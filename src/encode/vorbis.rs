use super::vorbis_lookup::VORBIS_LOOKUP;
use crate::header::StreamInfo;
use crate::read::{ReadError, Reader};
use lewton::{
    audio::get_decoded_sample_count,
    header::{read_header_ident, read_header_setup, IdentHeader, SetupHeader},
};
use std::{
    collections::HashMap,
    error::Error,
    fmt::{Display, Formatter, Result as FmtResult},
    io::{Error as IoError, Read, Write},
    sync::{Arc, LazyLock, Mutex},
};
use tap::Pipe;

const OGG_CAPTURE_PATTERN: &[u8; 4] = b"OggS";
const OGG_CONTINUED_PACKET: u8 = 0x01;
const OGG_BEGINNING_OF_STREAM: u8 = 0x02;
const OGG_END_OF_STREAM: u8 = 0x04;
const OGG_MAX_SEGMENTS_U8: u8 = u8::MAX;
const OGG_MAX_SEGMENTS: usize = OGG_MAX_SEGMENTS_U8 as usize;
const OGG_SEGMENT_SIZE_U8: u8 = u8::MAX;
const OGG_SEGMENT_SIZE: usize = OGG_SEGMENT_SIZE_U8 as usize;

const fn build_ogg_crc_table() -> [u32; 256] {
    let mut table = [0; 256];
    let mut index = 0;
    let mut prefix = 0_u32;
    while index < table.len() {
        let mut value = prefix << 24;
        let mut bit = 0;
        while bit < 8 {
            value = if value & 0x8000_0000 != 0 {
                (value << 1) ^ 0x04C1_1DB7
            } else {
                value << 1
            };
            bit += 1;
        }
        table[index] = value;
        index += 1;
        prefix += 1;
    }
    table
}

const OGG_CRC_TABLE: [u32; 256] = build_ogg_crc_table();
type SetupHeaderCache = Mutex<HashMap<(u32, u8), Arc<SetupHeader>>>;
static SETUP_HEADER_CACHE: LazyLock<SetupHeaderCache> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

struct VorbisHeaders {
    identification_data: Vec<u8>,
    setup_data: &'static [u8],
    identification: IdentHeader,
    setup: Arc<SetupHeader>,
}

fn update_ogg_crc(mut crc: u32, bytes: &[u8]) -> u32 {
    for &byte in bytes {
        let index = ((crc >> 24) as u8 ^ byte) as usize;
        crc = (crc << 8) ^ OGG_CRC_TABLE[index];
    }
    crc
}

struct OggWriter<W> {
    sink: W,
    serial: u32,
    sequence: u32,
}

impl<W: Write> OggWriter<W> {
    fn new(sink: W, serial: u32) -> Self {
        Self {
            sink,
            serial,
            sequence: 0,
        }
    }

    fn write_packet(
        &mut self,
        packet: &[u8],
        granule_position: u64,
        beginning_of_stream: bool,
        end_of_stream: bool,
    ) -> Result<(), IoError> {
        let mut offset = 0;
        let mut first_page = true;

        loop {
            let remaining = packet.len() - offset;
            let full_segments = remaining / OGG_SEGMENT_SIZE;
            let completes_packet = full_segments < OGG_MAX_SEGMENTS;
            let segment_count = if completes_packet {
                full_segments + 1
            } else {
                OGG_MAX_SEGMENTS
            };
            let page_data_len = if completes_packet {
                remaining
            } else {
                OGG_MAX_SEGMENTS * OGG_SEGMENT_SIZE
            };

            let mut header_type = 0;
            if !first_page {
                header_type |= OGG_CONTINUED_PACKET;
            }
            if beginning_of_stream && first_page {
                header_type |= OGG_BEGINNING_OF_STREAM;
            }
            if end_of_stream && completes_packet {
                header_type |= OGG_END_OF_STREAM;
            }

            let page_granule = if completes_packet {
                granule_position
            } else {
                u64::MAX
            };
            let page_data = &packet[offset..offset + page_data_len];
            self.write_page(header_type, page_granule, segment_count, page_data)?;

            offset += page_data_len;
            if completes_packet {
                break;
            }
            first_page = false;
        }

        Ok(())
    }

    fn write_page(
        &mut self,
        header_type: u8,
        granule_position: u64,
        segment_count: usize,
        data: &[u8],
    ) -> Result<(), IoError> {
        let mut header = Vec::with_capacity(27 + segment_count);
        header.extend_from_slice(OGG_CAPTURE_PATTERN);
        header.push(0);
        header.push(header_type);
        header.extend_from_slice(&granule_position.to_le_bytes());
        header.extend_from_slice(&self.serial.to_le_bytes());
        header.extend_from_slice(&self.sequence.to_le_bytes());
        header.extend_from_slice(&[0; 4]);
        header.push(u8::try_from(segment_count).expect("Ogg page has at most 255 segments"));

        let full_segments = data.len() / OGG_SEGMENT_SIZE;
        header.extend(std::iter::repeat_n(
            OGG_SEGMENT_SIZE_U8,
            full_segments.min(segment_count),
        ));
        if segment_count > full_segments {
            header.push(
                u8::try_from(data.len() % OGG_SEGMENT_SIZE)
                    .expect("Ogg lacing value is less than 255"),
            );
        }

        let crc = update_ogg_crc(update_ogg_crc(0, &header), data);
        header[22..26].copy_from_slice(&crc.to_le_bytes());

        self.sink.write_all(&header)?;
        self.sink.write_all(data)?;
        self.sequence = self.sequence.wrapping_add(1);
        Ok(())
    }

    fn finish(self) -> W {
        self.sink
    }
}

pub(super) fn encode<R: Read, W: Write>(
    info: &StreamInfo,
    source: &mut Reader<R>,
    sink: W,
) -> Result<W, VorbisError> {
    // The stream should have contained the CRC32 of a setup header in a header chunk.
    // Otherwise, the stream cannot be encoded correctly.
    let crc32 = info
        .vorbis_crc32
        .ok_or_else(|| VorbisError::new(VorbisErrorKind::MissingCrc32))?;

    // Construct the three Vorbis headers required by an Ogg stream. FSB stores
    // raw Vorbis packets and identifies the shared setup header by its CRC32.
    let headers = init_headers(info.sample_rate.get(), info.channels.get(), crc32)?;
    let comment_header = init_comment_header_data();
    let serial = crc32 ^ info.sample_rate.get() ^ u32::from(info.channels.get());
    let mut writer = OggWriter::new(sink, serial);
    writer
        .write_packet(&headers.identification_data, 0, true, false)
        .map_err(VorbisError::from_io(VorbisErrorKind::WriteStream))?;
    writer
        .write_packet(&comment_header, 0, false, false)
        .map_err(VorbisError::from_io(VorbisErrorKind::WriteStream))?;
    writer
        .write_packet(headers.setup_data, 0, false, false)
        .map_err(VorbisError::from_io(VorbisErrorKind::WriteStream))?;

    let start_pos = source.position();
    let stream_size = info.size.get() as usize;
    let mut granule_position = 0_u64;
    let mut first_audio_packet = true;
    let mut pending_packet = None;

    while source.position() - start_pos < stream_size {
        // FSB data is commonly padded or ends with an incomplete packet-size
        // field. Treat it as the end of the stream, matching FMOD's behavior.
        let Ok(packet_size) = source.le_u16() else {
            break;
        };

        // signals end of stream data
        if packet_size == u16::MIN || packet_size == u16::MAX {
            break;
        }

        let packet = source
            .take(packet_size as usize)
            .map_err(VorbisError::from_read(VorbisErrorKind::ReadPacket))?;

        let decoded_samples =
            get_decoded_sample_count(&headers.identification, &headers.setup, &packet)
                .map_err(Into::into)
                .map_err(VorbisError::from_lewton(VorbisErrorKind::DecodePacket))?;
        if first_audio_packet {
            first_audio_packet = false;
        } else {
            granule_position = granule_position.saturating_add(decoded_samples as u64);
        }

        if let Some((previous_packet, previous_granule)) =
            pending_packet.replace((packet, granule_position))
        {
            writer
                .write_packet(&previous_packet, previous_granule, false, false)
                .map_err(VorbisError::from_io(VorbisErrorKind::WriteStream))?;
        }
    }

    if let Some((last_packet, _)) = pending_packet {
        writer
            .write_packet(&last_packet, u64::from(info.num_samples.get()), false, true)
            .map_err(VorbisError::from_io(VorbisErrorKind::WriteStream))?;
    }

    Ok(writer.finish())
}

// default block sizes for FMOD sound banks:
// minimum 256 samples; maximum 2048 samples
const MIN_BLOCK_SIZE_EXP2: u8 = 8;
const MAX_BLOCK_SIZE_EXP2: u8 = 11;

fn init_headers(sample_rate: u32, channels: u8, crc32: u32) -> Result<VorbisHeaders, VorbisError> {
    // construct identification header from scratch
    let id_header_data = init_id_header_data(sample_rate, channels)
        .expect("writing to an in-memory buffer is infallible");
    let id_header = id_header_data
        .pipe_as_ref(read_header_ident)
        .map_err(Into::into)
        .map_err(VorbisError::from_lewton(VorbisErrorKind::CreateHeaders))?;

    // construct setup header from lookup table
    let setup_header_data = *VORBIS_LOOKUP
        .get(&crc32)
        .ok_or_else(|| VorbisError::new(VorbisErrorKind::Crc32Lookup))?;

    let cache_key = (crc32, channels);
    let setup_header = SETUP_HEADER_CACHE
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .get(&cache_key)
        .cloned();
    let setup_header = if let Some(setup_header) = setup_header {
        setup_header
    } else {
        let parsed = Arc::new(
            read_header_setup(
                setup_header_data,
                channels,
                (MIN_BLOCK_SIZE_EXP2, MAX_BLOCK_SIZE_EXP2),
            )
            .map_err(Into::into)
            .map_err(VorbisError::from_lewton(VorbisErrorKind::CreateHeaders))?,
        );
        SETUP_HEADER_CACHE
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .entry(cache_key)
            .or_insert_with(|| Arc::clone(&parsed))
            .clone()
    };

    Ok(VorbisHeaders {
        identification_data: id_header_data,
        setup_data: setup_header_data,
        identification: id_header,
        setup: setup_header,
    })
}

fn init_comment_header_data() -> Vec<u8> {
    const VENDOR: &[u8] = b"fsbex";

    let mut data = Vec::with_capacity(16 + VENDOR.len());
    data.extend_from_slice(&[3]);
    data.extend_from_slice(b"vorbis");
    data.extend_from_slice(
        &u32::try_from(VENDOR.len())
            .expect("Vorbis vendor string length fits in u32")
            .to_le_bytes(),
    );
    data.extend_from_slice(VENDOR);
    data.extend_from_slice(&0_u32.to_le_bytes());
    data.push(1);
    data
}

fn init_id_header_data(sample_rate: u32, channels: u8) -> Result<Vec<u8>, IoError> {
    // Vorbis file header information taken from:
    // [1]: https://www.xiph.org/vorbis/doc/Vorbis_I_spec.html (sections 4.2.1 and 4.2.2)

    const BLOCK_SIZES: u8 = (MAX_BLOCK_SIZE_EXP2 << 4) | (MIN_BLOCK_SIZE_EXP2);

    let mut data = Vec::with_capacity(30);

    data.write_all(&[1])?;
    data.write_all(b"vorbis")?;
    data.write_all(&[0; 4])?;
    data.write_all(&[channels])?;
    data.write_all(&sample_rate.to_le_bytes())?;
    data.write_all(&[0; 4])?;
    data.write_all(&[0; 4])?;
    data.write_all(&[0; 4])?;
    data.write_all(&[BLOCK_SIZES])?;
    data.write_all(&[1])?;

    Ok(data)
}

/// Represents an error that can occur when encoding a Vorbis stream.
///
/// See [`VorbisErrorKind`] for the different kinds of errors that can occur.
#[derive(Debug)]
pub struct VorbisError {
    kind: VorbisErrorKind,
    source: Option<VorbisErrorSource>,
}

/// A variant of a [`VorbisError`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum VorbisErrorKind {
    /// A CRC32 checksum was not found in the stream header within the sound bank.
    /// This checksum is needed to reconstruct the Vorbis decoder state and encode audio samples.
    MissingCrc32,
    /// Failed to create the file headers needed for the Vorbis decoder.
    CreateHeaders,
    /// The stream's associated CRC32 checksum was found, but it did not match any existing entries in the lookup table.
    Crc32Lookup,
    /// Failed to create the Vorbis encoder for writing audio samples.
    CreateEncoder,
    /// Failed to read an audio packet from the stream data.
    ReadPacket,
    /// Failed to decode an audio packet from the stream data into a sample.
    DecodePacket,
    /// Failed to encode an audio sample to the writer.
    EncodeBlock,
    /// Failed to flush the writer after encoding the entire stream.
    FinishStream,
    /// Failed to write the reconstructed Ogg stream.
    WriteStream,
}

#[derive(Debug)]
enum VorbisErrorSource {
    Decode(lewton::VorbisError),
    Io(IoError),
    Read(ReadError),
}

impl VorbisError {
    fn new(kind: VorbisErrorKind) -> Self {
        Self { kind, source: None }
    }

    fn from_lewton(kind: VorbisErrorKind) -> impl FnOnce(lewton::VorbisError) -> Self {
        move |source| Self {
            kind,
            source: Some(VorbisErrorSource::Decode(source)),
        }
    }

    fn from_io(kind: VorbisErrorKind) -> impl FnOnce(IoError) -> Self {
        move |source| Self {
            kind,
            source: Some(VorbisErrorSource::Io(source)),
        }
    }

    fn from_read(kind: VorbisErrorKind) -> impl FnOnce(ReadError) -> Self {
        move |source| Self {
            kind,
            source: Some(VorbisErrorSource::Read(source)),
        }
    }

    /// Returns the [`VorbisErrorKind`] associated with this error.
    #[must_use]
    pub fn kind(&self) -> VorbisErrorKind {
        self.kind
    }
}

impl Display for VorbisError {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        self.kind.fmt(f)
    }
}

impl Error for VorbisError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match &self.source {
            Some(source) => match source {
                VorbisErrorSource::Decode(e) => Some(e),
                VorbisErrorSource::Io(e) => Some(e),
                VorbisErrorSource::Read(e) => Some(e),
            },
            None => None,
        }
    }
}

impl Display for VorbisErrorKind {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        f.write_str(match self {
            Self::MissingCrc32 => "file header did not contain CRC32 of Vorbis setup header",
            Self::CreateHeaders => "failed to create dummy Vorbis headers",
            Self::Crc32Lookup => "CRC32 of Vorbis setup header was not found in lookup table",
            Self::CreateEncoder => "failed to create Vorbis stream encoder",
            Self::ReadPacket => "failed to read audio packet from Vorbis stream",
            Self::DecodePacket => "failed to decode audio packet from Vorbis stream",
            Self::EncodeBlock => "failed to encode block of samples",
            Self::FinishStream => "failed to finalize writing Vorbis stream data",
            Self::WriteStream => "failed to write reconstructed Ogg stream",
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn page_len(page: &[u8]) -> usize {
        let segment_count = usize::from(page[26]);
        27 + segment_count
            + page[27..27 + segment_count]
                .iter()
                .map(|&size| usize::from(size))
                .sum::<usize>()
    }

    fn assert_valid_page_crc(page: &[u8]) {
        let expected = u32::from_le_bytes(page[22..26].try_into().unwrap());
        let mut header = page[..27 + usize::from(page[26])].to_vec();
        header[22..26].fill(0);
        let actual = update_ogg_crc(update_ogg_crc(0, &header), &page[header.len()..]);
        assert_eq!(actual, expected);
    }

    #[test]
    fn ogg_writer_handles_packet_continuation_and_terminal_lacing() {
        let packet = vec![0xA5; OGG_MAX_SEGMENTS * OGG_SEGMENT_SIZE];
        let mut writer = OggWriter::new(Cursor::new(Vec::new()), 7);
        writer.write_packet(&packet, 42, true, true).unwrap();
        let output = writer.finish().into_inner();

        let first_len = page_len(&output);
        let first = &output[..first_len];
        let second = &output[first_len..];

        assert_eq!(&first[..4], OGG_CAPTURE_PATTERN);
        assert_eq!(first[5], OGG_BEGINNING_OF_STREAM);
        assert_eq!(u64::from_le_bytes(first[6..14].try_into().unwrap()), u64::MAX);
        assert_eq!(first[26], OGG_MAX_SEGMENTS_U8);
        assert!(first[27..27 + OGG_MAX_SEGMENTS]
            .iter()
            .all(|&lace| lace == OGG_SEGMENT_SIZE_U8));
        assert_valid_page_crc(first);

        assert_eq!(&second[..4], OGG_CAPTURE_PATTERN);
        assert_eq!(second[5], OGG_CONTINUED_PACKET | OGG_END_OF_STREAM);
        assert_eq!(u64::from_le_bytes(second[6..14].try_into().unwrap()), 42);
        assert_eq!(second[26], 1);
        assert_eq!(second[27], 0);
        assert_valid_page_crc(second);
    }

    #[test]
    fn vorbis_comment_header_has_required_framing_bit() {
        let header = init_comment_header_data();
        assert_eq!(&header[..7], b"\x03vorbis");
        assert_eq!(header.last(), Some(&1));
    }
}
