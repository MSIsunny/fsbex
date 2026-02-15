use super::mpeg_fix::fix_fsb5_mpeg;
use crate::{header::StreamInfo, read::Reader};
use std::{
    error::Error,
    fmt::{Display, Formatter, Result as FmtResult},
    io::{copy, Error as IoError, Read, Write},
};

/// Encodes an MPEG stream by directly copying the raw stream data to the provided sink.
///
/// Unlike PCM or Vorbis, MPEG data in FSB banks is already framed/encoded and should be
/// written verbatim without modification or header construction.
pub(super) fn encode<R: Read, W: Write>(
    info: &StreamInfo,
    source: &mut Reader<R>,
    mut sink: W,
) -> Result<W, MpegError> {
    let stream_size = info.size.get() as usize;

    // Read raw MPEG bytes into a buffer (limit to stream size)
    let mut raw = Vec::with_capacity(stream_size);
    let _bytes_copied = copy(&mut source.limit(stream_size), &mut raw)
        .map_err(MpegError::from_io(MpegErrorKind::EncodeStream))?;

    // Apply FSB5-specific MPEG padding removal
    let fixed = fix_fsb5_mpeg(&raw);

    // Write the repaired stream
    sink.write_all(&fixed)
        .map_err(MpegError::from_io(MpegErrorKind::EncodeStream))?;

    Ok(sink)
}

/// See [`MpegErrorKind`] for the different kinds of errors that can occur.
#[derive(Debug)]
pub struct MpegError {
    kind: MpegErrorKind,
    source: IoError,
}

/// A variant of a [`MpegError`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum MpegErrorKind {
    /// Failed to write the ID3v2 header due to an underlying I/O error.
    CreateHeader,
    /// Failed to encode the entire stream via copying from reader to writer.
    EncodeStream,
}

impl MpegError {
    fn from_io(kind: MpegErrorKind) -> impl FnOnce(IoError) -> Self {
        move |source| Self { kind, source }
    }

    /// Returns the [`MpegErrorKind`] associated with this error.
    #[must_use]
    pub fn kind(&self) -> MpegErrorKind {
        self.kind
    }
}

impl Display for MpegError {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        self.kind.fmt(f)
    }
}

impl Error for MpegError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        Some(&self.source)
    }
}

impl Display for MpegErrorKind {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        f.write_str(match self {
            Self::CreateHeader => "failed to encode ID3v2 header",
            Self::EncodeStream => "failed to encode full MPEG stream",
        })
    }
}
