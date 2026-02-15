use std::cmp::min;

/// Remove FSB5-specific padding from raw MPEG data, mirroring the provided C# logic.
/// This function scans frames, calculates their length based on MPEG header fields,
/// copies each valid frame, and skips inter-frame padding (alignment to 4-byte boundaries)
/// and runs of zero bytes that FSB5 may insert.
///
/// Behavior notes (following the C# reference):
/// - A frame is identified by the 4-byte header beginning with 0xFF and next byte's high 4 bits == 0xF (sync).
/// - MPEG version and layer are decoded from the header; bitrate and sample rate are resolved via tables.
/// - Frame length is computed as:
///   * Layer I: (12 * bitrate * 1000 / sample_rate + padding) * 4
///   * Layer II and III: 144 * bitrate * 1000 / sample_rate + padding
///   (This mirrors the original C# tool; it does not distinguish MPEG-2/2.5 Layer III's 72 factor.)
/// - After each frame, if the next two bytes do not look like a header, seek to the next 4-byte-aligned
///   offset for the next frame and skip runs of zero bytes.
/// - Stops when remaining bytes are insufficient to read a header or full frame payload.
pub(super) fn fix_fsb5_mpeg(input: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(input.len());
    let mut pos: usize = 0;
    let end = input.len();

    while pos + 4 <= end {
        // Read 4-byte header
        let b0 = input[pos];
        let b1 = input[pos + 1];
        let b2 = input[pos + 2];
        let _b3 = input[pos + 3];

        // Validate basic sync (0xFF, next high nibble 0xF)
        if b0 != 0xFF || (b1 & 0xF0) != 0xF0 {
            // Not a header; advance by 1 and keep scanning
            pos += 1;
            continue;
        }

        // Decode MPEG version as in C#:
        // mpegVersion = 3 - ((header[1] >> 3) & 0x03)
        // -> maps to { 0: MPEG1, 1: MPEG2, 2: MPEG2.5 }
        let mpeg_version_index = 3u8.wrapping_sub((b1 >> 3) & 0x03);
        // layer = 4 - ((header[1] >> 1) & 0x03) -> 1,2,3
        let layer = 4i32 - ((b1 >> 1) & 0x03) as i32;
        if !(1..=3).contains(&layer) {
            pos += 1;
            continue;
        }

        let bitrate_index = ((b2 >> 4) & 0x0F) as usize;
        let sample_rate_index = ((b2 >> 2) & 0x03) as usize;
        let padding = ((b2 >> 1) & 0x01) as i32;

        // Resolve bitrate and sample rate
        let bitrate_kbps = get_mpeg_bitrate(mpeg_version_index, layer, bitrate_index);
        if bitrate_kbps <= 0 {
            pos += 1;
            continue;
        }
        let sample_rate = get_mpeg_sample_rate(mpeg_version_index as usize, sample_rate_index);
        if sample_rate <= 0 {
            pos += 1;
            continue;
        }

        // Compute frame length in bytes
        let frame_len = get_mpeg_frame_len_bytes(layer, bitrate_kbps, sample_rate, padding);
        if frame_len < 4 {
            pos += 1;
            continue;
        }
        // Ensure we have the full frame payload
        if pos + frame_len as usize > end {
            // Not enough data for full frame; stop
            break;
        }

        // Copy header + payload
        out.extend_from_slice(&input[pos..pos + frame_len as usize]);

        // Advance position
        pos += frame_len as usize;

        // Peek next 2 bytes; if not looking like an MPEG header, align and skip zeros
        if pos + 2 <= end && !(input[pos] == 0xFF && (input[pos + 1] & 0xF0) == 0xF0) {
            // Align to next 4-byte boundary based on the frame length just processed
            // Seek the difference between next multiple of 4 and the frame length
            let seek = next_multiple_of_4(frame_len) - frame_len;
            pos = min(pos + seek as usize, end);

            // Skip trailing zeros
            while pos < end && input[pos] == 0 {
                pos += 1;
            }
            if pos < end {
                // Step back one byte like the C# logic
                pos = pos.saturating_sub(1);
            }
        }
    }

    out
}

// Tables ported from the C# reference code

// MPEG-1 bitrates (kbps): Layer I/II/III
const V1_BITRATES_L1: [i32; 16] = [
    0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, -1,
];
const V1_BITRATES_L2: [i32; 16] = [
    0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384, -1,
];
const V1_BITRATES_L3: [i32; 16] = [
    0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, -1,
];

// MPEG-2/2.5 bitrates (kbps): Layer I and Layer II/III share the same table in the C# reference
const V2_BITRATES_L1: [i32; 16] = [
    0, 32, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176, 192, 224, 256, -1,
];
const V2_BITRATES_L2L3: [i32; 16] = [
    0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, -1,
];

// Sample rates per MPEG version (index 0..3)
const SAMPLE_RATES_V1: [i32; 4] = [44100, 48000, 32000, -1];
const SAMPLE_RATES_V2: [i32; 4] = [22050, 24000, 16000, -1];
const SAMPLE_RATES_V25: [i32; 4] = [11025, 12000, 8000, -1];

/// Return bitrate in kbps based on mpegVersion index (0:MPEG1, 1:MPEG2, 2:MPEG2.5),
/// layer (1..3), and bitrate index (0..15). Mirrors the C# logic.
fn get_mpeg_bitrate(mpeg_version_index: u8, mut layer: i32, bitrate_index: usize) -> i32 {
    // If MPEG version is 2.0 or 2.5 and Layer III, use Layer II table (per C#)
    if mpeg_version_index >= 1 && layer == 3 {
        layer = 2;
    }
    // Layer is 1..3, decrement to index into tables
    let layer_idx = (layer - 1) as usize;
    match mpeg_version_index {
        0 => match layer_idx {
            0 => V1_BITRATES_L1[bitrate_index],
            1 => V1_BITRATES_L2[bitrate_index],
            2 => V1_BITRATES_L3[bitrate_index],
            _ => -1,
        },
        _ => {
            // MPEG-2 or 2.5
            match layer_idx {
                0 => V2_BITRATES_L1[bitrate_index],
                1 | 2 => V2_BITRATES_L2L3[bitrate_index],
                _ => -1,
            }
        }
    }
}

/// Return sample rate in Hz based on mpegVersion index (0:MPEG1, 1:MPEG2, 2:MPEG2.5).
fn get_mpeg_sample_rate(mpeg_version_index: usize, sample_rate_index: usize) -> i32 {
    match mpeg_version_index {
        0 => SAMPLE_RATES_V1[sample_rate_index],
        1 => SAMPLE_RATES_V2[sample_rate_index],
        2 => SAMPLE_RATES_V25[sample_rate_index],
        _ => -1,
    }
}

/// Compute frame length in bytes based on layer, bitrate (kbps), sample rate (Hz), and padding.
/// Mirrors the C# logic (Layer I has special formula; Layer II/III share the 144 factor).
fn get_mpeg_frame_len_bytes(
    layer: i32,
    bitrate_kbps: i32,
    sample_rate_hz: i32,
    padding: i32,
) -> i32 {
    if layer == 1 {
        // Layer I: (12 * bitrate * 1000 / sample_rate + padding) * 4
        ((12 * bitrate_kbps * 1000) / sample_rate_hz + padding) * 4
    } else {
        // Layer II/III: 144 * bitrate * 1000 / sample_rate + padding
        (144 * bitrate_kbps * 1000) / sample_rate_hz + padding
    }
}

/// Get next multiple of 4 for the given number
fn next_multiple_of_4(n: i32) -> i32 {
    let rem = n % 4;
    if rem == 0 {
        n
    } else {
        n + (4 - rem)
    }
}
