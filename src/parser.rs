use crate::constants::CRC_LEN;
use crate::constants::CRC8_DVBS2_POLY;
use crate::constants::HEADER_LEN;
use crate::constants::MIN_FRAME_LEN;
use crate::constants::MIN_FRAME_LEN_U32;
use crate::constants::PREAMBLE_LEN;
use crate::constants::V1_CHECKSUM_LEN;
use crate::constants::V1_ENCAPSULATED_CMD;
use crate::constants::V1_HEADER_LEN;
use crate::constants::V1_JUMBO_LENGTH_LEN;
use crate::constants::V1_JUMBO_SIZE;
use crate::constants::V1_MIN_FRAME_LEN;
use crate::constants::V2_INNER_OVERHEAD;
use crate::constants::V2_OVER_V1_HEADER_LEN;

/// MSP framing version. Jumbo frames retain their underlying version.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum MspVersion {
    /// MSP v1, ordinary or jumbo.
    V1,
    /// MSP v2 encapsulated in an ordinary or jumbo v1 frame.
    V2OverV1,
    /// Native MSP v2.
    V2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MspDirection {
    Request,  // '<'
    Response, // '>'
    Error,    // '!'
}

impl MspDirection {
    #[must_use]
    fn from_byte(b: u8) -> Option<Self> {
        match b {
            b'<' => Some(Self::Request),
            b'>' => Some(Self::Response),
            b'!' => Some(Self::Error),
            _ => None,
        }
    }
}

/// Updates a running CRC-8 (DVB-S2) with a single byte.
#[inline]
#[must_use]
fn crc8_dvbs2_byte(mut crc: u8, byte: u8) -> u8 {
    crc ^= byte;
    for _ in 0..8 {
        if (crc & 0x80) != 0 {
            crc = (crc << 1) ^ CRC8_DVBS2_POLY;
        } else {
            crc <<= 1;
        }
    }
    crc
}

/// Computes the CRC-8 (DVB-S2) checksum over a byte slice with an initial value.
#[inline]
#[must_use]
pub fn crc8_dvbs2_slice(initial: u8, data: &[u8]) -> u8 {
    data.iter()
        .fold(initial, |crc, &byte| crc8_dvbs2_byte(crc, byte))
}

/// Why a standalone byte slice could not be validated as an MSP frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum MspFrameError {
    /// More bytes are needed; `required` is the next validation threshold.
    Incomplete { required: usize, actual: usize },
    /// The frame does not begin with `$M` or `$X`.
    InvalidPreamble,
    /// The direction byte is neither `<`, `>`, nor `!`.
    InvalidDirection,
    /// The outer payload size differs from the inner header plus payload and CRC.
    /// `expected` is the minimum overhead when the outer size cannot hold an inner header.
    InvalidEncapsulationLength { expected: usize, actual: usize },
    /// The complete frame length cannot be represented on this target.
    LengthOverflow,
    /// A complete frame is followed by additional bytes.
    LengthMismatch { expected: usize, actual: usize },
    /// The wire checksum differs from the calculated checksum.
    ChecksumMismatch { expected: u8, actual: u8 },
}

impl core::fmt::Display for MspFrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Incomplete { required, actual } => {
                write!(
                    f,
                    "incomplete MSP frame: need {required} bytes, got {actual}"
                )
            }
            Self::InvalidPreamble => f.write_str("invalid MSP preamble"),
            Self::InvalidDirection => f.write_str("invalid MSP direction"),
            Self::InvalidEncapsulationLength { expected, actual } => write!(
                f,
                "invalid MSP encapsulation length: expected {expected} bytes, got {actual}"
            ),
            Self::LengthOverflow => f.write_str("MSP frame length exceeds target address space"),
            Self::LengthMismatch { expected, actual } => {
                write!(
                    f,
                    "MSP frame length mismatch: expected {expected} bytes, got {actual}"
                )
            }
            Self::ChecksumMismatch { expected, actual } => {
                write!(
                    f,
                    "MSP checksum mismatch: expected {expected:#04x}, got {actual:#04x}"
                )
            }
        }
    }
}

impl core::error::Error for MspFrameError {}

/// An internal parser failure, distinct from recoverable malformed wire data.
///
/// Reset or replace the parser before continuing after an error. Input may have
/// advanced before the error was reported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum MspParserError {
    /// Buffer positions or a requested frame range violate the parser invariant.
    InvalidBufferState,
    /// A requested append would exceed the spill buffer capacity.
    BufferCapacityExceeded,
}

impl core::fmt::Display for MspParserError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidBufferState => f.write_str("invalid MSP parser buffer state"),
            Self::BufferCapacityExceeded => {
                f.write_str("MSP parser spill buffer capacity exceeded")
            }
        }
    }
}

impl core::error::Error for MspParserError {}

/// A complete MSP frame with a valid header and checksum(s).
///
/// The borrowed bytes include the header, payload, and checksum. Construction is
/// private so this type always represents a validated frame. Header fields are
/// decoded during validation; payload and wire bytes remain borrowed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspFrame<'a> {
    bytes: &'a [u8],
    version: MspVersion,
    direction: MspDirection,
    flags: u8,
    command_id: u16,
    payload: &'a [u8],
    checksum: u8,
}

impl<'a> MspFrame<'a> {
    /// Validates a standalone byte slice as a complete MSP frame.
    ///
    /// Discards validation diagnostics. Use [`Self::try_from`] for detailed errors.
    #[must_use]
    pub fn from_slice(bytes: &'a [u8]) -> Option<Self> {
        Self::try_from(bytes).ok()
    }

    /// Returns the complete wire frame, including its checksum.
    #[must_use]
    pub fn as_bytes(&self) -> &'a [u8] {
        self.bytes
    }

    /// Returns the framing version (jumbo frames retain their underlying version).
    #[must_use]
    pub fn version(&self) -> MspVersion {
        self.version
    }

    /// Returns the direction of this frame (request, response, or error).
    #[must_use]
    pub fn direction(&self) -> MspDirection {
        self.direction
    }

    /// Returns the 8-bit flag field, or zero for v1.
    #[must_use]
    pub fn flags(&self) -> u8 {
        self.flags
    }

    /// Returns the 16-bit command ID.
    #[must_use]
    pub fn command_id(&self) -> u16 {
        self.command_id
    }

    /// Returns the payload slice borrowing directly from the frame.
    #[must_use]
    pub fn payload(&self) -> &'a [u8] {
        self.payload
    }

    /// Returns the final wire checksum: outer XOR for v1 formats, CRC for native v2.
    #[must_use]
    pub fn checksum(&self) -> u8 {
        self.checksum
    }
}

impl core::ops::Deref for MspFrame<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.bytes
    }
}

impl AsRef<[u8]> for MspFrame<'_> {
    fn as_ref(&self) -> &[u8] {
        self.bytes
    }
}

impl<'a> TryFrom<&'a [u8]> for MspFrame<'a> {
    type Error = MspFrameError;

    fn try_from(bytes: &'a [u8]) -> Result<Self, Self::Error> {
        match inspect_frame(bytes) {
            FrameInspection::Invalid(error) => Err(error),
            FrameInspection::Incomplete(required) => Err(MspFrameError::Incomplete {
                required,
                actual: bytes.len(),
            }),
            FrameInspection::Complete(frame) => frame.into_frame(bytes),
        }
    }
}

/// Owned frame metadata, so inspection does not borrow across buffer mutation.
/// Constructed by frame inspection after checking the complete frame and checksums.
struct ValidatedFrameMetadata {
    len: usize,
    payload_offset: usize,
    version: MspVersion,
    direction: MspDirection,
    flags: u8,
    command_id: u16,
    checksum: u8,
}

impl ValidatedFrameMetadata {
    /// Attaches the validated fields to the exact bytes that were inspected.
    /// Callers must preserve those bytes between inspection and borrowing.
    fn into_frame(self, bytes: &[u8]) -> Result<MspFrame<'_>, MspFrameError> {
        if bytes.len() != self.len {
            return Err(MspFrameError::LengthMismatch {
                expected: self.len,
                actual: bytes.len(),
            });
        }
        let trailer_len = match self.version {
            MspVersion::V1 => V1_CHECKSUM_LEN,
            MspVersion::V2OverV1 => CRC_LEN + V1_CHECKSUM_LEN,
            MspVersion::V2 => CRC_LEN,
        };
        let payload = bytes
            .get(self.payload_offset..self.len - trailer_len)
            .ok_or(MspFrameError::Incomplete {
                required: self.payload_offset + trailer_len,
                actual: bytes.len(),
            })?;
        Ok(MspFrame {
            bytes,
            version: self.version,
            direction: self.direction,
            flags: self.flags,
            command_id: self.command_id,
            payload,
            checksum: self.checksum,
        })
    }
}

enum FrameInspection {
    Invalid(MspFrameError),
    Incomplete(usize),
    Complete(ValidatedFrameMetadata),
}

fn inspect_frame(bytes: &[u8]) -> FrameInspection {
    if let [b'$', b'M', ..] = bytes {
        return inspect_v1_frame(bytes);
    }
    // Reject incorrect preamble bytes as soon as they arrive, but retain valid prefixes.
    if bytes
        .iter()
        .zip(b"$X")
        .any(|(actual, expected)| actual != expected)
    {
        return FrameInspection::Invalid(MspFrameError::InvalidPreamble);
    }
    if bytes.len() < 2 {
        return FrameInspection::Incomplete(2);
    }
    let Some(&direction) = bytes.get(2) else {
        return FrameInspection::Incomplete(HEADER_LEN);
    };
    let Some(direction) = MspDirection::from_byte(direction) else {
        return FrameInspection::Invalid(MspFrameError::InvalidDirection);
    };
    let Some(header) = bytes
        .get(..HEADER_LEN)
        .and_then(|header| <&[u8; HEADER_LEN]>::try_from(header).ok())
    else {
        return FrameInspection::Incomplete(HEADER_LEN);
    };
    let &[_, _, _, flags, command_id_lo, command_id_hi, len_lo, len_hi] = header;
    let command_id = u16::from_le_bytes([command_id_lo, command_id_hi]);
    // Add MIN_FRAME_LEN as u32 so even a 16-bit target cannot overflow before checking usize.
    let payload_len = u32::from(u16::from_le_bytes([len_lo, len_hi]));
    let Ok(frame_len) = usize::try_from(payload_len + MIN_FRAME_LEN_U32) else {
        return FrameInspection::Invalid(MspFrameError::LengthOverflow);
    };
    if bytes.len() < frame_len {
        return FrameInspection::Incomplete(frame_len);
    }
    let checksum_idx = frame_len - CRC_LEN;
    let Some(crc_slice) = bytes.get(PREAMBLE_LEN..checksum_idx) else {
        return FrameInspection::Invalid(MspFrameError::LengthMismatch {
            expected: frame_len,
            actual: bytes.len(),
        });
    };
    let Some(&received_crc) = bytes.get(checksum_idx) else {
        return FrameInspection::Invalid(MspFrameError::LengthMismatch {
            expected: frame_len,
            actual: bytes.len(),
        });
    };
    let calculated_crc = crc8_dvbs2_slice(0, crc_slice);
    if calculated_crc != received_crc {
        return FrameInspection::Invalid(MspFrameError::ChecksumMismatch {
            expected: calculated_crc,
            actual: received_crc,
        });
    }
    FrameInspection::Complete(ValidatedFrameMetadata {
        len: frame_len,
        payload_offset: HEADER_LEN,
        version: MspVersion::V2,
        direction,
        flags,
        command_id,
        checksum: received_crc,
    })
}

fn inspect_v1_frame(bytes: &[u8]) -> FrameInspection {
    let Some(&direction) = bytes.get(2) else {
        return FrameInspection::Incomplete(V1_HEADER_LEN);
    };
    let Some(direction) = MspDirection::from_byte(direction) else {
        return FrameInspection::Invalid(MspFrameError::InvalidDirection);
    };
    let Some(header) = bytes.get(..V1_HEADER_LEN) else {
        return FrameInspection::Incomplete(V1_HEADER_LEN);
    };
    let [_, _, _, size, command] = *header else {
        return FrameInspection::Incomplete(V1_HEADER_LEN);
    };
    let encapsulated = command == V1_ENCAPSULATED_CMD;
    let jumbo = size == V1_JUMBO_SIZE;
    if encapsulated && !jumbo && usize::from(size) < V2_INNER_OVERHEAD {
        return FrameInspection::Invalid(MspFrameError::InvalidEncapsulationLength {
            expected: V2_INNER_OVERHEAD,
            actual: usize::from(size),
        });
    }
    // Betaflight places the jumbo length AFTER the inner v2 header.
    let base_offset = if encapsulated {
        V2_OVER_V1_HEADER_LEN
    } else {
        V1_HEADER_LEN
    };
    let payload_offset = base_offset + if jumbo { V1_JUMBO_LENGTH_LEN } else { 0 };
    let Some(header) = bytes.get(..payload_offset) else {
        return FrameInspection::Incomplete(payload_offset);
    };
    let outer_size = if jumbo {
        let Some(&[lo, hi]) = header.get(base_offset..) else {
            return FrameInspection::Incomplete(payload_offset);
        };
        u16::from_le_bytes([lo, hi])
    } else {
        u16::from(size)
    };
    let (version, flags, command_id) = if encapsulated {
        let Some(&[flags, lo, hi, len_lo, len_hi]) =
            header.get(V1_HEADER_LEN..V2_OVER_V1_HEADER_LEN)
        else {
            return FrameInspection::Incomplete(V2_OVER_V1_HEADER_LEN);
        };
        let expected = u32::from(u16::from_le_bytes([len_lo, len_hi])) + V2_INNER_OVERHEAD as u32;
        if expected != u32::from(outer_size) {
            let Ok(expected) = usize::try_from(expected) else {
                return FrameInspection::Invalid(MspFrameError::LengthOverflow);
            };
            return FrameInspection::Invalid(MspFrameError::InvalidEncapsulationLength {
                expected,
                actual: usize::from(outer_size),
            });
        }
        (MspVersion::V2OverV1, flags, u16::from_le_bytes([lo, hi]))
    } else {
        (MspVersion::V1, 0, u16::from(command))
    };
    let Ok(frame_len) = usize::try_from(
        u32::from(outer_size)
            + V1_MIN_FRAME_LEN as u32
            + if jumbo { V1_JUMBO_LENGTH_LEN as u32 } else { 0 },
    ) else {
        return FrameInspection::Invalid(MspFrameError::LengthOverflow);
    };
    if bytes.len() < frame_len {
        return FrameInspection::Incomplete(frame_len);
    }
    let Some(frame_bytes) = bytes.get(..frame_len) else {
        return FrameInspection::Incomplete(frame_len);
    };
    let checksum_idx = frame_len - V1_CHECKSUM_LEN;
    let Some((&actual, body)) = frame_bytes.get(PREAMBLE_LEN..).and_then(|s| s.split_last()) else {
        return FrameInspection::Incomplete(frame_len);
    };
    let expected = body.iter().fold(0, |sum, byte| sum ^ byte);
    if expected != actual {
        return FrameInspection::Invalid(MspFrameError::ChecksumMismatch { expected, actual });
    }
    if encapsulated {
        let Some(inner_header) = frame_bytes.get(V1_HEADER_LEN..V2_OVER_V1_HEADER_LEN) else {
            return FrameInspection::Incomplete(V2_OVER_V1_HEADER_LEN);
        };
        let Some(inner_body) = frame_bytes
            .get(payload_offset..checksum_idx)
            .and_then(|s| s.split_last())
        else {
            return FrameInspection::Incomplete(frame_len);
        };
        let (&actual, payload) = inner_body;
        let expected = crc8_dvbs2_slice(crc8_dvbs2_slice(0, inner_header), payload);
        if expected != actual {
            return FrameInspection::Invalid(MspFrameError::ChecksumMismatch { expected, actual });
        }
    }
    FrameInspection::Complete(ValidatedFrameMetadata {
        len: frame_len,
        payload_offset,
        version,
        direction,
        flags,
        command_id,
        checksum: actual,
    })
}

/// Splits a byte stream into validated MSP frames without allocation.
///
/// Complete frames borrow the input directly. Frames spanning chunks use the
/// internal `N`-byte spill buffer. `N` limits only split frames: larger contiguous
/// frames are accepted, but larger incomplete candidates are skipped. Malformed
/// candidates are skipped by scanning for the next `$` to start validation again.
pub struct MspParser<const N: usize> {
    buffer: [u8; N],
    start: usize,
    end: usize,
}

impl<const N: usize> MspParser<N> {
    /// Creates an empty parser.
    ///
    /// `N` must be at least 9, the minimum native MSP v2 frame size.
    /// Smaller capacities fail at compile time, even for runtime construction.
    /// Split v2-over-v1 frames need at least 12 bytes (14 for jumbo), plus payload;
    /// smaller buffers skip those candidates. Complete contiguous frames are
    /// accepted regardless of spill capacity.
    ///
    /// ```compile_fail,E0080
    /// use uf_msp::MspParser;
    ///
    /// let parser = MspParser::<8>::new();
    /// ```
    #[must_use]
    pub const fn new() -> Self {
        const {
            assert!(
                N >= MIN_FRAME_LEN,
                "MSP spill capacity must be at least 9 bytes"
            );
        }
        Self {
            buffer: [0; N],
            start: 0,
            end: 0,
        }
    }

    /// Discards any retained bytes, for example after a transport disconnect.
    pub fn reset(&mut self) {
        self.start = 0;
        self.end = 0;
    }

    /// Returns the currently unconsumed bytes in the spill buffer.
    #[inline]
    fn buffered(&self) -> Result<&[u8], MspParserError> {
        self.buffer
            .get(self.start..self.end)
            .ok_or(MspParserError::InvalidBufferState)
    }

    /// Returns `true` if there are unconsumed bytes in the spill buffer.
    #[inline]
    fn has_buffered(&self) -> bool {
        self.start < self.end
    }

    /// Skips a rejected candidate and scans for the next possible preamble.
    /// Starting after the first byte guarantees progress, while searching only
    /// for `$` preserves partial preambles at chunk boundaries.
    #[inline]
    fn next_candidate(bytes: &[u8]) -> &[u8] {
        let remaining = bytes.get(1..).unwrap_or(&[]);
        let offset = remaining
            .iter()
            .position(|&byte| byte == b'$')
            .unwrap_or(remaining.len());
        remaining.get(offset..).unwrap_or(&[])
    }

    /// Advances the spill buffer to the next possible preamble.
    #[inline]
    fn skip_buffered_candidate(&mut self) -> Result<(), MspParserError> {
        self.start = self.end - Self::next_candidate(self.buffered()?).len();
        Ok(())
    }

    /// Compacts the spill buffer so remaining bytes begin at index 0.
    #[inline]
    fn compact(&mut self) {
        if self.start > 0 {
            self.buffer.copy_within(self.start..self.end, 0);
            self.end -= self.start;
            self.start = 0;
        }
    }

    /// Borrows an inspected frame from the spill buffer and consumes its bytes.
    #[inline]
    fn consume_buffered_frame(
        &mut self,
        frame: ValidatedFrameMetadata,
    ) -> Result<MspFrame<'_>, MspParserError> {
        let end = self
            .start
            .checked_add(frame.len)
            .ok_or(MspParserError::InvalidBufferState)?;
        if end > self.end {
            return Err(MspParserError::InvalidBufferState);
        }
        let bytes = self
            .buffer
            .get(self.start..end)
            .ok_or(MspParserError::InvalidBufferState)?;
        let frame = frame
            .into_frame(bytes)
            .map_err(|_invalid_frame| MspParserError::InvalidBufferState)?;
        self.start = end;
        Ok(frame)
    }

    /// Copies up to `count` bytes from `input` into the spill buffer, advancing `input`.
    #[inline]
    fn pull_from_input(&mut self, input: &mut &[u8], count: usize) -> Result<(), MspParserError> {
        if self.start > self.end || self.end > N {
            return Err(MspParserError::InvalidBufferState);
        }
        let take = count.min(input.len());
        let end = self
            .end
            .checked_add(take)
            .ok_or(MspParserError::BufferCapacityExceeded)?;
        let dst = self
            .buffer
            .get_mut(self.end..end)
            .ok_or(MspParserError::BufferCapacityExceeded)?;
        let (head, tail) = input.split_at(take);
        dst.copy_from_slice(head);
        self.end = end;
        *input = tail;
        Ok(())
    }

    /// Returns the next valid frame and advances `input` past consumed bytes.
    ///
    /// Call repeatedly until `Ok(None)`, which consumes all input and retains any
    /// incomplete candidate that fits the spill buffer. Empty input can drain
    /// frames retained during recovery from a corrupt candidate.
    ///
    /// The result borrows either the input or the spill buffer. Finish using it
    /// before calling the parser again.
    ///
    /// # Errors
    ///
    /// Reports invalid internal buffer state or an impossible append. Malformed
    /// wire candidates are skipped without returning an error. After an error,
    /// reset or replace the parser; input may already have advanced.
    pub fn next<'a, 'input: 'a>(
        &'a mut self,
        input: &mut &'input [u8],
    ) -> Result<Option<MspFrame<'a>>, MspParserError> {
        if self.start > self.end || self.end > N {
            return Err(MspParserError::InvalidBufferState);
        }
        loop {
            // Process any data currently in the spill buffer.
            if self.has_buffered() {
                match inspect_frame(self.buffered()?) {
                    FrameInspection::Invalid(_) => self.skip_buffered_candidate()?,
                    FrameInspection::Complete(frame) => {
                        return self.consume_buffered_frame(frame).map(Some);
                    }
                    FrameInspection::Incomplete(required) => {
                        if required > N {
                            // Candidate exceeds spill capacity; scan for the next preamble.
                            self.skip_buffered_candidate()?;
                            continue;
                        }
                        self.compact();
                        self.pull_from_input(input, required - self.end)?;
                        if self.end < required {
                            return Ok(None);
                        }
                    }
                }
                continue;
            }

            // Process input stream directly (zero-copy when contiguous).
            self.reset();
            if input.is_empty() {
                return Ok(None);
            }

            match inspect_frame(input) {
                FrameInspection::Invalid(_) => *input = Self::next_candidate(input),
                FrameInspection::Complete(frame) => {
                    let bytes = input
                        .get(..frame.len)
                        .ok_or(MspParserError::InvalidBufferState)?;
                    let remaining = input
                        .get(frame.len..)
                        .ok_or(MspParserError::InvalidBufferState)?;
                    let frame = frame
                        .into_frame(bytes)
                        .map_err(|_invalid_frame| MspParserError::InvalidBufferState)?;
                    *input = remaining;
                    return Ok(Some(frame));
                }
                FrameInspection::Incomplete(required) => {
                    if required > N {
                        // Candidate exceeds spill capacity; scan for the next preamble.
                        *input = Self::next_candidate(input);
                        continue;
                    }
                    // Spill partial candidate into internal buffer to await rest of frame.
                    self.pull_from_input(input, input.len())?;
                    return Ok(None);
                }
            }
        }
    }
}

impl<const N: usize> Default for MspParser<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[expect(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    reason = "tests use unwrap, expect, and direct slice indexing for conciseness"
)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    fn frame(payload: &[u8]) -> Vec<u8> {
        let mut bytes = std::vec![b'$', b'X', b'>', 0, 42, 0];
        bytes.extend_from_slice(&u16::try_from(payload.len()).unwrap().to_le_bytes());
        bytes.extend_from_slice(payload);
        bytes.push(crc8_dvbs2_slice(0, &bytes[3..]));
        bytes
    }

    fn v1_frame(payload: &[u8], encapsulated: bool, jumbo: bool) -> Vec<u8> {
        let outer_len = payload.len() + if encapsulated { 6 } else { 0 };
        let mut bytes = std::vec![
            b'$',
            b'M',
            b'>',
            if jumbo {
                255
            } else {
                u8::try_from(outer_len).unwrap()
            },
            if encapsulated { 255 } else { 42 }
        ];
        if encapsulated {
            bytes.extend_from_slice(&[0xa5, 0xef, 0xbe]);
            bytes.extend_from_slice(&u16::try_from(payload.len()).unwrap().to_le_bytes());
        }
        let crc = if encapsulated {
            crc8_dvbs2_slice(0, &bytes[5..10])
        } else {
            0
        };
        if jumbo {
            bytes.extend_from_slice(&u16::try_from(outer_len).unwrap().to_le_bytes());
        }
        bytes.extend_from_slice(payload);
        if encapsulated {
            bytes.push(crc8_dvbs2_slice(crc, payload));
        }
        bytes.push(bytes[3..].iter().fold(0, |sum, b| sum ^ b));
        bytes
    }

    #[test]
    fn fixed_v1_vectors() {
        for (wire, version, payload) in [
            (
                b"$M<\x00\x01\x01".as_slice(),
                MspVersion::V1,
                b"".as_slice(),
            ),
            (b"$M>\xff\x2a\x01\x00\x41\x95", MspVersion::V1, b"A"),
            (
                b"$M>\x06\xff\x00\x01\x00\x00\x00\x45\xbd",
                MspVersion::V2OverV1,
                b"",
            ),
            (
                b"$M>\xff\xff\x00\x01\x00\x00\x00\x06\x00\x45\x42",
                MspVersion::V2OverV1,
                b"",
            ),
        ] {
            let result = MspFrame::try_from(wire).unwrap();
            assert_eq!(result.version(), version);
            assert_eq!(result.payload(), payload);
            assert_eq!(result.checksum(), *wire.last().unwrap());
        }
    }

    #[test]
    fn v1_boundaries_and_borrowing() {
        for encapsulated in [false, true] {
            for len in [
                0,
                1,
                248,
                249,
                254,
                255,
                256,
                if encapsulated { 65529 } else { 65535 },
            ] {
                let payload = std::vec![0x73; len];
                let jumbo = len + if encapsulated { 6 } else { 0 } >= 255;
                let mut bytes = v1_frame(&payload, encapsulated, jumbo);
                for direction in *b"<>!" {
                    bytes[2] = direction;
                    let f = MspFrame::try_from(bytes.as_slice()).unwrap();
                    assert_eq!(
                        f.version(),
                        if encapsulated {
                            MspVersion::V2OverV1
                        } else {
                            MspVersion::V1
                        }
                    );
                    assert_eq!(f.command_id(), if encapsulated { 0xbeef } else { 42 });
                    assert_eq!(f.flags(), if encapsulated { 0xa5 } else { 0 });
                    assert_eq!(f.payload(), payload);
                    assert_eq!(f.as_bytes().as_ptr(), bytes.as_ptr());
                    let offset = if encapsulated { 10 } else { 5 } + if jumbo { 2 } else { 0 };
                    assert_eq!(f.payload().as_ptr(), bytes[offset..].as_ptr());
                }
                // Maximum frames borrow contiguous input even with a small spill buffer.
                assert_eq!(
                    MspParser::<9>::new()
                        .next(&mut bytes.as_slice())
                        .unwrap()
                        .unwrap()
                        .as_bytes(),
                    bytes
                );
                if bytes.len() > 300 {
                    continue;
                }
                for split in 0..bytes.len() {
                    let mut parser = MspParser::<300>::new();
                    assert_eq!(parser.next(&mut &bytes[..split]), Ok(None));
                    let f = parser.next(&mut &bytes[split..]).unwrap().unwrap();
                    assert_eq!(f.as_bytes(), bytes);
                    assert_eq!(f.payload(), payload);
                }
                let mut parser = MspParser::<300>::new();
                for (i, byte) in bytes.iter().enumerate() {
                    assert_eq!(
                        parser
                            .next(&mut core::slice::from_ref(byte))
                            .unwrap()
                            .is_some(),
                        i + 1 == bytes.len()
                    );
                }
                let mut trailing = bytes.clone();
                trailing.push(0);
                assert!(matches!(
                    MspFrame::try_from(trailing.as_slice()),
                    Err(MspFrameError::LengthMismatch { .. })
                ));
                for prefix in 0..bytes.len() {
                    assert!(matches!(
                        MspFrame::try_from(&bytes[..prefix]),
                        Err(MspFrameError::Incomplete { .. })
                    ));
                }
            }
        }
    }

    #[test]
    fn encapsulation_lengths_must_fit_outer_envelope() {
        for (header, expected, actual) in [
            (b"$M>\x00\xff\x00\x00\x00\x00\x00".as_slice(), 6, 0),
            (b"$M>\xff\xff\x00\x00\x00\xff\xff\xff\xff", 65541, 65535),
        ] {
            assert_eq!(
                MspFrame::try_from(header),
                Err(MspFrameError::InvalidEncapsulationLength { expected, actual })
            );
        }
    }

    #[test]
    fn undersized_encapsulation_rejects_at_outer_header() {
        let probe = v1_frame(&[], false, false);
        for size in 0..V2_INNER_OVERHEAD as u8 {
            let header = [b'$', b'M', b'>', size, V1_ENCAPSULATED_CMD];
            let expected = MspFrameError::InvalidEncapsulationLength {
                expected: V2_INNER_OVERHEAD,
                actual: usize::from(size),
            };
            assert_eq!(MspFrame::try_from(header.as_slice()), Err(expected));
            // Keep the parser below the inner-header threshold, then supply a
            // short v1 frame in isolation. No extra input should be necessary.
            for split in 0..=header.len() {
                let mut parser = MspParser::<32>::new();
                assert_eq!(parser.next(&mut &header[..split]), Ok(None));
                assert_eq!(parser.next(&mut &header[split..]), Ok(None));
                assert!(parser.buffered().unwrap().is_empty());
                assert_eq!(
                    parser
                        .next(&mut probe.as_slice())
                        .unwrap()
                        .unwrap()
                        .as_bytes(),
                    probe
                );
            }
        }
        assert_eq!(
            MspFrame::try_from(b"$M>\x00\xff\xff".as_slice()),
            Err(MspFrameError::InvalidEncapsulationLength {
                expected: V2_INNER_OVERHEAD,
                actual: 0
            })
        );
    }

    #[test]
    fn encapsulation_errors_and_mixed_recovery() {
        let mut stream = std::vec::Vec::new();
        let mut expected = std::vec::Vec::new();
        for encapsulated in [false, true] {
            for jumbo in [false, true] {
                let bytes = v1_frame(b"hello", encapsulated, jumbo);
                let mut corrupt = bytes.clone();
                *corrupt.last_mut().unwrap() ^= 1;
                assert!(matches!(
                    MspFrame::try_from(corrupt.as_slice()),
                    Err(MspFrameError::ChecksumMismatch { .. })
                ));
                stream.extend_from_slice(&corrupt);
                if encapsulated {
                    // Corrupt only the inner checksum, repairing outer XOR.
                    let n = corrupt.len();
                    corrupt[n - 2] ^= 1;
                    corrupt[n - 1] = corrupt[3..n - 1].iter().fold(0, |sum, b| sum ^ b);
                    assert!(matches!(
                        MspFrame::try_from(corrupt.as_slice()),
                        Err(MspFrameError::ChecksumMismatch { .. })
                    ));
                    stream.extend_from_slice(&corrupt);
                    corrupt[8] ^= 1;
                    assert!(matches!(
                        MspFrame::try_from(corrupt.as_slice()),
                        Err(MspFrameError::InvalidEncapsulationLength { .. })
                    ));
                    stream.extend_from_slice(&corrupt);
                }
                stream.extend_from_slice(&bytes);
                expected.push(bytes);
            }
        }
        let native = frame(b"native");
        stream.extend_from_slice(&native);
        expected.push(native);
        for chunk_size in [1, 7, stream.len()] {
            let mut parser = MspParser::<128>::new();
            let mut actual = std::vec::Vec::new();
            for chunk in stream.chunks(chunk_size) {
                let mut input = chunk;
                while let Some(f) = parser.next(&mut input).unwrap() {
                    actual.push(f.as_bytes().to_vec());
                }
            }
            assert_eq!(actual, expected);
        }
        for command in [0, 254] {
            let bytes = [b'$', b'M', b'!', 0, command, command];
            assert_eq!(
                MspFrame::try_from(bytes.as_slice()).unwrap().command_id(),
                u16::from(command)
            );
        }
        assert!(matches!(
            MspFrame::try_from(b"$M?".as_slice()),
            Err(MspFrameError::InvalidDirection)
        ));
        let bytes = v1_frame(b"hello", true, true);
        let mut parser = MspParser::<14>::new();
        assert_eq!(parser.next(&mut &bytes[..10]), Ok(None));
        let probe = v1_frame(&[], false, false);
        assert_eq!(
            parser
                .next(&mut probe.as_slice())
                .unwrap()
                .unwrap()
                .as_bytes(),
            probe
        );
        let mut parser = MspParser::<32>::new();
        assert_eq!(parser.next(&mut &bytes[..10]), Ok(None));
        parser.reset();
        assert_eq!(
            parser
                .next(&mut probe.as_slice())
                .unwrap()
                .unwrap()
                .as_bytes(),
            probe
        );
        let bytes = v1_frame(b"hi", true, true);
        let mut parser = MspParser::<16>::new();
        assert_eq!(parser.next(&mut &bytes[..10]), Ok(None));
        assert_eq!(
            parser.next(&mut &bytes[10..]).unwrap().unwrap().payload(),
            b"hi"
        );
    }

    #[test]
    fn checksum_known_vector() {
        assert_eq!(crc8_dvbs2_slice(0, b"123456789"), 0xbc);
        let partial = crc8_dvbs2_slice(0, b"1234");
        assert_eq!(crc8_dvbs2_slice(partial, b"56789"), 0xbc);
    }

    #[test]
    fn checksum_byte_matches_slice() {
        let data = b"123456789";
        let mut crc = 0;
        for &byte in data {
            crc = crc8_dvbs2_byte(crc, byte);
        }
        assert_eq!(crc, crc8_dvbs2_slice(0, data));
        assert_eq!(crc, 0xbc);
    }

    #[test]
    fn all_splits_and_borrow_sources() {
        let bytes = frame(b"hello");
        for split in 0..=bytes.len() {
            let mut parser = MspParser::<32>::new();
            let spill_ptr = parser.buffer.as_ptr();
            let mut first = &bytes[..split];
            if split == bytes.len() {
                let result = parser
                    .next(&mut first)
                    .expect("valid parser state")
                    .unwrap();
                assert_eq!(result.as_bytes().as_ptr(), bytes.as_ptr());
                assert_eq!(result.as_bytes(), bytes);
            } else {
                assert!(
                    parser
                        .next(&mut first)
                        .expect("valid parser state")
                        .is_none()
                );
                let mut second = &bytes[split..];
                let result = parser
                    .next(&mut second)
                    .expect("valid parser state")
                    .unwrap();
                assert_eq!(result.as_bytes(), bytes);
                assert_eq!(result.as_ref(), bytes);
                assert_eq!(
                    result.as_bytes().as_ptr(),
                    if split == 0 {
                        bytes.as_ptr()
                    } else {
                        spill_ptr
                    }
                );
                assert_eq!(second, []);
            }
            assert_eq!(first, []);
            assert!(
                parser
                    .next(&mut &[][..])
                    .expect("valid parser state")
                    .is_none()
            );
        }
    }

    #[test]
    fn byte_by_byte_at_exact_capacity() {
        let bytes = frame(b"hello");
        let mut parser = MspParser::<14>::new();
        for (index, byte) in bytes.iter().enumerate() {
            let mut input = core::slice::from_ref(byte);
            let result = parser.next(&mut input).expect("valid parser state");
            if index == bytes.len() - 1 {
                assert_eq!(result.unwrap().as_bytes(), bytes);
            } else {
                assert!(result.is_none());
            }
            assert_eq!(input, []);
        }
    }

    #[test]
    fn multiple_frames_and_trailing_partial() {
        let bytes = frame(b"hello");
        let mut stream = bytes.clone();
        stream.extend_from_slice(&bytes);
        stream.extend_from_slice(&bytes[..5]);
        let mut input = stream.as_slice();
        let mut parser = MspParser::<32>::default();
        assert_eq!(
            parser
                .next(&mut input)
                .expect("valid parser state")
                .unwrap()
                .as_bytes()
                .as_ptr(),
            stream.as_ptr()
        );
        assert_eq!(input.len(), bytes.len() + 5);
        assert_eq!(
            parser
                .next(&mut input)
                .expect("valid parser state")
                .unwrap()
                .as_bytes()
                .as_ptr(),
            stream[bytes.len()..].as_ptr()
        );
        assert!(
            parser
                .next(&mut input)
                .expect("valid parser state")
                .is_none()
        );
        assert_eq!(input, []);
        let mut tail = &bytes[5..];
        assert_eq!(
            parser
                .next(&mut tail)
                .expect("valid parser state")
                .unwrap()
                .as_bytes(),
            bytes
        );
        assert_eq!(tail, []);
    }

    #[test]
    fn spill_completion_leaves_next_frame_in_input() {
        let bytes = frame(b"hello");
        let mut parser = MspParser::<32>::new();
        assert!(
            parser
                .next(&mut &bytes[..4])
                .expect("valid parser state")
                .is_none()
        );
        let mut rest = bytes[4..].to_vec();
        rest.extend_from_slice(&bytes);
        let mut input = rest.as_slice();
        assert_eq!(
            parser
                .next(&mut input)
                .expect("valid parser state")
                .unwrap()
                .as_bytes(),
            bytes
        );
        assert_eq!(input, bytes);
        assert_eq!(
            parser
                .next(&mut input)
                .expect("valid parser state")
                .unwrap()
                .as_bytes()
                .as_ptr(),
            rest[bytes.len() - 4..].as_ptr()
        );
    }

    #[test]
    fn empty_payload_and_all_directions() {
        for direction in *b"<>!" {
            let mut bytes = frame(&[]);
            bytes[2] = direction;
            let mut parser = MspParser::<9>::new();
            assert_eq!(
                parser
                    .next(&mut bytes.as_slice())
                    .expect("valid parser state")
                    .unwrap()
                    .as_bytes(),
                bytes
            );
        }
    }

    #[test]
    fn noise_invalid_headers_and_checksums() {
        let bytes = frame(b"hello");
        let mut stream = b"noise$$X?$M>".to_vec();
        let mut corrupt = bytes.clone();
        *corrupt.last_mut().unwrap() ^= 1;
        stream.extend_from_slice(&corrupt);
        stream.extend_from_slice(&bytes);
        let mut parser = MspParser::<32>::new();
        let mut input = stream.as_slice();
        assert_eq!(
            parser
                .next(&mut input)
                .expect("valid parser state")
                .unwrap()
                .as_bytes(),
            bytes
        );
        assert_eq!(input, []);
    }

    #[test]
    fn recovery_preserves_partial_headers_after_noise() {
        let inner = frame(b"hello");
        for prefix_len in 0..HEADER_LEN {
            let mut payload = std::vec![b'n'; 64];
            payload.extend_from_slice(b"$$X?$Y>");
            payload.extend_from_slice(&inner[..prefix_len]);
            let mut outer = frame(&payload);
            // Replace the outer CRC with the next byte of the embedded frame.
            // Change a covered field if that byte happens to be the valid CRC.
            if *outer.last().unwrap() == inner[prefix_len] {
                outer[3] ^= 1;
            }
            *outer.last_mut().unwrap() = inner[prefix_len];
            assert!(MspFrame::from_slice(&outer).is_none());

            for split in [0, HEADER_LEN] {
                let mut parser = MspParser::<128>::new();
                assert_eq!(parser.next(&mut &outer[..split]), Ok(None));
                let mut input = &outer[split..];
                assert_eq!(parser.next(&mut input), Ok(None));
                assert!(input.is_empty());
                let mut tail = &inner[prefix_len + 1..];
                assert_eq!(parser.next(&mut tail).unwrap().unwrap().as_bytes(), inner);
                assert!(tail.is_empty());
            }
        }
    }

    #[test]
    fn buffered_recovery_drains_multiple_embedded_frames() {
        let inner = frame(&[]);
        let mut payload = inner.clone();
        payload.extend_from_slice(&inner);
        let mut outer = frame(&payload);
        // Invalid checksum with no accidental header in the trailing byte.
        let checksum = *outer.last().unwrap();
        *outer.last_mut().unwrap() = u8::from(checksum == 0);
        for split in 0..outer.len() {
            let mut parser = MspParser::<32>::new();
            assert!(
                parser
                    .next(&mut &outer[..split])
                    .expect("valid parser state")
                    .is_none()
            );
            let mut input = &outer[split..];
            assert_eq!(
                parser
                    .next(&mut input)
                    .expect("valid parser state")
                    .unwrap()
                    .as_bytes(),
                inner
            );
            assert_eq!(
                parser
                    .next(&mut input)
                    .expect("valid parser state")
                    .unwrap()
                    .as_bytes(),
                inner
            );
            assert!(
                parser
                    .next(&mut input)
                    .expect("valid parser state")
                    .is_none()
            );
            assert_eq!(input, []);
        }
    }

    #[test]
    fn recovery_compacts_a_partial_embedded_frame() {
        let inner = frame(b"hello");
        let mut outer = frame(&inner[..5]);
        // This byte is part of the recovered inner frame, not a valid outer CRC.
        assert_ne!(*outer.last().unwrap(), inner[5]);
        *outer.last_mut().unwrap() = inner[5];
        let mut parser = MspParser::<32>::new();
        assert!(
            parser
                .next(&mut &outer[..8])
                .expect("valid parser state")
                .is_none()
        );
        assert!(
            parser
                .next(&mut &outer[8..])
                .expect("valid parser state")
                .is_none()
        );
        assert!(
            parser
                .next(&mut &[][..])
                .expect("valid parser state")
                .is_none()
        );
        assert_eq!(
            parser
                .next(&mut &inner[6..])
                .expect("valid parser state")
                .unwrap()
                .as_bytes(),
            inner
        );
    }

    #[test]
    fn oversized_contiguous_frame_borrows_input() {
        let bytes = frame(&std::vec![0; usize::from(u16::MAX)]);
        let mut parser = MspParser::<9>::new();
        let mut input = bytes.as_slice();
        let result = parser
            .next(&mut input)
            .expect("valid parser state")
            .unwrap();
        assert_eq!(result.as_bytes(), bytes);
        assert_eq!(result.as_bytes().as_ptr(), bytes.as_ptr());
        assert_eq!(input, []);
    }

    #[test]
    fn oversized_split_candidates_are_skipped() {
        let oversized = frame(b"hello");
        let valid = frame(&[]);
        for split in 1..oversized.len() {
            let mut parser = MspParser::<9>::new();
            assert!(
                parser
                    .next(&mut &oversized[..split])
                    .expect("valid parser state")
                    .is_none()
            );
            let mut tail = oversized[split..].to_vec();
            tail.extend_from_slice(&valid);
            let mut input = tail.as_slice();
            assert_eq!(
                parser
                    .next(&mut input)
                    .expect("valid parser state")
                    .unwrap()
                    .as_bytes(),
                valid
            );
            assert_eq!(input, []);
        }
    }

    #[test]
    fn oversized_length_header_recovers() {
        let mut stream = b"$X>\0\0\0\xff\xff".to_vec();
        let valid = frame(&[]);
        stream.extend_from_slice(&valid);
        let mut parser = MspParser::<9>::new();
        let mut input = stream.as_slice();
        assert_eq!(
            parser
                .next(&mut input)
                .expect("valid parser state")
                .unwrap()
                .as_bytes(),
            valid
        );
        assert_eq!(input, []);
    }

    #[test]
    fn reset_discards_partial_frame() {
        let bytes = frame(b"hello");
        let mut parser = MspParser::<32>::new();
        assert!(
            parser
                .next(&mut &bytes[..8])
                .expect("valid parser state")
                .is_none()
        );
        parser.reset();
        assert!(
            parser
                .next(&mut &bytes[8..])
                .expect("valid parser state")
                .is_none()
        );
        assert_eq!(
            parser
                .next(&mut bytes.as_slice())
                .expect("valid parser state")
                .unwrap()
                .as_bytes(),
            bytes
        );
    }

    #[test]
    fn empty_input() {
        let mut parser = MspParser::<9>::new();
        assert!(
            parser
                .next(&mut &[][..])
                .expect("valid parser state")
                .is_none()
        );
        assert!(
            parser
                .next(&mut &[][..])
                .expect("valid parser state")
                .is_none()
        );
    }

    #[test]
    fn const_construction_at_minimum_capacity() {
        const PARSER: MspParser<9> = MspParser::new();
        let mut parser = PARSER;
        let bytes = frame(&[]);
        assert!(
            parser
                .next(&mut &bytes[..8])
                .expect("valid parser state")
                .is_none()
        );
        assert_eq!(
            parser
                .next(&mut &bytes[8..])
                .expect("valid parser state")
                .unwrap()
                .as_bytes(),
            bytes
        );
    }

    #[test]
    fn frame_field_accessors() {
        let payload = b"status_ok";
        let bytes = frame(payload);
        let frame = MspFrame::from_slice(&bytes).expect("valid frame");

        assert_eq!(frame.as_bytes(), bytes.as_slice());
        assert_eq!(&frame[..], bytes.as_slice()); // Deref test
        assert_eq!(frame.direction(), MspDirection::Response);
        assert_eq!(frame.flags(), 0);
        assert_eq!(frame.command_id(), 42);
        assert_eq!(frame.payload(), payload);
        assert_eq!(frame.checksum(), *bytes.last().unwrap());
    }

    #[test]
    fn decoded_fields_and_payload_borrows_match_across_all_splits() {
        for (direction_byte, direction) in [
            (b'<', MspDirection::Request),
            (b'>', MspDirection::Response),
            (b'!', MspDirection::Error),
        ] {
            for payload in [b"".as_slice(), b"hello$X!"] {
                let mut bytes = frame(payload);
                bytes[2] = direction_byte;
                bytes[3] = 0xa5;
                bytes[4..6].copy_from_slice(&0xbeef_u16.to_le_bytes());
                let checksum = crc8_dvbs2_slice(0, &bytes[PREAMBLE_LEN..bytes.len() - CRC_LEN]);
                *bytes.last_mut().unwrap() = checksum;

                let standalone = MspFrame::try_from(bytes.as_slice()).unwrap();
                assert_eq!(standalone.direction(), direction);
                assert_eq!(standalone.flags(), 0xa5);
                assert_eq!(standalone.command_id(), 0xbeef);
                assert_eq!(standalone.payload(), payload);
                assert_eq!(standalone.checksum(), checksum);
                assert_eq!(standalone.as_bytes().as_ptr(), bytes.as_ptr());
                assert_eq!(standalone.payload().as_ptr(), bytes[HEADER_LEN..].as_ptr());

                for split in 0..=bytes.len() {
                    let mut parser = MspParser::<32>::new();
                    let spill_payload_ptr = parser.buffer[HEADER_LEN..].as_ptr();
                    let mut first = &bytes[..split];
                    let parsed = if split == bytes.len() {
                        parser.next(&mut first).unwrap().unwrap()
                    } else {
                        assert_eq!(parser.next(&mut first), Ok(None));
                        let mut rest = &bytes[split..];
                        let parsed = parser.next(&mut rest).unwrap().unwrap();
                        assert_eq!(rest, []);
                        parsed
                    };
                    assert_eq!(parsed, standalone);
                    assert_eq!(parsed.as_ref(), bytes);
                    assert_eq!(&*parsed, bytes);
                    assert_eq!(
                        parsed.payload().as_ptr(),
                        if split == 0 || split == bytes.len() {
                            standalone.payload().as_ptr()
                        } else {
                            spill_payload_ptr
                        }
                    );
                    assert_eq!(first, []);
                }
            }
        }
    }

    #[test]
    fn frame_from_slice_invalid_inputs() {
        assert!(MspFrame::from_slice(b"").is_none());
        assert!(MspFrame::from_slice(b"$X").is_none());
        let mut corrupt = frame(b"test");
        *corrupt.last_mut().unwrap() ^= 1;
        assert!(MspFrame::from_slice(&corrupt).is_none());
    }

    #[test]
    fn standalone_validation_reports_header_errors() {
        for direction in *b"<>!" {
            let mut bytes = frame(&[]);
            bytes[2] = direction;
            for length in 0..HEADER_LEN {
                assert_eq!(
                    MspFrame::try_from(&bytes[..length]),
                    Err(MspFrameError::Incomplete {
                        required: if length < 2 { 2 } else { HEADER_LEN },
                        actual: length,
                    })
                );
            }
        }
        for bytes in [b"n".as_slice(), b"noise", b"$Y", b"$Y>"] {
            assert_eq!(
                MspFrame::try_from(bytes),
                Err(MspFrameError::InvalidPreamble)
            );
        }
        assert_eq!(
            MspFrame::try_from(b"$X?".as_slice()),
            Err(MspFrameError::InvalidDirection)
        );
    }

    #[test]
    fn standalone_validation_reports_payload_and_checksum_errors() {
        let bytes = frame(b"hello");
        for length in HEADER_LEN..bytes.len() {
            assert_eq!(
                MspFrame::try_from(&bytes[..length]),
                Err(MspFrameError::Incomplete {
                    required: bytes.len(),
                    actual: length,
                })
            );
        }

        let expected = *bytes.last().unwrap();
        let mut corrupt = bytes.clone();
        *corrupt.last_mut().unwrap() ^= 1;
        assert_eq!(
            MspFrame::try_from(corrupt.as_slice()),
            Err(MspFrameError::ChecksumMismatch {
                expected,
                actual: expected ^ 1,
            })
        );
        assert!(MspFrame::from_slice(&corrupt).is_none());

        let mut trailing = bytes.clone();
        trailing.push(0);
        assert_eq!(
            MspFrame::try_from(trailing.as_slice()),
            Err(MspFrameError::LengthMismatch {
                expected: bytes.len(),
                actual: trailing.len(),
            })
        );
        let validated = MspFrame::try_from(bytes.as_slice()).unwrap();
        assert_eq!(validated.as_bytes().as_ptr(), bytes.as_ptr());
        assert_eq!(MspFrame::from_slice(&bytes), Some(validated));
    }

    #[test]
    fn invalid_buffer_positions_are_reported_and_reset_recovers() {
        let bytes = frame(&[]);
        for (start, end) in [(1, 0), (0, 10), (10, 10), (usize::MAX, usize::MAX)] {
            let mut parser = MspParser::<9>::new();
            parser.start = start;
            parser.end = end;
            let mut input = bytes.as_slice();
            assert_eq!(
                parser.next(&mut input),
                Err(MspParserError::InvalidBufferState)
            );
            assert_eq!(input, bytes);
            parser.reset();
            assert_eq!(parser.next(&mut input).unwrap().unwrap().as_bytes(), bytes);
            assert_eq!(parser.next(&mut input), Ok(None));
        }
    }

    #[test]
    fn failed_buffer_append_does_not_consume_input_or_change_state() {
        let mut parser = MspParser::<9>::new();
        parser.end = 8;
        let before = parser.buffer;
        let mut input = b"ab".as_slice();
        assert_eq!(
            parser.pull_from_input(&mut input, 2),
            Err(MspParserError::BufferCapacityExceeded)
        );
        assert_eq!(input, b"ab");
        assert_eq!((parser.start, parser.end), (0, 8));
        assert_eq!(parser.buffer, before);
        let inspection = inspect_frame(&frame(&[]));
        assert!(matches!(inspection, FrameInspection::Complete(_)));
        if let FrameInspection::Complete(frame) = inspection {
            assert_eq!(
                parser.consume_buffered_frame(frame),
                Err(MspParserError::InvalidBufferState)
            );
        }
        assert_eq!((parser.start, parser.end), (0, 8));
    }

    #[test]
    fn public_errors_implement_error_and_display() {
        fn error_message(error: &dyn core::error::Error) -> std::string::String {
            std::format!("{error}")
        }
        assert_eq!(
            error_message(&MspFrameError::Incomplete {
                required: 9,
                actual: 8
            }),
            "incomplete MSP frame: need 9 bytes, got 8"
        );
        assert_eq!(
            error_message(&MspParserError::InvalidBufferState),
            "invalid MSP parser buffer state"
        );
    }
}
