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

/// DVB-S2 CRC-8 polynomial: `x^8 + x^7 + x^6 + x^4 + x^2 + 1`.
const CRC8_DVBS2_POLY: u8 = 0xD5;

/// MSP v2 header length: `$` (1), `X` (1), direction (1), flag (1), command ID (2), payload length (2).
pub const HEADER_LEN: usize = 8;

/// Length of the CRC checksum field at the end of an MSP v2 frame.
pub const CRC_LEN: usize = 1;

/// Length of the 3-byte preamble: `$` (1), `X` (1), direction (1).
pub const PREAMBLE_LEN: usize = 3;

/// Minimum valid MSP v2 frame size (empty payload: 8-byte header + 1-byte CRC).
pub const MIN_FRAME_LEN: usize = HEADER_LEN + CRC_LEN;

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

/// Why a standalone byte slice could not be validated as an MSP v2 frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum MspFrameError {
    /// More bytes are needed; `required` is the next validation threshold.
    Incomplete { required: usize, actual: usize },
    /// The frame does not begin with `$X`.
    InvalidPreamble,
    /// The direction byte is neither `<`, `>`, nor `!`.
    InvalidDirection,
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
            Self::InvalidPreamble => f.write_str("invalid MSP v2 preamble"),
            Self::InvalidDirection => f.write_str("invalid MSP direction"),
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

/// A complete native MSP v2 frame with a valid header and checksum.
///
/// The borrowed bytes include the header, payload, and checksum. Construction is
/// private so this type always represents a validated frame. Header fields are
/// decoded during validation; payload and wire bytes remain borrowed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspFrame<'a> {
    bytes: &'a [u8],
    direction: MspDirection,
    flags: u8,
    command_id: u16,
    payload: &'a [u8],
    checksum: u8,
}

impl<'a> MspFrame<'a> {
    /// Validates a standalone byte slice as a complete MSP v2 frame.
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

    /// Returns the direction of this frame (request, response, or error).
    #[must_use]
    pub fn direction(&self) -> MspDirection {
        self.direction
    }

    /// Returns the 8-bit flag field.
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

    /// Returns the CRC checksum byte at the end of the frame.
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
/// Only `inspect_frame` constructs this after checking the complete frame and CRC.
struct ValidatedFrameMetadata {
    len: usize,
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
        let payload = bytes
            .get(HEADER_LEN..)
            .and_then(|body| body.split_last())
            .map(|(_, payload)| payload)
            .ok_or(MspFrameError::Incomplete {
                required: MIN_FRAME_LEN,
                actual: bytes.len(),
            })?;
        Ok(MspFrame {
            bytes,
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
    match bytes {
        // Preamble partial prefixes: valid start, but need full header to inspect frame length.
        []
        | [b'$']
        | [b'$', b'X']
        | [b'$', b'X', b'<' | b'>' | b'!']
        | [b'$', b'X', b'<' | b'>' | b'!', _]
        | [b'$', b'X', b'<' | b'>' | b'!', _, _]
        | [b'$', b'X', b'<' | b'>' | b'!', _, _, _]
        | [b'$', b'X', b'<' | b'>' | b'!', _, _, _, _] => FrameInspection::Incomplete(HEADER_LEN),
        // Valid header: inspect payload length and checksum.
        [b'$', b'X', b'<' | b'>' | b'!', _, _, _, _, _, ..] => {
            let Some(header) = bytes
                .get(..HEADER_LEN)
                .and_then(|header| <&[u8; HEADER_LEN]>::try_from(header).ok())
            else {
                return FrameInspection::Incomplete(HEADER_LEN);
            };
            let &[
                _,
                _,
                direction,
                flags,
                command_id_lo,
                command_id_hi,
                len_lo,
                len_hi,
            ] = header;
            let Some(direction) = MspDirection::from_byte(direction) else {
                return FrameInspection::Invalid(MspFrameError::InvalidDirection);
            };
            let command_id = u16::from_le_bytes([command_id_lo, command_id_hi]);
            // Add MIN_FRAME_LEN as u32 so even a 16-bit target cannot overflow before checking usize.
            let payload_len = u32::from(u16::from_le_bytes([len_lo, len_hi]));
            let min_frame_len = u32::try_from(MIN_FRAME_LEN).unwrap_or(9);
            let Ok(frame_len) = usize::try_from(payload_len + min_frame_len) else {
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
                direction,
                flags,
                command_id,
                checksum: received_crc,
            })
        }
        [b'$', b'X', _, ..] => FrameInspection::Invalid(MspFrameError::InvalidDirection),
        _ => FrameInspection::Invalid(MspFrameError::InvalidPreamble),
    }
}

/// Splits a byte stream into validated native MSP v2 frames without allocation.
///
/// Complete frames borrow the input directly. Frames spanning chunks use the
/// internal `N`-byte spill buffer. `N` limits only split frames: larger contiguous
/// frames are accepted, but larger incomplete candidates are skipped. Malformed
/// candidates are skipped one byte at a time to search for the next valid frame.
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

    /// Discards one byte from the spill buffer to scan for the next candidate frame.
    #[inline]
    fn skip_buffered_byte(&mut self) {
        self.start += 1;
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
                    FrameInspection::Invalid(_) => self.skip_buffered_byte(),
                    FrameInspection::Complete(frame) => {
                        return self.consume_buffered_frame(frame).map(Some);
                    }
                    FrameInspection::Incomplete(required) => {
                        if required > N {
                            // Candidate frame exceeds spill capacity; discard byte to resync.
                            self.skip_buffered_byte();
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
                FrameInspection::Invalid(_) => *input = input.get(1..).unwrap_or(&[]),
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
                        // Candidate frame exceeds spill capacity; discard byte to resync.
                        *input = input.get(1..).unwrap_or(&[]);
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
        for bytes in [b"".as_slice(), b"$", b"$X", b"$X>", b"$X>\0\0\0\0"] {
            assert_eq!(
                MspFrame::try_from(bytes),
                Err(MspFrameError::Incomplete {
                    required: HEADER_LEN,
                    actual: bytes.len(),
                })
            );
        }
        for bytes in [b"noise".as_slice(), b"$M>"] {
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
