#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MspVersion {
    V1,
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
    pub fn from_byte(b: u8) -> Option<Self> {
        match b {
            b'<' => Some(Self::Request),
            b'>' => Some(Self::Response),
            b'!' => Some(Self::Error),
            _ => None,
        }
    }
}

/// Zero-copy packet directly borrowing the payload slice from input memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspPacket<'a> {
    pub version: MspVersion,
    pub direction: MspDirection,
    pub cmd: u16,
    pub payload: &'a [u8],
}

/// DVB-S2 CRC-8 polynomial: `x^8 + x^7 + x^6 + x^4 + x^2 + 1`.
pub const CRC8_DVBS2_POLY: u8 = 0xD5;

/// MSP v2 header length: `$` (1), `X` (1), direction (1), flag (1), cmd (2), payload length (2).
pub const HEADER_LEN: usize = 8;

/// Length of the CRC checksum field at the end of an MSP v2 frame.
pub const CRC_LEN: usize = 1;

/// Length of the 3-byte preamble: `$` (1), `X` (1), direction (1).
pub const PREAMBLE_LEN: usize = 3;

/// Byte offset of the direction indicator (`<`, `>`, `!`).
pub const DIRECTION_OFFSET: usize = 2;

/// Byte offset of the 8-bit flags field.
pub const FLAGS_OFFSET: usize = 3;

/// Byte offset of the 16-bit little-endian command ID.
pub const CMD_OFFSET: usize = 4;

/// Byte offset in the header where the 16-bit little-endian payload length is located.
pub const PAYLOAD_LEN_OFFSET: usize = 6;

/// Minimum valid MSP v2 frame size (empty payload: 8-byte header + 1-byte CRC).
pub const MIN_FRAME_LEN: usize = HEADER_LEN + CRC_LEN;

/// Updates a running CRC-8 (DVB-S2) with a single byte.
#[inline]
#[must_use]
pub fn crc8_dvbs2_byte(mut crc: u8, byte: u8) -> u8 {
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

/// A complete native MSP v2 frame with a valid header and checksum.
///
/// The borrowed bytes include the header, payload, and checksum. Construction is
/// private so this type always represents a validated frame; fields are not decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspFrame<'a> {
    bytes: &'a [u8],
}

impl<'a> MspFrame<'a> {
    /// Validates a standalone byte slice as a complete MSP v2 frame.
    #[must_use]
    pub fn from_slice(bytes: &'a [u8]) -> Option<Self> {
        match inspect(bytes) {
            Candidate::Complete(len) if len == bytes.len() => Some(Self { bytes }),
            _ => None,
        }
    }

    /// Returns the complete wire frame, including its checksum.
    #[must_use]
    pub fn as_bytes(&self) -> &'a [u8] {
        self.bytes
    }

    /// Returns the direction of this frame (request, response, or error).
    #[must_use]
    pub fn direction(&self) -> MspDirection {
        match self.bytes[DIRECTION_OFFSET] {
            b'<' => MspDirection::Request,
            b'>' => MspDirection::Response,
            b'!' => MspDirection::Error,
            _ => unreachable!("validated frame has a valid direction byte"),
        }
    }

    /// Returns the 8-bit flag field.
    #[must_use]
    pub fn flags(&self) -> u8 {
        self.bytes[FLAGS_OFFSET]
    }

    /// Returns the 16-bit command ID.
    #[must_use]
    pub fn cmd(&self) -> u16 {
        u16::from_le_bytes([self.bytes[CMD_OFFSET], self.bytes[CMD_OFFSET + 1]])
    }

    /// Returns the payload slice borrowing directly from the frame.
    #[must_use]
    pub fn payload(&self) -> &'a [u8] {
        &self.bytes[HEADER_LEN..self.bytes.len() - CRC_LEN]
    }

    /// Returns the CRC checksum byte at the end of the frame.
    #[must_use]
    pub fn checksum(&self) -> u8 {
        self.bytes[self.bytes.len() - CRC_LEN]
    }

    /// Decodes the frame into a zero-copy [`MspPacket`].
    #[must_use]
    pub fn packet(&self) -> MspPacket<'a> {
        MspPacket {
            version: MspVersion::V2,
            direction: self.direction(),
            cmd: self.cmd(),
            payload: self.payload(),
        }
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

impl<'a> From<MspFrame<'a>> for MspPacket<'a> {
    fn from(frame: MspFrame<'a>) -> Self {
        frame.packet()
    }
}

impl<'a> TryFrom<&'a [u8]> for MspFrame<'a> {
    type Error = ();

    fn try_from(bytes: &'a [u8]) -> Result<Self, Self::Error> {
        Self::from_slice(bytes).ok_or(())
    }
}

enum Candidate {
    Invalid,
    Incomplete(usize),
    Complete(usize),
}

fn inspect(bytes: &[u8]) -> Candidate {
    match bytes {
        // Preamble partial prefixes: valid start, but need full header to inspect frame length.
        [] | [b'$'] | [b'$', b'X'] => Candidate::Incomplete(HEADER_LEN),
        // Valid preamble: inspect header, payload length, and checksum.
        [b'$', b'X', b'<' | b'>' | b'!', ..] => {
            if bytes.len() < HEADER_LEN {
                return Candidate::Incomplete(HEADER_LEN);
            }
            // Add MIN_FRAME_LEN as u32 so even a 16-bit target cannot overflow before checking usize.
            let payload_len = u32::from(u16::from_le_bytes([
                bytes[PAYLOAD_LEN_OFFSET],
                bytes[PAYLOAD_LEN_OFFSET + 1],
            ]));
            let Ok(frame_len) = usize::try_from(payload_len + MIN_FRAME_LEN as u32) else {
                return Candidate::Invalid;
            };
            if bytes.len() < frame_len {
                return Candidate::Incomplete(frame_len);
            }
            let checksum_idx = frame_len - CRC_LEN;
            if crc8_dvbs2_slice(0, &bytes[PREAMBLE_LEN..checksum_idx]) != bytes[checksum_idx] {
                return Candidate::Invalid;
            }
            Candidate::Complete(frame_len)
        }
        _ => Candidate::Invalid,
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
    fn buffered(&self) -> &[u8] {
        &self.buffer[self.start..self.end]
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

    /// Consumes `length` bytes from the spill buffer as an [`MspFrame`].
    #[inline]
    fn consume_buffered_frame<'a>(&'a mut self, length: usize) -> MspFrame<'a> {
        let begin = self.start;
        self.start += length;
        MspFrame {
            bytes: &self.buffer[begin..self.start],
        }
    }

    /// Copies up to `count` bytes from `input` into the spill buffer, advancing `input`.
    #[inline]
    fn pull_from_input(&mut self, input: &mut &[u8], count: usize) {
        let take = count.min(input.len());
        self.buffer[self.end..self.end + take].copy_from_slice(&input[..take]);
        self.end += take;
        *input = &input[take..];
    }

    /// Returns the next valid frame and advances `input` past consumed bytes.
    ///
    /// Call repeatedly until `None`, which consumes all input and retains any
    /// incomplete candidate that fits the spill buffer. Empty input can drain
    /// frames retained during recovery from a corrupt candidate.
    ///
    /// The result borrows either the input or the spill buffer. Finish using it
    /// before calling the parser again.
    pub fn next<'a, 'input: 'a>(&'a mut self, input: &mut &'input [u8]) -> Option<MspFrame<'a>> {
        loop {
            // Process any data currently in the spill buffer.
            if self.has_buffered() {
                match inspect(self.buffered()) {
                    Candidate::Invalid => self.skip_buffered_byte(),
                    Candidate::Complete(length) => {
                        return Some(self.consume_buffered_frame(length));
                    }
                    Candidate::Incomplete(required) => {
                        if required > N {
                            // Candidate exceeds spill capacity; discard byte to resync.
                            self.skip_buffered_byte();
                            continue;
                        }
                        self.compact();
                        self.pull_from_input(input, required - self.end);
                        if self.end < required {
                            return None;
                        }
                    }
                }
                continue;
            }

            //Process input stream directly (zero-copy when contiguous).
            self.reset();
            if input.is_empty() {
                return None;
            }

            match inspect(input) {
                Candidate::Invalid => *input = &input[1..],
                Candidate::Complete(length) => {
                    let (bytes, remaining) = input.split_at(length);
                    *input = remaining;
                    return Some(MspFrame { bytes });
                }
                Candidate::Incomplete(required) => {
                    if required > N {
                        // Candidate exceeds spill capacity; discard byte to resync.
                        *input = &input[1..];
                        continue;
                    }
                    // Spill partial candidate into internal buffer to await rest of frame.
                    self.pull_from_input(input, input.len());
                    return None;
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
                let result = parser.next(&mut first).unwrap();
                assert_eq!(result.as_bytes().as_ptr(), bytes.as_ptr());
                assert_eq!(result.as_bytes(), bytes);
            } else {
                assert!(parser.next(&mut first).is_none());
                let mut second = &bytes[split..];
                let result = parser.next(&mut second).unwrap();
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
            assert!(parser.next(&mut &[][..]).is_none());
        }
    }

    #[test]
    fn byte_by_byte_at_exact_capacity() {
        let bytes = frame(b"hello");
        let mut parser = MspParser::<14>::new();
        for (index, byte) in bytes.iter().enumerate() {
            let mut input = core::slice::from_ref(byte);
            let result = parser.next(&mut input);
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
            parser.next(&mut input).unwrap().as_bytes().as_ptr(),
            stream.as_ptr()
        );
        assert_eq!(input.len(), bytes.len() + 5);
        assert_eq!(
            parser.next(&mut input).unwrap().as_bytes().as_ptr(),
            stream[bytes.len()..].as_ptr()
        );
        assert!(parser.next(&mut input).is_none());
        assert_eq!(input, []);
        let mut tail = &bytes[5..];
        assert_eq!(parser.next(&mut tail).unwrap().as_bytes(), bytes);
        assert_eq!(tail, []);
    }

    #[test]
    fn spill_completion_leaves_next_frame_in_input() {
        let bytes = frame(b"hello");
        let mut parser = MspParser::<32>::new();
        assert!(parser.next(&mut &bytes[..4]).is_none());
        let mut rest = bytes[4..].to_vec();
        rest.extend_from_slice(&bytes);
        let mut input = rest.as_slice();
        assert_eq!(parser.next(&mut input).unwrap().as_bytes(), bytes);
        assert_eq!(input, bytes);
        assert_eq!(
            parser.next(&mut input).unwrap().as_bytes().as_ptr(),
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
                parser.next(&mut bytes.as_slice()).unwrap().as_bytes(),
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
        assert_eq!(parser.next(&mut input).unwrap().as_bytes(), bytes);
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
            assert!(parser.next(&mut &outer[..split]).is_none());
            let mut input = &outer[split..];
            assert_eq!(parser.next(&mut input).unwrap().as_bytes(), inner);
            assert_eq!(parser.next(&mut input).unwrap().as_bytes(), inner);
            assert!(parser.next(&mut input).is_none());
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
        assert!(parser.next(&mut &outer[..8]).is_none());
        assert!(parser.next(&mut &outer[8..]).is_none());
        assert!(parser.next(&mut &[][..]).is_none());
        assert_eq!(parser.next(&mut &inner[6..]).unwrap().as_bytes(), inner);
    }

    #[test]
    fn oversized_contiguous_frame_borrows_input() {
        let bytes = frame(&std::vec![0; usize::from(u16::MAX)]);
        let mut parser = MspParser::<9>::new();
        let mut input = bytes.as_slice();
        let result = parser.next(&mut input).unwrap();
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
            assert!(parser.next(&mut &oversized[..split]).is_none());
            let mut tail = oversized[split..].to_vec();
            tail.extend_from_slice(&valid);
            let mut input = tail.as_slice();
            assert_eq!(parser.next(&mut input).unwrap().as_bytes(), valid);
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
        assert_eq!(parser.next(&mut input).unwrap().as_bytes(), valid);
        assert_eq!(input, []);
    }

    #[test]
    fn reset_discards_partial_frame() {
        let bytes = frame(b"hello");
        let mut parser = MspParser::<32>::new();
        assert!(parser.next(&mut &bytes[..8]).is_none());
        parser.reset();
        assert!(parser.next(&mut &bytes[8..]).is_none());
        assert_eq!(
            parser.next(&mut bytes.as_slice()).unwrap().as_bytes(),
            bytes
        );
    }

    #[test]
    fn empty_input() {
        let mut parser = MspParser::<9>::new();
        assert!(parser.next(&mut &[][..]).is_none());
        assert!(parser.next(&mut &[][..]).is_none());
    }

    #[test]
    fn const_construction_at_minimum_capacity() {
        const PARSER: MspParser<9> = MspParser::new();
        let mut parser = PARSER;
        let bytes = frame(&[]);
        assert!(parser.next(&mut &bytes[..8]).is_none());
        assert_eq!(parser.next(&mut &bytes[8..]).unwrap().as_bytes(), bytes);
    }

    #[test]
    fn frame_field_accessors_and_packet_conversion() {
        let payload = b"status_ok";
        let bytes = frame(payload);
        let frame = MspFrame::from_slice(&bytes).expect("valid frame");

        assert_eq!(frame.as_bytes(), bytes.as_slice());
        assert_eq!(&frame[..], bytes.as_slice()); // Deref test
        assert_eq!(frame.direction(), MspDirection::Response);
        assert_eq!(frame.flags(), 0);
        assert_eq!(frame.cmd(), 42);
        assert_eq!(frame.payload(), payload);
        assert_eq!(frame.checksum(), *bytes.last().unwrap());

        let packet = frame.packet();
        assert_eq!(packet.version, MspVersion::V2);
        assert_eq!(packet.direction, MspDirection::Response);
        assert_eq!(packet.cmd, 42);
        assert_eq!(packet.payload, payload);

        let packet_from: MspPacket = frame.into();
        assert_eq!(packet_from, packet);
    }

    #[test]
    fn frame_from_slice_invalid_inputs() {
        assert!(MspFrame::from_slice(b"").is_none());
        assert!(MspFrame::from_slice(b"$X").is_none());
        let mut corrupt = frame(b"test");
        *corrupt.last_mut().unwrap() ^= 1;
        assert!(MspFrame::from_slice(&corrupt).is_none());
    }
}
