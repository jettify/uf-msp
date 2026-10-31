/// Ordinary MSP v1 header size, including preamble.
pub const V1_HEADER_LEN: usize = 5;
/// Jumbo MSP v1 header size, including preamble and extended length.
pub const V1_JUMBO_HEADER_LEN: usize = 7;
/// Minimum MSP v1 wire frame size.
pub const V1_MIN_FRAME_LEN: usize = V1_HEADER_LEN + V1_CHECKSUM_LEN;

/// DVB-S2 CRC-8 polynomial: `x^8 + x^7 + x^6 + x^4 + x^2 + 1`.
pub(crate) const CRC8_DVBS2_POLY: u8 = 0xD5;

/// MSP v2 header length: `$` (1), `X` (1), direction (1), flag (1), command ID (2), payload length (2).
/// See [`V1_HEADER_LEN`] and [`V1_JUMBO_HEADER_LEN`] for v1 headers.
pub const HEADER_LEN: usize = 8;

/// Length of the CRC checksum field at the end of an MSP v2 frame.
pub const CRC_LEN: usize = 1;

/// Length of the 3-byte preamble: `$` (1), `X` (1), direction (1).
pub const PREAMBLE_LEN: usize = 3;

/// Minimum valid MSP v2 frame size (empty payload: 8-byte header + 1-byte CRC).
/// See [`V1_MIN_FRAME_LEN`] for the minimum v1 frame size.
pub const MIN_FRAME_LEN: usize = HEADER_LEN + CRC_LEN;

pub(crate) const V1_ENCAPSULATED_CMD: u8 = 255;
pub(crate) const V1_JUMBO_SIZE: u8 = 255;
pub(crate) const V1_CHECKSUM_LEN: usize = 1;
pub(crate) const V1_JUMBO_LENGTH_LEN: usize = V1_JUMBO_HEADER_LEN - V1_HEADER_LEN;
pub(crate) const V2_INNER_HEADER_LEN: usize = HEADER_LEN - PREAMBLE_LEN;
pub(crate) const V2_INNER_OVERHEAD: usize = V2_INNER_HEADER_LEN + CRC_LEN;
pub(crate) const V2_OVER_V1_HEADER_LEN: usize = V1_HEADER_LEN + V2_INNER_HEADER_LEN;

pub(crate) const MIN_FRAME_LEN_U32: u32 = MIN_FRAME_LEN as u32;
