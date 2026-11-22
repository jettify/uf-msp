// Shared framing constants.

/// Length of the 3-byte preamble: `$` (1), version `M`/`X` (1), direction (1).
pub(crate) const PREAMBLE_LEN: usize = 3;

/// DVB-S2 CRC-8 polynomial: `x^8 + x^7 + x^6 + x^4 + x^2 + 1`.
pub(crate) const CRC8_DVBS2_POLY: u8 = 0xD5;

/// Length of the CRC-8 checksum field at the end of an MSP v2 payload.
pub(crate) const CRC_LEN: usize = 1;

// MSP v1 framing constants.

/// Ordinary MSP v1 header size, including preamble: `$` (1), `M` (1), direction (1), size (1), command (1).
pub(crate) const V1_HEADER_LEN: usize = 5;

/// Jumbo MSP v1 header size, including preamble and 2-byte extended length.
pub(crate) const V1_JUMBO_HEADER_LEN: usize = 7;

/// Length of the outer XOR checksum field at the end of an MSP v1 frame.
pub(crate) const V1_CHECKSUM_LEN: usize = 1;

/// Minimum MSP v1 wire frame size (5-byte header + 1-byte XOR checksum).
pub(crate) const V1_MIN_FRAME_LEN: usize = V1_HEADER_LEN + V1_CHECKSUM_LEN;
pub(crate) const V1_MIN_FRAME_LEN_U32: u32 = V1_MIN_FRAME_LEN as u32;

/// Size of the extended length field in a jumbo MSP v1 frame.
pub(crate) const V1_JUMBO_LENGTH_LEN: usize = V1_JUMBO_HEADER_LEN - V1_HEADER_LEN;
pub(crate) const V1_JUMBO_LENGTH_LEN_U32: u32 = V1_JUMBO_LENGTH_LEN as u32;

/// MSP v1 `size` field value indicating a 2-byte jumbo length follows.
pub(crate) const V1_JUMBO_SIZE: u8 = 255;

/// MSP v1 `command` field value (`MSP_V2_FRAME_ID`) indicating an encapsulated MSP v2 frame.
pub(crate) const V1_ENCAPSULATED_CMD: u8 = 255;

// MSP v2 framing constants (native and v2-over-v1).

/// MSP v2 header length: `$` (1), `X` (1), direction (1), flags (1), command ID (2), payload length (2).
/// See [`V1_HEADER_LEN`] and [`V1_JUMBO_HEADER_LEN`] for v1 headers.
pub(crate) const V2_HEADER_LEN: usize = 8;

/// Minimum valid MSP v2 frame size (empty payload: 8-byte header + 1-byte CRC).
/// See [`V1_MIN_FRAME_LEN`] for the minimum v1 frame size.
pub(crate) const V2_MIN_FRAME_LEN: usize = V2_HEADER_LEN + CRC_LEN;
pub(crate) const V2_MIN_FRAME_LEN_U32: u32 = V2_MIN_FRAME_LEN as u32;

/// Inner MSP v2 header size inside a v1 envelope: flags (1), command ID (2), payload length (2).
pub(crate) const V2_INNER_HEADER_LEN: usize = V2_HEADER_LEN - PREAMBLE_LEN;

/// Additional bytes inside the outer v1 `size` for an encapsulated v2 frame (5-byte inner header + 1-byte inner CRC).
pub(crate) const V2_INNER_OVERHEAD: usize = V2_INNER_HEADER_LEN + CRC_LEN;
pub(crate) const V2_INNER_OVERHEAD_U32: u32 = V2_INNER_OVERHEAD as u32;

/// Offset where the inner MSP v2 header ends inside a v1 frame (`5 + 5 = 10`).
pub(crate) const V2_OVER_V1_HEADER_LEN: usize = V1_HEADER_LEN + V2_INNER_HEADER_LEN;
