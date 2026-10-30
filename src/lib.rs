#![no_std]
#![doc = include_str!("../README.md")]

mod parser;

pub use parser::{
    COMMAND_ID_OFFSET, CRC_LEN, CRC8_DVBS2_POLY, DIRECTION_OFFSET, FLAGS_OFFSET, HEADER_LEN,
    MIN_FRAME_LEN, MspDirection, MspFrame, MspFrameError, MspParser, MspParserError,
    PAYLOAD_LEN_OFFSET, PREAMBLE_LEN, crc8_dvbs2_byte, crc8_dvbs2_slice,
};
