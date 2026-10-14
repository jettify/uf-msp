#![no_std]
#![allow(clippy::needless_doctest_main)]
#![doc = include_str!("../README.md")]

mod parser;

pub use parser::{
    CMD_OFFSET, CRC_LEN, CRC8_DVBS2_POLY, DIRECTION_OFFSET, FLAGS_OFFSET, HEADER_LEN,
    MIN_FRAME_LEN, MspDirection, MspFrame, MspPacket, MspParser, MspVersion, PAYLOAD_LEN_OFFSET,
    PREAMBLE_LEN, crc8_dvbs2_byte, crc8_dvbs2_slice,
};
