#![no_std]
#![doc = include_str!("../README.md")]

mod parser;

pub use parser::{
    CRC_LEN, HEADER_LEN, MIN_FRAME_LEN, MspDirection, MspFrame, MspFrameError, MspParser,
    MspParserError, PREAMBLE_LEN, crc8_dvbs2_slice,
};
