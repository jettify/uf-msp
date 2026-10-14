#![no_std]
#![allow(clippy::needless_doctest_main)]
#![doc = include_str!("../README.md")]

mod parser;

pub use parser::{
    CRC_LEN, CRC8_DVBS2_POLY, HEADER_LEN, MIN_FRAME_LEN, MspDirection, MspFrame, MspPacket,
    MspParser, MspVersion, crc8_dvbs2_byte, crc8_dvbs2_slice,
};
