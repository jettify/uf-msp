#![no_std]
#![doc = include_str!("../README.md")]

mod constants;
mod parser;

pub use constants::CRC_LEN;
pub use constants::HEADER_LEN;
pub use constants::MIN_FRAME_LEN;
pub use constants::PREAMBLE_LEN;
pub use parser::MspDirection;
pub use parser::MspFrame;
pub use parser::MspFrameError;
pub use parser::MspParser;
pub use parser::MspParserError;
pub use parser::crc8_dvbs2_slice;

pub use constants::V1_HEADER_LEN;
pub use constants::V1_JUMBO_HEADER_LEN;
pub use constants::V1_MIN_FRAME_LEN;
pub use parser::MspVersion;
