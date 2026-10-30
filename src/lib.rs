#![no_std]
#![doc = include_str!("../README.md")]

mod parser;

pub use parser::CRC_LEN;
pub use parser::HEADER_LEN;
pub use parser::MIN_FRAME_LEN;
pub use parser::MspDirection;
pub use parser::MspFrame;
pub use parser::MspFrameError;
pub use parser::MspParser;
pub use parser::MspParserError;
pub use parser::PREAMBLE_LEN;
pub use parser::crc8_dvbs2_slice;
