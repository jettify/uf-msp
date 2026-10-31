#![no_std]
#![doc = include_str!("../README.md")]

mod constants;
mod parser;

pub use parser::MspDirection;
pub use parser::MspFrame;
pub use parser::MspFrameError;
pub use parser::MspParser;
pub use parser::MspParserError;
pub use parser::MspVersion;
pub use parser::crc8_dvbs2_slice;
