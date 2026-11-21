#![no_std]
#![doc = include_str!("../README.md")]

mod constants;
mod packet;
mod parser;

/// MSP command IDs.
pub mod commands {
    pub use crate::constants::MSP_ALTITUDE;
    pub use crate::constants::MSP_ATTITUDE;
    pub use crate::constants::MSP_RAW_IMU;
}

pub use packet::MspAltitude;
pub use packet::MspAttitude;
pub use packet::MspDecodeError;
pub use packet::MspMessage;
pub use packet::MspPacket;
pub use packet::MspPacketError;
pub use packet::MspPackets;
pub use packet::MspRawImu;
pub use parser::MspDirection;
pub use parser::MspFrame;
pub use parser::MspFrameError;
pub use parser::MspParser;
pub use parser::MspParserError;
pub use parser::MspVersion;
pub use parser::crc8_dvbs2_slice;
