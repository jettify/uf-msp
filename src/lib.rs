#![no_std]
#![doc = include_str!("../README.md")]

mod codec;
/// MSP command IDs.
pub mod commands;
mod constants;
mod packets;
mod parser;

pub use codec::MspDecodeError;
pub use codec::MspEncodeError;
pub use codec::MspPayload;
pub use codec::write_packet_to_buffer;
pub use codec::write_raw_frame_to_buffer;
pub use codec::write_request_to_buffer;
pub use packets::MspAltitude;
pub use packets::MspAttitude;
pub use packets::MspMessage;
pub use packets::MspPacket;
pub use packets::MspPackets;
pub use packets::MspRawImu;
pub use parser::MspDirection;
pub use parser::MspFrame;
pub use parser::MspFrameError;
pub use parser::MspParser;
pub use parser::MspVersion;
pub use parser::crc8_dvbs2_slice;
