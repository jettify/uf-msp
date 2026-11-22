#![no_std]
#![doc = include_str!("../README.md")]

mod codec;
/// MSP command IDs.
pub mod commands;
mod constants;
mod error;
mod packets;
mod parser;

pub use codec::MspPayload;
pub use codec::write_packet_to_buffer;
pub use codec::write_raw_frame_to_buffer;
pub use codec::write_request_to_buffer;
pub use error::CapacityError;
pub use error::MspDecodeError;
pub use error::MspEncodeError;
pub use error::MspError;
pub use error::MspFrameError;
pub use error::Result;
pub use packets::MAX_CRAFT_NAME_LEN;
pub use packets::MAX_FLIGHT_MODE_EXT_BYTES;
pub use packets::MAX_RC_CHANNELS;
pub use packets::MspAltitude;
pub use packets::MspApiVersion;
pub use packets::MspAttitude;
pub use packets::MspFcVariant;
pub use packets::MspFcVersion;
pub use packets::MspMessage;
pub use packets::MspName;
pub use packets::MspPacket;
pub use packets::MspPackets;
pub use packets::MspRawImu;
pub use packets::MspRc;
pub use packets::MspRebootRequest;
pub use packets::MspRebootResponse;
pub use packets::MspSetName;
pub use packets::MspSetRawRc;
pub use packets::MspStatus;
pub use packets::MspStatusExtended;
pub use packets::MspStatusTail;
pub use parser::MspDirection;
pub use parser::MspFrame;
pub use parser::MspParser;
pub use parser::MspVersion;
pub use parser::crc8_dvbs2_slice;
