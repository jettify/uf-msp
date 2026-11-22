use crate::MspDecodeError;
use crate::MspEncodeError;
use crate::MspPayload;
use crate::codec::PayloadReader;
use crate::codec::PayloadWriter;
use crate::commands::MSP_ALTITUDE;
use crate::commands::MSP_ATTITUDE;
use crate::commands::MSP_RAW_IMU;

/// Attitude in the integer units transmitted by `MSP_ATTITUDE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspAttitude {
    pub roll_decidegrees: i16,
    pub pitch_decidegrees: i16,
    pub yaw_degrees: i16,
}

impl MspPayload for MspAttitude {
    const COMMAND_ID: u16 = MSP_ATTITUDE;
    const MIN_PAYLOAD_SIZE: usize = 6;

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        Ok(Self {
            roll_decidegrees: reader.read_i16()?,
            pitch_decidegrees: reader.read_i16()?,
            yaw_degrees: reader.read_i16()?,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, Self::MIN_PAYLOAD_SIZE)?;
        writer.write_i16(self.roll_decidegrees)?;
        writer.write_i16(self.pitch_decidegrees)?;
        writer.write_i16(self.yaw_degrees)?;
        Ok(writer.written())
    }
}

/// `MSP_RAW_IMU` values in X, Y, Z order.
///
/// Values retain their wire representation. Sensor scaling depends on the
/// flight controller firmware; no universal physical-unit conversion is applied.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspRawImu {
    pub acc: [i16; 3],
    pub gyro: [i16; 3],
    pub mag: [i16; 3],
}

impl MspPayload for MspRawImu {
    const COMMAND_ID: u16 = MSP_RAW_IMU;
    const MIN_PAYLOAD_SIZE: usize = 18;

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        Ok(Self {
            acc: [reader.read_i16()?, reader.read_i16()?, reader.read_i16()?],
            gyro: [reader.read_i16()?, reader.read_i16()?, reader.read_i16()?],
            mag: [reader.read_i16()?, reader.read_i16()?, reader.read_i16()?],
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, Self::MIN_PAYLOAD_SIZE)?;
        for value in self.acc {
            writer.write_i16(value)?;
        }
        for value in self.gyro {
            writer.write_i16(value)?;
        }
        for value in self.mag {
            writer.write_i16(value)?;
        }
        Ok(writer.written())
    }
}

/// Altitude and vertical speed transmitted by `MSP_ALTITUDE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspAltitude {
    pub altitude_cm: i32,
    pub vario_cm_s: i16,
}

impl MspPayload for MspAltitude {
    const COMMAND_ID: u16 = MSP_ALTITUDE;
    const MIN_PAYLOAD_SIZE: usize = 6;

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        Ok(Self {
            altitude_cm: reader.read_i32()?,
            vario_cm_s: reader.read_i16()?,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, Self::MIN_PAYLOAD_SIZE)?;
        writer.write_i32(self.altitude_cm)?;
        writer.write_i16(self.vario_cm_s)?;
        Ok(writer.written())
    }
}
