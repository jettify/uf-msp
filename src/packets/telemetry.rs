use crate::MspDecodeError;
use crate::MspEncodeError;
use crate::MspPayload;
use crate::codec::PayloadReader;
use crate::codec::PayloadWriter;
use crate::commands::MSP_ALTITUDE;
use crate::commands::MSP_ATTITUDE;
use crate::commands::MSP_COMP_GPS;
use crate::commands::MSP_RAW_GPS;
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

/// Position and ground velocity from `MSP_RAW_GPS`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspRawGps {
    /// Firmware-specific fix byte. Betaflight uses zero for no fix and nonzero for a fix.
    pub fix: u8,
    pub satellites: u8,
    pub latitude_e7: i32,
    pub longitude_e7: i32,
    pub altitude_m: u16,
    pub ground_speed_cm_s: u16,
    pub ground_course_decidegrees: u16,
    pub dop: Option<MspRawGpsDop>,
}

/// Optional dilution of precision fields from Betaflight's `MSP_RAW_GPS` response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspRawGpsDop {
    /// HDOP in Betaflight 4.4, and PDOP in Betaflight 4.5 and later.
    pub dop_centi: u16,
    pub extended: Option<MspRawGpsExtended>,
}

/// GPS fields appended after PDOP starting in Betaflight MSP API 1.49.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspRawGpsExtended {
    pub hdop_centi: u16,
    pub vdop_centi: u16,
    pub satellites_in_view: u8,
}

impl MspPayload for MspRawGps {
    const COMMAND_ID: u16 = MSP_RAW_GPS;
    const MIN_PAYLOAD_SIZE: usize = 16;

    fn payload_size(&self) -> usize {
        match &self.dop {
            None => Self::MIN_PAYLOAD_SIZE,
            Some(dop) if dop.extended.is_some() => Self::MIN_PAYLOAD_SIZE + 2 + 5,
            Some(_) => Self::MIN_PAYLOAD_SIZE + 2,
        }
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        let mut gps = Self {
            fix: reader.read_u8()?,
            satellites: reader.read_u8()?,
            latitude_e7: reader.read_i32()?,
            longitude_e7: reader.read_i32()?,
            altitude_m: reader.read_u16()?,
            ground_speed_cm_s: reader.read_u16()?,
            ground_course_decidegrees: reader.read_u16()?,
            dop: None,
        };
        if reader.remaining_len() > 0 {
            let dop_centi = reader.read_u16()?;
            let extended = if reader.remaining_len() > 0 {
                Some(MspRawGpsExtended {
                    hdop_centi: reader.read_u16()?,
                    vdop_centi: reader.read_u16()?,
                    satellites_in_view: reader.read_u8()?,
                })
            } else {
                None
            };
            gps.dop = Some(MspRawGpsDop {
                dop_centi,
                extended,
            });
        }
        Ok(gps)
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, self.payload_size())?;
        writer.write_u8(self.fix)?;
        writer.write_u8(self.satellites)?;
        writer.write_i32(self.latitude_e7)?;
        writer.write_i32(self.longitude_e7)?;
        writer.write_u16(self.altitude_m)?;
        writer.write_u16(self.ground_speed_cm_s)?;
        writer.write_u16(self.ground_course_decidegrees)?;
        if let Some(dop) = self.dop {
            writer.write_u16(dop.dop_centi)?;
            if let Some(extended) = dop.extended {
                writer.write_u16(extended.hdop_centi)?;
                writer.write_u16(extended.vdop_centi)?;
                writer.write_u8(extended.satellites_in_view)?;
            }
        }
        Ok(writer.written())
    }
}

/// Distance, direction, and GPS update byte from `MSP_COMP_GPS`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspCompGps {
    pub distance_to_home_m: u16,
    pub direction_to_home_degrees: i16,
    pub update: u8,
}

impl MspPayload for MspCompGps {
    const COMMAND_ID: u16 = MSP_COMP_GPS;
    const MIN_PAYLOAD_SIZE: usize = 5;

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        Ok(Self {
            distance_to_home_m: reader.read_u16()?,
            direction_to_home_degrees: reader.read_i16()?,
            update: reader.read_u8()?,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, Self::MIN_PAYLOAD_SIZE)?;
        writer.write_u16(self.distance_to_home_m)?;
        writer.write_i16(self.direction_to_home_degrees)?;
        writer.write_u8(self.update)?;
        Ok(writer.written())
    }
}
