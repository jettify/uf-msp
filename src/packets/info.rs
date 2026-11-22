use crate::CapacityError;
use crate::MspDecodeError;
use crate::MspEncodeError;
use crate::MspPayload;
use crate::codec::PayloadReader;
use crate::codec::PayloadWriter;
use crate::commands::MSP_API_VERSION;
use crate::commands::MSP_FC_VARIANT;
use crate::commands::MSP_FC_VERSION;
use crate::commands::MSP_NAME;
use crate::commands::MSP_REBOOT;
use crate::commands::MSP_SET_NAME;
use crate::commands::MSP_STATUS;

/// Maximum craft name length in bytes transmitted by `MSP_NAME` / `MSP_SET_NAME`.
pub const MAX_CRAFT_NAME_LEN: usize = 16;

/// Maximum extended flight-mode flag bytes transmitted by `MSP_STATUS`.
pub const MAX_FLIGHT_MODE_EXT_BYTES: usize = 15;

/// `MSP_API_VERSION` response payload (`0x01`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspApiVersion {
    pub protocol_version: u8,
    pub api_version_major: u8,
    pub api_version_minor: u8,
}

impl MspPayload for MspApiVersion {
    const COMMAND_ID: u16 = MSP_API_VERSION;
    const MIN_PAYLOAD_SIZE: usize = 3;

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        Ok(Self {
            protocol_version: reader.read_u8()?,
            api_version_major: reader.read_u8()?,
            api_version_minor: reader.read_u8()?,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, Self::MIN_PAYLOAD_SIZE)?;
        writer.write_u8(self.protocol_version)?;
        writer.write_u8(self.api_version_major)?;
        writer.write_u8(self.api_version_minor)?;
        Ok(writer.written())
    }
}

/// `MSP_FC_VARIANT` response payload (`0x02`): 4-byte ASCII flight controller identifier (e.g. `*b"BTFL"`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspFcVariant {
    pub identifier: [u8; 4],
}

impl MspFcVariant {
    /// Returns the 4-byte identifier as a `&str` if it is valid UTF-8.
    #[must_use]
    pub fn as_str(&self) -> Option<&str> {
        core::str::from_utf8(&self.identifier).ok()
    }
}

impl MspPayload for MspFcVariant {
    const COMMAND_ID: u16 = MSP_FC_VARIANT;
    const MIN_PAYLOAD_SIZE: usize = 4;

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        Ok(Self {
            identifier: reader.read_array::<4>()?,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, Self::MIN_PAYLOAD_SIZE)?;
        writer.write_bytes(&self.identifier)?;
        Ok(writer.written())
    }
}

/// `MSP_FC_VERSION` response payload (`0x03`): 3-byte flight controller version (`major.minor.patch`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspFcVersion {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
}

impl MspPayload for MspFcVersion {
    const COMMAND_ID: u16 = MSP_FC_VERSION;
    const MIN_PAYLOAD_SIZE: usize = 3;

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        Ok(Self {
            major: reader.read_u8()?,
            minor: reader.read_u8()?,
            patch: reader.read_u8()?,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, Self::MIN_PAYLOAD_SIZE)?;
        writer.write_u8(self.major)?;
        writer.write_u8(self.minor)?;
        writer.write_u8(self.patch)?;
        Ok(writer.written())
    }
}

/// Craft name payload for `MSP_NAME` (`0x0A`) responses.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspName {
    name: heapless::String<MAX_CRAFT_NAME_LEN, u8>,
}

impl MspName {
    /// Copies a craft name into an owned response.
    ///
    /// # Errors
    ///
    /// Returns a capacity error if the name exceeds MAX_CRAFT_NAME_LEN bytes.
    pub fn new(name: &str) -> Result<Self, CapacityError> {
        copy_craft_name(name).map(|name| Self { name })
    }

    /// Returns the craft name.
    #[must_use]
    pub fn name(&self) -> &str {
        self.name.as_str()
    }
}

impl MspPayload for MspName {
    const COMMAND_ID: u16 = MSP_NAME;
    const MIN_PAYLOAD_SIZE: usize = 0;

    fn payload_size(&self) -> usize {
        self.name.len()
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        decode_craft_name(Self::COMMAND_ID, data).map(|name| Self { name })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, self.payload_size())?;
        writer.write_bytes(self.name.as_bytes())?;
        Ok(writer.written())
    }
}

/// Craft name setter request payload for `MSP_SET_NAME` (`0x0B`).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspSetName {
    name: heapless::String<MAX_CRAFT_NAME_LEN, u8>,
}

impl MspSetName {
    /// Copies a craft name into an owned request.
    ///
    /// # Errors
    ///
    /// Returns a capacity error if the name exceeds MAX_CRAFT_NAME_LEN bytes.
    pub fn new(name: &str) -> Result<Self, CapacityError> {
        copy_craft_name(name).map(|name| Self { name })
    }

    /// Returns the requested craft name.
    #[must_use]
    pub fn name(&self) -> &str {
        self.name.as_str()
    }
}

impl MspPayload for MspSetName {
    const COMMAND_ID: u16 = MSP_SET_NAME;
    const MIN_PAYLOAD_SIZE: usize = 0;

    fn payload_size(&self) -> usize {
        self.name.len()
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        decode_craft_name(Self::COMMAND_ID, data).map(|name| Self { name })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, self.payload_size())?;
        writer.write_bytes(self.name.as_bytes())?;
        Ok(writer.written())
    }
}

fn copy_craft_name(name: &str) -> Result<heapless::String<MAX_CRAFT_NAME_LEN, u8>, CapacityError> {
    let mut stored = heapless::String::new();
    stored
        .push_str(name)
        .map_err(|_| CapacityError::new(MAX_CRAFT_NAME_LEN, name.len()))?;
    Ok(stored)
}

fn decode_craft_name(
    command_id: u16,
    data: &[u8],
) -> Result<heapless::String<MAX_CRAFT_NAME_LEN, u8>, MspDecodeError> {
    PayloadReader::new(command_id, data).read_remaining_string()
}

/// Cumulative trailing fields appended to [`MspStatusExtended`] in newer Betaflight releases.
///
/// Using an enum makes out-of-order extension combinations (such as a battery profile
/// without a core temperature or control-rate profile count) unrepresentable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MspStatusTail {
    /// Betaflight 4.5 (initial API 1.46): 2-byte signed CPU core temperature in °C.
    CoreTemp { core_temp_celsius: i16 },
    /// Later API 1.46+: CPU core temperature and control-rate profile count.
    ControlRateProfileCount {
        core_temp_celsius: i16,
        control_rate_profile_count: u8,
    },
    /// Betaflight 4.6+ (API 1.48+): adds battery profile count and active battery profile index.
    BatteryProfile {
        core_temp_celsius: i16,
        control_rate_profile_count: u8,
        battery_profile_count: u8,
        current_battery_profile_index: u8,
    },
}

impl MspStatusTail {
    #[must_use]
    pub const fn encoded_size(&self) -> usize {
        match self {
            Self::CoreTemp { .. } => 2,
            Self::ControlRateProfileCount { .. } => 3,
            Self::BatteryProfile { .. } => 5,
        }
    }

    #[must_use]
    pub const fn core_temp_celsius(&self) -> i16 {
        match *self {
            Self::CoreTemp { core_temp_celsius }
            | Self::ControlRateProfileCount {
                core_temp_celsius, ..
            }
            | Self::BatteryProfile {
                core_temp_celsius, ..
            } => core_temp_celsius,
        }
    }

    #[must_use]
    pub const fn control_rate_profile_count(&self) -> Option<u8> {
        match *self {
            Self::CoreTemp { .. } => None,
            Self::ControlRateProfileCount {
                control_rate_profile_count,
                ..
            }
            | Self::BatteryProfile {
                control_rate_profile_count,
                ..
            } => Some(control_rate_profile_count),
        }
    }

    #[must_use]
    pub const fn battery_profile(&self) -> Option<(u8, u8)> {
        match *self {
            Self::CoreTemp { .. } | Self::ControlRateProfileCount { .. } => None,
            Self::BatteryProfile {
                battery_profile_count,
                current_battery_profile_index,
                ..
            } => Some((battery_profile_count, current_battery_profile_index)),
        }
    }
}

/// Optional trailing fields appended to `MSP_STATUS` in modern Betaflight API revisions.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspStatusExtended {
    pub average_system_load_percent: u16,
    pub gyro_cycle_time: u16,
    flight_mode_flags_ext: heapless::Vec<u8, MAX_FLIGHT_MODE_EXT_BYTES, u8>,
    pub arming_disable_flags_count: u8,
    pub arming_disable_flags: u32,
    pub config_state_flags: u8,
    pub tail: Option<MspStatusTail>,
}

impl MspStatusExtended {
    /// Returns the flight mode flag bytes following the first 32 bits.
    #[must_use]
    pub fn flight_mode_flags_ext(&self) -> &[u8] {
        self.flight_mode_flags_ext.as_slice()
    }

    /// Replaces the flight mode flag bytes following the first 32 bits.
    ///
    /// # Errors
    ///
    /// Returns a capacity error if the input exceeds MAX_FLIGHT_MODE_EXT_BYTES.
    /// On error, the previous flag bytes are preserved.
    pub fn set_flight_mode_flags_ext(&mut self, flags: &[u8]) -> Result<(), CapacityError> {
        self.flight_mode_flags_ext = heapless::Vec::from_slice(flags)
            .map_err(|_| CapacityError::new(MAX_FLIGHT_MODE_EXT_BYTES, flags.len()))?;
        Ok(())
    }

    /// Returns the signed CPU core temperature in °C, if present (`API >= 1.46`).
    #[must_use]
    pub const fn core_temp_celsius(&self) -> Option<i16> {
        match &self.tail {
            Some(tail) => Some(tail.core_temp_celsius()),
            None => None,
        }
    }

    /// Returns the control-rate profile count, if present.
    #[must_use]
    pub const fn control_rate_profile_count(&self) -> Option<u8> {
        match &self.tail {
            Some(tail) => tail.control_rate_profile_count(),
            None => None,
        }
    }

    /// Returns `(battery_profile_count, current_battery_profile_index)`, if present (`API >= 1.48`).
    #[must_use]
    pub const fn battery_profile(&self) -> Option<(u8, u8)> {
        match &self.tail {
            Some(tail) => tail.battery_profile(),
            None => None,
        }
    }
}

/// `MSP_STATUS` response payload (`0x65`).
///
/// Supports both the classic 11-byte MultiWii/Cleanflight prefix and the modern
/// Betaflight extended status fields (variable flight mode bitmask, arming disable flags,
/// CPU temperature, and profile counts).
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspStatus {
    pub cycle_time_us: u16,
    pub i2c_error_count: u16,
    pub sensors: u16,
    pub flight_mode_flags: u32,
    pub current_pid_profile_index: u8,
    pub extended: Option<MspStatusExtended>,
}

impl MspPayload for MspStatus {
    const COMMAND_ID: u16 = MSP_STATUS;
    const MIN_PAYLOAD_SIZE: usize = 11;

    fn payload_size(&self) -> usize {
        let Some(ext) = &self.extended else {
            return Self::MIN_PAYLOAD_SIZE;
        };
        let tail_size = ext.tail.as_ref().map_or(0, MspStatusTail::encoded_size);
        Self::MIN_PAYLOAD_SIZE + 5 + ext.flight_mode_flags_ext.len() + 6 + tail_size
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        let cycle_time_us = reader.read_u16()?;
        let i2c_error_count = reader.read_u16()?;
        let sensors = reader.read_u16()?;
        let flight_mode_flags = reader.read_u32()?;
        let current_pid_profile_index = reader.read_u8()?;

        let extended = if reader.remaining_len() >= 11 {
            let average_system_load_percent = reader.read_u16()?;
            let gyro_cycle_time = reader.read_u16()?;
            let ext_len = usize::from(reader.read_u8()? & 0x0F);
            let flight_mode_flags_ext = reader.read_vec(ext_len)?;
            let arming_disable_flags_count = reader.read_u8()?;
            let arming_disable_flags = reader.read_u32()?;
            let config_state_flags = reader.read_u8()?;

            let tail = if reader.remaining_len() >= 5 {
                let core_temp_celsius = reader.read_i16()?;
                let control_rate_profile_count = reader.read_u8()?;
                let battery_profile_count = reader.read_u8()?;
                let current_battery_profile_index = reader.read_u8()?;
                Some(MspStatusTail::BatteryProfile {
                    core_temp_celsius,
                    control_rate_profile_count,
                    battery_profile_count,
                    current_battery_profile_index,
                })
            } else if reader.remaining_len() >= 3 {
                let core_temp_celsius = reader.read_i16()?;
                let control_rate_profile_count = reader.read_u8()?;
                Some(MspStatusTail::ControlRateProfileCount {
                    core_temp_celsius,
                    control_rate_profile_count,
                })
            } else if reader.remaining_len() >= 2 {
                let core_temp_celsius = reader.read_i16()?;
                Some(MspStatusTail::CoreTemp { core_temp_celsius })
            } else {
                None
            };

            Some(MspStatusExtended {
                average_system_load_percent,
                gyro_cycle_time,
                flight_mode_flags_ext,
                arming_disable_flags_count,
                arming_disable_flags,
                config_state_flags,
                tail,
            })
        } else {
            None
        };

        Ok(Self {
            cycle_time_us,
            i2c_error_count,
            sensors,
            flight_mode_flags,
            current_pid_profile_index,
            extended,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, self.payload_size())?;
        writer.write_u16(self.cycle_time_us)?;
        writer.write_u16(self.i2c_error_count)?;
        writer.write_u16(self.sensors)?;
        writer.write_u32(self.flight_mode_flags)?;
        writer.write_u8(self.current_pid_profile_index)?;

        if let Some(ext) = &self.extended {
            writer.write_u16(ext.average_system_load_percent)?;
            writer.write_u16(ext.gyro_cycle_time)?;
            let flags = ext.flight_mode_flags_ext();
            writer.write_u8(flags.len() as u8)?;
            writer.write_bytes(flags)?;
            writer.write_u8(ext.arming_disable_flags_count)?;
            writer.write_u32(ext.arming_disable_flags)?;
            writer.write_u8(ext.config_state_flags)?;

            if let Some(tail) = &ext.tail {
                match *tail {
                    MspStatusTail::CoreTemp { core_temp_celsius } => {
                        writer.write_i16(core_temp_celsius)?;
                    }
                    MspStatusTail::ControlRateProfileCount {
                        core_temp_celsius,
                        control_rate_profile_count,
                    } => {
                        writer.write_i16(core_temp_celsius)?;
                        writer.write_u8(control_rate_profile_count)?;
                    }
                    MspStatusTail::BatteryProfile {
                        core_temp_celsius,
                        control_rate_profile_count,
                        battery_profile_count,
                        current_battery_profile_index,
                    } => {
                        writer.write_i16(core_temp_celsius)?;
                        writer.write_u8(control_rate_profile_count)?;
                        writer.write_u8(battery_profile_count)?;
                        writer.write_u8(current_battery_profile_index)?;
                    }
                }
            }
        }
        Ok(writer.written())
    }
}

/// Parameterized request payload for `MSP_REBOOT` (`0x44`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspRebootRequest {
    /// Optional reboot mode (`0` = firmware, `1` = ROM bootloader, `2` = MSC, `3` = MSC UTC, `4` = flash bootloader).
    pub mode: Option<u8>,
}

impl MspPayload for MspRebootRequest {
    const COMMAND_ID: u16 = MSP_REBOOT;
    const MIN_PAYLOAD_SIZE: usize = 0;

    fn payload_size(&self) -> usize {
        usize::from(self.mode.is_some())
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader = PayloadReader::new(Self::COMMAND_ID, data);
        let mode = if reader.remaining_len() >= 1 {
            Some(reader.read_u8()?)
        } else {
            None
        };
        Ok(Self { mode })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, self.payload_size())?;
        if let Some(mode) = self.mode {
            writer.write_u8(mode)?;
        }
        Ok(writer.written())
    }
}

/// Response payload for `MSP_REBOOT` (`0x44`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspRebootResponse {
    pub mode: u8,
    /// Present when `mode == MSP_REBOOT_MSC` (`2`) on targets with USB MSC support.
    pub msc_ready: Option<bool>,
}

impl MspPayload for MspRebootResponse {
    const COMMAND_ID: u16 = MSP_REBOOT;
    const MIN_PAYLOAD_SIZE: usize = 1;

    fn payload_size(&self) -> usize {
        1 + usize::from(self.msc_ready.is_some())
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        let mode = reader.read_u8()?;
        let msc_ready = if reader.remaining_len() >= 1 {
            Some(reader.read_bool()?)
        } else {
            None
        };
        Ok(Self { mode, msc_ready })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, self.payload_size())?;
        writer.write_u8(self.mode)?;
        if let Some(ready) = self.msc_ready {
            writer.write_bool(ready)?;
        }
        Ok(writer.written())
    }
}
