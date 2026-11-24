use crate::MspDecodeError;
use crate::MspEncodeError;
use crate::MspPayload;
use crate::codec::PayloadReader;
use crate::codec::PayloadWriter;
use crate::commands::MSP_ANALOG;
use crate::commands::MSP_BATTERY_STATE;
use crate::commands::MSP2_BATTERY_PROFILE;

/// Voltage, current, consumption, and signal strength from `MSP_ANALOG`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspAnalog {
    pub voltage_decivolts: u8,
    pub consumed_mah: u16,
    /// Raw receiver signal strength.
    pub rssi: u16,
    pub current_centiamps: i16,
    /// Higher-resolution voltage supplied by newer Betaflight firmware.
    pub voltage_centivolts: Option<u16>,
}

impl MspPayload for MspAnalog {
    const COMMAND_ID: u16 = MSP_ANALOG;
    const MIN_PAYLOAD_SIZE: usize = 7;

    fn payload_size(&self) -> usize {
        if self.voltage_centivolts.is_some() {
            Self::MIN_PAYLOAD_SIZE + 2
        } else {
            Self::MIN_PAYLOAD_SIZE
        }
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        Ok(Self {
            voltage_decivolts: reader.read_u8()?,
            consumed_mah: reader.read_u16()?,
            rssi: reader.read_u16()?,
            current_centiamps: reader.read_i16()?,
            voltage_centivolts: (reader.remaining_len() > 0)
                .then(|| reader.read_u16())
                .transpose()?,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, self.payload_size())?;
        writer.write_u8(self.voltage_decivolts)?;
        writer.write_u16(self.consumed_mah)?;
        writer.write_u16(self.rssi)?;
        writer.write_i16(self.current_centiamps)?;
        if let Some(voltage) = self.voltage_centivolts {
            writer.write_u16(voltage)?;
        }
        Ok(writer.written())
    }
}

/// Betaflight battery status from `MSP_BATTERY_STATE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspBatteryState {
    /// Zero indicates no battery was detected.
    pub cell_count: u8,
    pub capacity_mah: u16,
    pub voltage_decivolts: u8,
    pub consumed_mah: u16,
    pub current_centiamps: i16,
    /// Firmware battery status and alert code.
    pub state: u8,
    /// Higher-resolution voltage supplied by newer Betaflight firmware.
    pub voltage_centivolts: Option<u16>,
}

impl MspPayload for MspBatteryState {
    const COMMAND_ID: u16 = MSP_BATTERY_STATE;
    const MIN_PAYLOAD_SIZE: usize = 9;

    fn payload_size(&self) -> usize {
        if self.voltage_centivolts.is_some() {
            Self::MIN_PAYLOAD_SIZE + 2
        } else {
            Self::MIN_PAYLOAD_SIZE
        }
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        Ok(Self {
            cell_count: reader.read_u8()?,
            capacity_mah: reader.read_u16()?,
            voltage_decivolts: reader.read_u8()?,
            consumed_mah: reader.read_u16()?,
            current_centiamps: reader.read_i16()?,
            state: reader.read_u8()?,
            voltage_centivolts: (reader.remaining_len() > 0)
                .then(|| reader.read_u16())
                .transpose()?,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, self.payload_size())?;
        writer.write_u8(self.cell_count)?;
        writer.write_u16(self.capacity_mah)?;
        writer.write_u8(self.voltage_decivolts)?;
        writer.write_u16(self.consumed_mah)?;
        writer.write_i16(self.current_centiamps)?;
        writer.write_u8(self.state)?;
        if let Some(voltage) = self.voltage_centivolts {
            writer.write_u16(voltage)?;
        }
        Ok(writer.written())
    }
}

/// Requests a Betaflight battery profile via `MSP2_BATTERY_PROFILE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspBatteryProfileRequest {
    /// Zero-based profile index. An empty request selects the active profile.
    pub profile_index: Option<u8>,
}

impl MspPayload for MspBatteryProfileRequest {
    const COMMAND_ID: u16 = MSP2_BATTERY_PROFILE;
    const MIN_PAYLOAD_SIZE: usize = 0;

    fn payload_size(&self) -> usize {
        usize::from(self.profile_index.is_some())
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        Ok(Self {
            profile_index: data.first().copied(),
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, self.payload_size())?;
        if let Some(index) = self.profile_index {
            writer.write_u8(index)?;
        }
        Ok(writer.written())
    }
}

/// Per-cell voltage thresholds and capacity from `MSP2_BATTERY_PROFILE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspBatteryProfile {
    pub profile_index: u8,
    pub min_cell_centivolts: u16,
    pub max_cell_centivolts: u16,
    pub warning_cell_centivolts: u16,
    pub full_cell_centivolts: u16,
    pub capacity_mah: u16,
    /// Zero enables automatic cell-count detection.
    pub forced_cell_count: u8,
    pub consumption_warning_percent: u8,
}

impl MspPayload for MspBatteryProfile {
    const COMMAND_ID: u16 = MSP2_BATTERY_PROFILE;
    const MIN_PAYLOAD_SIZE: usize = 13;

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        let mut reader =
            PayloadReader::with_min_size(Self::COMMAND_ID, data, Self::MIN_PAYLOAD_SIZE)?;
        Ok(Self {
            profile_index: reader.read_u8()?,
            min_cell_centivolts: reader.read_u16()?,
            max_cell_centivolts: reader.read_u16()?,
            warning_cell_centivolts: reader.read_u16()?,
            full_cell_centivolts: reader.read_u16()?,
            capacity_mah: reader.read_u16()?,
            forced_cell_count: reader.read_u8()?,
            consumption_warning_percent: reader.read_u8()?,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let mut writer = PayloadWriter::with_min_size(buffer, Self::MIN_PAYLOAD_SIZE)?;
        writer.write_u8(self.profile_index)?;
        writer.write_u16(self.min_cell_centivolts)?;
        writer.write_u16(self.max_cell_centivolts)?;
        writer.write_u16(self.warning_cell_centivolts)?;
        writer.write_u16(self.full_cell_centivolts)?;
        writer.write_u16(self.capacity_mah)?;
        writer.write_u8(self.forced_cell_count)?;
        writer.write_u8(self.consumption_warning_percent)?;
        Ok(writer.written())
    }
}
