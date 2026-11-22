use crate::CapacityError;
use crate::MspDecodeError;
use crate::MspEncodeError;
use crate::MspPayload;
use crate::codec::PayloadReader;
use crate::codec::PayloadWriter;
use crate::commands::MSP_RC;
use crate::commands::MSP_SET_RAW_RC;

/// Maximum number of RC channels supported by Betaflight in `MSP_RC` and `MSP_SET_RAW_RC`.
pub const MAX_RC_CHANNELS: usize = 18;

type Channels = heapless::Vec<u16, MAX_RC_CHANNELS, u8>;

fn copy_channels(channels: &[u16]) -> Result<Channels, CapacityError> {
    Channels::from_slice(channels).map_err(|_| CapacityError::new(MAX_RC_CHANNELS, channels.len()))
}

fn decode_channels(command_id: u16, data: &[u8]) -> Result<Channels, MspDecodeError> {
    if !data.len().is_multiple_of(2) {
        return Err(MspDecodeError::InvalidPayload { command_id });
    }
    let count = data.len() / 2;
    if count > MAX_RC_CHANNELS {
        return Err(MspDecodeError::CapacityExceeded {
            command_id,
            capacity: MAX_RC_CHANNELS,
            requested: count,
        });
    }
    let mut reader = PayloadReader::with_min_size(command_id, data, data.len())?;
    let mut channels = Channels::new();
    for _ in 0..count {
        let ch = reader.read_u16()?;
        channels
            .push(ch)
            .map_err(|_| MspDecodeError::CapacityExceeded {
                command_id,
                capacity: MAX_RC_CHANNELS,
                requested: count,
            })?;
    }
    Ok(channels)
}

fn encode_channels(channels: &[u16], buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
    let mut writer = PayloadWriter::with_min_size(buffer, channels.len() * 2)?;
    for &ch in channels {
        writer.write_u16(ch)?;
    }
    Ok(writer.written())
}

/// `MSP_RC` response payload (`0x69`): variable array of up to 18 RC channel pulse widths in microseconds.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspRc {
    channels: Channels,
}

impl MspRc {
    /// Copies RC channel pulse widths in microseconds into the payload.
    ///
    /// # Errors
    ///
    /// Returns a capacity error if the input exceeds `MAX_RC_CHANNELS`.
    pub fn new(channels: &[u16]) -> Result<Self, CapacityError> {
        copy_channels(channels).map(|channels| Self { channels })
    }

    /// Returns the channel pulse widths in microseconds.
    #[must_use]
    pub fn channels(&self) -> &[u16] {
        self.channels.as_slice()
    }
}

impl MspPayload for MspRc {
    const COMMAND_ID: u16 = MSP_RC;
    const MIN_PAYLOAD_SIZE: usize = 0;

    fn payload_size(&self) -> usize {
        self.channels.len() * 2
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        Ok(Self {
            channels: decode_channels(Self::COMMAND_ID, data)?,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        encode_channels(self.channels(), buffer)
    }
}

/// `MSP_SET_RAW_RC` setter request payload (`0xC8`): variable array of up to 18 RC channel pulse widths.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspSetRawRc {
    channels: Channels,
}

impl MspSetRawRc {
    /// Copies RC channel pulse widths in microseconds into the payload.
    ///
    /// # Errors
    ///
    /// Returns a capacity error if the input exceeds `MAX_RC_CHANNELS`.
    pub fn new(channels: &[u16]) -> Result<Self, CapacityError> {
        copy_channels(channels).map(|channels| Self { channels })
    }

    /// Returns the channel pulse widths in microseconds.
    #[must_use]
    pub fn channels(&self) -> &[u16] {
        self.channels.as_slice()
    }
}

impl MspPayload for MspSetRawRc {
    const COMMAND_ID: u16 = MSP_SET_RAW_RC;
    const MIN_PAYLOAD_SIZE: usize = 0;

    fn payload_size(&self) -> usize {
        self.channels.len() * 2
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        Ok(Self {
            channels: decode_channels(Self::COMMAND_ID, data)?,
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        encode_channels(self.channels(), buffer)
    }
}
