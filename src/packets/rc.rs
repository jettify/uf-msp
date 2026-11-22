use crate::MspDecodeError;
use crate::MspEncodeError;
use crate::MspPayload;
use crate::MspVec;
use crate::codec::PayloadReader;
use crate::codec::PayloadWriter;
use crate::commands::MSP_RC;
use crate::commands::MSP_SET_RAW_RC;

/// Maximum number of RC channels supported by Betaflight in `MSP_RC` and `MSP_SET_RAW_RC`.
pub const MAX_RC_CHANNELS: usize = 18;

fn decode_channels(
    command_id: u16,
    data: &[u8],
) -> Result<MspVec<u16, MAX_RC_CHANNELS>, MspDecodeError> {
    if !data.len().is_multiple_of(2) {
        return Err(MspDecodeError::InvalidPayload { command_id });
    }
    let count = data.len() / 2;
    if count > MAX_RC_CHANNELS {
        return Err(MspDecodeError::InvalidPayload { command_id });
    }
    let mut reader = PayloadReader::with_min_size(command_id, data, data.len())?;
    let mut channels = MspVec::new();
    for _ in 0..count {
        let ch = reader.read_u16()?;
        channels
            .push(ch)
            .map_err(|_| MspDecodeError::InvalidPayload { command_id })?;
    }
    Ok(channels)
}

fn encode_channels(
    channels: &MspVec<u16, MAX_RC_CHANNELS>,
    buffer: &mut [u8],
) -> Result<usize, MspEncodeError> {
    let mut writer = PayloadWriter::with_min_size(buffer, channels.len() * 2)?;
    for &ch in channels.as_slice() {
        writer.write_u16(ch)?;
    }
    Ok(writer.written())
}

/// `MSP_RC` response payload (`0x69`): variable array of up to 18 RC channel pulse widths in microseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspRc {
    pub channels: MspVec<u16, MAX_RC_CHANNELS>,
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
        encode_channels(&self.channels, buffer)
    }
}

impl TryFrom<&[u8]> for MspRc {
    type Error = MspDecodeError;

    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        Self::from_bytes(payload)
    }
}

/// `MSP_SET_RAW_RC` setter request payload (`0xC8`): variable array of up to 18 RC channel pulse widths.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspSetRawRc {
    pub channels: MspVec<u16, MAX_RC_CHANNELS>,
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
        encode_channels(&self.channels, buffer)
    }
}

impl TryFrom<&[u8]> for MspSetRawRc {
    type Error = MspDecodeError;

    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        Self::from_bytes(payload)
    }
}
