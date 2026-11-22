mod telemetry;

pub use telemetry::MspAltitude;
pub use telemetry::MspAttitude;
pub use telemetry::MspRawImu;

use crate::MspDecodeError;
use crate::MspDirection;
use crate::MspEncodeError;
use crate::MspFrame;
use crate::MspParser;
use crate::MspPayload;
use crate::MspVersion;
use crate::commands::MSP_ALTITUDE;
use crate::commands::MSP_ATTITUDE;
use crate::commands::MSP_RAW_IMU;
use crate::write_packet_to_buffer;
use crate::write_request_to_buffer;

/// Supported polling requests and telemetry responses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum MspMessage {
    AttitudeRequest,
    Attitude(MspAttitude),
    RawImuRequest,
    RawImu(MspRawImu),
    AltitudeRequest,
    Altitude(MspAltitude),
}

impl MspMessage {
    /// Returns the MSP command ID implied by this message.
    #[must_use]
    pub const fn command_id(&self) -> u16 {
        match self {
            Self::AttitudeRequest | Self::Attitude(_) => MSP_ATTITUDE,
            Self::RawImuRequest | Self::RawImu(_) => MSP_RAW_IMU,
            Self::AltitudeRequest | Self::Altitude(_) => MSP_ALTITUDE,
        }
    }

    /// Returns the request or response direction implied by this message.
    #[must_use]
    pub const fn direction(&self) -> MspDirection {
        match self {
            Self::AttitudeRequest | Self::RawImuRequest | Self::AltitudeRequest => {
                MspDirection::Request
            }
            Self::Attitude(_) | Self::RawImu(_) | Self::Altitude(_) => MspDirection::Response,
        }
    }
}

/// An owned decoded packet, independent of the parser and its input bytes.
///
/// Wire bytes, checksums, and ignored trailing payload bytes are not retained.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MspPacket {
    pub version: MspVersion,
    pub flags: u8,
    pub message: MspMessage,
}

impl MspPacket {
    #[must_use]
    pub const fn command_id(&self) -> u16 {
        self.message.command_id()
    }

    #[must_use]
    pub const fn direction(&self) -> MspDirection {
        self.message.direction()
    }

    /// Serializes this packet as a complete MSP wire frame into `buffer`,
    /// returning the total frame length in bytes.
    ///
    /// # Errors
    ///
    /// Returns [`MspEncodeError::BufferTooSmall`] if `buffer` cannot hold the complete wire frame,
    /// [`MspEncodeError::InvalidV1Command`] if `version` is [`MspVersion::V1`] and the command ID
    /// is at least 255, or [`MspEncodeError::PayloadTooLarge`] if the payload exceeds wire limits.
    pub fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        match &self.message {
            MspMessage::AttitudeRequest
            | MspMessage::RawImuRequest
            | MspMessage::AltitudeRequest => {
                write_request_to_buffer(buffer, self.version, self.flags, self.command_id())
            }
            MspMessage::Attitude(attitude) => write_packet_to_buffer(
                buffer,
                self.version,
                MspDirection::Response,
                self.flags,
                attitude,
            ),
            MspMessage::RawImu(imu) => write_packet_to_buffer(
                buffer,
                self.version,
                MspDirection::Response,
                self.flags,
                imu,
            ),
            MspMessage::Altitude(altitude) => write_packet_to_buffer(
                buffer,
                self.version,
                MspDirection::Response,
                self.flags,
                altitude,
            ),
        }
    }
}

fn decode_command<P: MspPayload>(
    direction: MspDirection,
    payload: &[u8],
    request: MspMessage,
    wrap: fn(P) -> MspMessage,
) -> Result<MspMessage, MspDecodeError> {
    match direction {
        MspDirection::Error => Err(MspDecodeError::RemoteError {
            command_id: P::COMMAND_ID,
        }),
        MspDirection::Request => Ok(request),
        MspDirection::Response => P::from_bytes(payload).map(wrap),
    }
}

impl TryFrom<MspFrame<'_>> for MspPacket {
    type Error = MspDecodeError;

    /// Decodes the known response prefix, ignoring extra payload bytes.
    /// Polling request payloads are ignored. The frame is already validated,
    /// including checksums covering all declared payload bytes.
    fn try_from(frame: MspFrame<'_>) -> Result<Self, Self::Error> {
        let command_id = frame.command_id();
        let direction = frame.direction();
        let payload = frame.payload();
        let message = match command_id {
            MSP_ATTITUDE => decode_command(
                direction,
                payload,
                MspMessage::AttitudeRequest,
                MspMessage::Attitude,
            )?,
            MSP_RAW_IMU => decode_command(
                direction,
                payload,
                MspMessage::RawImuRequest,
                MspMessage::RawImu,
            )?,
            MSP_ALTITUDE => decode_command(
                direction,
                payload,
                MspMessage::AltitudeRequest,
                MspMessage::Altitude,
            )?,
            _ => return Err(MspDecodeError::UnsupportedCommand { command_id }),
        };
        Ok(Self {
            version: frame.version(),
            flags: frame.flags(),
            message,
        })
    }
}

impl<const N: usize> MspParser<N> {
    /// Consumes the next validated frame and decodes it into an owned packet.
    ///
    /// Incomplete input is retained according to the same spill-buffer rules
    /// as [`Self::next`].
    ///
    /// # Errors
    ///
    /// Decode errors consume one frame and allow parsing to continue.
    pub fn next_packet(&mut self, input: &mut &[u8]) -> Result<Option<MspPacket>, MspDecodeError> {
        self.next(input).map(MspPacket::try_from).transpose()
    }

    /// Iterates over owned packets, advancing the caller's input cursor.
    ///
    /// Decode errors are yielded and iteration continues. Incomplete input ends
    /// this iterator; create another with the next chunk to continue parsing.
    /// Dropping early leaves the cursor at the first unconsumed byte.
    pub fn packets<'a, 'input>(
        &'a mut self,
        input: &'a mut &'input [u8],
    ) -> MspPackets<'a, 'input, N> {
        MspPackets {
            parser: self,
            input,
        }
    }
}

/// An iterator borrowing a parser and input cursor, yielding owned packets.
#[must_use = "iterators do nothing unless consumed"]
pub struct MspPackets<'a, 'input, const N: usize> {
    parser: &'a mut MspParser<N>,
    input: &'a mut &'input [u8],
}

impl<const N: usize> core::fmt::Debug for MspPackets<'_, '_, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("MspPackets")
            .field("parser", &self.parser)
            .field("remaining_len", &self.input.len())
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl<const N: usize> defmt::Format for MspPackets<'_, '_, N> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "MspPackets {{ parser: {}, remaining_len: {} }}",
            self.parser,
            self.input.len()
        );
    }
}

impl<const N: usize> Iterator for MspPackets<'_, '_, N> {
    type Item = Result<MspPacket, MspDecodeError>;

    fn next(&mut self) -> Option<Self::Item> {
        self.parser.next_packet(self.input).transpose()
    }
}

impl<const N: usize> core::iter::FusedIterator for MspPackets<'_, '_, N> {}
