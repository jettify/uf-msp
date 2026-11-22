mod info;
mod rc;
mod telemetry;

pub use info::MAX_CRAFT_NAME_LEN;
pub use info::MAX_FLIGHT_MODE_EXT_BYTES;
pub use info::MspApiVersion;
pub use info::MspFcVariant;
pub use info::MspFcVersion;
pub use info::MspName;
pub use info::MspRebootRequest;
pub use info::MspRebootResponse;
pub use info::MspSetName;
pub use info::MspStatus;
pub use info::MspStatusExtended;
pub use info::MspStatusTail;
pub use rc::MAX_RC_CHANNELS;
pub use rc::MspRc;
pub use rc::MspSetRawRc;
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
use crate::commands::MSP_ACC_CALIBRATION;
use crate::commands::MSP_ALTITUDE;
use crate::commands::MSP_API_VERSION;
use crate::commands::MSP_ATTITUDE;
use crate::commands::MSP_EEPROM_WRITE;
use crate::commands::MSP_FC_VARIANT;
use crate::commands::MSP_FC_VERSION;
use crate::commands::MSP_NAME;
use crate::commands::MSP_RAW_IMU;
use crate::commands::MSP_RC;
use crate::commands::MSP_REBOOT;
use crate::commands::MSP_SET_NAME;
use crate::commands::MSP_SET_RAW_RC;
use crate::commands::MSP_STATUS;
use crate::write_packet_to_buffer;
use crate::write_raw_frame_to_buffer;
use crate::write_request_to_buffer;

/// Supported MSP requests, responses, and setter acknowledgments.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum MspMessage {
    // Identity & Status (Out)
    ApiVersionRequest,
    ApiVersion(MspApiVersion),
    FcVariantRequest,
    FcVariant(MspFcVariant),
    FcVersionRequest,
    FcVersion(MspFcVersion),
    StatusRequest,
    Status(MspStatus),

    // Variable-length String (Out & In Setter)
    NameRequest,
    Name(MspName),
    SetName(MspSetName),
    SetNameResponse,

    // Parameterized Request -> Typed Response (In/Out)
    RebootRequest(MspRebootRequest),
    Reboot(MspRebootResponse),

    // Telemetry & Sensors (Out)
    RawImuRequest,
    RawImu(MspRawImu),
    AttitudeRequest,
    Attitude(MspAttitude),
    AltitudeRequest,
    Altitude(MspAltitude),

    // RC Channels (Out & In Setter)
    RcRequest,
    Rc(MspRc),
    SetRawRc(MspSetRawRc),
    SetRawRcResponse,

    // Action Triggers (Empty Request -> Empty ACK Response)
    AccCalibrationRequest,
    AccCalibrationResponse,
    EepromWriteRequest,
    EepromWriteResponse,
}

impl MspMessage {
    /// Returns the MSP command ID implied by this message.
    #[must_use]
    pub const fn command_id(&self) -> u16 {
        match self {
            Self::ApiVersionRequest | Self::ApiVersion(_) => MSP_API_VERSION,
            Self::FcVariantRequest | Self::FcVariant(_) => MSP_FC_VARIANT,
            Self::FcVersionRequest | Self::FcVersion(_) => MSP_FC_VERSION,
            Self::NameRequest | Self::Name(_) => MSP_NAME,
            Self::SetName(_) | Self::SetNameResponse => MSP_SET_NAME,
            Self::RebootRequest(_) | Self::Reboot(_) => MSP_REBOOT,
            Self::StatusRequest | Self::Status(_) => MSP_STATUS,
            Self::RawImuRequest | Self::RawImu(_) => MSP_RAW_IMU,
            Self::RcRequest | Self::Rc(_) => MSP_RC,
            Self::AttitudeRequest | Self::Attitude(_) => MSP_ATTITUDE,
            Self::AltitudeRequest | Self::Altitude(_) => MSP_ALTITUDE,
            Self::SetRawRc(_) | Self::SetRawRcResponse => MSP_SET_RAW_RC,
            Self::AccCalibrationRequest | Self::AccCalibrationResponse => MSP_ACC_CALIBRATION,
            Self::EepromWriteRequest | Self::EepromWriteResponse => MSP_EEPROM_WRITE,
        }
    }

    /// Returns the request or response direction implied by this message.
    #[must_use]
    pub const fn direction(&self) -> MspDirection {
        match self {
            Self::ApiVersionRequest
            | Self::FcVariantRequest
            | Self::FcVersionRequest
            | Self::StatusRequest
            | Self::NameRequest
            | Self::SetName(_)
            | Self::RebootRequest(_)
            | Self::RawImuRequest
            | Self::AttitudeRequest
            | Self::AltitudeRequest
            | Self::RcRequest
            | Self::SetRawRc(_)
            | Self::AccCalibrationRequest
            | Self::EepromWriteRequest => MspDirection::Request,
            Self::ApiVersion(_)
            | Self::FcVariant(_)
            | Self::FcVersion(_)
            | Self::Status(_)
            | Self::Name(_)
            | Self::SetNameResponse
            | Self::Reboot(_)
            | Self::RawImu(_)
            | Self::Attitude(_)
            | Self::Altitude(_)
            | Self::Rc(_)
            | Self::SetRawRcResponse
            | Self::AccCalibrationResponse
            | Self::EepromWriteResponse => MspDirection::Response,
        }
    }
}

/// An owned decoded packet, independent of the parser and its input bytes.
///
/// Wire bytes, checksums, and ignored trailing payload bytes are not retained.
#[derive(Debug, Clone, PartialEq, Eq)]
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
        let dir = self.direction();
        match &self.message {
            MspMessage::ApiVersionRequest
            | MspMessage::FcVariantRequest
            | MspMessage::FcVersionRequest
            | MspMessage::StatusRequest
            | MspMessage::NameRequest
            | MspMessage::RawImuRequest
            | MspMessage::AttitudeRequest
            | MspMessage::AltitudeRequest
            | MspMessage::RcRequest
            | MspMessage::AccCalibrationRequest
            | MspMessage::EepromWriteRequest => {
                write_request_to_buffer(buffer, self.version, self.flags, self.command_id())
            }
            MspMessage::SetNameResponse
            | MspMessage::SetRawRcResponse
            | MspMessage::AccCalibrationResponse
            | MspMessage::EepromWriteResponse => write_raw_frame_to_buffer(
                buffer,
                self.version,
                MspDirection::Response,
                self.flags,
                self.command_id(),
                &[],
            ),
            MspMessage::ApiVersion(p) => {
                write_packet_to_buffer(buffer, self.version, dir, self.flags, p)
            }
            MspMessage::FcVariant(p) => {
                write_packet_to_buffer(buffer, self.version, dir, self.flags, p)
            }
            MspMessage::FcVersion(p) => {
                write_packet_to_buffer(buffer, self.version, dir, self.flags, p)
            }
            MspMessage::Status(p) => {
                write_packet_to_buffer(buffer, self.version, dir, self.flags, p)
            }
            MspMessage::Name(p) => write_packet_to_buffer(buffer, self.version, dir, self.flags, p),
            MspMessage::SetName(p) => {
                write_packet_to_buffer(buffer, self.version, dir, self.flags, p)
            }
            MspMessage::RebootRequest(p) => {
                write_packet_to_buffer(buffer, self.version, dir, self.flags, p)
            }
            MspMessage::Reboot(p) => {
                write_packet_to_buffer(buffer, self.version, dir, self.flags, p)
            }
            MspMessage::Attitude(p) => {
                write_packet_to_buffer(buffer, self.version, dir, self.flags, p)
            }
            MspMessage::RawImu(p) => {
                write_packet_to_buffer(buffer, self.version, dir, self.flags, p)
            }
            MspMessage::Altitude(p) => {
                write_packet_to_buffer(buffer, self.version, dir, self.flags, p)
            }
            MspMessage::Rc(p) => write_packet_to_buffer(buffer, self.version, dir, self.flags, p),
            MspMessage::SetRawRc(p) => {
                write_packet_to_buffer(buffer, self.version, dir, self.flags, p)
            }
        }
    }
}

/// Decodes an "Out" command: empty `<` polling request, typed `>` response.
fn decode_out_command<P: MspPayload>(
    direction: MspDirection,
    payload: &[u8],
    request: MspMessage,
    wrap_response: fn(P) -> MspMessage,
) -> Result<MspMessage, MspDecodeError> {
    match direction {
        MspDirection::Error => Err(MspDecodeError::RemoteError {
            command_id: P::COMMAND_ID,
        }),
        MspDirection::Request => Ok(request),
        MspDirection::Response => P::from_bytes(payload).map(wrap_response),
    }
}

/// Decodes an "In" setter command: typed `<` request, empty `>` ACK response.
fn decode_in_command<P: MspPayload>(
    direction: MspDirection,
    payload: &[u8],
    wrap_request: fn(P) -> MspMessage,
    response_ack: MspMessage,
) -> Result<MspMessage, MspDecodeError> {
    match direction {
        MspDirection::Error => Err(MspDecodeError::RemoteError {
            command_id: P::COMMAND_ID,
        }),
        MspDirection::Request => P::from_bytes(payload).map(wrap_request),
        MspDirection::Response => Ok(response_ack),
    }
}

/// Decodes a bidirectional "In/Out" command: typed `<` request, typed `>` response.
fn decode_inout_command<Req: MspPayload, Resp: MspPayload>(
    direction: MspDirection,
    payload: &[u8],
    wrap_request: fn(Req) -> MspMessage,
    wrap_response: fn(Resp) -> MspMessage,
) -> Result<MspMessage, MspDecodeError> {
    match direction {
        MspDirection::Error => Err(MspDecodeError::RemoteError {
            command_id: Req::COMMAND_ID,
        }),
        MspDirection::Request => Req::from_bytes(payload).map(wrap_request),
        MspDirection::Response => Resp::from_bytes(payload).map(wrap_response),
    }
}

/// Decodes a parameterless action trigger: empty `<` request, empty `>` ACK response.
fn decode_action_command(
    command_id: u16,
    direction: MspDirection,
    request: MspMessage,
    response_ack: MspMessage,
) -> Result<MspMessage, MspDecodeError> {
    match direction {
        MspDirection::Error => Err(MspDecodeError::RemoteError { command_id }),
        MspDirection::Request => Ok(request),
        MspDirection::Response => Ok(response_ack),
    }
}

impl TryFrom<MspFrame<'_>> for MspPacket {
    type Error = MspDecodeError;

    /// Decodes the known command payload, ignoring extra trailing bytes on fixed-prefix
    /// responses. The frame is already validated, including checksums covering all
    /// declared payload bytes.
    fn try_from(frame: MspFrame<'_>) -> Result<Self, Self::Error> {
        let command_id = frame.command_id();
        let direction = frame.direction();
        let payload = frame.payload();
        let message = match command_id {
            MSP_API_VERSION => decode_out_command(
                direction,
                payload,
                MspMessage::ApiVersionRequest,
                MspMessage::ApiVersion,
            )?,
            MSP_FC_VARIANT => decode_out_command(
                direction,
                payload,
                MspMessage::FcVariantRequest,
                MspMessage::FcVariant,
            )?,
            MSP_FC_VERSION => decode_out_command(
                direction,
                payload,
                MspMessage::FcVersionRequest,
                MspMessage::FcVersion,
            )?,
            MSP_NAME => decode_out_command(
                direction,
                payload,
                MspMessage::NameRequest,
                MspMessage::Name,
            )?,
            MSP_SET_NAME => decode_in_command(
                direction,
                payload,
                MspMessage::SetName,
                MspMessage::SetNameResponse,
            )?,
            MSP_REBOOT => decode_inout_command(
                direction,
                payload,
                MspMessage::RebootRequest,
                MspMessage::Reboot,
            )?,
            MSP_STATUS => decode_out_command(
                direction,
                payload,
                MspMessage::StatusRequest,
                MspMessage::Status,
            )?,
            MSP_RAW_IMU => decode_out_command(
                direction,
                payload,
                MspMessage::RawImuRequest,
                MspMessage::RawImu,
            )?,
            MSP_RC => {
                decode_out_command(direction, payload, MspMessage::RcRequest, MspMessage::Rc)?
            }
            MSP_ATTITUDE => decode_out_command(
                direction,
                payload,
                MspMessage::AttitudeRequest,
                MspMessage::Attitude,
            )?,
            MSP_ALTITUDE => decode_out_command(
                direction,
                payload,
                MspMessage::AltitudeRequest,
                MspMessage::Altitude,
            )?,
            MSP_SET_RAW_RC => decode_in_command(
                direction,
                payload,
                MspMessage::SetRawRc,
                MspMessage::SetRawRcResponse,
            )?,
            MSP_ACC_CALIBRATION => decode_action_command(
                MSP_ACC_CALIBRATION,
                direction,
                MspMessage::AccCalibrationRequest,
                MspMessage::AccCalibrationResponse,
            )?,
            MSP_EEPROM_WRITE => decode_action_command(
                MSP_EEPROM_WRITE,
                direction,
                MspMessage::EepromWriteRequest,
                MspMessage::EepromWriteResponse,
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
