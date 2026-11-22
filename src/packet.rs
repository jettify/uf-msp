use crate::MspDirection;
use crate::MspFrame;
use crate::MspParser;
use crate::MspVersion;
use crate::commands::MSP_ALTITUDE;
use crate::commands::MSP_ATTITUDE;
use crate::commands::MSP_RAW_IMU;

/// Attitude in the integer units transmitted by `MSP_ATTITUDE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspAttitude {
    pub roll_decidegrees: i16,
    pub pitch_decidegrees: i16,
    pub yaw_degrees: i16,
}

impl MspAttitude {
    /// MSP command ID (`MSP_ATTITUDE`).
    pub const COMMAND_ID: u16 = MSP_ATTITUDE;

    /// Minimum response payload length in bytes.
    pub const PAYLOAD_LEN: usize = 6;

    /// Decodes an `MSP_ATTITUDE` response payload prefix, ignoring trailing bytes.
    ///
    /// # Errors
    ///
    /// Returns [`MspDecodeError::PayloadTooShort`] if `payload` has fewer than
    /// [`Self::PAYLOAD_LEN`] bytes.
    pub fn decode(payload: &[u8]) -> Result<Self, MspDecodeError> {
        let [r0, r1, p0, p1, y0, y1] = prefix::<{ Self::PAYLOAD_LEN }>(Self::COMMAND_ID, payload)?;
        Ok(Self {
            roll_decidegrees: i16::from_le_bytes([r0, r1]),
            pitch_decidegrees: i16::from_le_bytes([p0, p1]),
            yaw_degrees: i16::from_le_bytes([y0, y1]),
        })
    }
}

impl TryFrom<&[u8]> for MspAttitude {
    type Error = MspDecodeError;

    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        Self::decode(payload)
    }
}

/// `MSP_RAW_IMU` values in X, Y, Z order.
///
/// Values retain their wire representation. Sensor scaling depends on the
/// flight controller firmware; no universal physical-unit conversion is applied.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspRawImu {
    pub acc: [i16; 3],
    pub gyro: [i16; 3],
    pub mag: [i16; 3],
}

impl MspRawImu {
    /// MSP command ID (`MSP_RAW_IMU`).
    pub const COMMAND_ID: u16 = MSP_RAW_IMU;

    /// Minimum response payload length in bytes.
    pub const PAYLOAD_LEN: usize = 18;

    /// Decodes an `MSP_RAW_IMU` response payload prefix, ignoring trailing bytes.
    ///
    /// # Errors
    ///
    /// Returns [`MspDecodeError::PayloadTooShort`] if `payload` has fewer than
    /// [`Self::PAYLOAD_LEN`] bytes.
    pub fn decode(payload: &[u8]) -> Result<Self, MspDecodeError> {
        let [
            a0,
            a1,
            a2,
            a3,
            a4,
            a5,
            g0,
            g1,
            g2,
            g3,
            g4,
            g5,
            m0,
            m1,
            m2,
            m3,
            m4,
            m5,
        ] = prefix::<{ Self::PAYLOAD_LEN }>(Self::COMMAND_ID, payload)?;
        Ok(Self {
            acc: [
                i16::from_le_bytes([a0, a1]),
                i16::from_le_bytes([a2, a3]),
                i16::from_le_bytes([a4, a5]),
            ],
            gyro: [
                i16::from_le_bytes([g0, g1]),
                i16::from_le_bytes([g2, g3]),
                i16::from_le_bytes([g4, g5]),
            ],
            mag: [
                i16::from_le_bytes([m0, m1]),
                i16::from_le_bytes([m2, m3]),
                i16::from_le_bytes([m4, m5]),
            ],
        })
    }
}

impl TryFrom<&[u8]> for MspRawImu {
    type Error = MspDecodeError;

    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        Self::decode(payload)
    }
}

/// Altitude and vertical speed transmitted by `MSP_ALTITUDE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspAltitude {
    pub altitude_cm: i32,
    pub vario_cm_s: i16,
}

impl MspAltitude {
    /// MSP command ID (`MSP_ALTITUDE`).
    pub const COMMAND_ID: u16 = MSP_ALTITUDE;

    /// Minimum response payload length in bytes.
    pub const PAYLOAD_LEN: usize = 6;

    /// Decodes an `MSP_ALTITUDE` response payload prefix, ignoring trailing bytes.
    ///
    /// # Errors
    ///
    /// Returns [`MspDecodeError::PayloadTooShort`] if `payload` has fewer than
    /// [`Self::PAYLOAD_LEN`] bytes.
    pub fn decode(payload: &[u8]) -> Result<Self, MspDecodeError> {
        let [a0, a1, a2, a3, v0, v1] = prefix::<{ Self::PAYLOAD_LEN }>(Self::COMMAND_ID, payload)?;
        Ok(Self {
            altitude_cm: i32::from_le_bytes([a0, a1, a2, a3]),
            vario_cm_s: i16::from_le_bytes([v0, v1]),
        })
    }
}

impl TryFrom<&[u8]> for MspAltitude {
    type Error = MspDecodeError;

    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        Self::decode(payload)
    }
}

/// Supported polling requests and telemetry responses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
}

/// Why a validated frame could not be decoded as a supported packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum MspDecodeError {
    UnsupportedCommand {
        command_id: u16,
    },
    RemoteError {
        command_id: u16,
    },
    /// A response has fewer than the required prefix bytes.
    PayloadTooShort {
        command_id: u16,
        required: usize,
        actual: usize,
    },
}

impl core::fmt::Display for MspDecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnsupportedCommand { command_id } => {
                write!(f, "unsupported MSP command {command_id}")
            }
            Self::RemoteError { command_id } => {
                write!(f, "remote MSP error for command {command_id}")
            }
            Self::PayloadTooShort {
                command_id,
                required,
                actual,
            } => write!(
                f,
                "MSP command {command_id} payload too short: need {required} bytes, got {actual}"
            ),
        }
    }
}

impl core::error::Error for MspDecodeError {}

fn prefix<const N: usize>(command_id: u16, payload: &[u8]) -> Result<[u8; N], MspDecodeError> {
    payload
        .get(..N)
        .and_then(|bytes| bytes.try_into().ok())
        .ok_or(MspDecodeError::PayloadTooShort {
            command_id,
            required: N,
            actual: payload.len(),
        })
}

fn decode_command<T>(
    command_id: u16,
    direction: MspDirection,
    payload: &[u8],
    request: MspMessage,
    decode: fn(&[u8]) -> Result<T, MspDecodeError>,
    wrap: fn(T) -> MspMessage,
) -> Result<MspMessage, MspDecodeError> {
    match direction {
        MspDirection::Error => Err(MspDecodeError::RemoteError { command_id }),
        MspDirection::Request => Ok(request),
        MspDirection::Response => decode(payload).map(wrap),
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
                command_id,
                direction,
                payload,
                MspMessage::AttitudeRequest,
                MspAttitude::decode,
                MspMessage::Attitude,
            )?,
            MSP_RAW_IMU => decode_command(
                command_id,
                direction,
                payload,
                MspMessage::RawImuRequest,
                MspRawImu::decode,
                MspMessage::RawImu,
            )?,
            MSP_ALTITUDE => decode_command(
                command_id,
                direction,
                payload,
                MspMessage::AltitudeRequest,
                MspAltitude::decode,
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

impl<const N: usize> Iterator for MspPackets<'_, '_, N> {
    type Item = Result<MspPacket, MspDecodeError>;

    fn next(&mut self) -> Option<Self::Item> {
        self.parser.next_packet(self.input).transpose()
    }
}

impl<const N: usize> core::iter::FusedIterator for MspPackets<'_, '_, N> {}
