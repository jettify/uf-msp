use crate::MSP_ALTITUDE;
use crate::MSP_ATTITUDE;
use crate::MSP_RAW_IMU;
use crate::MspDirection;
use crate::MspFrame;
use crate::MspParser;
use crate::MspParserError;
use crate::MspVersion;

/// Attitude in the integer units transmitted by `MSP_ATTITUDE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspAttitude {
    pub roll_decidegrees: i16,
    pub pitch_decidegrees: i16,
    pub yaw_degrees: i16,
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

/// Altitude and vertical speed transmitted by `MSP_ALTITUDE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MspAltitude {
    pub altitude_cm: i32,
    pub vario_cm_s: i16,
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

fn prefix<const N: usize>(frame: MspFrame<'_>) -> Result<[u8; N], MspDecodeError> {
    frame
        .payload()
        .get(..N)
        .and_then(|bytes| bytes.try_into().ok())
        .ok_or(MspDecodeError::PayloadTooShort {
            command_id: frame.command_id(),
            required: N,
            actual: frame.payload().len(),
        })
}

impl TryFrom<MspFrame<'_>> for MspPacket {
    type Error = MspDecodeError;

    /// Decodes the known response prefix, ignoring extra payload bytes.
    /// Polling request payloads are ignored. The frame is already validated,
    /// including checksums covering all declared payload bytes.
    fn try_from(frame: MspFrame<'_>) -> Result<Self, Self::Error> {
        let command_id = frame.command_id();
        let request = match command_id {
            MSP_ATTITUDE => MspMessage::AttitudeRequest,
            MSP_RAW_IMU => MspMessage::RawImuRequest,
            MSP_ALTITUDE => MspMessage::AltitudeRequest,
            _ => return Err(MspDecodeError::UnsupportedCommand { command_id }),
        };
        let message = match frame.direction() {
            MspDirection::Error => return Err(MspDecodeError::RemoteError { command_id }),
            MspDirection::Request => request,
            MspDirection::Response => match command_id {
                MSP_ATTITUDE => {
                    let [r0, r1, p0, p1, y0, y1] = prefix(frame)?;
                    MspMessage::Attitude(MspAttitude {
                        roll_decidegrees: i16::from_le_bytes([r0, r1]),
                        pitch_decidegrees: i16::from_le_bytes([p0, p1]),
                        yaw_degrees: i16::from_le_bytes([y0, y1]),
                    })
                }
                MSP_RAW_IMU => {
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
                    ] = prefix(frame)?;
                    MspMessage::RawImu(MspRawImu {
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
                MSP_ALTITUDE => {
                    let [a0, a1, a2, a3, v0, v1] = prefix(frame)?;
                    MspMessage::Altitude(MspAltitude {
                        altitude_cm: i32::from_le_bytes([a0, a1, a2, a3]),
                        vario_cm_s: i16::from_le_bytes([v0, v1]),
                    })
                }
                _ => return Err(MspDecodeError::UnsupportedCommand { command_id }),
            },
        };
        Ok(Self {
            version: frame.version(),
            flags: frame.flags(),
            message,
        })
    }
}

/// A framing parser failure or a recoverable packet decoding error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum MspPacketError {
    Parser(MspParserError),
    Decode(MspDecodeError),
}

impl From<MspParserError> for MspPacketError {
    fn from(error: MspParserError) -> Self {
        Self::Parser(error)
    }
}

impl From<MspDecodeError> for MspPacketError {
    fn from(error: MspDecodeError) -> Self {
        Self::Decode(error)
    }
}

impl core::fmt::Display for MspPacketError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Parser(error) => error.fmt(f),
            Self::Decode(error) => error.fmt(f),
        }
    }
}

impl core::error::Error for MspPacketError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        Some(match self {
            Self::Parser(error) => error,
            Self::Decode(error) => error,
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
    /// Decode errors consume one frame and allow parsing to continue. Internal
    /// parser errors require resetting the parser before reuse.
    pub fn next_packet(&mut self, input: &mut &[u8]) -> Result<Option<MspPacket>, MspPacketError> {
        Ok(self.next(input)?.map(MspPacket::try_from).transpose()?)
    }

    /// Iterates over owned packets, advancing the caller's input cursor.
    ///
    /// Decode errors are yielded and iteration continues. An internal parser
    /// error is yielded once and terminates the iterator. Incomplete input ends
    /// this iterator; create another with the next chunk to continue parsing.
    /// Dropping early leaves the cursor at the first unconsumed byte.
    pub fn packets<'a, 'input>(
        &'a mut self,
        input: &'a mut &'input [u8],
    ) -> MspPackets<'a, 'input, N> {
        MspPackets {
            parser: self,
            input,
            finished: false,
        }
    }
}

/// An iterator borrowing a parser and input cursor, yielding owned packets.
#[must_use = "iterators do nothing unless consumed"]
pub struct MspPackets<'a, 'input, const N: usize> {
    parser: &'a mut MspParser<N>,
    input: &'a mut &'input [u8],
    finished: bool,
}

impl<const N: usize> core::fmt::Debug for MspPackets<'_, '_, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("MspPackets")
            .field("parser", &self.parser)
            .field("remaining_len", &self.input.len())
            .field("finished", &self.finished)
            .finish()
    }
}

impl<const N: usize> Iterator for MspPackets<'_, '_, N> {
    type Item = Result<MspPacket, MspPacketError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }
        match self.parser.next_packet(self.input) {
            Ok(Some(packet)) => Some(Ok(packet)),
            Ok(None) => {
                self.finished = true;
                None
            }
            Err(error) => {
                if matches!(error, MspPacketError::Parser(_)) {
                    self.finished = true;
                }
                Some(Err(error))
            }
        }
    }
}

impl<const N: usize> core::iter::FusedIterator for MspPackets<'_, '_, N> {}
