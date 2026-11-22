use crate::MspDirection;
use crate::MspFrame;
use crate::MspParser;
use crate::MspVersion;
use crate::commands::MSP_ALTITUDE;
use crate::commands::MSP_ATTITUDE;
use crate::commands::MSP_RAW_IMU;
use crate::constants::CRC_LEN;
use crate::constants::PREAMBLE_LEN;
use crate::constants::V1_CHECKSUM_LEN;
use crate::constants::V1_ENCAPSULATED_CMD;
use crate::constants::V1_HEADER_LEN;
use crate::constants::V1_JUMBO_HEADER_LEN;
use crate::constants::V1_JUMBO_LENGTH_LEN;
use crate::constants::V1_JUMBO_SIZE;
use crate::constants::V2_HEADER_LEN;
use crate::constants::V2_INNER_OVERHEAD;
use crate::constants::V2_MIN_FRAME_LEN;
use crate::constants::V2_OVER_V1_HEADER_LEN;
use crate::crc8_dvbs2_slice;

/// A typed MSP command payload that can be decoded from and encoded into byte slices.
pub trait MspPayload: Sized {
    /// MSP command ID for this payload.
    const COMMAND_ID: u16;

    /// Minimum expected length of the payload in bytes.
    const MIN_PAYLOAD_SIZE: usize;

    /// Exact number of payload bytes [`Self::to_bytes`] will write.
    #[must_use]
    fn payload_size(&self) -> usize {
        Self::MIN_PAYLOAD_SIZE
    }

    /// Creates a payload instance from a byte slice, ignoring trailing extension bytes.
    ///
    /// # Errors
    ///
    /// Returns [`MspDecodeError::PayloadTooShort`] if `data` has fewer than
    /// [`Self::MIN_PAYLOAD_SIZE`] bytes.
    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError>;

    /// Writes this payload into the beginning of `buffer`, returning the number of bytes written.
    ///
    /// # Errors
    ///
    /// Returns [`MspEncodeError::BufferTooSmall`] if `buffer` has fewer than
    /// [`Self::payload_size`] bytes.
    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError>;
}

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
        let [r0, r1, p0, p1, y0, y1] =
            prefix::<{ Self::MIN_PAYLOAD_SIZE }>(Self::COMMAND_ID, data)?;
        Ok(Self {
            roll_decidegrees: i16::from_le_bytes([r0, r1]),
            pitch_decidegrees: i16::from_le_bytes([p0, p1]),
            yaw_degrees: i16::from_le_bytes([y0, y1]),
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let [r0, r1] = self.roll_decidegrees.to_le_bytes();
        let [p0, p1] = self.pitch_decidegrees.to_le_bytes();
        let [y0, y1] = self.yaw_degrees.to_le_bytes();
        *prefix_mut::<{ Self::MIN_PAYLOAD_SIZE }>(buffer)? = [r0, r1, p0, p1, y0, y1];
        Ok(Self::MIN_PAYLOAD_SIZE)
    }
}

impl TryFrom<&[u8]> for MspAttitude {
    type Error = MspDecodeError;

    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        Self::from_bytes(payload)
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
        ] = prefix::<{ Self::MIN_PAYLOAD_SIZE }>(Self::COMMAND_ID, data)?;
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

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let [ax, ay, az] = self.acc;
        let [gx, gy, gz] = self.gyro;
        let [mx, my, mz] = self.mag;
        let [a0, a1] = ax.to_le_bytes();
        let [a2, a3] = ay.to_le_bytes();
        let [a4, a5] = az.to_le_bytes();
        let [g0, g1] = gx.to_le_bytes();
        let [g2, g3] = gy.to_le_bytes();
        let [g4, g5] = gz.to_le_bytes();
        let [m0, m1] = mx.to_le_bytes();
        let [m2, m3] = my.to_le_bytes();
        let [m4, m5] = mz.to_le_bytes();
        *prefix_mut::<{ Self::MIN_PAYLOAD_SIZE }>(buffer)? = [
            a0, a1, a2, a3, a4, a5, g0, g1, g2, g3, g4, g5, m0, m1, m2, m3, m4, m5,
        ];
        Ok(Self::MIN_PAYLOAD_SIZE)
    }
}

impl TryFrom<&[u8]> for MspRawImu {
    type Error = MspDecodeError;

    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        Self::from_bytes(payload)
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
        let [a0, a1, a2, a3, v0, v1] =
            prefix::<{ Self::MIN_PAYLOAD_SIZE }>(Self::COMMAND_ID, data)?;
        Ok(Self {
            altitude_cm: i32::from_le_bytes([a0, a1, a2, a3]),
            vario_cm_s: i16::from_le_bytes([v0, v1]),
        })
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let [a0, a1, a2, a3] = self.altitude_cm.to_le_bytes();
        let [v0, v1] = self.vario_cm_s.to_le_bytes();
        *prefix_mut::<{ Self::MIN_PAYLOAD_SIZE }>(buffer)? = [a0, a1, a2, a3, v0, v1];
        Ok(Self::MIN_PAYLOAD_SIZE)
    }
}

impl TryFrom<&[u8]> for MspAltitude {
    type Error = MspDecodeError;

    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        Self::from_bytes(payload)
    }
}

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

/// Why a validated frame could not be decoded as a supported packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

/// Why a payload or frame could not be serialized into a caller buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum MspEncodeError {
    /// The destination buffer cannot hold the serialized payload or frame.
    BufferTooSmall {
        required: usize,
        actual: usize,
    },
    /// MSP v1 only supports command IDs `0..=254` (`255` is reserved for v2 encapsulation).
    InvalidV1Command {
        command_id: u16,
    },
    /// The payload exceeds the maximum size representable by the chosen wire format.
    PayloadTooLarge {
        max: usize,
        actual: usize,
    },
    PayloadSizeMismatch {
        expected: usize,
        written: usize,
    },
}

impl core::fmt::Display for MspEncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BufferTooSmall { required, actual } => {
                write!(
                    f,
                    "MSP buffer too small: need {required} bytes, got {actual}"
                )
            }
            Self::InvalidV1Command { command_id } => {
                write!(
                    f,
                    "MSP command {command_id} cannot be encoded as raw MSP v1 (must be < 255)"
                )
            }
            Self::PayloadTooLarge { max, actual } => {
                write!(
                    f,
                    "MSP payload too large: maximum {max} bytes, got {actual}"
                )
            }
            Self::PayloadSizeMismatch { expected, written } => {
                write!(
                    f,
                    "MSP payload size mismatch: expected {expected} bytes, wrote {written}"
                )
            }
        }
    }
}

impl core::error::Error for MspEncodeError {}

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

fn prefix_mut<const N: usize>(buffer: &mut [u8]) -> Result<&mut [u8; N], MspEncodeError> {
    let actual = buffer.len();
    buffer
        .get_mut(..N)
        .and_then(|bytes| bytes.try_into().ok())
        .ok_or(MspEncodeError::BufferTooSmall {
            required: N,
            actual,
        })
}

fn encode_v2_frame(
    buffer: &mut [u8],
    direction: MspDirection,
    flags: u8,
    command_id: u16,
    payload_len: usize,
    write_payload: impl FnOnce(&mut [u8]) -> Result<usize, MspEncodeError>,
) -> Result<usize, MspEncodeError> {
    let max_payload = usize::from(u16::MAX);
    let Ok(payload_len_u16) = u16::try_from(payload_len) else {
        return Err(MspEncodeError::PayloadTooLarge {
            max: max_payload,
            actual: payload_len,
        });
    };
    let required =
        payload_len
            .checked_add(V2_MIN_FRAME_LEN)
            .ok_or(MspEncodeError::PayloadTooLarge {
                max: max_payload,
                actual: payload_len,
            })?;
    let actual = buffer.len();
    let frame_buf = buffer
        .get_mut(..required)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;

    let [cmd_lo, cmd_hi] = command_id.to_le_bytes();
    let [len_lo, len_hi] = payload_len_u16.to_le_bytes();
    *prefix_mut::<V2_HEADER_LEN>(frame_buf)? = [
        b'$',
        b'X',
        direction.to_byte(),
        flags,
        cmd_lo,
        cmd_hi,
        len_lo,
        len_hi,
    ];

    let crc_idx = required - CRC_LEN;
    let payload_dst = frame_buf
        .get_mut(V2_HEADER_LEN..crc_idx)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    write_payload(payload_dst)?;

    let crc_slice = frame_buf
        .get(PREAMBLE_LEN..crc_idx)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    let crc = crc8_dvbs2_slice(0, crc_slice);
    let crc_slot = frame_buf
        .get_mut(crc_idx)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    *crc_slot = crc;
    Ok(required)
}

fn encode_v1_frame(
    buffer: &mut [u8],
    direction: MspDirection,
    command_id: u16,
    payload_len: usize,
    write_payload: impl FnOnce(&mut [u8]) -> Result<usize, MspEncodeError>,
) -> Result<usize, MspEncodeError> {
    let Ok(command_byte) = u8::try_from(command_id) else {
        return Err(MspEncodeError::InvalidV1Command { command_id });
    };
    if command_byte == V1_ENCAPSULATED_CMD {
        return Err(MspEncodeError::InvalidV1Command { command_id });
    }
    let max_payload = usize::from(u16::MAX);
    let Ok(payload_len_u16) = u16::try_from(payload_len) else {
        return Err(MspEncodeError::PayloadTooLarge {
            max: max_payload,
            actual: payload_len,
        });
    };
    let is_jumbo = payload_len >= usize::from(V1_JUMBO_SIZE);
    let payload_offset = if is_jumbo {
        V1_JUMBO_HEADER_LEN
    } else {
        V1_HEADER_LEN
    };
    let required = payload_offset
        .checked_add(payload_len)
        .and_then(|n| n.checked_add(V1_CHECKSUM_LEN))
        .ok_or(MspEncodeError::PayloadTooLarge {
            max: max_payload,
            actual: payload_len,
        })?;
    let actual = buffer.len();
    let frame_buf = buffer
        .get_mut(..required)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;

    if let Ok(size_byte) = u8::try_from(payload_len)
        && !is_jumbo
    {
        *prefix_mut::<V1_HEADER_LEN>(frame_buf)? =
            [b'$', b'M', direction.to_byte(), size_byte, command_byte];
    } else {
        let [size_lo, size_hi] = payload_len_u16.to_le_bytes();
        *prefix_mut::<V1_JUMBO_HEADER_LEN>(frame_buf)? = [
            b'$',
            b'M',
            direction.to_byte(),
            V1_JUMBO_SIZE,
            command_byte,
            size_lo,
            size_hi,
        ];
    }

    let xor_idx = required - V1_CHECKSUM_LEN;
    let payload_dst = frame_buf
        .get_mut(payload_offset..xor_idx)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    write_payload(payload_dst)?;

    let xor_slice = frame_buf
        .get(PREAMBLE_LEN..xor_idx)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    let xor = xor_slice.iter().fold(0, |acc, &b| acc ^ b);
    let xor_slot = frame_buf
        .get_mut(xor_idx)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    *xor_slot = xor;
    Ok(required)
}

fn encode_v2_over_v1_frame(
    buffer: &mut [u8],
    direction: MspDirection,
    flags: u8,
    command_id: u16,
    payload_len: usize,
    write_payload: impl FnOnce(&mut [u8]) -> Result<usize, MspEncodeError>,
) -> Result<usize, MspEncodeError> {
    let max_payload = usize::from(u16::MAX) - V2_INNER_OVERHEAD;
    let Some(outer_size) = payload_len
        .checked_add(V2_INNER_OVERHEAD)
        .filter(|_| payload_len <= max_payload)
    else {
        return Err(MspEncodeError::PayloadTooLarge {
            max: max_payload,
            actual: payload_len,
        });
    };
    let (Ok(outer_size_u16), Ok(payload_len_u16)) =
        (u16::try_from(outer_size), u16::try_from(payload_len))
    else {
        return Err(MspEncodeError::PayloadTooLarge {
            max: max_payload,
            actual: payload_len,
        });
    };
    let is_jumbo = outer_size >= usize::from(V1_JUMBO_SIZE);
    let payload_offset = V2_OVER_V1_HEADER_LEN + if is_jumbo { V1_JUMBO_LENGTH_LEN } else { 0 };
    let required = payload_offset
        .checked_add(payload_len)
        .and_then(|n| n.checked_add(CRC_LEN + V1_CHECKSUM_LEN))
        .ok_or(MspEncodeError::PayloadTooLarge {
            max: max_payload,
            actual: payload_len,
        })?;
    let actual = buffer.len();
    let frame_buf = buffer
        .get_mut(..required)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;

    let [cmd_lo, cmd_hi] = command_id.to_le_bytes();
    let [len_lo, len_hi] = payload_len_u16.to_le_bytes();
    if let Ok(size_byte) = u8::try_from(outer_size)
        && !is_jumbo
    {
        *prefix_mut::<V2_OVER_V1_HEADER_LEN>(frame_buf)? = [
            b'$',
            b'M',
            direction.to_byte(),
            size_byte,
            V1_ENCAPSULATED_CMD,
            flags,
            cmd_lo,
            cmd_hi,
            len_lo,
            len_hi,
        ];
    } else {
        let [outer_lo, outer_hi] = outer_size_u16.to_le_bytes();
        *prefix_mut::<{ V2_OVER_V1_HEADER_LEN + V1_JUMBO_LENGTH_LEN }>(frame_buf)? = [
            b'$',
            b'M',
            direction.to_byte(),
            V1_JUMBO_SIZE,
            V1_ENCAPSULATED_CMD,
            flags,
            cmd_lo,
            cmd_hi,
            len_lo,
            len_hi,
            outer_lo,
            outer_hi,
        ];
    }

    let xor_idx = required - V1_CHECKSUM_LEN;
    let crc_idx = xor_idx - CRC_LEN;
    let payload_dst = frame_buf
        .get_mut(payload_offset..crc_idx)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    write_payload(payload_dst)?;

    let inner_header = frame_buf
        .get(V1_HEADER_LEN..V2_OVER_V1_HEADER_LEN)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    let payload_slice = frame_buf
        .get(payload_offset..crc_idx)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    let inner_crc = crc8_dvbs2_slice(crc8_dvbs2_slice(0, inner_header), payload_slice);
    let crc_slot = frame_buf
        .get_mut(crc_idx)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    *crc_slot = inner_crc;

    let xor_slice = frame_buf
        .get(PREAMBLE_LEN..xor_idx)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    let xor = xor_slice.iter().fold(0, |acc, &b| acc ^ b);
    let xor_slot = frame_buf
        .get_mut(xor_idx)
        .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
    *xor_slot = xor;
    Ok(required)
}

fn encode_frame_with(
    buffer: &mut [u8],
    version: MspVersion,
    direction: MspDirection,
    flags: u8,
    command_id: u16,
    payload_len: usize,
    write_payload: impl FnOnce(&mut [u8]) -> Result<usize, MspEncodeError>,
) -> Result<usize, MspEncodeError> {
    match version {
        MspVersion::V1 => {
            encode_v1_frame(buffer, direction, command_id, payload_len, write_payload)
        }
        MspVersion::V2 => encode_v2_frame(
            buffer,
            direction,
            flags,
            command_id,
            payload_len,
            write_payload,
        ),
        MspVersion::V2OverV1 => encode_v2_over_v1_frame(
            buffer,
            direction,
            flags,
            command_id,
            payload_len,
            write_payload,
        ),
    }
}

/// Serializes a typed [`MspPayload`] into `buffer` as a complete MSP wire frame,
/// returning the total frame length in bytes.
///
/// The payload is encoded directly in-place into `buffer` without intermediate buffers.
///
/// # Errors
///
/// Returns [`MspEncodeError::BufferTooSmall`] if `buffer` cannot hold the complete frame,
/// [`MspEncodeError::InvalidV1Command`] if `version` is [`MspVersion::V1`] and `P::COMMAND_ID >= 255`,
/// or [`MspEncodeError::PayloadTooLarge`] if `payload.payload_size()` exceeds the wire limit.
pub fn write_packet_to_buffer<P: MspPayload>(
    buffer: &mut [u8],
    version: MspVersion,
    direction: MspDirection,
    flags: u8,
    payload: &P,
) -> Result<usize, MspEncodeError> {
    let expected = payload.payload_size();
    encode_frame_with(
        buffer,
        version,
        direction,
        flags,
        P::COMMAND_ID,
        expected,
        |dst| {
            let written = payload.to_bytes(dst)?;
            if written != expected {
                return Err(MspEncodeError::PayloadSizeMismatch { expected, written });
            }
            Ok(written)
        },
    )
}

/// Serializes a raw byte slice `payload` into `buffer` as a complete MSP wire frame,
/// returning the total frame length in bytes.
///
/// Useful for encoding empty acknowledgments (`>`), error replies (`!`), or custom
/// commands without defining an [`MspPayload`] type.
///
/// # Errors
///
/// Returns [`MspEncodeError::BufferTooSmall`] if `buffer` cannot hold the complete frame,
/// [`MspEncodeError::InvalidV1Command`] if `version` is [`MspVersion::V1`] and `command_id >= 255`,
/// or [`MspEncodeError::PayloadTooLarge`] if `payload.len()` exceeds the wire limit.
pub fn write_raw_frame_to_buffer(
    buffer: &mut [u8],
    version: MspVersion,
    direction: MspDirection,
    flags: u8,
    command_id: u16,
    payload: &[u8],
) -> Result<usize, MspEncodeError> {
    encode_frame_with(
        buffer,
        version,
        direction,
        flags,
        command_id,
        payload.len(),
        |dst| {
            dst.copy_from_slice(payload);
            Ok(payload.len())
        },
    )
}

/// Serializes an empty MSP polling request (`<`) into `buffer`, returning the
/// total frame length in bytes.
///
/// # Errors
///
/// Returns [`MspEncodeError::BufferTooSmall`] if `buffer` cannot hold the frame, or
/// [`MspEncodeError::InvalidV1Command`] if `version` is [`MspVersion::V1`] and `command_id >= 255`.
pub fn write_request_to_buffer(
    buffer: &mut [u8],
    version: MspVersion,
    flags: u8,
    command_id: u16,
) -> Result<usize, MspEncodeError> {
    write_raw_frame_to_buffer(
        buffer,
        version,
        MspDirection::Request,
        flags,
        command_id,
        &[],
    )
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
