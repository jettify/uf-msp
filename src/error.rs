/// Error returned when a variable-length MSP field exceeds its capacity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CapacityError {
    pub capacity: usize,
    pub requested: usize,
}

impl CapacityError {
    /// Creates a new capacity error for a container of `capacity` when `requested` items/bytes were given.
    #[must_use]
    pub const fn new(capacity: usize, requested: usize) -> Self {
        Self {
            capacity,
            requested,
        }
    }
}

impl core::fmt::Display for CapacityError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "MSP fixed container capacity exceeded: capacity {}, requested {}",
            self.capacity, self.requested
        )
    }
}

impl core::error::Error for CapacityError {}

/// Why a standalone byte slice could not be validated as an MSP frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum MspFrameError {
    /// More bytes are needed; `required` is the next validation threshold.
    Incomplete { required: usize, actual: usize },
    /// The frame does not begin with `$M` or `$X`.
    InvalidPreamble,
    /// The direction byte is neither `<`, `>`, nor `!`.
    InvalidDirection,
    /// The outer payload size differs from the inner header plus payload and CRC.
    /// `expected` is the minimum overhead when the outer size cannot hold an inner header.
    InvalidEncapsulationLength { expected: usize, actual: usize },
    /// The complete frame length cannot be represented on this target.
    LengthOverflow,
    /// A complete frame is followed by additional bytes.
    LengthMismatch { expected: usize, actual: usize },
    /// The wire checksum differs from the calculated checksum.
    ChecksumMismatch { expected: u8, actual: u8 },
}

impl MspFrameError {
    /// Returns `true` if the byte slice is an incomplete prefix requiring more bytes.
    #[must_use]
    pub const fn is_incomplete(&self) -> bool {
        matches!(self, Self::Incomplete { .. })
    }
}

impl core::fmt::Display for MspFrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Incomplete { required, actual } => {
                write!(
                    f,
                    "incomplete MSP frame: need {required} bytes, got {actual}"
                )
            }
            Self::InvalidPreamble => f.write_str("invalid MSP preamble"),
            Self::InvalidDirection => f.write_str("invalid MSP direction"),
            Self::InvalidEncapsulationLength { expected, actual } => write!(
                f,
                "invalid MSP encapsulation length: expected {expected} bytes, got {actual}"
            ),
            Self::LengthOverflow => f.write_str("MSP frame length exceeds target address space"),
            Self::LengthMismatch { expected, actual } => {
                write!(
                    f,
                    "MSP frame length mismatch: expected {expected} bytes, got {actual}"
                )
            }
            Self::ChecksumMismatch { expected, actual } => {
                write!(
                    f,
                    "MSP checksum mismatch: expected {expected:#04x}, got {actual:#04x}"
                )
            }
        }
    }
}

impl core::error::Error for MspFrameError {}

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
    /// A payload has fewer than the required prefix bytes.
    PayloadTooShort {
        command_id: u16,
        required: usize,
        actual: usize,
    },
    /// A text field in the payload is not valid UTF-8.
    InvalidUtf8 {
        command_id: u16,
    },
    /// A variable-length field in the payload exceeds the fixed container capacity.
    CapacityExceeded {
        command_id: u16,
        capacity: usize,
        requested: usize,
    },
    /// A payload meets the minimum size, but contains malformed inner fields
    /// (such as an unaligned element byte count or an unknown discriminant).
    InvalidPayload {
        command_id: u16,
    },
}

impl MspDecodeError {
    /// Returns the MSP command ID associated with this decode error.
    #[must_use]
    pub const fn command_id(&self) -> u16 {
        match *self {
            Self::UnsupportedCommand { command_id }
            | Self::RemoteError { command_id }
            | Self::PayloadTooShort { command_id, .. }
            | Self::InvalidUtf8 { command_id }
            | Self::CapacityExceeded { command_id, .. }
            | Self::InvalidPayload { command_id } => command_id,
        }
    }

    /// Returns `true` if the command ID is not supported by [`crate::MspPacket`].
    #[must_use]
    pub const fn is_unsupported(&self) -> bool {
        matches!(self, Self::UnsupportedCommand { .. })
    }

    /// Returns `true` if the frame had the error direction (`!`).
    #[must_use]
    pub const fn is_remote_error(&self) -> bool {
        matches!(self, Self::RemoteError { .. })
    }

    /// Returns the underlying [`CapacityError`] if this error was caused by exceeding fixed capacity.
    #[must_use]
    pub const fn capacity_error(&self) -> Option<CapacityError> {
        match *self {
            Self::CapacityExceeded {
                capacity,
                requested,
                ..
            } => Some(CapacityError {
                capacity,
                requested,
            }),
            _ => None,
        }
    }
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
            Self::InvalidUtf8 { command_id } => {
                write!(f, "invalid UTF-8 in MSP payload for command {command_id}")
            }
            Self::CapacityExceeded {
                command_id,
                capacity,
                requested,
            } => write!(
                f,
                "MSP command {command_id} payload exceeds capacity: capacity {capacity}, requested {requested}"
            ),
            Self::InvalidPayload { command_id } => {
                write!(f, "invalid MSP payload for command {command_id}")
            }
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

/// Unified error type encompassing frame validation, payload decoding, frame encoding,
/// and fixed-capacity container errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum MspError {
    /// Wire frame validation failed.
    Frame(MspFrameError),
    /// Command or payload decoding failed.
    Decode(MspDecodeError),
    /// Payload or frame serialization failed.
    Encode(MspEncodeError),
    /// A variable-length field exceeded its fixed container capacity.
    Capacity(CapacityError),
}

impl core::fmt::Display for MspError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Frame(err) => err.fmt(f),
            Self::Decode(err) => err.fmt(f),
            Self::Encode(err) => err.fmt(f),
            Self::Capacity(err) => err.fmt(f),
        }
    }
}

impl core::error::Error for MspError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Frame(err) => Some(err),
            Self::Decode(err) => Some(err),
            Self::Encode(err) => Some(err),
            Self::Capacity(err) => Some(err),
        }
    }
}

impl From<MspFrameError> for MspError {
    fn from(err: MspFrameError) -> Self {
        Self::Frame(err)
    }
}

impl From<MspDecodeError> for MspError {
    fn from(err: MspDecodeError) -> Self {
        Self::Decode(err)
    }
}

impl From<MspEncodeError> for MspError {
    fn from(err: MspEncodeError) -> Self {
        Self::Encode(err)
    }
}

impl From<CapacityError> for MspError {
    fn from(err: CapacityError) -> Self {
        Self::Capacity(err)
    }
}

/// Crate-wide result type alias defaulting to [`MspError`].
pub type Result<T, E = MspError> = core::result::Result<T, E>;
