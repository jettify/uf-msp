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

/// Decodes a matched MSP command by dispatching on frame direction.
fn decode_command(
    command_id: u16,
    direction: MspDirection,
    decode_request: impl FnOnce() -> Result<MspMessage, MspDecodeError>,
    decode_response: impl FnOnce() -> Result<MspMessage, MspDecodeError>,
) -> Result<MspMessage, MspDecodeError> {
    match direction {
        MspDirection::Error => Err(MspDecodeError::RemoteError { command_id }),
        MspDirection::Request => decode_request(),
        MspDirection::Response => decode_response(),
    }
}

// Generates `TryFrom<&[u8]>` delegating to `MspPayload::from_bytes`.
macro_rules! impl_try_from_bytes {
    ($( $ty:ty ),* $(,)?) => {
        $(
            impl TryFrom<&[u8]> for $ty {
                type Error = MspDecodeError;

                fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
                    <$ty as MspPayload>::from_bytes(payload)
                }
            }
        )*
    };
}

// Each row declares a request and response. A variant without a type has an empty payload.
macro_rules! define_msp_messages {
    (
        $(
            $command:ident => (
                $request:ident $(($request_payload:ty))?,
                $response:ident $(($response_payload:ty))?
            )
        ),+ $(,)?
    ) => {
        const _: () = {
            $(
                $(assert!(<$request_payload as MspPayload>::COMMAND_ID == $command);)?
                $(assert!(<$response_payload as MspPayload>::COMMAND_ID == $command);)?
            )+
        };

        $(
            $(impl_try_from_bytes!($request_payload);)?
            $(impl_try_from_bytes!($response_payload);)?
        )+

        /// Supported MSP requests, responses, and setter acknowledgments.
        #[derive(Debug, Clone, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[non_exhaustive]
        pub enum MspMessage {
            $(
                $request $(($request_payload))?,
                $response $(($response_payload))?,
            )+
        }

        impl MspMessage {
            /// Returns the MSP command ID implied by this message.
            #[must_use]
            pub const fn command_id(&self) -> u16 {
                match self {
                    $(
                        define_msp_messages!(@pattern $request $(($request_payload))?, _)
                        | define_msp_messages!(@pattern $response $(($response_payload))?, _) => $command,
                    )+
                }
            }

            /// Returns the request or response direction implied by this message.
            #[must_use]
            pub const fn direction(&self) -> MspDirection {
                match self {
                    $(
                        define_msp_messages!(@pattern $request $(($request_payload))?, _) => {
                            MspDirection::Request
                        }
                        define_msp_messages!(@pattern $response $(($response_payload))?, _) => {
                            MspDirection::Response
                        }
                    )+
                }
            }

            fn encode_frame(
                &self,
                buffer: &mut [u8],
                version: MspVersion,
                flags: u8,
            ) -> Result<usize, MspEncodeError> {
                let direction = self.direction();
                match self {
                    $(
                        define_msp_messages!(@pattern $request $(($request_payload))?, payload) => {
                            define_msp_messages!(
                                @encode buffer, version, direction, flags, $command
                                $(, payload: $request_payload)?
                            )
                        }
                        define_msp_messages!(@pattern $response $(($response_payload))?, payload) => {
                            define_msp_messages!(
                                @encode buffer, version, direction, flags, $command
                                $(, payload: $response_payload)?
                            )
                        }
                    )+
                }
            }

            #[deny(unreachable_patterns)]
            fn decode_payload(
                command_id: u16,
                direction: MspDirection,
                payload: &[u8],
            ) -> Result<Self, MspDecodeError> {
                match command_id {
                    $(
                        $command => decode_command(
                            command_id,
                            direction,
                            || define_msp_messages!(@decode payload, $request $(($request_payload))?),
                            || define_msp_messages!(@decode payload, $response $(($response_payload))?),
                        ),
                    )+
                    _ => Err(MspDecodeError::UnsupportedCommand { command_id }),
                }
            }
        }
    };

    (@pattern $variant:ident, $binding:pat) => {
        Self::$variant
    };
    (@pattern $variant:ident ($ty:ty), $binding:pat) => {
        Self::$variant($binding)
    };

    (@encode $buffer:ident, $version:ident, $direction:ident, $flags:ident, $command:ident) => {
        write_raw_frame_to_buffer($buffer, $version, $direction, $flags, $command, &[])
    };
    (@encode $buffer:ident, $version:ident, $direction:ident, $flags:ident, $command:ident,
        $payload:ident: $ty:ty
    ) => {
        write_packet_to_buffer($buffer, $version, $direction, $flags, $payload)
    };

    (@decode $payload:ident, $variant:ident) => {
        Ok(Self::$variant)
    };
    (@decode $payload:ident, $variant:ident ($ty:ty)) => {
        <$ty as MspPayload>::from_bytes($payload).map(Self::$variant)
    };
}

define_msp_messages! {
    MSP_API_VERSION     => (ApiVersionRequest, ApiVersion(MspApiVersion)),
    MSP_FC_VARIANT      => (FcVariantRequest, FcVariant(MspFcVariant)),
    MSP_FC_VERSION      => (FcVersionRequest, FcVersion(MspFcVersion)),
    MSP_STATUS          => (StatusRequest, Status(MspStatus)),
    MSP_NAME            => (NameRequest, Name(MspName)),
    MSP_RAW_IMU         => (RawImuRequest, RawImu(MspRawImu)),
    MSP_ATTITUDE        => (AttitudeRequest, Attitude(MspAttitude)),
    MSP_ALTITUDE        => (AltitudeRequest, Altitude(MspAltitude)),
    MSP_RC              => (RcRequest, Rc(MspRc)),
    MSP_SET_NAME        => (SetName(MspSetName), SetNameResponse),
    MSP_SET_RAW_RC      => (SetRawRc(MspSetRawRc), SetRawRcResponse),
    MSP_REBOOT          => (RebootRequest(MspRebootRequest), Reboot(MspRebootResponse)),
    MSP_ACC_CALIBRATION => (AccCalibrationRequest, AccCalibrationResponse),
    MSP_EEPROM_WRITE    => (EepromWriteRequest, EepromWriteResponse),
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

    /// Validates a standalone wire frame slice and decodes it into an owned packet.
    ///
    /// # Errors
    ///
    /// Returns [`MspError::Frame`](crate::MspError::Frame) if `bytes` is not a complete,
    /// valid MSP frame, or [`MspError::Decode`](crate::MspError::Decode) if the command
    /// or payload cannot be decoded.
    pub fn from_frame_bytes(bytes: &[u8]) -> crate::Result<Self> {
        let frame = MspFrame::try_from(bytes)?;
        Ok(Self::try_from(frame)?)
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
        self.message.encode_frame(buffer, self.version, self.flags)
    }
}

impl TryFrom<MspFrame<'_>> for MspPacket {
    type Error = MspDecodeError;

    /// Decodes the known command payload, ignoring extra trailing bytes on fixed-prefix
    /// responses. The frame is already validated, including checksums covering all
    /// declared payload bytes.
    fn try_from(frame: MspFrame<'_>) -> Result<Self, Self::Error> {
        let message =
            MspMessage::decode_payload(frame.command_id(), frame.direction(), frame.payload())?;
        Ok(Self {
            version: frame.version(),
            flags: frame.flags(),
            message,
        })
    }
}

impl TryFrom<&[u8]> for MspPacket {
    type Error = crate::MspError;

    fn try_from(bytes: &[u8]) -> crate::Result<Self> {
        Self::from_frame_bytes(bytes)
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
