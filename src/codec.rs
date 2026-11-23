use crate::MspDecodeError;
use crate::MspDirection;
use crate::MspEncodeError;
use crate::MspVersion;
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

    /// Creates a payload instance from a byte slice using the command's wire layout.
    ///
    /// # Errors
    ///
    /// Returns [`MspDecodeError::PayloadTooShort`] if `data` has fewer than
    /// [`Self::MIN_PAYLOAD_SIZE`] bytes or ends partway through a required field.
    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError>;

    /// Writes this payload into the beginning of `buffer`, returning the number of bytes written.
    ///
    /// # Errors
    ///
    /// Returns [`MspEncodeError::BufferTooSmall`] if `buffer` has fewer than
    /// [`Self::payload_size`] bytes.
    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError>;
}

/// Sequential zero-copy reader for decoding little-endian MSP payload fields.
///
/// Similar to Betaflight's `sbuf_t` reader helpers (`sbufReadU8`, `sbufReadU16`, etc.),
/// tracking the cumulative byte offset and producing [`MspDecodeError::PayloadTooShort`]
/// on underflow.
#[derive(Debug, Clone)]
pub(crate) struct PayloadReader<'a> {
    command_id: u16,
    data: &'a [u8],
    pos: usize,
    min_required: usize,
}

impl<'a> PayloadReader<'a> {
    /// Creates a reader that validates `data.len() >= min_required` upfront.
    #[inline]
    pub(crate) fn with_min_size(
        command_id: u16,
        data: &'a [u8],
        min_required: usize,
    ) -> Result<Self, MspDecodeError> {
        if data.len() < min_required {
            return Err(MspDecodeError::PayloadTooShort {
                command_id,
                required: min_required,
                actual: data.len(),
            });
        }
        Ok(Self {
            command_id,
            data,
            pos: 0,
            min_required,
        })
    }

    /// Creates a reader without upfront minimum size check (requirement grows as fields are read).
    #[inline]
    pub(crate) const fn new(command_id: u16, data: &'a [u8]) -> Self {
        Self {
            command_id,
            data,
            pos: 0,
            min_required: 0,
        }
    }

    /// Returns the unread trailing slice of the payload.
    #[cfg(test)]
    #[inline]
    pub(crate) fn remaining(&self) -> &'a [u8] {
        self.data.get(self.pos..).unwrap_or(&[])
    }

    /// Returns the number of unread bytes remaining.
    #[inline]
    pub(crate) const fn remaining_len(&self) -> usize {
        self.data.len().saturating_sub(self.pos)
    }

    /// Borrows `len` bytes from the cursor and advances it.
    #[inline]
    pub(crate) fn read_bytes(&mut self, len: usize) -> Result<&'a [u8], MspDecodeError> {
        let next_pos = self
            .pos
            .checked_add(len)
            .ok_or(MspDecodeError::PayloadTooShort {
                command_id: self.command_id,
                required: usize::MAX,
                actual: self.data.len(),
            })?;
        let required = next_pos.max(self.min_required);
        let slice = self
            .data
            .get(self.pos..next_pos)
            .ok_or(MspDecodeError::PayloadTooShort {
                command_id: self.command_id,
                required,
                actual: self.data.len(),
            })?;
        self.pos = next_pos;
        Ok(slice)
    }

    /// Copies a fixed-size byte array `[u8; N]` from the cursor and advances it.
    #[inline]
    pub(crate) fn read_array<const N: usize>(&mut self) -> Result<[u8; N], MspDecodeError> {
        let slice = self.read_bytes(N)?;
        let required = self.pos.max(self.min_required);
        slice
            .try_into()
            .map_err(|_slice_length_error| MspDecodeError::PayloadTooShort {
                command_id: self.command_id,
                required,
                actual: self.data.len(),
            })
    }

    #[inline]
    pub(crate) fn read_u8(&mut self) -> Result<u8, MspDecodeError> {
        let [b0] = self.read_array::<1>()?;
        Ok(b0)
    }

    /// Reads a single byte and interprets `0` as `false` and any non-zero value as `true`.
    #[inline]
    pub(crate) fn read_bool(&mut self) -> Result<bool, MspDecodeError> {
        Ok(self.read_u8()? != 0)
    }

    #[cfg(test)]
    #[inline]
    pub(crate) fn read_i8(&mut self) -> Result<i8, MspDecodeError> {
        let [b0] = self.read_array::<1>()?;
        Ok(i8::from_le_bytes([b0]))
    }

    #[inline]
    pub(crate) fn read_u16(&mut self) -> Result<u16, MspDecodeError> {
        Ok(u16::from_le_bytes(self.read_array::<2>()?))
    }

    #[inline]
    pub(crate) fn read_i16(&mut self) -> Result<i16, MspDecodeError> {
        Ok(i16::from_le_bytes(self.read_array::<2>()?))
    }

    #[inline]
    pub(crate) fn read_u32(&mut self) -> Result<u32, MspDecodeError> {
        Ok(u32::from_le_bytes(self.read_array::<4>()?))
    }

    #[inline]
    pub(crate) fn read_i32(&mut self) -> Result<i32, MspDecodeError> {
        Ok(i32::from_le_bytes(self.read_array::<4>()?))
    }

    #[cfg(test)]
    #[inline]
    pub(crate) fn read_u64(&mut self) -> Result<u64, MspDecodeError> {
        Ok(u64::from_le_bytes(self.read_array::<8>()?))
    }

    /// Reads `len` bytes from the cursor into a fixed-capacity `heapless::Vec<u8, N, u8>`.
    #[inline]
    pub(crate) fn read_vec<const N: usize>(
        &mut self,
        len: usize,
    ) -> Result<heapless::Vec<u8, N, u8>, MspDecodeError> {
        let bytes = self.read_bytes(len)?;
        heapless::Vec::from_slice(bytes).map_err(|_capacity_error| {
            MspDecodeError::CapacityExceeded {
                command_id: self.command_id,
                capacity: N,
                requested: len,
            }
        })
    }

    /// Reads a 1-byte length prefix (`u8`) followed by that many bytes.
    #[cfg(test)]
    #[inline]
    pub(crate) fn read_len_prefixed_bytes(&mut self) -> Result<&'a [u8], MspDecodeError> {
        let len = usize::from(self.read_u8()?);
        self.read_bytes(len)
    }

    /// Reads a 1-byte length prefix (`u8`) followed by a UTF-8 string slice of that byte length.
    #[cfg(test)]
    #[inline]
    pub(crate) fn read_len_prefixed_str(&mut self) -> Result<&'a str, MspDecodeError> {
        let bytes = self.read_len_prefixed_bytes()?;
        core::str::from_utf8(bytes).map_err(|_utf8_error| MspDecodeError::InvalidUtf8 {
            command_id: self.command_id,
        })
    }

    /// Reads a 1-byte length prefix (`u8`) followed by a UTF-8 string into a `heapless::String<N, u8>`.
    #[cfg(test)]
    #[inline]
    pub(crate) fn read_len_prefixed_string<const N: usize>(
        &mut self,
    ) -> Result<heapless::String<N, u8>, MspDecodeError> {
        let s = self.read_len_prefixed_str()?;
        let mut out = heapless::String::new();
        out.push_str(s)
            .map_err(|_capacity_error| MspDecodeError::CapacityExceeded {
                command_id: self.command_id,
                capacity: N,
                requested: s.len(),
            })?;
        Ok(out)
    }

    /// Consumes all remaining bytes in the payload and validates them as UTF-8.
    #[inline]
    pub(crate) fn read_remaining_str(&mut self) -> Result<&'a str, MspDecodeError> {
        let bytes = self.read_bytes(self.remaining_len())?;
        core::str::from_utf8(bytes).map_err(|_utf8_error| MspDecodeError::InvalidUtf8 {
            command_id: self.command_id,
        })
    }

    /// Consumes all remaining bytes in the payload as a UTF-8 `heapless::String<N, u8>`.
    #[inline]
    pub(crate) fn read_remaining_string<const N: usize>(
        &mut self,
    ) -> Result<heapless::String<N, u8>, MspDecodeError> {
        let s = self.read_remaining_str()?;
        let mut out = heapless::String::new();
        out.push_str(s)
            .map_err(|_capacity_error| MspDecodeError::CapacityExceeded {
                command_id: self.command_id,
                capacity: N,
                requested: s.len(),
            })?;
        Ok(out)
    }
}

/// Sequential writer for encoding little-endian MSP payload fields into a caller buffer.
#[derive(Debug)]
pub(crate) struct PayloadWriter<'a> {
    buffer: &'a mut [u8],
    pos: usize,
    min_required: usize,
}

impl<'a> PayloadWriter<'a> {
    /// Creates a writer that validates `buffer.len() >= min_required` upfront.
    #[inline]
    pub(crate) fn with_min_size(
        buffer: &'a mut [u8],
        min_required: usize,
    ) -> Result<Self, MspEncodeError> {
        let actual = buffer.len();
        if actual < min_required {
            return Err(MspEncodeError::BufferTooSmall {
                required: min_required,
                actual,
            });
        }
        Ok(Self {
            buffer,
            pos: 0,
            min_required,
        })
    }

    /// Creates a writer without upfront minimum size check (requirement grows as fields are written).
    #[cfg(test)]
    #[inline]
    pub(crate) const fn new(buffer: &'a mut [u8]) -> Self {
        Self {
            buffer,
            pos: 0,
            min_required: 0,
        }
    }

    /// Returns the total number of bytes written so far.
    #[inline]
    pub(crate) const fn written(&self) -> usize {
        self.pos
    }

    /// Writes `bytes` into the buffer at the current cursor position and advances it.
    #[inline]
    pub(crate) fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), MspEncodeError> {
        let actual = self.buffer.len();
        let next_pos = self
            .pos
            .checked_add(bytes.len())
            .ok_or(MspEncodeError::BufferTooSmall {
                required: usize::MAX,
                actual,
            })?;
        let required = next_pos.max(self.min_required);
        let dst = self
            .buffer
            .get_mut(self.pos..next_pos)
            .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
        dst.copy_from_slice(bytes);
        self.pos = next_pos;
        Ok(())
    }

    #[inline]
    pub(crate) fn write_u8(&mut self, value: u8) -> Result<(), MspEncodeError> {
        self.write_bytes(&[value])
    }

    /// Writes a boolean as a single byte (`0` for `false`, `1` for `true`).
    #[inline]
    pub(crate) fn write_bool(&mut self, value: bool) -> Result<(), MspEncodeError> {
        self.write_u8(u8::from(value))
    }

    #[cfg(test)]
    #[inline]
    pub(crate) fn write_i8(&mut self, value: i8) -> Result<(), MspEncodeError> {
        self.write_bytes(&value.to_le_bytes())
    }

    #[inline]
    pub(crate) fn write_u16(&mut self, value: u16) -> Result<(), MspEncodeError> {
        self.write_bytes(&value.to_le_bytes())
    }

    #[inline]
    pub(crate) fn write_i16(&mut self, value: i16) -> Result<(), MspEncodeError> {
        self.write_bytes(&value.to_le_bytes())
    }

    #[inline]
    pub(crate) fn write_u32(&mut self, value: u32) -> Result<(), MspEncodeError> {
        self.write_bytes(&value.to_le_bytes())
    }

    #[inline]
    pub(crate) fn write_i32(&mut self, value: i32) -> Result<(), MspEncodeError> {
        self.write_bytes(&value.to_le_bytes())
    }

    #[cfg(test)]
    #[inline]
    pub(crate) fn write_u64(&mut self, value: u64) -> Result<(), MspEncodeError> {
        self.write_bytes(&value.to_le_bytes())
    }

    /// Writes `bytes` prefixed by its 1-byte (`u8`) length.
    #[inline]
    pub(crate) fn write_len_prefixed_bytes(&mut self, bytes: &[u8]) -> Result<(), MspEncodeError> {
        let len = u8::try_from(bytes.len()).map_err(|_length_conversion_error| {
            MspEncodeError::PayloadTooLarge {
                max: usize::from(u8::MAX),
                actual: bytes.len(),
            }
        })?;
        let actual = self.buffer.len();
        let next_pos = self
            .pos
            .checked_add(1)
            .and_then(|n| n.checked_add(bytes.len()))
            .ok_or(MspEncodeError::BufferTooSmall {
                required: usize::MAX,
                actual,
            })?;
        let required = next_pos.max(self.min_required);
        if actual < required {
            return Err(MspEncodeError::BufferTooSmall { required, actual });
        }
        self.write_u8(len)?;
        self.write_bytes(bytes)
    }

    /// Writes a UTF-8 string slice `s` prefixed by its 1-byte (`u8`) byte length.
    #[cfg(test)]
    #[inline]
    pub(crate) fn write_len_prefixed_str(&mut self, s: &str) -> Result<(), MspEncodeError> {
        self.write_len_prefixed_bytes(s.as_bytes())
    }
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
    let too_large = MspEncodeError::PayloadTooLarge {
        max: usize::from(u16::MAX),
        actual: payload_len,
    };
    let payload_len_u16 =
        u16::try_from(payload_len).map_err(|_length_conversion_error| too_large)?;
    let crc_idx = V2_HEADER_LEN.checked_add(payload_len).ok_or(too_large)?;
    let required = crc_idx.checked_add(CRC_LEN).ok_or(too_large)?;
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
    let too_large = MspEncodeError::PayloadTooLarge {
        max: usize::from(u16::MAX),
        actual: payload_len,
    };
    let payload_len_u16 =
        u16::try_from(payload_len).map_err(|_length_conversion_error| too_large)?;
    let is_jumbo = payload_len >= usize::from(V1_JUMBO_SIZE);
    let payload_offset = if is_jumbo {
        V1_JUMBO_HEADER_LEN
    } else {
        V1_HEADER_LEN
    };
    let xor_idx = payload_offset.checked_add(payload_len).ok_or(too_large)?;
    let required = xor_idx.checked_add(V1_CHECKSUM_LEN).ok_or(too_large)?;
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
    const MAX_PAYLOAD: usize = u16::MAX as usize - V2_INNER_OVERHEAD;
    const JUMBO_HEADER_LEN: usize = V2_OVER_V1_HEADER_LEN + V1_JUMBO_LENGTH_LEN;

    let too_large = MspEncodeError::PayloadTooLarge {
        max: MAX_PAYLOAD,
        actual: payload_len,
    };
    let outer_size = payload_len
        .checked_add(V2_INNER_OVERHEAD)
        .ok_or(too_large)?;
    let outer_size_u16 = u16::try_from(outer_size).map_err(|_length_conversion_error| too_large)?;
    let payload_len_u16 =
        u16::try_from(payload_len).map_err(|_length_conversion_error| too_large)?;
    let is_jumbo = outer_size >= usize::from(V1_JUMBO_SIZE);
    let payload_offset = if is_jumbo {
        JUMBO_HEADER_LEN
    } else {
        V2_OVER_V1_HEADER_LEN
    };
    let crc_idx = payload_offset.checked_add(payload_len).ok_or(too_large)?;
    let xor_idx = crc_idx.checked_add(CRC_LEN).ok_or(too_large)?;
    let required = xor_idx.checked_add(V1_CHECKSUM_LEN).ok_or(too_large)?;
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
        *prefix_mut::<JUMBO_HEADER_LEN>(frame_buf)? = [
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn payload_reader_and_writer_roundtrip_primitives() {
        let mut buf = [0u8; 25];
        let mut writer = PayloadWriter::with_min_size(&mut buf, 22).unwrap();
        writer.write_u8(0x12).unwrap();
        writer.write_i8(-42).unwrap();
        writer.write_u16(0x3456).unwrap();
        writer.write_i16(-12345).unwrap();
        writer.write_u32(0x789a_bcde).unwrap();
        writer.write_i32(-123_456_789).unwrap();
        writer.write_u64(0x0123_4567_89ab_cdef).unwrap();
        writer.write_bytes(b"xy").unwrap();
        assert_eq!(writer.written(), 24);

        let mut reader = PayloadReader::with_min_size(108, &buf, 22).unwrap();
        assert_eq!(reader.read_u8().unwrap(), 0x12);
        assert_eq!(reader.read_i8().unwrap(), -42);
        assert_eq!(reader.read_u16().unwrap(), 0x3456);
        assert_eq!(reader.read_i16().unwrap(), -12345);
        assert_eq!(reader.read_u32().unwrap(), 0x789a_bcde);
        assert_eq!(reader.read_i32().unwrap(), -123_456_789);
        assert_eq!(reader.read_u64().unwrap(), 0x0123_4567_89ab_cdef);
        assert_eq!(reader.read_array::<2>().unwrap(), *b"xy");
        assert_eq!(reader.remaining_len(), 1);
        assert_eq!(reader.remaining(), &[0]);
    }

    #[test]
    fn payload_reader_and_writer_report_accurate_bounds_errors() {
        assert_eq!(
            PayloadReader::with_min_size(108, &[1, 2, 3], 6).unwrap_err(),
            MspDecodeError::PayloadTooShort {
                command_id: 108,
                required: 6,
                actual: 3,
            }
        );

        let mut reader = PayloadReader::new(109, &[1, 2, 3]);
        assert_eq!(reader.read_u16(), Ok(0x0201));
        assert_eq!(
            reader.read_u16(),
            Err(MspDecodeError::PayloadTooShort {
                command_id: 109,
                required: 4,
                actual: 3,
            })
        );

        let mut short_buf = [0u8; 3];
        assert_eq!(
            PayloadWriter::with_min_size(&mut short_buf, 6).unwrap_err(),
            MspEncodeError::BufferTooSmall {
                required: 6,
                actual: 3,
            }
        );

        let mut writer = PayloadWriter::new(&mut short_buf);
        assert_eq!(writer.write_u16(0x1234), Ok(()));
        assert_eq!(
            writer.write_u16(0x5678),
            Err(MspEncodeError::BufferTooSmall {
                required: 4,
                actual: 3,
            })
        );
    }

    #[test]
    fn payload_reader_and_writer_handle_bool_and_length_prefixed_fields() {
        let mut buf = [0u8; 32];
        let mut writer = PayloadWriter::new(&mut buf);
        writer.write_bool(false).unwrap();
        writer.write_bool(true).unwrap();
        writer.write_len_prefixed_bytes(&[0xaa, 0xbb]).unwrap();
        writer.write_len_prefixed_str("BTFL").unwrap();
        let written = writer.written();
        assert_eq!(written, 1 + 1 + 3 + 5);

        // Non-zero byte decodes as true
        let mut raw = [0u8; 16];
        raw[..written].copy_from_slice(&buf[..written]);
        raw[1] = 42;
        raw[written..written + 4].copy_from_slice(b"tail");

        let mut reader = PayloadReader::new(20, &raw[..written + 4]);
        assert!(!reader.read_bool().unwrap());
        assert!(reader.read_bool().unwrap());
        assert_eq!(reader.read_len_prefixed_bytes().unwrap(), &[0xaa, 0xbb]);
        let s: heapless::String<4, u8> = reader.read_len_prefixed_string().unwrap();
        assert_eq!(s.as_str(), "BTFL");
        let tail: heapless::String<4, u8> = reader.read_remaining_string().unwrap();
        assert_eq!(tail.as_str(), "tail");
        assert_eq!(reader.remaining_len(), 0);

        // CapacityExceeded on read_len_prefixed_string and read_vec
        let mut reader = PayloadReader::new(21, &[4, b'B', b'T', b'F', b'L']);
        assert_eq!(
            reader.read_len_prefixed_string::<3>(),
            Err(MspDecodeError::CapacityExceeded {
                command_id: 21,
                capacity: 3,
                requested: 4,
            })
        );
        let mut reader = PayloadReader::new(21, &[1, 2, 3, 4]);
        assert_eq!(
            reader.read_vec::<3>(4),
            Err(MspDecodeError::CapacityExceeded {
                command_id: 21,
                capacity: 3,
                requested: 4,
            })
        );

        // InvalidUtf8 on read_len_prefixed_str
        let mut reader = PayloadReader::new(22, &[2, 0xff, 0xfe]);
        assert_eq!(
            reader.read_len_prefixed_str(),
            Err(MspDecodeError::InvalidUtf8 { command_id: 22 })
        );

        // Truncated length-prefixed payload reports PayloadTooShort
        let mut reader = PayloadReader::new(23, &[5, b'a', b'b']);
        assert_eq!(
            reader.read_len_prefixed_bytes(),
            Err(MspDecodeError::PayloadTooShort {
                command_id: 23,
                required: 6,
                actual: 3,
            })
        );

        // BufferTooSmall and PayloadTooLarge on write_len_prefixed_bytes
        let mut small = [0u8; 3];
        let mut writer = PayloadWriter::new(&mut small);
        assert_eq!(
            writer.write_len_prefixed_bytes(b"abcd"),
            Err(MspEncodeError::BufferTooSmall {
                required: 5,
                actual: 3,
            })
        );
        let oversized = [0u8; 256];
        let mut large_buf = [0u8; 300];
        let mut writer = PayloadWriter::new(&mut large_buf);
        assert_eq!(
            writer.write_len_prefixed_bytes(&oversized),
            Err(MspEncodeError::PayloadTooLarge {
                max: 255,
                actual: 256,
            })
        );
    }
}
