use uf_msp::MspAltitude;
use uf_msp::MspAttitude;
use uf_msp::MspDecodeError;
use uf_msp::MspDirection;
use uf_msp::MspEncodeError;
use uf_msp::MspFrame;
use uf_msp::MspMessage;
use uf_msp::MspPacket;
use uf_msp::MspParser;
use uf_msp::MspPayload;
use uf_msp::MspRawImu;
use uf_msp::MspVersion;
use uf_msp::commands::MSP_ATTITUDE;
use uf_msp::crc8_dvbs2_slice;
use uf_msp::write_packet_to_buffer;
use uf_msp::write_raw_frame_to_buffer;
use uf_msp::write_request_to_buffer;

const VERSIONS: [MspVersion; 3] = [MspVersion::V1, MspVersion::V2, MspVersion::V2OverV1];

fn wire(version: MspVersion, direction: u8, command: u16, payload: &[u8]) -> Vec<u8> {
    let mut inner = vec![0xa5];
    inner.extend_from_slice(&command.to_le_bytes());
    inner.extend_from_slice(&u16::try_from(payload.len()).unwrap().to_le_bytes());
    inner.extend_from_slice(payload);
    inner.push(crc8_dvbs2_slice(0, &inner));
    match version {
        MspVersion::V2 => {
            let mut bytes = vec![b'$', b'X', direction];
            bytes.extend(inner);
            bytes
        }
        MspVersion::V1 | MspVersion::V2OverV1 => {
            let (command, payload) = if version == MspVersion::V1 {
                (u8::try_from(command).unwrap(), payload)
            } else {
                (255, inner.as_slice())
            };
            let mut bytes = vec![b'$', b'M', direction];
            let jumbo = payload.len() >= 255;
            bytes.push(if jumbo {
                255
            } else {
                u8::try_from(payload.len()).unwrap()
            });
            bytes.push(command);
            let (header, body) = if version == MspVersion::V2OverV1 {
                payload.split_at(5)
            } else {
                (&[][..], payload)
            };
            bytes.extend_from_slice(header);
            if jumbo {
                bytes.extend_from_slice(&u16::try_from(payload.len()).unwrap().to_le_bytes());
            }
            bytes.extend_from_slice(body);
            bytes.push(bytes[3..].iter().fold(0, |sum, byte| sum ^ byte));
            bytes
        }
        _ => unreachable!(),
    }
}

fn samples() -> [(MspMessage, Vec<u8>); 3] {
    let imu_values = [i16::MIN, i16::MAX, -1, 0, 1, -1234, 4321, -2, 256];
    [
        (
            MspMessage::Attitude(MspAttitude {
                roll_decidegrees: -1234,
                pitch_decidegrees: 2345,
                yaw_degrees: -180,
            }),
            [-1234_i16, 2345, -180]
                .into_iter()
                .flat_map(i16::to_le_bytes)
                .collect(),
        ),
        (
            MspMessage::RawImu(MspRawImu {
                acc: [i16::MIN, i16::MAX, -1],
                gyro: [0, 1, -1234],
                mag: [4321, -2, 256],
            }),
            imu_values.into_iter().flat_map(i16::to_le_bytes).collect(),
        ),
        (
            MspMessage::Altitude(MspAltitude {
                altitude_cm: -123_456_789,
                vario_cm_s: i16::MIN,
            }),
            (-123_456_789_i32)
                .to_le_bytes()
                .into_iter()
                .chain(i16::MIN.to_le_bytes())
                .collect(),
        ),
    ]
}

#[test]
fn responses_and_requests_across_formats_and_every_split() {
    for version in VERSIONS {
        for (response, payload) in samples() {
            let request = match response {
                MspMessage::Attitude(_) => MspMessage::AttitudeRequest,
                MspMessage::RawImu(_) => MspMessage::RawImuRequest,
                MspMessage::Altitude(_) => MspMessage::AltitudeRequest,
                _ => unreachable!(),
            };
            for (direction, message, payload) in
                [(b'>', response, payload.as_slice()), (b'<', request, &[])]
            {
                let command_id = message.command_id();
                let bytes = wire(version, direction, command_id, payload);
                let expected = MspPacket {
                    version,
                    flags: if version == MspVersion::V1 { 0 } else { 0xa5 },
                    message,
                };
                assert_eq!(expected.command_id(), command_id);
                assert_eq!(
                    expected.direction(),
                    if direction == b'<' {
                        MspDirection::Request
                    } else {
                        MspDirection::Response
                    }
                );
                assert_eq!(
                    MspPacket::try_from(MspFrame::try_from(bytes.as_slice()).unwrap()).as_ref(),
                    Ok(&expected)
                );
                let mut encoded = vec![0xee_u8; bytes.len() + 4];
                assert_eq!(expected.to_bytes(&mut encoded), Ok(bytes.len()));
                assert_eq!(&encoded[..bytes.len()], bytes.as_slice());
                assert_eq!(&encoded[bytes.len()..], &[0xee; 4]);
                for short_len in 0..bytes.len() {
                    let mut short_buf = vec![0u8; short_len];
                    assert_eq!(
                        expected.to_bytes(&mut short_buf),
                        Err(MspEncodeError::BufferTooSmall {
                            required: bytes.len(),
                            actual: short_len,
                        })
                    );
                }
                for split in 0..=bytes.len() {
                    let mut parser = MspParser::<64>::new();
                    let mut packets = Vec::new();
                    for chunk in [&bytes[..split], &bytes[split..]] {
                        let mut input = chunk;
                        packets.extend(parser.packets(&mut input).map(Result::unwrap));
                        assert_eq!(input, []);
                    }
                    assert_eq!(packets, core::slice::from_ref(&expected));
                }
                // Every byte can also arrive in its own transport chunk.
                let mut parser = MspParser::<64>::new();
                let packets: Vec<_> = bytes
                    .chunks(1)
                    .flat_map(|chunk| parser.packets(&mut &chunk[..]).collect::<Vec<_>>())
                    .collect();
                assert_eq!(packets, [Ok(expected)]);
            }
        }
    }
}

#[test]
fn extra_payload_is_ignored_but_still_checksummed() {
    for version in VERSIONS {
        for (message, mut payload) in samples() {
            // Also exercises jumbo v1 and jumbo v2-over-v1 envelopes.
            payload.extend_from_slice(&[0xab; 260]);
            let bytes = wire(version, b'>', message.command_id(), &payload);
            let packet =
                MspPacket::try_from(MspFrame::try_from(bytes.as_slice()).unwrap()).unwrap();
            assert_eq!(packet.message, message);
            for split in 0..=bytes.len() {
                let mut parser = MspParser::<512>::new();
                let mut packets = Vec::new();
                for chunk in [&bytes[..split], &bytes[split..]] {
                    packets.extend(parser.packets(&mut &chunk[..]));
                }
                assert_eq!(packets.len(), 1);
                assert_eq!(packets[0].as_ref(), Ok(&packet));
            }
            let mut corrupt = bytes.clone();
            // Last payload byte, before native CRC or outer XOR.
            let index = corrupt.len()
                - if version == MspVersion::V2OverV1 {
                    3
                } else {
                    2
                };
            corrupt[index] ^= 1;
            assert!(MspFrame::try_from(corrupt.as_slice()).is_err());
            let mut stream = corrupt;
            stream.extend_from_slice(&bytes);
            let mut parser = MspParser::<512>::new();
            assert_eq!(
                parser.packets(&mut stream.as_slice()).collect::<Vec<_>>(),
                [Ok(packet)]
            );

            let request = wire(version, b'<', message.command_id(), &payload);
            let decoded =
                MspPacket::try_from(MspFrame::try_from(request.as_slice()).unwrap()).unwrap();
            assert_eq!(decoded.direction(), MspDirection::Request);
            assert_eq!(decoded.command_id(), message.command_id());
        }
    }
}

#[test]
fn short_payloads_and_remote_errors_are_reported_and_consumed() {
    for version in VERSIONS {
        for (message, payload) in samples() {
            let command_id = message.command_id();
            let good = wire(version, b'>', command_id, &payload);
            let mut stream = Vec::new();
            let mut expected = Vec::new();
            for length in 0..payload.len() {
                stream.extend(wire(version, b'>', command_id, &payload[..length]));
                expected.push(Err(MspDecodeError::PayloadTooShort {
                    command_id,
                    required: payload.len(),
                    actual: length,
                }));
            }
            stream.extend(wire(version, b'!', command_id, &[]));
            expected.push(Err(MspDecodeError::RemoteError { command_id }));
            stream.extend(good);
            expected.push(Ok(MspPacket {
                version,
                flags: if version == MspVersion::V1 { 0 } else { 0xa5 },
                message,
            }));
            let mut parser = MspParser::<64>::new();
            let mut input = stream.as_slice();
            assert_eq!(parser.packets(&mut input).collect::<Vec<_>>(), expected);
            assert_eq!(input, []);
        }
    }
}

#[test]
fn unsupported_commands_take_precedence_and_iteration_continues() {
    for version in VERSIONS {
        let mut stream = b"noise".to_vec();
        for direction in *b"<>!" {
            stream.extend(wire(version, direction, 42, &[]));
        }
        stream.extend(wire(version, b'<', 108, &[]));
        let mut parser = MspParser::<64>::new();
        let mut input = stream.as_slice();
        let mut packets = parser.packets(&mut input);
        for _ in 0..3 {
            assert_eq!(
                packets.next(),
                Some(Err(MspDecodeError::UnsupportedCommand { command_id: 42 }))
            );
        }
        assert_eq!(
            packets.next().unwrap().unwrap().message,
            MspMessage::AttitudeRequest
        );
        assert_eq!(packets.next(), None);
        assert_eq!(packets.next(), None);
        assert_eq!(packets.next(), None);
    }
    let bytes = wire(MspVersion::V2, b'>', 0x3012, &[]);
    assert_eq!(
        MspPacket::try_from(MspFrame::try_from(bytes.as_slice()).unwrap()),
        Err(MspDecodeError::UnsupportedCommand { command_id: 0x3012 })
    );
}

#[test]
fn early_drop_preserves_cursor_and_owned_packets_survive_parser_reuse() {
    let first = wire(MspVersion::V2, b'<', 108, &[]);
    let second = wire(MspVersion::V1, b'<', 109, &[]);
    let mut stream = first.clone();
    stream.extend_from_slice(&second);
    let mut parser = MspParser::<64>::new();
    let mut input = stream.as_slice();
    let owned = parser.packets(&mut input).next().unwrap().unwrap();
    assert_eq!(input, second);
    let other = parser.next_packet(&mut input).unwrap().unwrap();
    parser.reset();
    drop(stream);
    assert_eq!(owned.message, MspMessage::AttitudeRequest);
    assert_eq!(other.message, MspMessage::AltitudeRequest);
}

#[test]
fn oversized_split_frames_follow_existing_spill_rules() {
    let large = wire(MspVersion::V2, b'<', 108, &[0; 64]);
    let mut parser = MspParser::<9>::new();
    // Contiguous frames need not fit the spill buffer.
    assert_eq!(
        parser
            .next_packet(&mut large.as_slice())
            .unwrap()
            .unwrap()
            .message,
        MspMessage::AttitudeRequest
    );
    let good = wire(MspVersion::V2, b'<', 109, &[]);
    for split in 1..large.len() {
        parser.reset();
        assert_eq!(parser.next_packet(&mut &large[..split]), Ok(None));
        let mut tail = large[split..].to_vec();
        tail.extend_from_slice(&good);
        let packets: Vec<_> = parser.packets(&mut tail.as_slice()).collect();
        assert_eq!(packets.len(), 1);
        assert_eq!(
            packets[0].as_ref().unwrap().message,
            MspMessage::AltitudeRequest
        );
    }
}

#[test]
fn iterator_drains_buffered_frames_recovered_from_corrupt_envelope() {
    let first = wire(MspVersion::V2, b'<', 108, &[]);
    let second = wire(MspVersion::V1, b'<', 109, &[]);
    let mut payload = first;
    payload.extend(second);
    let mut outer = wire(MspVersion::V2, b'>', 42, &payload);
    *outer.last_mut().unwrap() ^= 1;

    let mut parser = MspParser::<64>::new();
    assert_eq!(parser.next_packet(&mut &outer[..4]), Ok(None));
    let mut input = &outer[4..];
    {
        let mut packets = parser.packets(&mut input);
        assert_eq!(
            packets.next().unwrap().unwrap().message,
            MspMessage::AttitudeRequest
        );
    }
    // Recovery already consumed the transport input, but a second frame remains
    // in the spill buffer. A fresh iterator must drain it even with empty input.
    assert_eq!(input, []);
    let mut packets = parser.packets(&mut input);
    assert_eq!(
        packets.next().unwrap().unwrap().message,
        MspMessage::AltitudeRequest
    );
    assert_eq!(packets.next(), None);
    assert_eq!(packets.next(), None);
}

#[test]
fn errors_implement_display_and_error_trait() {
    use std::error::Error;
    let errors = [
        (
            MspDecodeError::UnsupportedCommand { command_id: 42 },
            "unsupported MSP command 42",
        ),
        (
            MspDecodeError::RemoteError { command_id: 108 },
            "remote MSP error for command 108",
        ),
        (
            MspDecodeError::PayloadTooShort {
                command_id: 108,
                required: 6,
                actual: 5,
            },
            "MSP command 108 payload too short: need 6 bytes, got 5",
        ),
        (
            MspDecodeError::InvalidUtf8 { command_id: 10 },
            "invalid UTF-8 in MSP payload for command 10",
        ),
        (
            MspDecodeError::CapacityExceeded {
                command_id: 10,
                capacity: 16,
                requested: 17,
            },
            "MSP command 10 payload exceeds capacity: capacity 16, requested 17",
        ),
        (
            MspDecodeError::InvalidPayload { command_id: 105 },
            "invalid MSP payload for command 105",
        ),
    ];
    for (error, expected) in errors {
        assert_eq!(error.to_string(), expected);
        assert!(error.source().is_none());
    }

    let encode_errors = [
        (
            MspEncodeError::BufferTooSmall {
                required: 18,
                actual: 12,
            },
            "MSP buffer too small: need 18 bytes, got 12",
        ),
        (
            MspEncodeError::InvalidV1Command { command_id: 255 },
            "MSP command 255 cannot be encoded as raw MSP v1 (must be < 255)",
        ),
        (
            MspEncodeError::PayloadTooLarge {
                max: 65_529,
                actual: 65_530,
            },
            "MSP payload too large: maximum 65529 bytes, got 65530",
        ),
    ];
    for (error, expected) in encode_errors {
        assert_eq!(error.to_string(), expected);
        assert!(error.source().is_none());
    }

    let capacity_err = uf_msp::CapacityError::new(16, 20);
    assert_eq!(
        capacity_err.to_string(),
        "MSP fixed container capacity exceeded: capacity 16, requested 20"
    );
    assert!(capacity_err.source().is_none());

    let frame_err = uf_msp::MspFrameError::InvalidPreamble;
    let decode_err = MspDecodeError::UnsupportedCommand { command_id: 99 };
    let encode_err = MspEncodeError::InvalidV1Command { command_id: 255 };

    let top_errors = [
        (
            uf_msp::MspError::from(frame_err),
            frame_err.to_string(),
            frame_err.to_string(),
        ),
        (
            uf_msp::MspError::from(decode_err),
            decode_err.to_string(),
            decode_err.to_string(),
        ),
        (
            uf_msp::MspError::from(encode_err),
            encode_err.to_string(),
            encode_err.to_string(),
        ),
        (
            uf_msp::MspError::from(capacity_err),
            capacity_err.to_string(),
            capacity_err.to_string(),
        ),
    ];
    for (error, expected_display, expected_source) in top_errors {
        assert_eq!(error.to_string(), expected_display);
        assert_eq!(error.source().unwrap().to_string(), expected_source);
    }
}

#[test]
fn manual_decoding_with_question_mark() {
    fn decode(
        parser: &mut MspParser<64>,
        input: &mut &[u8],
    ) -> Result<Option<MspPacket>, MspDecodeError> {
        let Some(frame) = parser.next(input) else {
            return Ok(None);
        };
        Ok(Some(MspPacket::try_from(frame)?))
    }

    let mut parser = MspParser::<64>::new();
    let request = wire(MspVersion::V2, b'<', MSP_ATTITUDE, &[]);
    assert_eq!(
        decode(&mut parser, &mut request.as_slice())
            .unwrap()
            .unwrap()
            .message,
        MspMessage::AttitudeRequest
    );
    let short = wire(MspVersion::V2, b'>', MSP_ATTITUDE, &[]);
    assert_eq!(
        decode(&mut parser, &mut short.as_slice()),
        Err(MspDecodeError::PayloadTooShort {
            command_id: MSP_ATTITUDE,
            required: 6,
            actual: 0,
        })
    );
}

#[test]
fn public_command_constants_match_wire_ids() {
    use uf_msp::commands::*;

    assert_eq!(MSP_API_VERSION, 1);
    assert_eq!(MSP_FC_VARIANT, 2);
    assert_eq!(MSP_FC_VERSION, 3);
    assert_eq!(MSP_BOARD_INFO, 4);
    assert_eq!(MSP_BUILD_INFO, 5);
    assert_eq!(MSP_NAME, 10);
    assert_eq!(MSP_SET_NAME, 11);
    assert_eq!(MSP_BATTERY_CONFIG, 32);
    assert_eq!(MSP_SET_BATTERY_CONFIG, 33);
    assert_eq!(MSP_ARMING_CONFIG, 61);
    assert_eq!(MSP_SET_ARMING_CONFIG, 62);
    assert_eq!(MSP_RX_MAP, 64);
    assert_eq!(MSP_REBOOT, 68);
    assert_eq!(MSP_DATAFLASH_READ, 71);
    assert_eq!(MSP_VTX_CONFIG, 88);
    assert_eq!(MSP_SET_VTX_CONFIG, 89);
    assert_eq!(MSP_ADVANCED_CONFIG, 90);
    assert_eq!(MSP_SET_ARMING_DISABLED, 99);
    assert_eq!(MSP_STATUS, 101);
    assert_eq!(MSP_RAW_IMU, 102);
    assert_eq!(MSP_SERVO, 103);
    assert_eq!(MSP_MOTOR, 104);
    assert_eq!(MSP_RC, 105);
    assert_eq!(MSP_RAW_GPS, 106);
    assert_eq!(MSP_COMP_GPS, 107);
    assert_eq!(MSP_ATTITUDE, 108);
    assert_eq!(MSP_ALTITUDE, 109);
    assert_eq!(MSP_ANALOG, 110);
    assert_eq!(MSP_RC_TUNING, 111);
    assert_eq!(MSP_PID, 112);
    assert_eq!(MSP_BOXNAMES, 116);
    assert_eq!(MSP_PIDNAMES, 117);
    assert_eq!(MSP_WP, 118);
    assert_eq!(MSP_BOXIDS, 119);
    assert_eq!(MSP_BATTERY_STATE, 130);
    assert_eq!(MSP_MOTOR_TELEMETRY, 139);
    assert_eq!(MSP_SIMPLIFIED_TUNING, 140);
    assert_eq!(MSP_VALIDATE_SIMPLIFIED_TUNING, 145);
    assert_eq!(MSP_STATUS_EX, 150);
    assert_eq!(MSP_UID, 160);
    assert_eq!(MSP_ATTITUDE_QUATERNION, 167);
    assert_eq!(MSP_PITOT, 170);
    assert_eq!(MSP_OSD_VIDEO_CONFIG, 180);
    assert_eq!(MSP_OSD_CANVAS, 189);
    assert_eq!(MSP_SET_RAW_RC, 200);
    assert_eq!(MSP_SET_PID, 202);
    assert_eq!(MSP_ACC_CALIBRATION, 205);
    assert_eq!(MSP_MAG_CALIBRATION, 206);
    assert_eq!(MSP_RESET_CONF, 208);
    assert_eq!(MSP_SET_MOTOR, 214);
    assert_eq!(MSP_SET_VTXTABLE_POWERLEVEL, 228);
    assert_eq!(MSP_MULTIPLE_MSP, 230);
    assert_eq!(MSP_EEPROM_WRITE, 250);
    assert_eq!(MSP_DEBUGMSG, 253);
    assert_eq!(MSP_DEBUG, 254);
    assert_eq!(MSP_V2_FRAME, 255);

    assert_eq!(MSP2_COMMON_SERIAL_CONFIG, 0x1009);
    assert_eq!(MSP2_SENSOR_RANGEFINDER_LIDARMT, 0x1F01);
    assert_eq!(MSP2_SENSOR_OPTICALFLOW_MT, 0x1F02);
    assert_eq!(MSP2_SENSOR_GPS, 0x1F03);
    assert_eq!(MSP2_BETAFLIGHT_BIND, 0x3000);
    assert_eq!(MSP2_CLI_SETTING, 0x3010);
    assert_eq!(MSP2_CLI_SETTING_INFO, 0x3011);
    assert_eq!(MSP2_CLI_COMMAND, 0x3012);

    assert_eq!(MspRawImu::COMMAND_ID, MSP_RAW_IMU);
    assert_eq!(MspAttitude::COMMAND_ID, MSP_ATTITUDE);
    assert_eq!(MspAltitude::COMMAND_ID, MSP_ALTITUDE);
    assert_eq!(MspRawImu::MIN_PAYLOAD_SIZE, 18);
    assert_eq!(MspAttitude::MIN_PAYLOAD_SIZE, 6);
    assert_eq!(MspAltitude::MIN_PAYLOAD_SIZE, 6);
    assert_eq!(MspMessage::RawImuRequest.command_id(), MSP_RAW_IMU);
    assert_eq!(MspMessage::AttitudeRequest.command_id(), MSP_ATTITUDE);
    assert_eq!(MspMessage::AltitudeRequest.command_id(), MSP_ALTITUDE);
}

fn check_payload_codec<P>(expected: P, wire_bytes: &[u8])
where
    P: MspPayload + Copy + PartialEq + core::fmt::Debug,
{
    assert_eq!(expected.payload_size(), P::MIN_PAYLOAD_SIZE);
    assert_eq!(expected.payload_size(), wire_bytes.len());

    let mut exact = vec![0u8; P::MIN_PAYLOAD_SIZE];
    assert_eq!(expected.to_bytes(&mut exact), Ok(P::MIN_PAYLOAD_SIZE));
    assert_eq!(exact, wire_bytes);
    assert_eq!(P::from_bytes(&exact), Ok(expected));

    let mut oversized = vec![0xee_u8; P::MIN_PAYLOAD_SIZE + 4];
    assert_eq!(expected.to_bytes(&mut oversized), Ok(P::MIN_PAYLOAD_SIZE));
    assert_eq!(&oversized[..P::MIN_PAYLOAD_SIZE], wire_bytes);
    assert_eq!(
        &oversized[P::MIN_PAYLOAD_SIZE..],
        &[0xee; 4],
        "to_bytes must not clobber trailing bytes"
    );
    assert_eq!(P::from_bytes(&oversized), Ok(expected));

    for short_len in 0..P::MIN_PAYLOAD_SIZE {
        let mut short_buf = vec![0u8; short_len];
        assert_eq!(
            expected.to_bytes(&mut short_buf),
            Err(MspEncodeError::BufferTooSmall {
                required: P::MIN_PAYLOAD_SIZE,
                actual: short_len,
            })
        );
        assert_eq!(
            P::from_bytes(&wire_bytes[..short_len]),
            Err(MspDecodeError::PayloadTooShort {
                command_id: P::COMMAND_ID,
                required: P::MIN_PAYLOAD_SIZE,
                actual: short_len,
            })
        );
    }
}

#[test]
fn per_message_payload_codecs_roundtrip_and_check_lengths() {
    for (message, payload) in samples() {
        match message {
            MspMessage::Attitude(expected) => {
                check_payload_codec(expected, &payload);
                assert_eq!(MspAttitude::try_from(payload.as_slice()), Ok(expected));
            }
            MspMessage::RawImu(expected) => {
                check_payload_codec(expected, &payload);
                assert_eq!(MspRawImu::try_from(payload.as_slice()), Ok(expected));
            }
            MspMessage::Altitude(expected) => {
                check_payload_codec(expected, &payload);
                assert_eq!(MspAltitude::try_from(payload.as_slice()), Ok(expected));
            }
            _ => unreachable!(),
        }
    }
}

#[test]
fn packet_iterator_implements_debug() {
    let bytes = wire(MspVersion::V2, b'<', MSP_ATTITUDE, &[]);
    let mut parser = MspParser::<32>::new();
    let mut input = bytes.as_slice();
    let mut packets = parser.packets(&mut input);
    assert_eq!(
        format!("{packets:?}"),
        format!(
            "MspPackets {{ parser: MspParser {{ capacity: 32, start: 0, end: 0, buffered_len: 0, .. }}, remaining_len: {} }}",
            bytes.len()
        )
    );
    assert!(packets.next().is_some());
    assert!(packets.next().is_none());
    assert_eq!(
        format!("{packets:?}"),
        "MspPackets { parser: MspParser { capacity: 32, start: 0, end: 0, buffered_len: 0, .. }, remaining_len: 0 }"
    );
}

#[derive(Debug, PartialEq, Eq)]
struct VariablePayload<const CMD: u16>(Vec<u8>);

impl<const CMD: u16> MspPayload for VariablePayload<CMD> {
    const COMMAND_ID: u16 = CMD;
    const MIN_PAYLOAD_SIZE: usize = 0;

    fn payload_size(&self) -> usize {
        self.0.len()
    }

    fn from_bytes(data: &[u8]) -> Result<Self, MspDecodeError> {
        Ok(Self(data.to_vec()))
    }

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
        let required = self.0.len();
        let actual = buffer.len();
        let dst = buffer
            .get_mut(..required)
            .ok_or(MspEncodeError::BufferTooSmall { required, actual })?;
        dst.copy_from_slice(&self.0);
        Ok(required)
    }
}

#[test]
fn write_packet_and_request_across_all_versions_and_jumbo_boundaries() {
    for version in VERSIONS {
        for len in [0, 1, 248, 249, 254, 255, 256, 512] {
            let payload = VariablePayload::<108>(vec![0x5a; len]);
            for (dir_byte, direction) in [
                (b'<', MspDirection::Request),
                (b'>', MspDirection::Response),
                (b'!', MspDirection::Error),
            ] {
                let expected = wire(version, dir_byte, 108, &payload.0);
                let mut buf = vec![0u8; expected.len()];
                assert_eq!(
                    write_packet_to_buffer(&mut buf, version, direction, 0xa5, &payload),
                    Ok(expected.len())
                );
                assert_eq!(buf, expected);
                let frame = MspFrame::try_from(buf.as_slice()).unwrap();
                assert_eq!(frame.version(), version);
                assert_eq!(frame.direction(), direction);
                assert_eq!(frame.command_id(), 108);
                assert_eq!(
                    VariablePayload::<108>::from_bytes(frame.payload()),
                    Ok(VariablePayload(payload.0.clone()))
                );
            }
        }

        let expected_req = wire(version, b'<', 108, &[]);
        let mut req_buf = vec![0u8; expected_req.len()];
        assert_eq!(
            write_request_to_buffer(&mut req_buf, version, 0xa5, 108),
            Ok(expected_req.len())
        );
        assert_eq!(req_buf, expected_req);

        for (dir_byte, direction) in [
            (b'<', MspDirection::Request),
            (b'>', MspDirection::Response),
            (b'!', MspDirection::Error),
        ] {
            for len in [0, 1, 7, 255, 300] {
                let raw_payload = vec![0x3c; len];
                let expected_raw = wire(version, dir_byte, 108, &raw_payload);
                let mut raw_buf = vec![0u8; expected_raw.len()];
                assert_eq!(
                    write_raw_frame_to_buffer(
                        &mut raw_buf,
                        version,
                        direction,
                        0xa5,
                        108,
                        &raw_payload,
                    ),
                    Ok(expected_raw.len())
                );
                assert_eq!(raw_buf, expected_raw);
            }
        }
    }
}

#[cfg(feature = "defmt")]
#[test]
fn public_types_implement_defmt_format() {
    fn assert_defmt_format<T: defmt::Format>() {}

    assert_defmt_format::<MspVersion>();
    assert_defmt_format::<MspDirection>();
    assert_defmt_format::<uf_msp::MspFrameError>();
    assert_defmt_format::<MspFrame<'_>>();
    assert_defmt_format::<MspParser<16>>();
    assert_defmt_format::<MspAttitude>();
    assert_defmt_format::<MspRawImu>();
    assert_defmt_format::<MspAltitude>();
    assert_defmt_format::<MspMessage>();
    assert_defmt_format::<MspPacket>();
    assert_defmt_format::<uf_msp::MspPackets<'_, '_, 16>>();
    assert_defmt_format::<MspDecodeError>();
    assert_defmt_format::<MspEncodeError>();
    assert_defmt_format::<uf_msp::CapacityError>();
    assert_defmt_format::<uf_msp::MspError>();
    assert_defmt_format::<uf_msp::MspApiVersion>();
    assert_defmt_format::<uf_msp::MspFcVariant>();
    assert_defmt_format::<uf_msp::MspFcVersion>();
    assert_defmt_format::<uf_msp::MspName>();
    assert_defmt_format::<uf_msp::MspSetName>();
    assert_defmt_format::<uf_msp::MspStatus>();
    assert_defmt_format::<uf_msp::MspStatusExtended>();
    assert_defmt_format::<uf_msp::MspStatusTail>();
    assert_defmt_format::<uf_msp::MspRebootRequest>();
    assert_defmt_format::<uf_msp::MspRebootResponse>();
    assert_defmt_format::<uf_msp::MspRc>();
    assert_defmt_format::<uf_msp::MspSetRawRc>();
}

#[test]
fn variable_payloads_accept_native_views_up_to_their_capacity() {
    use uf_msp::CapacityError;
    use uf_msp::MAX_CRAFT_NAME_LEN;
    use uf_msp::MAX_FLIGHT_MODE_EXT_BYTES;
    use uf_msp::MAX_RC_CHANNELS;
    use uf_msp::MspName;
    use uf_msp::MspRc;
    use uf_msp::MspSetName;
    use uf_msp::MspSetRawRc;
    use uf_msp::MspStatusExtended;

    let name = "é".repeat(MAX_CRAFT_NAME_LEN / 2);
    let long_name = name.clone() + "x";
    let name_error = CapacityError {
        capacity: MAX_CRAFT_NAME_LEN,
        requested: MAX_CRAFT_NAME_LEN + 1,
    };
    assert_eq!(MspName::new("").unwrap().name(), "");
    assert_eq!(MspSetName::new("").unwrap().name(), "");
    assert_eq!(MspName::new(&name).unwrap().name(), name);
    assert_eq!(MspSetName::new(&name).unwrap().name(), name);
    assert_eq!(MspName::from_bytes(name.as_bytes()).unwrap().name(), name);
    assert_eq!(
        MspSetName::from_bytes(name.as_bytes()).unwrap().name(),
        name
    );
    assert_eq!(MspName::new(&long_name), Err(name_error));
    assert_eq!(MspSetName::new(&long_name), Err(name_error));

    let channels = [1500; MAX_RC_CHANNELS];
    let long_channels = [1500; MAX_RC_CHANNELS + 1];
    let channel_error = CapacityError {
        capacity: MAX_RC_CHANNELS,
        requested: MAX_RC_CHANNELS + 1,
    };
    assert_eq!(MspRc::new(&[]).unwrap().channels(), []);
    assert_eq!(MspSetRawRc::new(&[]).unwrap().channels(), []);
    assert_eq!(MspRc::new(&channels).unwrap().channels(), &channels);
    assert_eq!(MspSetRawRc::new(&channels).unwrap().channels(), &channels);
    assert_eq!(MspRc::new(&long_channels), Err(channel_error));
    assert_eq!(MspSetRawRc::new(&long_channels), Err(channel_error));

    let mut extended = MspStatusExtended::default();
    let flags = [0x5a; MAX_FLIGHT_MODE_EXT_BYTES];
    let long_flags = [0xff; MAX_FLIGHT_MODE_EXT_BYTES + 1];
    assert_eq!(extended.flight_mode_flags_ext(), []);
    extended.set_flight_mode_flags_ext(&flags).unwrap();
    assert_eq!(extended.flight_mode_flags_ext(), &flags);
    assert_eq!(
        extended.set_flight_mode_flags_ext(&long_flags),
        Err(CapacityError {
            capacity: MAX_FLIGHT_MODE_EXT_BYTES,
            requested: MAX_FLIGHT_MODE_EXT_BYTES + 1,
        })
    );
    assert_eq!(extended.flight_mode_flags_ext(), &flags);
    extended.set_flight_mode_flags_ext(&[]).unwrap();
    assert_eq!(extended.flight_mode_flags_ext(), []);
}

#[test]
fn owned_packet_stack_footprint() {
    // Guard embedded stack footprint: owned packets must stay compact (<= 64 bytes).
    assert!(
        core::mem::size_of::<MspMessage>() <= 64,
        "MspMessage grew too large for embedded stacks: {} bytes",
        core::mem::size_of::<MspMessage>()
    );
    assert!(
        core::mem::size_of::<MspPacket>() <= 64,
        "MspPacket grew too large for embedded stacks: {} bytes",
        core::mem::size_of::<MspPacket>()
    );
}

#[test]
fn archetype_packets_roundtrip_across_all_versions_and_directions() {
    use uf_msp::MspApiVersion;
    use uf_msp::MspFcVariant;
    use uf_msp::MspFcVersion;
    use uf_msp::MspName;
    use uf_msp::MspRc;
    use uf_msp::MspRebootRequest;
    use uf_msp::MspRebootResponse;
    use uf_msp::MspSetName;
    use uf_msp::MspSetRawRc;

    let cases: &[(MspMessage, Vec<u8>)] = &[
        // 1. Out Fixed & Optional Trailing Telemetry
        (MspMessage::ApiVersionRequest, vec![]),
        (
            MspMessage::ApiVersion(MspApiVersion {
                protocol_version: 0,
                api_version_major: 1,
                api_version_minor: 49,
            }),
            vec![0, 1, 49],
        ),
        (MspMessage::FcVariantRequest, vec![]),
        (
            MspMessage::FcVariant(MspFcVariant {
                identifier: *b"BTFL",
            }),
            b"BTFL".to_vec(),
        ),
        (MspMessage::FcVersionRequest, vec![]),
        (
            MspMessage::FcVersion(MspFcVersion {
                major: 4,
                minor: 6,
                patch: 0,
            }),
            vec![4, 6, 0],
        ),
        // 2. Out Variable-Length Array & String
        (MspMessage::NameRequest, vec![]),
        (
            MspMessage::Name(MspName::new("uflight").unwrap()),
            b"uflight".to_vec(),
        ),
        (MspMessage::RcRequest, vec![]),
        (
            MspMessage::Rc(MspRc::new(&[1500, 1500, 1000, 1500]).unwrap()),
            [1500_u16, 1500, 1000, 1500]
                .into_iter()
                .flat_map(u16::to_le_bytes)
                .collect(),
        ),
        // 3. In Setter & Action Trigger (`<` Request with payload/empty, `>` empty ACK)
        (
            MspMessage::SetName(MspSetName::new("race-quad").unwrap()),
            b"race-quad".to_vec(),
        ),
        (MspMessage::SetNameResponse, vec![]),
        (
            MspMessage::SetRawRc(MspSetRawRc::new(&[1000, 1200, 1400, 1600, 1800, 2000]).unwrap()),
            [1000_u16, 1200, 1400, 1600, 1800, 2000]
                .into_iter()
                .flat_map(u16::to_le_bytes)
                .collect(),
        ),
        (MspMessage::SetRawRcResponse, vec![]),
        (MspMessage::AccCalibrationRequest, vec![]),
        (MspMessage::AccCalibrationResponse, vec![]),
        (MspMessage::EepromWriteRequest, vec![]),
        (MspMessage::EepromWriteResponse, vec![]),
        // 4. Parameterized Request -> Typed Response (`MSP_REBOOT`)
        (
            MspMessage::RebootRequest(MspRebootRequest { mode: None }),
            vec![],
        ),
        (
            MspMessage::RebootRequest(MspRebootRequest { mode: Some(2) }),
            vec![2],
        ),
        (
            MspMessage::Reboot(MspRebootResponse {
                mode: 0,
                msc_ready: None,
            }),
            vec![0],
        ),
        (
            MspMessage::Reboot(MspRebootResponse {
                mode: 2,
                msc_ready: Some(true),
            }),
            vec![2, 1],
        ),
    ];

    assert_packets_roundtrip(cases);
}

#[test]
fn status_packets_roundtrip_across_all_versions_and_directions() {
    use uf_msp::MspStatus;
    use uf_msp::MspStatusExtended;
    use uf_msp::MspStatusTail;

    let status_legacy = MspStatus {
        cycle_time_us: 125,
        i2c_error_count: 0,
        sensors: 0x27,
        flight_mode_flags: 0x0000_0003,
        current_pid_profile_index: 1,
        extended: None,
    };
    // Betaflight 4.5.2 carries a signed core temperature.
    let mut extended_bf45 = MspStatusExtended::default();
    extended_bf45.average_system_load_percent = 30;
    extended_bf45.arming_disable_flags_count = 28;
    extended_bf45.tail = Some(MspStatusTail::CoreTemp {
        core_temp_celsius: -5,
    });
    assert_eq!(extended_bf45.core_temp_celsius(), Some(-5));
    assert_eq!(extended_bf45.control_rate_profile_count(), None);
    assert_eq!(extended_bf45.battery_profile(), None);

    let status_bf45 = MspStatus {
        cycle_time_us: 125,
        i2c_error_count: 0,
        sensors: 0x27,
        flight_mode_flags: 0x0000_0003,
        current_pid_profile_index: 0,
        extended: Some(extended_bf45),
    };

    let mut extended = MspStatusExtended::default();
    extended.set_flight_mode_flags_ext(&[0xaa, 0xbb]).unwrap();
    extended.average_system_load_percent = 42;
    extended.arming_disable_flags_count = 28;
    extended.arming_disable_flags = 0x0010_0004;
    extended.config_state_flags = 1;
    extended.tail = Some(MspStatusTail::BatteryProfile {
        core_temp_celsius: 55,
        control_rate_profile_count: 4,
        battery_profile_count: 3,
        current_battery_profile_index: 1,
    });
    assert_eq!(extended.core_temp_celsius(), Some(55));
    assert_eq!(extended.control_rate_profile_count(), Some(4));
    assert_eq!(extended.battery_profile(), Some((3, 1)));

    let status_ext = MspStatus {
        cycle_time_us: 125,
        i2c_error_count: 2,
        sensors: 0x2f,
        flight_mode_flags: 0x1234_5678,
        current_pid_profile_index: 0,
        extended: Some(extended),
    };

    assert_packets_roundtrip(&[
        (MspMessage::StatusRequest, vec![]),
        (
            MspMessage::Status(status_legacy),
            vec![125, 0, 0, 0, 0x27, 0, 3, 0, 0, 0, 1],
        ),
        (
            MspMessage::Status(status_bf45),
            vec![
                125, 0, 0, 0, 0x27, 0, 3, 0, 0, 0, 0, 30, 0, 0, 0, 0, 28, 0, 0, 0, 0, 0, 0xfb, 0xff,
            ],
        ),
        (
            MspMessage::Status(status_ext),
            vec![
                125, 0, 2, 0, 0x2f, 0, 0x78, 0x56, 0x34, 0x12, 0, 42, 0, 0, 0, 2, 0xaa, 0xbb, 28,
                4, 0, 0x10, 0, 1, 55, 0, 4, 3, 1,
            ],
        ),
    ]);
}

fn assert_packets_roundtrip(cases: &[(MspMessage, Vec<u8>)]) {
    assert_packets_roundtrip_on_versions(&VERSIONS, cases);
}

fn assert_packets_roundtrip_on_versions(versions: &[MspVersion], cases: &[(MspMessage, Vec<u8>)]) {
    for &version in versions {
        for (message, expected_payload) in cases {
            let dir_byte = match message.direction() {
                MspDirection::Request => b'<',
                MspDirection::Response => b'>',
                MspDirection::Error => b'!',
            };
            let expected_wire = wire(version, dir_byte, message.command_id(), expected_payload);
            let packet = MspPacket {
                version,
                flags: if version == MspVersion::V1 { 0 } else { 0xa5 },
                message: message.clone(),
            };

            let mut buf = vec![0u8; expected_wire.len()];
            assert_eq!(packet.to_bytes(&mut buf), Ok(expected_wire.len()));
            assert_eq!(buf, expected_wire);

            let frame = MspFrame::try_from(buf.as_slice()).unwrap();
            assert_eq!(MspPacket::try_from(frame).as_ref(), Ok(&packet));
            assert_eq!(MspPacket::from_frame_bytes(&buf).as_ref(), Ok(&packet));
            assert_eq!(MspPacket::try_from(buf.as_slice()).as_ref(), Ok(&packet));

            // Also verify streaming parser across chunk splits
            for split in [0, 1, expected_wire.len() / 2, expected_wire.len()] {
                let mut parser = MspParser::<128>::new();
                let mut decoded = Vec::new();
                for chunk in [&expected_wire[..split], &expected_wire[split..]] {
                    let mut input = chunk;
                    decoded.extend(parser.packets(&mut input).map(Result::unwrap));
                }
                assert_eq!(decoded, core::slice::from_ref(&packet));
            }
        }
    }
}

#[test]
fn variable_payload_decoding_checks() {
    use uf_msp::MspFcVariant;
    use uf_msp::MspName;
    use uf_msp::MspRc;
    use uf_msp::MspSetName;
    use uf_msp::MspSetRawRc;
    use uf_msp::commands::MSP_NAME;
    use uf_msp::commands::MSP_RC;
    use uf_msp::commands::MSP_SET_NAME;
    use uf_msp::commands::MSP_SET_RAW_RC;

    // Verify MspFcVariant::as_str and invalid payload rejection for variable packets
    let variant = MspFcVariant::try_from(b"BTFL".as_slice()).unwrap();
    assert_eq!(variant.as_str(), Some("BTFL"));

    // Odd byte count in MSP_RC / MSP_SET_RAW_RC is rejected as InvalidPayload
    assert_eq!(
        MspRc::try_from([1_u8, 2, 3].as_slice()),
        Err(MspDecodeError::InvalidPayload { command_id: MSP_RC })
    );
    // > 18 channels (38 bytes) in MSP_SET_RAW_RC is rejected as CapacityExceeded
    assert_eq!(
        MspSetRawRc::try_from([0_u8; 38].as_slice()),
        Err(MspDecodeError::CapacityExceeded {
            command_id: MSP_SET_RAW_RC,
            capacity: uf_msp::MAX_RC_CHANNELS,
            requested: 19,
        })
    );
    // > 16 bytes in MSP_NAME / MSP_SET_NAME is rejected as CapacityExceeded
    assert_eq!(
        MspName::try_from(b"0123456789abcdefX".as_slice()),
        Err(MspDecodeError::CapacityExceeded {
            command_id: MSP_NAME,
            capacity: uf_msp::MAX_CRAFT_NAME_LEN,
            requested: 17,
        })
    );
    assert_eq!(
        MspSetName::try_from([0xff, 0xfe].as_slice()),
        Err(MspDecodeError::InvalidUtf8 {
            command_id: MSP_SET_NAME
        })
    );
}

#[test]
fn error_accessors_predicates_and_unified_question_mark_ergonomics() {
    use uf_msp::CapacityError;
    use uf_msp::MspError;
    use uf_msp::MspFrameError;
    use uf_msp::MspName;
    use uf_msp::commands::MSP_NAME;

    // End-to-end helper using uf_msp::Result and `?` across CapacityError,
    // MspEncodeError, MspFrameError, and MspDecodeError without manual mapping.
    fn roundtrip_craft_name(raw_name: &str, buf: &mut [u8]) -> uf_msp::Result<MspPacket> {
        let payload = MspName::new(raw_name)?;
        let len = write_packet_to_buffer(buf, MspVersion::V2, MspDirection::Response, 0, &payload)?;
        MspPacket::from_frame_bytes(&buf[..len])
    }

    let unsupported = MspDecodeError::UnsupportedCommand { command_id: 42 };
    assert_eq!(unsupported.command_id(), 42);
    assert!(unsupported.is_unsupported());
    assert!(!unsupported.is_remote_error());
    assert_eq!(unsupported.capacity_error(), None);

    let remote = MspDecodeError::RemoteError { command_id: 108 };
    assert_eq!(remote.command_id(), 108);
    assert!(!remote.is_unsupported());
    assert!(remote.is_remote_error());

    let short = MspDecodeError::PayloadTooShort {
        command_id: 109,
        required: 6,
        actual: 2,
    };
    assert_eq!(short.command_id(), 109);

    let utf8 = MspDecodeError::InvalidUtf8 { command_id: 10 };
    assert_eq!(utf8.command_id(), 10);

    let cap = MspDecodeError::CapacityExceeded {
        command_id: 10,
        capacity: 16,
        requested: 20,
    };
    assert_eq!(cap.command_id(), 10);
    assert_eq!(cap.capacity_error(), Some(CapacityError::new(16, 20)));

    let invalid = MspDecodeError::InvalidPayload { command_id: 105 };
    assert_eq!(invalid.command_id(), 105);

    assert!(
        MspFrameError::Incomplete {
            required: 9,
            actual: 3
        }
        .is_incomplete()
    );
    assert!(!MspFrameError::InvalidPreamble.is_incomplete());

    let mut buf = [0u8; 32];
    let packet = roundtrip_craft_name("uflight", &mut buf).unwrap();
    assert_eq!(
        packet.message,
        MspMessage::Name(MspName::new("uflight").unwrap())
    );

    assert_eq!(
        roundtrip_craft_name("0123456789abcdefX", &mut buf),
        Err(MspError::Capacity(CapacityError::new(16, 17)))
    );

    let mut tiny_buf = [0u8; 4];
    assert_eq!(
        roundtrip_craft_name("uflight", &mut tiny_buf),
        Err(MspError::Encode(MspEncodeError::BufferTooSmall {
            required: 16,
            actual: 4,
        }))
    );

    assert_eq!(
        MspPacket::try_from(b"$Y".as_slice()),
        Err(MspError::Frame(MspFrameError::InvalidPreamble))
    );

    let bad_utf8_frame = wire(MspVersion::V2, b'>', MSP_NAME, &[0xff, 0xfe]);
    assert_eq!(
        MspPacket::try_from(bad_utf8_frame.as_slice()),
        Err(MspError::Decode(MspDecodeError::InvalidUtf8 {
            command_id: MSP_NAME,
        }))
    );
}

#[test]
fn frame_encoding_rejects_mismatched_payload_sizes() {
    struct MiscountedPayload(usize);

    impl MspPayload for MiscountedPayload {
        const COMMAND_ID: u16 = MSP_ATTITUDE;
        const MIN_PAYLOAD_SIZE: usize = 2;

        fn from_bytes(_data: &[u8]) -> Result<Self, MspDecodeError> {
            Ok(Self(Self::MIN_PAYLOAD_SIZE))
        }

        fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
            buffer[0] = 0x11;
            Ok(self.0)
        }
    }

    for version in VERSIONS {
        for written in [1, 3] {
            let mut buffer = [0xee; 32];
            assert_eq!(
                write_packet_to_buffer(
                    &mut buffer,
                    version,
                    MspDirection::Response,
                    0,
                    &MiscountedPayload(written),
                ),
                Err(MspEncodeError::PayloadSizeMismatch {
                    expected: 2,
                    written,
                }),
                "{version:?}"
            );
        }
    }
}

#[test]
fn frame_encoding_validates_v1_commands_and_payload_limits() {
    struct OversizedPayload(usize);
    impl MspPayload for OversizedPayload {
        const COMMAND_ID: u16 = 108;
        const MIN_PAYLOAD_SIZE: usize = 0;

        fn payload_size(&self) -> usize {
            self.0
        }

        fn from_bytes(_data: &[u8]) -> Result<Self, MspDecodeError> {
            Ok(Self(0))
        }

        fn to_bytes(&self, _buffer: &mut [u8]) -> Result<usize, MspEncodeError> {
            Ok(self.0)
        }
    }

    let mut buf = [0u8; 32];
    for command_id in [255_u16, 256, 0x3012] {
        assert_eq!(
            write_request_to_buffer(&mut buf, MspVersion::V1, 0, command_id),
            Err(MspEncodeError::InvalidV1Command { command_id })
        );
        for version in [MspVersion::V2, MspVersion::V2OverV1] {
            let expected = wire(version, b'<', command_id, &[]);
            let len = write_request_to_buffer(&mut buf, version, 0xa5, command_id).unwrap();
            assert_eq!(&buf[..len], expected.as_slice());
        }
    }

    let v2_cmd = VariablePayload::<0x3012>(vec![1, 2, 3]);
    assert_eq!(
        write_packet_to_buffer(&mut buf, MspVersion::V1, MspDirection::Response, 0, &v2_cmd),
        Err(MspEncodeError::InvalidV1Command { command_id: 0x3012 })
    );

    for version in [MspVersion::V1, MspVersion::V2] {
        assert_eq!(
            write_packet_to_buffer(
                &mut buf,
                version,
                MspDirection::Response,
                0,
                &OversizedPayload(65_536)
            ),
            Err(MspEncodeError::PayloadTooLarge {
                max: 65_535,
                actual: 65_536,
            })
        );
    }
    assert_eq!(
        write_packet_to_buffer(
            &mut buf,
            MspVersion::V2OverV1,
            MspDirection::Response,
            0,
            &OversizedPayload(65_530)
        ),
        Err(MspEncodeError::PayloadTooLarge {
            max: 65_529,
            actual: 65_530,
        })
    );
}

#[test]
fn gps_and_power_packets_roundtrip_with_legacy_and_extended_payloads() {
    use uf_msp::MspAnalog;
    use uf_msp::MspBatteryState;
    use uf_msp::MspCompGps;
    use uf_msp::MspRawGps;
    use uf_msp::MspRawGpsDop;
    use uf_msp::MspRawGpsExtended;

    let legacy_gps = MspRawGps {
        fix: 2,
        satellites: 9,
        latitude_e7: -1234,
        longitude_e7: 5678,
        altitude_m: 300,
        ground_speed_cm_s: 513,
        ground_course_decidegrees: 2700,
        dop: None,
    };
    let legacy_gps_bytes = [
        2, 9, 0x2e, 0xfb, 0xff, 0xff, 0x2e, 0x16, 0, 0, 0x2c, 1, 1, 2, 0x8c, 0x0a,
    ];
    let mut gps_with_dop = legacy_gps_bytes.to_vec();
    gps_with_dop.extend([0x23, 1]);
    let mut extended_gps = gps_with_dop.clone();
    extended_gps.extend([78, 0, 0x90, 1, 15]);

    let analog = MspAnalog {
        voltage_decivolts: 168,
        consumed_mah: 321,
        rssi: 1023,
        current_centiamps: -300,
        voltage_centivolts: None,
    };
    let battery = MspBatteryState {
        cell_count: 4,
        capacity_mah: 1500,
        voltage_decivolts: 168,
        consumed_mah: 321,
        current_centiamps: -300,
        state: 2,
        voltage_centivolts: None,
    };
    assert_packets_roundtrip_on_versions(
        &VERSIONS,
        &[
            (MspMessage::RawGpsRequest, vec![]),
            (MspMessage::RawGps(legacy_gps), legacy_gps_bytes.to_vec()),
            (
                MspMessage::RawGps(MspRawGps {
                    dop: Some(MspRawGpsDop {
                        dop_centi: 291,
                        extended: None,
                    }),
                    ..legacy_gps
                }),
                gps_with_dop,
            ),
            (
                MspMessage::RawGps(MspRawGps {
                    dop: Some(MspRawGpsDop {
                        dop_centi: 291,
                        extended: Some(MspRawGpsExtended {
                            hdop_centi: 78,
                            vdop_centi: 400,
                            satellites_in_view: 15,
                        }),
                    }),
                    ..legacy_gps
                }),
                extended_gps,
            ),
            (MspMessage::CompGpsRequest, vec![]),
            (
                MspMessage::CompGps(MspCompGps {
                    distance_to_home_m: 321,
                    direction_to_home_degrees: -90,
                    update: 1,
                }),
                vec![0x41, 1, 0xa6, 0xff, 1],
            ),
            (MspMessage::AnalogRequest, vec![]),
            (
                MspMessage::Analog(analog),
                vec![168, 0x41, 1, 0xff, 3, 0xd4, 0xfe],
            ),
            (
                MspMessage::Analog(MspAnalog {
                    voltage_centivolts: Some(1678),
                    ..analog
                }),
                vec![168, 0x41, 1, 0xff, 3, 0xd4, 0xfe, 0x8e, 6],
            ),
            (MspMessage::BatteryStateRequest, vec![]),
            (
                MspMessage::BatteryState(battery),
                vec![4, 0xdc, 5, 168, 0x41, 1, 0xd4, 0xfe, 2],
            ),
            (
                MspMessage::BatteryState(MspBatteryState {
                    voltage_centivolts: Some(1678),
                    ..battery
                }),
                vec![4, 0xdc, 5, 168, 0x41, 1, 0xd4, 0xfe, 2, 0x8e, 6],
            ),
        ],
    );
}

#[test]
fn gps_and_power_packets_reject_truncated_extensions() {
    use uf_msp::commands::MSP_ANALOG;
    use uf_msp::commands::MSP_BATTERY_STATE;
    use uf_msp::commands::MSP_RAW_GPS;

    for (command_id, actual, required) in [
        (MSP_RAW_GPS, 15, 16),
        (MSP_RAW_GPS, 17, 18),
        (MSP_RAW_GPS, 19, 20),
        (MSP_RAW_GPS, 21, 22),
        (MSP_RAW_GPS, 22, 23),
        (MSP_ANALOG, 8, 9),
        (MSP_BATTERY_STATE, 10, 11),
    ] {
        let frame = wire(MspVersion::V2, b'>', command_id, &[0; 23][..actual]);
        assert_eq!(
            MspPacket::from_frame_bytes(&frame),
            Err(uf_msp::MspError::Decode(MspDecodeError::PayloadTooShort {
                command_id,
                required,
                actual,
            }))
        );
    }
}

#[test]
fn device_and_battery_profile_packets_roundtrip_on_supported_versions() {
    use uf_msp::MspBatteryProfile;
    use uf_msp::MspBatteryProfileRequest;
    use uf_msp::MspFeatureConfig;
    use uf_msp::MspUid;
    use uf_msp::commands::MSP2_BATTERY_PROFILE;

    assert_packets_roundtrip_on_versions(
        &VERSIONS,
        &[
            (MspMessage::UidRequest, vec![]),
            (
                MspMessage::Uid(MspUid {
                    words: [0x1234_5678, 0x90ab_cdef, 0xfe01_0203],
                }),
                vec![
                    0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab, 0x90, 3, 2, 1, 0xfe,
                ],
            ),
            (MspMessage::FeatureConfigRequest, vec![]),
            (
                MspMessage::FeatureConfig(MspFeatureConfig {
                    enabled_features: 0x8004_0002,
                }),
                vec![2, 0, 4, 0x80],
            ),
        ],
    );

    let active = MspMessage::BatteryProfileRequest(MspBatteryProfileRequest {
        profile_index: None,
    });
    assert_eq!(
        MspPacket {
            version: MspVersion::V1,
            flags: 0,
            message: active.clone(),
        }
        .to_bytes(&mut [0; 32]),
        Err(MspEncodeError::InvalidV1Command {
            command_id: MSP2_BATTERY_PROFILE,
        })
    );
    assert_packets_roundtrip_on_versions(
        &[MspVersion::V2, MspVersion::V2OverV1],
        &[
            (active, vec![]),
            (
                MspMessage::BatteryProfileRequest(MspBatteryProfileRequest {
                    profile_index: Some(0),
                }),
                vec![0],
            ),
            (
                MspMessage::BatteryProfile(MspBatteryProfile {
                    profile_index: 2,
                    min_cell_centivolts: 330,
                    max_cell_centivolts: 430,
                    warning_cell_centivolts: 350,
                    full_cell_centivolts: 410,
                    capacity_mah: 1500,
                    forced_cell_count: 4,
                    consumption_warning_percent: 20,
                }),
                vec![2, 0x4a, 1, 0xae, 1, 0x5e, 1, 0x9a, 1, 0xdc, 5, 4, 20],
            ),
        ],
    );
}
