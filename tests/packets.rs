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
                let bytes = wire(version, direction, message.command_id(), payload);
                let expected = MspPacket {
                    version,
                    flags: if version == MspVersion::V1 { 0 } else { 0xa5 },
                    message,
                };
                assert_eq!(expected.command_id(), message.command_id());
                assert_eq!(
                    expected.direction(),
                    if direction == b'<' {
                        MspDirection::Request
                    } else {
                        MspDirection::Response
                    }
                );
                assert_eq!(
                    MspPacket::try_from(MspFrame::try_from(bytes.as_slice()).unwrap()),
                    Ok(expected)
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
                        assert!(input.is_empty());
                    }
                    assert_eq!(packets, [expected]);
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
                assert_eq!(packets, [Ok(packet)]);
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
            assert!(input.is_empty());
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
        assert_eq!(packets[0].unwrap().message, MspMessage::AltitudeRequest);
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
    assert!(input.is_empty());
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
