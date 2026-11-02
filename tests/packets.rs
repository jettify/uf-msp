use uf_msp::MspAltitude;
use uf_msp::MspAttitude;
use uf_msp::MspDecodeError;
use uf_msp::MspDirection;
use uf_msp::MspFrame;
use uf_msp::MspMessage;
use uf_msp::MspPacket;
use uf_msp::MspPacketError;
use uf_msp::MspParser;
use uf_msp::MspRawImu;
use uf_msp::MspVersion;
use uf_msp::crc8_dvbs2_slice;

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
                expected.push(Err(MspPacketError::Decode(
                    MspDecodeError::PayloadTooShort {
                        command_id,
                        required: payload.len(),
                        actual: length,
                    },
                )));
            }
            stream.extend(wire(version, b'!', command_id, &[]));
            expected.push(Err(MspPacketError::Decode(MspDecodeError::RemoteError {
                command_id,
            })));
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
                Some(Err(MspPacketError::Decode(
                    MspDecodeError::UnsupportedCommand { command_id: 42 }
                )))
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
fn errors_implement_display_and_source() {
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
        let wrapped = MspPacketError::Decode(error);
        assert_eq!(wrapped.to_string(), expected);
        assert_eq!(wrapped.source().unwrap().to_string(), expected);
    }
    let error = MspPacketError::Parser(uf_msp::MspParserError::InvalidBufferState);
    assert_eq!(error.source().unwrap().to_string(), error.to_string());
}
use uf_msp::MSP_ALTITUDE;
use uf_msp::MSP_ATTITUDE;
use uf_msp::MSP_RAW_IMU;

#[test]
fn packet_errors_support_question_mark_in_manual_decoding() {
    fn decode(
        parser: &mut MspParser<64>,
        input: &mut &[u8],
    ) -> Result<Option<MspPacket>, MspPacketError> {
        let Some(frame) = parser.next(input)? else {
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
        Err(MspPacketError::Decode(MspDecodeError::PayloadTooShort {
            command_id: MSP_ATTITUDE,
            required: 6,
            actual: 0,
        }))
    );
    let parser_error = uf_msp::MspParserError::InvalidBufferState;
    assert_eq!(
        MspPacketError::from(parser_error),
        MspPacketError::Parser(parser_error)
    );
}

#[test]
fn public_command_constants_match_wire_ids() {
    assert_eq!(MSP_RAW_IMU, 102);
    assert_eq!(MSP_ATTITUDE, 108);
    assert_eq!(MSP_ALTITUDE, 109);
    assert_eq!(MspMessage::RawImuRequest.command_id(), MSP_RAW_IMU);
    assert_eq!(MspMessage::AttitudeRequest.command_id(), MSP_ATTITUDE);
    assert_eq!(MspMessage::AltitudeRequest.command_id(), MSP_ALTITUDE);
}
