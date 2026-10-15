# uf-msp

[![CI](https://github.com/jettify/uf-msp/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-msp/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-msp/graph/badge.svg?token=2N16CN1OZX)](https://codecov.io/gh/jettify/uf-msp)
[![crates.io](https://img.shields.io/crates/v/uf-msp)](https://crates.io/crates/uf-msp)
[![docs.rs](https://img.shields.io/docsrs/uf-msp)](https://docs.rs/uf-msp/latest/uf_msp/)

A `no_std`, allocation-free Rust library for MSP framing.

## Implementation status

**Legend:**

- `🟢` - Implemented
- `🟡` - Partial implementation or known protocol compatibility gaps
- `🔴` - Not Implemented


| Packet Name | Command ID | Status | Notes |
| :--- | :--- | :--- | :--- |
| **Legacy MSP commands** |  |  |  |
| `MSP_API_VERSION` | `0x01` | 🔴 | Get API version |
| `MSP_FC_VARIANT` | `0x02` | 🔴 | Get flight controller variant |
| `MSP_FC_VERSION` | `0x03` | 🔴 | Get flight controller version |
| `MSP_BOARD_INFO` | `0x04` | 🔴 | Get board information |
| `MSP_BUILD_INFO` | `0x05` | 🔴 | Get build information |
| `MSP_NAME` | `0x0A` | 🔴 | Returns user set board name - betaflight |
| `MSP_SET_NAME` | `0x0B` | 🔴 | Sets board name - betaflight |
| `MSP_BATTERY_CONFIG` | `0x20` | 🔴 | Get battery configuration |
| `MSP_SET_BATTERY_CONFIG` | `0x21` | 🔴 | Set battery configuration |
| `MSP_MODE_RANGES` | `0x22` | 🔴 | Returns all mode ranges |
| `MSP_SET_MODE_RANGE` | `0x23` | 🔴 | Sets a single mode range |
| `MSP_FEATURE_CONFIG` | `0x24` | 🔴 | Get feature configuration |
| `MSP_SET_FEATURE_CONFIG` | `0x25` | 🔴 | Set feature configuration |
| `MSP_BOARD_ALIGNMENT_CONFIG` | `0x26` | 🔴 | Get board alignment configuration |
| `MSP_SET_BOARD_ALIGNMENT_CONFIG` | `0x27` | 🔴 | Set board alignment configuration |
| `MSP_CURRENT_METER_CONFIG` | `0x28` | 🔴 | Get current meter configuration |
| `MSP_SET_CURRENT_METER_CONFIG` | `0x29` | 🔴 | Set current meter configuration |
| `MSP_MIXER_CONFIG` | `0x2A` | 🔴 | Get mixer configuration |
| `MSP_SET_MIXER_CONFIG` | `0x2B` | 🔴 | Set mixer configuration |
| `MSP_RX_CONFIG` | `0x2C` | 🔴 | Get RX configuration |
| `MSP_SET_RX_CONFIG` | `0x2D` | 🔴 | Set RX configuration |
| `MSP_LED_COLORS` | `0x2E` | 🔴 | Get LED colors |
| `MSP_SET_LED_COLORS` | `0x2F` | 🔴 | Set LED colors |
| `MSP_LED_STRIP_CONFIG` | `0x30` | 🔴 | Get LED strip configuration |
| `MSP_SET_LED_STRIP_CONFIG` | `0x31` | 🔴 | Set LED strip configuration |
| `MSP_RSSI_CONFIG` | `0x32` | 🔴 | Get RSSI configuration |
| `MSP_SET_RSSI_CONFIG` | `0x33` | 🔴 | Set RSSI configuration |
| `MSP_ADJUSTMENT_RANGES` | `0x34` | 🔴 | Get adjustment ranges |
| `MSP_SET_ADJUSTMENT_RANGE` | `0x35` | 🔴 | Set adjustment range |
| `MSP_CF_SERIAL_CONFIG` | `0x36` | 🔴 | Get Cleanflight serial configuration |
| `MSP_VOLTAGE_METER_CONFIG` | `0x38` | 🔴 | Get voltage meter configuration |
| `MSP_SET_VOLTAGE_METER_CONFIG` | `0x39` | 🔴 | Set voltage meter configuration |
| `MSP_SONAR_ALTITUDE` | `0x3A` | 🔴 | Get sonar altitude [cm] |
| `MSP_PID_CONTROLLER` | `0x3B` | 🔴 | Get PID controller |
| `MSP_SET_PID_CONTROLLER` | `0x3C` | 🔴 | Set PID controller |
| `MSP_ARMING_CONFIG` | `0x3D` | 🔴 | Get arming configuration |
| `MSP_SET_ARMING_CONFIG` | `0x3E` | 🔴 | Set arming configuration |
| `MSP_RX_MAP` | `0x40` | 🔴 | Get RX map (also returns number of channels total) |
| `MSP_SET_RX_MAP` | `0x41` | 🔴 | Set RX map, numchannels to set comes from MSP_RX_MAP |
| `MSP_REBOOT` | `0x44` | 🔴 | Reboot settings |
| `MSP_DATAFLASH_SUMMARY` | `0x46` | 🔴 | Get description of dataflash chip |
| `MSP_DATAFLASH_READ` | `0x47` | 🔴 | Get content of dataflash chip |
| `MSP_DATAFLASH_ERASE` | `0x48` | 🔴 | Erase dataflash chip |
| `MSP_FAILSAFE_CONFIG` | `0x4B` | 🔴 | Get failsafe settings |
| `MSP_SET_FAILSAFE_CONFIG` | `0x4C` | 🔴 | Set failsafe settings |
| `MSP_RXFAIL_CONFIG` | `0x4D` | 🔴 | Get RX failsafe settings |
| `MSP_SET_RXFAIL_CONFIG` | `0x4E` | 🔴 | Set RX failsafe settings |
| `MSP_SDCARD_SUMMARY` | `0x4F` | 🔴 | Get SD card state |
| `MSP_BLACKBOX_CONFIG` | `0x50` | 🔴 | Get blackbox settings |
| `MSP_SET_BLACKBOX_CONFIG` | `0x51` | 🔴 | Set blackbox settings |
| `MSP_TRANSPONDER_CONFIG` | `0x52` | 🔴 | Get transponder settings |
| `MSP_SET_TRANSPONDER_CONFIG` | `0x53` | 🔴 | Set transponder settings |
| `MSP_OSD_CONFIG` | `0x54` | 🔴 | Get OSD settings |
| `MSP_SET_OSD_CONFIG` | `0x55` | 🔴 | Set OSD settings |
| `MSP_OSD_CHAR_READ` | `0x56` | 🔴 | Get OSD characters |
| `MSP_OSD_CHAR_WRITE` | `0x57` | 🔴 | Set OSD characters |
| `MSP_VTX_CONFIG` | `0x58` | 🔴 | Get VTX settings |
| `MSP_SET_VTX_CONFIG` | `0x59` | 🔴 | Set VTX settings |
| `MSP_ADVANCED_CONFIG` | `0x5A` | 🔴 | Get advanced configuration |
| `MSP_SET_ADVANCED_CONFIG` | `0x5B` | 🔴 | Set advanced configuration |
| `MSP_FILTER_CONFIG` | `0x5C` | 🔴 | Get filter configuration |
| `MSP_SET_FILTER_CONFIG` | `0x5D` | 🔴 | Set filter configuration |
| `MSP_PID_ADVANCED` | `0x5E` | 🔴 | Get advanced PID settings |
| `MSP_SET_PID_ADVANCED` | `0x5F` | 🔴 | Set advanced PID settings |
| `MSP_SENSOR_CONFIG` | `0x60` | 🔴 | Get sensor configuration |
| `MSP_SET_SENSOR_CONFIG` | `0x61` | 🔴 | Set sensor configuration |
| `MSP_CAMERA_CONTROL` | `0x62` | 🔴 | Camera control |
| `MSP_SET_ARMING_DISABLED` | `0x63` | 🔴 | Enable/disable arming |
| `MSP_STATUS` | `0x65` | 🔴 | Cycletime & errors_count & sensor present & box activation & current setting number |
| `MSP_RAW_IMU` | `0x66` | 🔴 | 9 DOF |
| `MSP_SERVO` | `0x67` | 🔴 | Servos |
| `MSP_MOTOR` | `0x68` | 🔴 | Motors |
| `MSP_RC` | `0x69` | 🔴 | RC channels and more |
| `MSP_RAW_GPS` | `0x6A` | 🔴 | Fix, numsat, lat, lon, alt, speed, ground course |
| `MSP_COMP_GPS` | `0x6B` | 🔴 | Distance home, direction home |
| `MSP_ATTITUDE` | `0x6C` | 🔴 | 2 angles 1 heading |
| `MSP_ALTITUDE` | `0x6D` | 🔴 | Altitude, variometer |
| `MSP_ANALOG` | `0x6E` | 🔴 | Vbat, powermetersum, rssi if available on RX |
| `MSP_RC_TUNING` | `0x6F` | 🔴 | RC rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID |
| `MSP_PID` | `0x70` | 🔴 | P I D coeff (9 are used currently) |
| `MSP_BOXNAMES` | `0x74` | 🔴 | The aux switch names |
| `MSP_PIDNAMES` | `0x75` | 🔴 | The PID names |
| `MSP_WP` | `0x76` | 🔴 | Get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold |
| `MSP_BOXIDS` | `0x77` | 🔴 | Get the permanent IDs associated to BOXes |
| `MSP_SERVO_CONFIGURATIONS` | `0x78` | 🔴 | All servo configurations |
| `MSP_NAV_STATUS` | `0x79` | 🔴 | Returns navigation status |
| `MSP_NAV_CONFIG` | `0x7A` | 🔴 | Returns navigation parameters |
| `MSP_MOTOR_3D_CONFIG` | `0x7C` | 🔴 | Settings needed for reversible ESCs |
| `MSP_RC_DEADBAND` | `0x7D` | 🔴 | Deadbands for yaw alt pitch roll |
| `MSP_SENSOR_ALIGNMENT` | `0x7E` | 🔴 | Orientation of acc,gyro,mag |
| `MSP_LED_STRIP_MODECOLOR` | `0x7F` | 🔴 | Get LED strip mode_color settings |
| `MSP_VOLTAGE_METERS` | `0x80` | 🔴 | Voltage (per meter) |
| `MSP_CURRENT_METERS` | `0x81` | 🔴 | Amperage (per meter) |
| `MSP_BATTERY_STATE` | `0x82` | 🔴 | Connected/Disconnected, Voltage, Current Used |
| `MSP_MOTOR_CONFIG` | `0x83` | 🔴 | Motor configuration (min/max throttle, etc) |
| `MSP_GPS_CONFIG` | `0x84` | 🔴 | GPS configuration |
| `MSP_COMPASS_CONFIG` | `0x85` | 🔴 | Compass configuration |
| `MSP_ESC_SENSOR_DATA` | `0x86` | 🔴 | Extra ESC data from 32-Bit ESCs (Temperature, RPM) |
| `MSP_GPS_RESCUE` | `0x87` | 🔴 | GPS Rescue angle, returnAltitude, descentDistance, groundSpeed, sanityChecks and minSats |
| `MSP_GPS_RESCUE_PIDS` | `0x88` | 🔴 | GPS Rescue throttleP and velocity PIDS + yaw P |
| `MSP_VTXTABLE_BAND` | `0x89` | 🔴 | VTX table band/channel data |
| `MSP_VTXTABLE_POWERLEVEL` | `0x8A` | 🔴 | VTX table powerLevel data |
| `MSP_MOTOR_TELEMETRY` | `0x8B` | 🔴 | Per-motor telemetry data (RPM, packet stats, ESC temp, etc.) |
| `MSP_SIMPLIFIED_TUNING` | `0x8C` | 🔴 | Get simplified tuning values and enabled state |
| `MSP_SET_SIMPLIFIED_TUNING` | `0x8D` | 🔴 | Set simplified tuning positions and apply calculated tuning |
| `MSP_CALCULATE_SIMPLIFIED_PID` | `0x8E` | 🔴 | Calculate PID values based on sliders without saving |
| `MSP_CALCULATE_SIMPLIFIED_GYRO` | `0x8F` | 🔴 | Calculate gyro filter values based on sliders without saving |
| `MSP_CALCULATE_SIMPLIFIED_DTERM` | `0x90` | 🔴 | Calculate D term filter values based on sliders without saving |
| `MSP_VALIDATE_SIMPLIFIED_TUNING` | `0x91` | 🔴 | Returns array of true/false showing which simplified tuning groups match values |
| `MSP_STATUS_EX` | `0x96` | 🔴 | Cycletime, errors_count, CPU load, sensor present etc |
| `MSP_UID` | `0xA0` | 🔴 | Unique device ID |
| `MSP_GPSSVINFO` | `0xA4` | 🔴 | Get Signal Strength (only U-Blox) |
| `MSP_GPSSTATISTICS` | `0xA6` | 🔴 | Get GPS debugging data |
| `MSP_ATTITUDE_QUATERNION` | `0xA7` | 🔴 | Orientation quaternion components (w, x, y, z) |
| `MSP_WING` | `0xA9` | 🔴 | WING settings |
| `MSP_PITOT` | `0xAA` | 🔴 | Pitot data |
| `MSP_OSD_VIDEO_CONFIG` | `0xB4` | 🔴 | Get OSD video settings |
| `MSP_SET_OSD_VIDEO_CONFIG` | `0xB5` | 🔴 | Set OSD video settings |
| `MSP_DISPLAYPORT` | `0xB6` | 🔴 | External OSD displayport mode |
| `MSP_COPY_PROFILE` | `0xB7` | 🔴 | Copy settings between profiles |
| `MSP_BEEPER_CONFIG` | `0xB8` | 🔴 | Get beeper configuration |
| `MSP_SET_BEEPER_CONFIG` | `0xB9` | 🔴 | Set beeper configuration |
| `MSP_SET_TX_INFO` | `0xBA` | 🔴 | Set runtime information from TX lua scripts |
| `MSP_TX_INFO` | `0xBB` | 🔴 | Get runtime information for TX lua scripts |
| `MSP_SET_OSD_CANVAS` | `0xBC` | 🔴 | Set OSD canvas size COLSxROWS |
| `MSP_OSD_CANVAS` | `0xBD` | 🔴 | Get OSD canvas size COLSxROWS |
| `MSP_SET_RAW_RC` | `0xC8` | 🔴 | 8 rc chan |
| `MSP_SET_RAW_GPS` | `0xC9` | 🔴 | Fix, numsat, lat, lon, alt, speed |
| `MSP_SET_PID` | `0xCA` | 🔴 | P I D coeff (9 are used currently) |
| `MSP_SET_WING` | `0xCB` | 🔴 | Set WING settings |
| `MSP_SET_RC_TUNING` | `0xCC` | 🔴 | RC rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID, yaw expo |
| `MSP_ACC_CALIBRATION` | `0xCD` | 🔴 | No param - calibrate accelerometer |
| `MSP_MAG_CALIBRATION` | `0xCE` | 🔴 | No param - calibrate magnetometer |
| `MSP_RESET_CONF` | `0xD0` | 🔴 | No param - reset settings |
| `MSP_SET_WP` | `0xD1` | 🔴 | Sets a given WP (WP#,lat, lon, alt, flags) |
| `MSP_SELECT_SETTING` | `0xD2` | 🔴 | Select setting number (0-2) |
| `MSP_SET_HEADING` | `0xD3` | 🔴 | Define a new heading hold direction |
| `MSP_SET_SERVO_CONFIGURATION` | `0xD4` | 🔴 | Servo settings |
| `MSP_SET_MOTOR` | `0xD6` | 🔴 | PropBalance function |
| `MSP_SET_NAV_CONFIG` | `0xD7` | 🔴 | Sets nav config parameters |
| `MSP_SET_MOTOR_3D_CONFIG` | `0xD9` | 🔴 | Settings needed for reversible ESCs |
| `MSP_SET_RC_DEADBAND` | `0xDA` | 🔴 | Deadbands for yaw alt pitch roll |
| `MSP_SET_RESET_CURR_PID` | `0xDB` | 🔴 | Reset current PID profile to defaults |
| `MSP_SET_SENSOR_ALIGNMENT` | `0xDC` | 🔴 | Set the orientation of acc,gyro,mag |
| `MSP_SET_LED_STRIP_MODECOLOR` | `0xDD` | 🔴 | Set LED strip mode_color settings |
| `MSP_SET_MOTOR_CONFIG` | `0xDE` | 🔴 | Motor configuration (min/max throttle, etc) |
| `MSP_SET_GPS_CONFIG` | `0xDF` | 🔴 | GPS configuration |
| `MSP_SET_COMPASS_CONFIG` | `0xE0` | 🔴 | Compass configuration |
| `MSP_SET_GPS_RESCUE` | `0xE1` | 🔴 | Set GPS Rescue parameters |
| `MSP_SET_GPS_RESCUE_PIDS` | `0xE2` | 🔴 | Set GPS Rescue PID values |
| `MSP_SET_VTXTABLE_BAND` | `0xE3` | 🔴 | Set vtxTable band/channel data |
| `MSP_SET_VTXTABLE_POWERLEVEL` | `0xE4` | 🔴 | Set vtxTable powerLevel data |
| `MSP_MULTIPLE_MSP` | `0xE6` | 🔴 | Request multiple MSPs in one request |
| `MSP_MODE_RANGES_EXTRA` | `0xEE` | 🔴 | Extra mode range data |
| `MSP_SET_ACC_TRIM` | `0xEF` | 🔴 | Set acc angle trim values |
| `MSP_ACC_TRIM` | `0xF0` | 🔴 | Get acc angle trim values |
| `MSP_SERVO_MIX_RULES` | `0xF1` | 🔴 | Get servo mixer configuration |
| `MSP_SET_SERVO_MIX_RULE` | `0xF2` | 🔴 | Set servo mixer configuration |
| `MSP_SET_PASSTHROUGH` | `0xF5` | 🔴 | Set passthrough to peripherals |
| `MSP_SET_RTC` | `0xF6` | 🔴 | Set the RTC clock |
| `MSP_RTC` | `0xF7` | 🔴 | Get the RTC clock |
| `MSP_SET_BOARD_INFO` | `0xF8` | 🔴 | Set the board information |
| `MSP_SET_SIGNATURE` | `0xF9` | 🔴 | Set the signature of the board and serial number |
| `MSP_EEPROM_WRITE` | `0xFA` | 🔴 | Write settings to EEPROM |
| `MSP_DEBUGMSG` | `0xFD` | 🔴 | debug string buffer |
| `MSP_DEBUG` | `0xFE` | 🔴 | debug1,debug2,debug3,debug4 |
| **MSPv2 common commands** |  |  |  |
| `MSP2_COMMON_SERIAL_CONFIG` | `0x1009` | 🔴 |  |
| `MSP2_SENSOR_GPS` | `0x1F03` | 🔴 |  |
| `MSP2_SENSOR_RANGEFINDER_LIDARMT` | `0x1F01` | 🔴 |  |
| `MSP2_SENSOR_OPTICALFLOW_MT` | `0x1F02` | 🔴 |  |
| **MSPv2 Betaflight commands** |  |  |  |
| `MSP2_BETAFLIGHT_BIND` | `0x3000` | 🔴 |  |
| `MSP2_MOTOR_OUTPUT_REORDERING` | `0x3001` | 🔴 |  |
| `MSP2_SET_MOTOR_OUTPUT_REORDERING` | `0x3002` | 🔴 |  |
| `MSP2_SEND_DSHOT_COMMAND` | `0x3003` | 🔴 |  |
| `MSP2_GET_VTX_DEVICE_STATUS` | `0x3004` | 🔴 |  |
| `MSP2_GET_OSD_WARNINGS` | `0x3005` | 🔴 | returns active OSD warning message text |
| `MSP2_GET_TEXT` | `0x3006` | 🔴 |  |
| `MSP2_SET_TEXT` | `0x3007` | 🔴 |  |
| `MSP2_GET_LED_STRIP_CONFIG_VALUES` | `0x3008` | 🔴 |  |
| `MSP2_SET_LED_STRIP_CONFIG_VALUES` | `0x3009` | 🔴 |  |
| `MSP2_SENSOR_CONFIG_ACTIVE` | `0x300A` | 🔴 |  |
| `MSP2_SENSOR_OPTICALFLOW` | `0x300B` | 🔴 |  |
| `MSP2_MCU_INFO` | `0x300C` | 🔴 |  |
| `MSP2_GYRO_SENSOR_ACTIVE` | `0x300D` | 🔴 |  |
| `MSP2_BATTERY_PROFILE` | `0x300E` | 🔴 |  |
| `MSP2_SET_BATTERY_PROFILE` | `0x300F` | 🔴 |  |
| `MSP2_CLI_SETTING` | `0x3010` | 🔴 |  |
| `MSP2_CLI_SETTING_INFO` | `0x3011` | 🔴 |  |
| `MSP2_CLI_COMMAND` | `0x3012` | 🔴 |  |
