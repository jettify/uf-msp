// Legacy MSP v1 command IDs (`docs/msp_protocol.h`).

pub const MSP_API_VERSION: u16 = 1;
pub const MSP_FC_VARIANT: u16 = 2;
pub const MSP_FC_VERSION: u16 = 3;
pub const MSP_BOARD_INFO: u16 = 4;
pub const MSP_BUILD_INFO: u16 = 5;

pub const MSP_NAME: u16 = 10;
pub const MSP_SET_NAME: u16 = 11;

// Cleanflight original features (32-62)
pub const MSP_BATTERY_CONFIG: u16 = 32;
pub const MSP_SET_BATTERY_CONFIG: u16 = 33;
pub const MSP_MODE_RANGES: u16 = 34;
pub const MSP_SET_MODE_RANGE: u16 = 35;
pub const MSP_FEATURE_CONFIG: u16 = 36;
pub const MSP_SET_FEATURE_CONFIG: u16 = 37;
pub const MSP_BOARD_ALIGNMENT_CONFIG: u16 = 38;
pub const MSP_SET_BOARD_ALIGNMENT_CONFIG: u16 = 39;
pub const MSP_CURRENT_METER_CONFIG: u16 = 40;
pub const MSP_SET_CURRENT_METER_CONFIG: u16 = 41;
pub const MSP_MIXER_CONFIG: u16 = 42;
pub const MSP_SET_MIXER_CONFIG: u16 = 43;
pub const MSP_RX_CONFIG: u16 = 44;
pub const MSP_SET_RX_CONFIG: u16 = 45;
pub const MSP_LED_COLORS: u16 = 46;
pub const MSP_SET_LED_COLORS: u16 = 47;
pub const MSP_LED_STRIP_CONFIG: u16 = 48;
pub const MSP_SET_LED_STRIP_CONFIG: u16 = 49;
pub const MSP_RSSI_CONFIG: u16 = 50;
pub const MSP_SET_RSSI_CONFIG: u16 = 51;
pub const MSP_ADJUSTMENT_RANGES: u16 = 52;
pub const MSP_SET_ADJUSTMENT_RANGE: u16 = 53;
pub const MSP_CF_SERIAL_CONFIG: u16 = 54;
pub const MSP_VOLTAGE_METER_CONFIG: u16 = 56;
pub const MSP_SET_VOLTAGE_METER_CONFIG: u16 = 57;
pub const MSP_SONAR_ALTITUDE: u16 = 58;
pub const MSP_PID_CONTROLLER: u16 = 59;
pub const MSP_SET_PID_CONTROLLER: u16 = 60;
pub const MSP_ARMING_CONFIG: u16 = 61;
pub const MSP_SET_ARMING_CONFIG: u16 = 62;

// Baseflight MSP commands (64-89)
pub const MSP_RX_MAP: u16 = 64;
pub const MSP_SET_RX_MAP: u16 = 65;
pub const MSP_REBOOT: u16 = 68;
pub const MSP_DATAFLASH_SUMMARY: u16 = 70;
pub const MSP_DATAFLASH_READ: u16 = 71;
pub const MSP_DATAFLASH_ERASE: u16 = 72;
pub const MSP_FAILSAFE_CONFIG: u16 = 75;
pub const MSP_SET_FAILSAFE_CONFIG: u16 = 76;
pub const MSP_RXFAIL_CONFIG: u16 = 77;
pub const MSP_SET_RXFAIL_CONFIG: u16 = 78;
pub const MSP_SDCARD_SUMMARY: u16 = 79;
pub const MSP_BLACKBOX_CONFIG: u16 = 80;
pub const MSP_SET_BLACKBOX_CONFIG: u16 = 81;
pub const MSP_TRANSPONDER_CONFIG: u16 = 82;
pub const MSP_SET_TRANSPONDER_CONFIG: u16 = 83;
pub const MSP_OSD_CONFIG: u16 = 84;
pub const MSP_SET_OSD_CONFIG: u16 = 85;
pub const MSP_OSD_CHAR_READ: u16 = 86;
pub const MSP_OSD_CHAR_WRITE: u16 = 87;
pub const MSP_VTX_CONFIG: u16 = 88;
pub const MSP_SET_VTX_CONFIG: u16 = 89;

// Betaflight Additional Commands (90-99)
pub const MSP_ADVANCED_CONFIG: u16 = 90;
pub const MSP_SET_ADVANCED_CONFIG: u16 = 91;
pub const MSP_FILTER_CONFIG: u16 = 92;
pub const MSP_SET_FILTER_CONFIG: u16 = 93;
pub const MSP_PID_ADVANCED: u16 = 94;
pub const MSP_SET_PID_ADVANCED: u16 = 95;
pub const MSP_SENSOR_CONFIG: u16 = 96;
pub const MSP_SET_SENSOR_CONFIG: u16 = 97;
pub const MSP_CAMERA_CONTROL: u16 = 98;
pub const MSP_SET_ARMING_DISABLED: u16 = 99;

// MultiWii original MSP commands (101-139)
pub const MSP_STATUS: u16 = 101;
/// Command ID for raw IMU telemetry and polling requests.
pub const MSP_RAW_IMU: u16 = 102;
pub const MSP_SERVO: u16 = 103;
pub const MSP_MOTOR: u16 = 104;
pub const MSP_RC: u16 = 105;
pub const MSP_RAW_GPS: u16 = 106;
pub const MSP_COMP_GPS: u16 = 107;
/// Command ID for attitude telemetry and polling requests.
pub const MSP_ATTITUDE: u16 = 108;
/// Command ID for altitude telemetry and polling requests.
pub const MSP_ALTITUDE: u16 = 109;
pub const MSP_ANALOG: u16 = 110;
pub const MSP_RC_TUNING: u16 = 111;
pub const MSP_PID: u16 = 112;
pub const MSP_BOXNAMES: u16 = 116;
pub const MSP_PIDNAMES: u16 = 117;
pub const MSP_WP: u16 = 118;
pub const MSP_BOXIDS: u16 = 119;
pub const MSP_SERVO_CONFIGURATIONS: u16 = 120;
pub const MSP_NAV_STATUS: u16 = 121;
pub const MSP_NAV_CONFIG: u16 = 122;
pub const MSP_MOTOR_3D_CONFIG: u16 = 124;
pub const MSP_RC_DEADBAND: u16 = 125;
pub const MSP_SENSOR_ALIGNMENT: u16 = 126;
pub const MSP_LED_STRIP_MODECOLOR: u16 = 127;
pub const MSP_VOLTAGE_METERS: u16 = 128;
pub const MSP_CURRENT_METERS: u16 = 129;
pub const MSP_BATTERY_STATE: u16 = 130;
pub const MSP_MOTOR_CONFIG: u16 = 131;
pub const MSP_GPS_CONFIG: u16 = 132;
pub const MSP_COMPASS_CONFIG: u16 = 133;
pub const MSP_ESC_SENSOR_DATA: u16 = 134;
pub const MSP_GPS_RESCUE: u16 = 135;
pub const MSP_GPS_RESCUE_PIDS: u16 = 136;
pub const MSP_VTXTABLE_BAND: u16 = 137;
pub const MSP_VTXTABLE_POWERLEVEL: u16 = 138;
pub const MSP_MOTOR_TELEMETRY: u16 = 139;

// Simplified tuning commands (140-145)
pub const MSP_SIMPLIFIED_TUNING: u16 = 140;
pub const MSP_SET_SIMPLIFIED_TUNING: u16 = 141;
pub const MSP_CALCULATE_SIMPLIFIED_PID: u16 = 142;
pub const MSP_CALCULATE_SIMPLIFIED_GYRO: u16 = 143;
pub const MSP_CALCULATE_SIMPLIFIED_DTERM: u16 = 144;
pub const MSP_VALIDATE_SIMPLIFIED_TUNING: u16 = 145;

// Additional non-MultiWii commands (150-170)
pub const MSP_STATUS_EX: u16 = 150;
pub const MSP_UID: u16 = 160;
pub const MSP_GPSSVINFO: u16 = 164;
pub const MSP_GPSSTATISTICS: u16 = 166;
pub const MSP_ATTITUDE_QUATERNION: u16 = 167;
pub const MSP_WING: u16 = 169;
pub const MSP_PITOT: u16 = 170;

// OSD specific commands (180-189)
pub const MSP_OSD_VIDEO_CONFIG: u16 = 180;
pub const MSP_SET_OSD_VIDEO_CONFIG: u16 = 181;
pub const MSP_DISPLAYPORT: u16 = 182;
pub const MSP_COPY_PROFILE: u16 = 183;
pub const MSP_BEEPER_CONFIG: u16 = 184;
pub const MSP_SET_BEEPER_CONFIG: u16 = 185;
pub const MSP_SET_TX_INFO: u16 = 186;
pub const MSP_TX_INFO: u16 = 187;
pub const MSP_SET_OSD_CANVAS: u16 = 188;
pub const MSP_OSD_CANVAS: u16 = 189;

// Set commands (200-228)
pub const MSP_SET_RAW_RC: u16 = 200;
pub const MSP_SET_RAW_GPS: u16 = 201;
pub const MSP_SET_PID: u16 = 202;
pub const MSP_SET_WING: u16 = 203;
pub const MSP_SET_RC_TUNING: u16 = 204;
pub const MSP_ACC_CALIBRATION: u16 = 205;
pub const MSP_MAG_CALIBRATION: u16 = 206;
pub const MSP_RESET_CONF: u16 = 208;
pub const MSP_SET_WP: u16 = 209;
pub const MSP_SELECT_SETTING: u16 = 210;
pub const MSP_SET_HEADING: u16 = 211;
pub const MSP_SET_SERVO_CONFIGURATION: u16 = 212;
pub const MSP_SET_MOTOR: u16 = 214;
pub const MSP_SET_NAV_CONFIG: u16 = 215;
pub const MSP_SET_MOTOR_3D_CONFIG: u16 = 217;
pub const MSP_SET_RC_DEADBAND: u16 = 218;
pub const MSP_SET_RESET_CURR_PID: u16 = 219;
pub const MSP_SET_SENSOR_ALIGNMENT: u16 = 220;
pub const MSP_SET_LED_STRIP_MODECOLOR: u16 = 221;
pub const MSP_SET_MOTOR_CONFIG: u16 = 222;
pub const MSP_SET_GPS_CONFIG: u16 = 223;
pub const MSP_SET_COMPASS_CONFIG: u16 = 224;
pub const MSP_SET_GPS_RESCUE: u16 = 225;
pub const MSP_SET_GPS_RESCUE_PIDS: u16 = 226;
pub const MSP_SET_VTXTABLE_BAND: u16 = 227;
pub const MSP_SET_VTXTABLE_POWERLEVEL: u16 = 228;

// Multiple MSP and special commands (230-255)
pub const MSP_MULTIPLE_MSP: u16 = 230;
pub const MSP_MODE_RANGES_EXTRA: u16 = 238;
pub const MSP_SET_ACC_TRIM: u16 = 239;
pub const MSP_ACC_TRIM: u16 = 240;
pub const MSP_SERVO_MIX_RULES: u16 = 241;
pub const MSP_SET_SERVO_MIX_RULE: u16 = 242;
pub const MSP_SET_PASSTHROUGH: u16 = 245;
pub const MSP_SET_RTC: u16 = 246;
pub const MSP_RTC: u16 = 247;
pub const MSP_SET_BOARD_INFO: u16 = 248;
pub const MSP_SET_SIGNATURE: u16 = 249;
pub const MSP_EEPROM_WRITE: u16 = 250;
pub const MSP_RESERVE_1: u16 = 251;
pub const MSP_RESERVE_2: u16 = 252;
pub const MSP_DEBUGMSG: u16 = 253;
pub const MSP_DEBUG: u16 = 254;
pub const MSP_V2_FRAME: u16 = 255;

// MSPv2 common commands (`docs/msp_protocol_v2_common.h`)
pub const MSP2_COMMON_SERIAL_CONFIG: u16 = 0x1009;
pub const MSP2_SENSOR_RANGEFINDER_LIDARMT: u16 = 0x1F01;
pub const MSP2_SENSOR_OPTICALFLOW_MT: u16 = 0x1F02;
pub const MSP2_SENSOR_GPS: u16 = 0x1F03;

// MSPv2 Betaflight commands (`docs/msp_protocol_v2_betaflight.h`)
pub const MSP2_BETAFLIGHT_BIND: u16 = 0x3000;
pub const MSP2_MOTOR_OUTPUT_REORDERING: u16 = 0x3001;
pub const MSP2_SET_MOTOR_OUTPUT_REORDERING: u16 = 0x3002;
pub const MSP2_SEND_DSHOT_COMMAND: u16 = 0x3003;
pub const MSP2_GET_VTX_DEVICE_STATUS: u16 = 0x3004;
pub const MSP2_GET_OSD_WARNINGS: u16 = 0x3005;
pub const MSP2_GET_TEXT: u16 = 0x3006;
pub const MSP2_SET_TEXT: u16 = 0x3007;
pub const MSP2_GET_LED_STRIP_CONFIG_VALUES: u16 = 0x3008;
pub const MSP2_SET_LED_STRIP_CONFIG_VALUES: u16 = 0x3009;
pub const MSP2_SENSOR_CONFIG_ACTIVE: u16 = 0x300A;
pub const MSP2_SENSOR_OPTICALFLOW: u16 = 0x300B;
pub const MSP2_MCU_INFO: u16 = 0x300C;
pub const MSP2_GYRO_SENSOR_ACTIVE: u16 = 0x300D;
pub const MSP2_BATTERY_PROFILE: u16 = 0x300E;
pub const MSP2_SET_BATTERY_PROFILE: u16 = 0x300F;
pub const MSP2_CLI_SETTING: u16 = 0x3010;
pub const MSP2_CLI_SETTING_INFO: u16 = 0x3011;
pub const MSP2_CLI_COMMAND: u16 = 0x3012;
