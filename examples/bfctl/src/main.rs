mod client;

use std::io::IsTerminal;
use std::io::Read;
use std::io::Write;
use std::io::{self};
use std::process::ExitCode;
use std::thread;
use std::time::Duration;
use std::time::Instant;

use clap::Parser;
use clap::Subcommand;
use client::Client;
use client::Result;
use serialport::SerialPortType;
use serialport::UsbPortInfo;
use uf_msp::MspApiVersion;
use uf_msp::MspAttitude;
use uf_msp::MspBatteryState;
use uf_msp::MspBoardInfo;
use uf_msp::MspFcVersion;
use uf_msp::MspName;
use uf_msp::MspRc;
use uf_msp::MspRxMap;
use uf_msp::MspStatus;

#[derive(Parser)]
#[command(version, about = "Read-only Betaflight bench diagnostics over MSP")]
struct Args {
    /// Serial port path, for example /dev/cu.usbmodem101 or COM3
    #[arg(long, global = true)]
    port: Option<String>,

    /// Serial baud rate
    #[arg(long, global = true, default_value_t = 115_200)]
    baud: u32,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List serial ports and available USB product names
    Ports,
    /// Show the Betaflight version, craft name, and board target
    Info,
    /// Show sensors, loop timing, CPU load, and arming blockers
    Status,
    /// Show receiver channels in microseconds
    Rc {
        /// Update continuously and show session min/max values
        #[arg(long)]
        watch: bool,
    },
    /// Show roll, pitch, and heading in degrees
    Attitude {
        /// Update continuously
        #[arg(long)]
        watch: bool,
    },
    /// Show battery detection, voltage, and current
    Battery,
}

fn main() -> ExitCode {
    let args = Args::parse();
    let stdout = io::stdout();
    let terminal =
        stdout.is_terminal() && std::env::var_os("TERM").as_deref() != Some("dumb".as_ref());

    match run(&args, &mut stdout.lock(), terminal) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e)
            if e.downcast_ref::<io::Error>()
                .is_some_and(|e| e.kind() == io::ErrorKind::BrokenPipe) =>
        {
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("bfctl: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(args: &Args, out: &mut impl Write, terminal: bool) -> Result<()> {
    if matches!(args.command, Command::Ports) {
        return list_ports(out);
    }

    let path = args
        .port
        .as_deref()
        .ok_or("Specify --port <device>; run bfctl ports to list devices")?;
    let port = serialport::new(path, args.baud)
        .timeout(Duration::from_millis(50))
        .dtr_on_open(true)
        .open()
        .map_err(|e| format!("Could not open {path}: {e}"))?;

    let mut client = Client::new(port, Duration::from_secs(1));
    let api = client.identify()?;

    match args.command {
        Command::Ports => Ok(()),
        Command::Info => show_info(&mut client, api, out),
        Command::Status => show_status(&client.query()?, api, out),
        Command::Battery => show_battery(client.query()?, out),
        Command::Rc { watch } => show_rc(&mut client, out, watch, terminal),
        Command::Attitude { watch } => show_samples(out, watch, terminal, || {
            let att: MspAttitude = client.query()?;
            Ok(format!(
                "Roll {:+6.1}°  Pitch {:+6.1}°  Heading {:>3}°",
                f64::from(att.roll_decidegrees) / 10.0,
                f64::from(att.pitch_decidegrees) / 10.0,
                att.yaw_degrees,
            ))
        }),
    }
}

fn list_ports(out: &mut impl Write) -> Result<()> {
    let mut ports = serialport::available_ports()?;
    ports.sort_by(|a, b| a.port_name.cmp(&b.port_name));
    if ports.is_empty() {
        writeln!(out, "No serial ports found.")?;
    }
    for p in ports {
        match p.port_type {
            SerialPortType::UsbPort(UsbPortInfo {
                product: Some(prod),
                ..
            }) => {
                writeln!(out, "{}  {}", p.port_name, clean_str(&prod))?;
            }
            _ => writeln!(out, "{}", p.port_name)?,
        }
    }
    Ok(())
}

fn show_info(
    client: &mut Client<impl Read + Write>,
    api: MspApiVersion,
    out: &mut impl Write,
) -> Result<()> {
    let ver: MspFcVersion = client.query()?;
    let craft = client
        .optional::<MspName>()?
        .and_then(|c| non_empty(c.name()));
    let board: Option<MspBoardInfo> = client.optional()?;

    let target = board
        .as_ref()
        .and_then(MspBoardInfo::target_name)
        .and_then(non_empty);
    let board_id = board
        .as_ref()
        .and_then(|b| non_empty(&String::from_utf8_lossy(&b.identifier)));

    let fw = if (api.api_version_major, api.api_version_minor) >= (1, 47) {
        format!(
            "{}.{:02}.{}",
            u16::from(ver.major) + 2000,
            ver.minor,
            ver.patch
        )
    } else {
        format!("{}.{}.{}", ver.major, ver.minor, ver.patch)
    };

    writeln!(
        out,
        "Betaflight {fw}  (MSP API {}.{})",
        api.api_version_major, api.api_version_minor
    )?;
    if let Some(name) = craft {
        writeln!(out, "Craft name   {name}")?;
    }
    writeln!(
        out,
        "Board target {}",
        target
            .as_deref()
            .or(board_id.as_deref())
            .unwrap_or("Unavailable")
    )?;
    if let (Some(t), Some(id)) = (&target, &board_id)
        && t != id
    {
        writeln!(out, "Board ID     {id}")?;
    }
    Ok(())
}

fn clean_str(text: &str) -> String {
    text.trim_end_matches('\0')
        .chars()
        .filter(|c| !c.is_control())
        .collect()
}

fn non_empty(text: &str) -> Option<String> {
    let s = clean_str(text);
    (!s.is_empty()).then_some(s)
}

fn show_status(status: &MspStatus, api: MspApiVersion, out: &mut impl Write) -> Result<()> {
    const SENSORS: [&str; 8] = [
        "ACC",
        "BARO",
        "MAG",
        "GPS",
        "RANGEFINDER",
        "GYRO",
        "OPTICALFLOW",
        "PITOT",
    ];
    let sensors = format_bits(u32::from(status.sensors), |bit| SENSORS.get(bit).copied());

    writeln!(out, "Sensors          {sensors}")?;
    writeln!(out, "PID loop         {} µs", status.cycle_time_us)?;
    writeln!(out, "I2C errors       {}", status.i2c_error_count)?;
    writeln!(
        out,
        "PID profile      {}",
        u16::from(status.current_pid_profile_index) + 1
    )?;

    if let Some(ext) = &status.extended {
        writeln!(out, "CPU load         {}%", ext.average_system_load_percent)?;
        if let Some(temp) = ext.core_temp_celsius().filter(|&t| t != 0) {
            writeln!(out, "CPU temperature  {temp} °C")?;
        }
        let blockers = format_bits(ext.arming_disable_flags, |bit| {
            (bit < usize::from(ext.arming_disable_flags_count))
                .then(|| arming_label(api, bit))
                .flatten()
        });
        writeln!(out, "Arming blockers  {blockers}")?;
    } else {
        writeln!(out, "Arming blockers  Unavailable")?;
    }
    Ok(())
}

fn format_bits(mask: u32, label: impl Fn(usize) -> Option<&'static str>) -> String {
    if mask == 0 {
        return "None reported".to_owned();
    }
    (0..u32::BITS as usize)
        .filter(|&bit| mask & (1 << bit) != 0)
        .map(|bit| label(bit).map_or_else(|| format!("BIT_{bit}"), str::to_owned))
        .collect::<Vec<_>>()
        .join(", ")
}

fn arming_label(api: MspApiVersion, bit: usize) -> Option<&'static str> {
    const COMMON: [&str; 25] = [
        "NOGYRO",
        "FAILSAFE",
        "RXLOSS",
        "NOT_DISARMED",
        "BOXFAILSAFE",
        "RUNAWAY",
        "CRASH",
        "THROTTLE",
        "ANGLE",
        "BOOTGRACE",
        "NOPREARM",
        "LOAD",
        "CALIB",
        "CLI",
        "CMS",
        "BST",
        "MSP",
        "PARALYZE",
        "GPS",
        "RESCUE_SW",
        "DSHOT_TELEM",
        "REBOOT_REQD",
        "DSHOT_BBANG",
        "NO_ACC_CAL",
        "MOTOR_PROTO",
    ];
    let (1, minor @ 45..=49) = (api.api_version_major, api.api_version_minor) else {
        return None;
    };
    if bit == 20 && minor == 45 {
        return Some("RPMFILTER");
    }
    let tail: &[_] = match minor {
        45 | 46 => &["ARM_SWITCH"],
        47 => &["FLIP_SWITCH", "ALT_HOLD_SW", "POS_HOLD_SW", "ARM_SWITCH"],
        _ => &[
            "FLIP_SWITCH",
            "ALT_HOLD_SW",
            "POS_HOLD_SW",
            "AUTOPILOT_SW",
            "ARM_SWITCH",
        ],
    };
    COMMON.into_iter().chain(tail.iter().copied()).nth(bit)
}

fn show_battery(bat: MspBatteryState, out: &mut impl Write) -> Result<()> {
    if bat.cell_count == 0 {
        writeln!(out, "No battery detected.")?;
        return Ok(());
    }
    let volts = bat.voltage_centivolts.map_or_else(
        || f64::from(bat.voltage_decivolts) / 10.0,
        |v| f64::from(v) / 100.0,
    );
    let amps = f64::from(bat.current_centiamps) / 100.0;
    writeln!(
        out,
        "{}S  {volts:.2} V  {amps:.2} A  {} mAh consumed",
        bat.cell_count, bat.consumed_mah
    )?;
    Ok(())
}

fn input_order(map: MspRxMap) -> Option<String> {
    let mut labels = ['?'; 8];
    for (idx, ch) in map.input_indexes.into_iter().zip("AERT1234".chars()) {
        let slot = labels.get_mut(usize::from(idx))?;
        if *slot != '?' {
            return None;
        }
        *slot = ch;
    }
    Some(labels.into_iter().collect())
}

fn show_rc(
    client: &mut Client<impl Read + Write>,
    out: &mut impl Write,
    watch: bool,
    terminal: bool,
) -> Result<()> {
    if let Some(order) = client.optional::<MspRxMap>()?.and_then(input_order) {
        writeln!(out, "Receiver input order  {order}")?;
    }
    let mut ranges = [(u16::MAX, 0_u16); uf_msp::MAX_RC_CHANNELS];
    let sep = if watch && !terminal { " | " } else { "\n" };

    show_samples(out, watch, terminal, || {
        let rc: MspRc = client.query()?;
        if rc.channels().is_empty() {
            return Ok("No receiver channels reported.".to_owned());
        }
        let lines: Vec<_> = rc
            .channels()
            .iter()
            .zip(&mut ranges)
            .enumerate()
            .map(|(i, (&val, (min, max)))| {
                let name = ["Roll", "Pitch", "Yaw", "Throttle"]
                    .get(i)
                    .map_or_else(|| format!("AUX{}", i - 3), |&s| s.to_owned());
                if watch {
                    *min = (*min).min(val);
                    *max = (*max).max(val);
                    format!("{name:<8} {val:>4} µs  [{min}–{max}]")
                } else {
                    format!("{name:<8} {val:>4} µs")
                }
            })
            .collect();
        Ok(lines.join(sep))
    })
}

fn show_samples(
    out: &mut impl Write,
    watch: bool,
    terminal: bool,
    mut sample: impl FnMut() -> Result<String>,
) -> Result<()> {
    let mut prev_lines = 0;
    loop {
        let start = Instant::now();
        let display = sample()?;
        if prev_lines > 0 {
            write!(out, "\x1b[{prev_lines}A\r\x1b[J")?;
        }
        writeln!(out, "{display}")?;
        out.flush()?;
        if !watch {
            return Ok(());
        }
        if terminal {
            prev_lines = display.lines().count();
        }
        thread::sleep(Duration::from_millis(100).saturating_sub(start.elapsed()));
    }
}
