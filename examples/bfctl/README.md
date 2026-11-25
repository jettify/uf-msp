# bfctl

A read-only Betaflight bench CLI using `uf-msp`. Connect the flight controller over USB and close Betaflight Configurator to free the serial port. On Linux, building with the default serial backend requires the `libudev` development package.

From the repository root, list the available ports.

```sh
cargo run --manifest-path examples/bfctl/Cargo.toml -- ports
```

Use the path from that list for `--port`. Run these commands from `examples/bfctl`.

```sh
cargo run -- --port /dev/cu.usbmodem0x80000001 info
cargo run -- --port /dev/cu.usbmodem0x80000001 status
cargo run -- --port /dev/cu.usbmodem0x80000001 rc --watch
cargo run -- --port /dev/cu.usbmodem0x80000001 attitude --watch
cargo run -- --port /dev/cu.usbmodem0x80000001 battery
```

- `info` shows firmware and API versions, the craft name, and board target when available.
- `status` shows detected sensors, PID loop time, I2C errors, CPU load and temperature when available, and reported arming blockers. Unknown flags appear as `BIT_n` with a zero-based bit number.
- `rc` shows the configured input order when available and live logical channel values in microseconds. Watch mode tracks each channel’s session minimum and maximum.
- `attitude` shows roll, pitch, and heading in degrees.
- `battery` shows pack voltage, cell count, current, and consumed capacity when a pack is detected.

Press Ctrl+C to stop a watch. The serial baud rate defaults to 115200 and can be set with `--baud`.
