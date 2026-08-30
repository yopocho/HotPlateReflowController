# Hot Plate Reflow Controller

[![Rust](https://img.shields.io/badge/Language-Rust-orange)](https://www.rust-lang.org/)
[![Embassy](https://img.shields.io/badge/Framework-Embassy-blueviolet)](https://embassy.dev/)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Development%20Paused-yellow)]()

A custom PCB-based reflow controller for surface-mount PCB assembly with thermocouple-based temperature feedback and embedded firmware control built with rust.

<div align="center">
  <img src="docs/images/pcb.jpg" width="75%" alt="Screenshot of PCB in Altium" />
</div>

## Overview

This repository contains the complete design and implementation of a hot plate reflow controller, including:
- **Firmware**: Embedded Rust implementation using the Embassy framework
- **Hardware**: PCB design files (Altium Designer) and electrical schematics
- **Mechanical**: Enclosure design for integration with heating element

### Target Hardware
- **Microcontroller**: STM32C071FBP6 (ARM Cortex-M0+)
- **Temperature Sensing**: K-type thermocouple input with signal conditioning
- **Power Stage**: In-series control of mains power (230V, 4A max)
- **Heating Element**: Electric flat-top griddle (900W max)

## Features

- OLED display with EC11 encoder-based menu UI
- K-Type thermocouple temperature feedback
- Configurable target temperature setpoint
- Automatic reflow profile following (TS319SNL and GC10 solder paste curve)
- Simple temperature measurement mode

## Project Structure

```
.
├── src/
│   ├── main.rs              # Main firmware entry point
│   ├── reflow_profiles.rs   # Reflow profile settings
│   └── rotary_encoder.rs     # Rotary encoder decoder
├── hardware/                # PCB design files (Altium) & enclosure design
├── docs/                    # Documentation
│   ├── images/              # Images for docs
│   ├── ERRORS_REFERENCE.md  # Reference for error codes
│   └── TODO.md              # Known issues and TODOs
├── Cargo.toml               # Rust project manifest
├── Cargo.lock               # Locked dependency versions
├── build.rs                 # Build script
├── rust-toolchain.toml      # Pinned Rust version
└── README.md
```

## Firmware

### Prerequisites
- Rust toolchain (cargo 1.90.0)
- `Cortex-M0+` target: `rustup target add thumbv6m-none-eabi`
- `probe-rs` for flashing: `cargo install probe-rs-tools --locked`

### Building

```bash
cargo build --release
```

### Flashing

With attaching debugger:
```bash
cargo run --release
```

Without attaching debugger:
```bash
cargo flash --chip STM32C071FBPx --release
```

### Debugging

RTT with `probe-rs` while device is running:
```bash
probe-rs attach --chip STM32C071FBPx target\thumbv6m-none-eabi\release\HotPlateReflowController
```

## Hardware

### PCB & Enclosure
- **Design Tool**: Altium Designer
- **Files**: Located in `hardware/` directory
- **MCU**: STM32C071FBP6 (ARM Cortex-M0+)
- **Mains Interface**: TRIAC-based control for 230V AC switching
- **Current Rating**: 4A nominal (900W @ 230V)
- **Temperature Feedback**: Thermocouple signal conditioning and ADC interface

### Manufacturing
Designed for JLCPCB standard (most economical) manufacturing specifications.

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) file for details.

## References

- [STM32C071FB Datasheet](https://www.st.com/content/st_com/en.html)
- [Embassy Framework Documentation](https://embassy.dev/)
- [Statig FSM Crate](https://crates.io/crates/statig/0.4.1)

- See [TODO.md](docs/TODO.md) for known issues in hardware and software

- See [ERRORS_REFERENCE.md](docs/ERRORS_REFERENCE.md) for matching error codes to messages

---

**Status**: Development Paused

**Last Updated**: August 2026
