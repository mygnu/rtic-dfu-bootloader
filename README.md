# RTIC DFU Bootloader

This project is a Rust-based bootloader using RTIC (Real-Time Interrupt-driven Concurrency) and DFU (Device Firmware Upgrade) for embedded systems.
16KB of flash memory is reserved for the bootloader, which allows for firmware updates over USB.
tested on STM32F103C8T6 (Blue Pill) development board with 64KB of flash memory.

## Features

- **RTIC Framework**: Leverages the RTIC framework for real-time, concurrent, and interrupt-driven applications.
- **DFU Support**: Enables firmware updates over USB using the DFU protocol.
- **Embedded Systems**: Designed specifically for embedded systems with constrained resources.

## Getting Started

### Prerequisites

- Rust toolchain
- `cargo` package manager
- Embedded development tools (e.g., `probe-rs`)

### Installation

1. Clone the repository:

   ```sh
   git clone https://github.com/yourusername/rtic-dfu-bootloader.git
   cd rtic-dfu-bootloader
   ```

2. Build the project:

   ```sh
   cargo build --release
   ```

3. Flash the bootloader to your device:

   ```sh
   cargo flash --release
   ```

### Usage

1. Connect your device via USB.
2. Use a DFU utility to upload new firmware:

   ```sh
   dfu-util -D path/to/firmware.bin
   ```

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgements

- [RTIC Framework](https://rtic.rs/)
- [DFU Specification](https://www.usb.org/document-library/device-firmware-upgrade-version-11)
