# `eurorack-pmod` LiteX examples

Basic example of using `eurorack-pmod` inside a LiteX environment firmware written in Rust.

It consists of a basic SoC that includes one instance of `eurorack-pmod` working alongside a soft RISCV CPU. Currently only tested on Colorlight i5 but should be quite easy to port to other ECP5 boards.

# Dependencies

For building the bitstream and LiteX BIOS:
- [oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build)
- [RISCV toolchain](https://xpack.github.io/dev-tools/riscv-none-elf-gcc/install/)

For building the firmware:
- Rust toolchain with some extras:
    - `rustup target add riscv32imac-unknown-elf`
    - `cargo install svd2rust`
    - `cargo install cargo-binutils`

The bitstream and firmware are built in CI inside `.github/workflows/main.yml`, it may be useful to look at if you are missing some dependencies.

# Building a bitstream


```
<from this repository>
git submodule update --init --recursive
python3 example-colorlight-i5.py --ecppack-compress --cpu-type vexriscv --cpu-variant imac --csr-svd build/colorlight_i5/csr.svd --build
```

# Flashing the bitstream

```
openFPGALoader -b colorlight-i5 build/colorlight_i5/gateware/colorlight_i5.bit
```

# Building the firmware (Rust)

Note: the `eurorack-pmod` gateware will respond to inputs (i.e LEDs changing) even if no firmware is flashed to the softcore. So you can see if things are kind of working even without firmware.

To build the bindings for LiteX CSRs (`litex-pac`), compile the firmware, and copy it into a binary file ready for uploading with `litex_term` --

```
cd firmware/
./build.sh
```

To see the LiteX terminal and download firmware to the device --

```
<from repo root>
./bin/litex_term --kernel build/colorlight_i5/rust-fw.bin /dev/ttyACM0
```

Note -- firmware uses the `defmt` rust framework for logging such that we can print things quite a bit faster than ordinary UART/printf. Basically all strings are stripped from firmware on the device and the original .elf file is used to decode strings inside the `defmt` stream from the device. You must use `defmt-print` to reinterpret the UART stream into something you can read. To flash the firmware and see the output of the application you normally want something like:

```
./bin/litex_term --kernel build/colorlight_i5/rust-fw.bin /dev/ttyACM0 | defmt-print -e firmware/litex-fw/target/riscv32imac-unknown-none-elf/release/litex-fw
```

You can build your own `defmt-print` by cloning the `defmt` repository (see their documentation).
