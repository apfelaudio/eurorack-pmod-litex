**WARN:** if you're just getting started with `eurorack-pmod` please [start here instead](https://github.com/apfelaudio/eurorack-pmod), this repository is a template for more advanced projects :)

# `eurorack-pmod` LiteX examples

Basic example of using `eurorack-pmod` inside a LiteX environment with firmware written in Rust.

It consists of a basic SoC that includes one instance of `eurorack-pmod` working alongside a soft RISCV CPU. Currently only tested on Colorlight i5, ECPIX-5 but should be quite easy to port to other ECP5 boards.

# Dependencies

For building the bitstream and LiteX BIOS:
- [oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build)
- [RISCV toolchain](https://xpack.github.io/dev-tools/riscv-none-elf-gcc/install/)

For building the firmware:
- Rust toolchain with some extras:
    - `rustup target add riscv32imac-unknown-elf`
    - `cargo install svd2rust`

The bitstream and firmware are built in CI inside `.github/workflows/main.yml`, it may be useful to look at if you are missing some dependencies.

# Building a bitstream


```
<from this repository>
git submodule update --init --recursive
# e.g. for Colorlight i5
python3 example-colorlight-i5.py --ecppack-compress --cpu-type vexriscv --cpu-variant imac --csr-svd build/colorlight_i5/csr.svd --build
# e.g. for ECPIX-5
python3 example-ecpix-5.py --ecppack-compress --cpu-type vexriscv --cpu-variant imac --csr-svd build/lambdaconcept_ecpix5/csr.svd --build
```

# Flashing the bitstream

```
# e.g. for Colorlight i5
openFPGALoader -b colorlight-i5 build/colorlight_i5/gateware/colorlight_i5.bit
# e.g. for ECPIX-5
openFPGALoader -b ecpix5 --vid 0x0403 --pid 0x6011 build/lambdaconcept_ecpix5/gateware/lambdaconcept_ecpix5.bit
```

# Building the firmware (Rust)

Note: the `eurorack-pmod` gateware will respond to inputs (i.e LEDs changing) even if no firmware is flashed to the softcore. So you can see if things are kind of working even without firmware.

To build the bindings for LiteX CSRs (`litex-pac`), compile the firmware, and copy it into a binary file ready for uploading with `litex_term` --

```
cd firmware/
# e.g. for Colorlight i5
BOARD=colorlight_i5 ./build.sh
# e.g. for ECPIX-5 with your own OBJCOPY binary --
BOARD=lambdaconcept_ecpix5 OBJCOPY=riscv64-elf-objcopy ./build.sh
```

To see the LiteX terminal and download firmware to the device (again replacing all instances of the board with your own type) --

```
<from repo root>
./bin/litex_term --kernel build/colorlight_i5/rust-fw.bin /dev/ttyACM0
```

Note -- firmware uses the `defmt` rust framework for logging such that we can print things quite a bit faster than ordinary UART/printf. Basically all strings are stripped from firmware on the device and the original .elf file is used to decode strings inside the `defmt` stream from the device. You must use `defmt-print` to reinterpret the UART stream into something you can read. To flash the firmware and see the output of the application you normally want something like:

```
./bin/litex_term --kernel build/colorlight_i5/rust-fw.bin /dev/ttyACM0 | defmt-print -e firmware/litex-fw/target/riscv32imac-unknown-none-elf/release/litex-fw
```

You can build your own `defmt-print` by cloning the `defmt` repository (see their documentation).
