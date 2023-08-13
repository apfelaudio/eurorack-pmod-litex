#!/bin/bash
set -e

export BUILD_DIR=`pwd`/../build/colorlight_i5

# DEFMT logging level is fixed at firmware compile time.
# You can use this to switch logging off completely.
export DEFMT_LOG=debug

FW_ROOT=`pwd`

# Generate the Rust CSR bindings spat out by the LiteX SOC generated.
cd $FW_ROOT/litex-pac/src
svd2rust -i $BUILD_DIR/csr.svd --target riscv

# Build the firmware .elf file
cd $FW_ROOT/litex-fw
cargo build --target=riscv32imac-unknown-none-elf --release

# Copy it into a binary that litex_term can upload.
riscv-none-elf-objcopy target/riscv32imac-unknown-none-elf/release/litex-fw -O binary $BUILD_DIR/rust-fw.bin
