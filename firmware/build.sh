#!/bin/bash
set -e

if [[ -z "$BOARD" ]]; then
    echo "Must provide BOARD (e.g. $ BOARD='colorlight_i5' ./build.sh)" 1>&2
    echo "Existing builds are" $(ls `pwd`/../build)
    exit 1
fi

export BUILD_DIR=`pwd`/../build/${BOARD}

FW_ROOT=`pwd`

OBJCOPY=${OBJCOPY:=riscv-none-elf-objcopy}

# Generate the Rust CSR bindings spat out by the LiteX SOC generated.
cd $FW_ROOT/deps/generated-litex-pac/src
svd2rust --log error -i $BUILD_DIR/csr.svd --target riscv

# Build the firmware .elf file
cd $FW_ROOT/litex-fw
cargo build -Zbuild-std=core --target=riscv32im-unknown-none-elf --release

# Copy it into a binary that litex_term can upload.
${OBJCOPY} target/riscv32im-unknown-none-elf/release/litex-fw -O binary $BUILD_DIR/rust-fw.bin
