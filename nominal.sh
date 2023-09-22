#!/bin/bash
set -e

# SVD only
echo "Generate SVD"
./sim.py --integrated-main-ram-size=0x10000 --cpu-type vexriscv --cpu-variant imac --csr-svd build/sim/csr.svd --no-compile-gateware
# FW only
echo "Build FW"
cd firmware
OBJCOPY=riscv64-linux-gnu-objcopy BOARD=sim ./build.sh
cd ..
# Boot firmware after BIOS
echo "Simulate GW+FW"
./sim.py --integrated-main-ram-size=0x10000 --ram-init build/sim/rust-fw.bin --cpu-type vexriscv --cpu-variant imac
