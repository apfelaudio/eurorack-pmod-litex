# Build Rust FW
OBJCOPY=riscv64-elf-objcopy BOARD=sim ./build.sh
# SVD only
./sim.py --integrated-main-ram-size=0x10000 --cpu-type vexriscv --cpu-variant imac --csr-svd build/sim/csr.svd --no-compile-gateware
# BIOS only
./sim.py --integrated-main-ram-size=0x10000 --cpu-type vexriscv --cpu-variant imac
# BIOS with FST tracing
./sim.py --integrated-main-ram-size=0x10000 --cpu-type vexriscv --cpu-variant imac --gtkwave-savefile --trace --trace-fst
# Boot firmware after BIOS
./sim.py --integrated-main-ram-size=0x10000 --ram-init build/sim/rust-fw.bin --cpu-type vexriscv --cpu-variant imac
# Boot firmware after BIOS w/ tracing
./sim.py --integrated-main-ram-size=0x10000 --ram-init build/sim/rust-fw.bin --cpu-type vexriscv --cpu-variant imac  --gtkwave-savefile --trace --trace-fst

# --threads 4 for speedup?
./sim.py --integrated-main-ram-size=0x100000 --ram-init build/sim/rust-fw.bin --cpu-type vexriscv --cpu-variant imac --timer-uptime --csr-svd build/sim/csr.svd --gtkwave-savefile --trace --trace-fst
