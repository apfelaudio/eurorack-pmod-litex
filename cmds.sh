./example-ecpix-5.py --ecppack-compress --cpu-type vexriscv --cpu-variant imac --csr-svd build/lambdaconcept_ecpix5/csr.svd --build

openFPGALoader -b ecpix5 --vid 0x0403 --pid 0x6011 build/lambdaconcept_ecpix5/gateware/lambdaconcept_ecpix5.bit

cd firmware && ./build.sh

./bin/litex_term --kernel build/lambdaconcept_ecpix5/rust-fw.bin /dev/ttyUSB3 | ~/dev/defmt/target/release/defmt-print --show-skipped-frames -e firmware/litex-fw/target/riscv32imac-unknown-none-elf/release/litex-fw
