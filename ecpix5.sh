sudo openFPGALoader -b ecpix5 --vid 0x0403 --pid 0x6011 build/lambdaconcept_ecpix5/gateware/lambdaconcept_ecpix5.bit
python3 example-ecpix-5.py --ecppack-compress --cpu-type vexriscv --cpu-variant imac --csr-svd build/lambdaconcept_ecpix5/csr.svd --build --lx-ignore-deps --timer-uptime
sudo ./bin/litex_term --kernel build/lambdaconcept_ecpix5/rust-fw.bin /dev/ttyUSB3
OBJCOPY=riscv64-linux-gnu-objcopy BOARD=lambdaconcept_ecpix5 ./build.sh
sudo ./bin/litex_term --kernel build/lambdaconcept_ecpix5/rust-fw.bin /dev/ttyUSB3
sudo ./bin/litex_term --kernel build/lambdaconcept_ecpix5/rust-fw.bin /dev/ttyUSB3
