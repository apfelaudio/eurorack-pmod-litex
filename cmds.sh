# BOOTLOADER BITSTREAM

# Build the bitstream (provide --flash-boot = 0x200000 (memmapped spi flash) + 0xE0000 (firmware offset in flash))
python3 example-colorlight-i5.py --ecppack-compress --flash-boot=0x2E0000 --ecppack-bootaddr 0x100000 --cpu-type vexriscv --cpu-variant imac --csr-svd build/colorlight_i5/csr.svd --uart-baudrate=1000000 --build
# Flash at correct address using dev board / JTAG
openFPGALoader -b colorlight-i5 -o 0x0 -f <bitstream>.bit

# BOOTLOADER FIRMWARE

# Turn into FBI with
./bin/crcfbigen.py bootloader/oc-fw.bin -f -l -o oc-fw.fbi
# Flash at correct address using dev board / JTAG
openFPGALoader -b colorlight-i5 -o 0x0E0000 -f oc-fw.fbi

# USER BITSTREAM

# Build the bitstream (provide --flash-boot = 0x200000 (memmapped spi flash) + 0x1E0000 (firmware offset in flash))
python3 example-colorlight-i5.py --ecppack-compress --flash-boot=0x3E0000 --ecppack-bootaddr 0x100000 --cpu-type vexriscv --cpu-variant imac --csr-svd build/colorlight_i5/csr.svd --uart-baudrate=1000000 --build
# Hold BUTTON while turning on, then flash over DFU device ALT 0
sudo dfu-util --alt 0 --download build/colorlight_i5/gateware/colorlight_i5.bit

# USER FIRMWARE

# Create the FBI
./bin/crcfbigen.py build/colorlight_i5/rust-fw.bin -f -l -o rust-fw.fbi
# Download it
sudo dfu-util --alt 1 --download rust-fw.fbi --reset


# You can choose when to do the --reset into the user bitstream
