# UNPROTECT COMPLETE FLASH
openFPGALoader -b colorlight-i9 -f --bulk-erase --unprotect-flash

# BOOTLOADER BITSTREAM

# Build the bitstream (provide --flash-boot = 0x800000 (memmapped spi flash) + 0xE0000 (firmware offset in flash))
python3 example-colorlight-i5.py --ecppack-compress --flash-boot=0x8E0000 --ecppack-bootaddr 0x100000 --cpu-type vexriscv --cpu-variant imac --csr-svd build/colorlight_i5/csr.svd --uart-baudrate=1000000 --timer-uptime --build
# Flash at correct address using dev board / JTAG
openFPGALoader -b colorlight-i9 -o 0x0 -f <bitstream>.bit

# BOOTLOADER FIRMWARE

# Turn into FBI with
./bin/crcfbigen.py build/colorlight_i5/bootloader.bin -f -l -o bootloader.fbi
# Flash at correct address using dev board / JTAG
openFPGALoader -b colorlight-i9 -o 0x0E0000 -f bootloader.fbi

# USER BITSTREAM

# Build the bitstream (provide --flash-boot = 0x800000 (memmapped spi flash) + 0x1E0000 (firmware offset in flash))
# ecppack-bootaddr set to switch back to bootloader on PROGRAMN
python3 example-colorlight-i5.py --ecppack-compress --flash-boot=0x9E0000 --ecppack-bootaddr 0x000000 --cpu-type vexriscv --cpu-variant imac --csr-svd build/colorlight_i5/csr.svd --uart-baudrate=1000000 --timer-uptime --build
# Hold BUTTON while turning on, then flash over DFU device ALT 0
sudo dfu-util --alt 0 --download build/colorlight_i5/gateware/colorlight_i5.bit

# USER FIRMWARE

# Create the FBI
./bin/crcfbigen.py build/colorlight_i5/rust-fw.bin -f -l -o rust-fw.fbi
# Download it
sudo dfu-util --alt 1 --download rust-fw.fbi --reset


# You can choose when to do the --reset into the user bitstream
