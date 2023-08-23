include ../build/lambdaconcept_ecpix5/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

FW_DIRECTORY=.
BUILD_DIR=../build/lambdaconcept_ecpix5/software

CFLAGS += -I$(FW_DIRECTORY)/include
CFLAGS += -DI2C_FREQ_HZ=400

TINYUSB_DIR := $(FW_DIRECTORY)/deps/tinyusb

TINYUSB_SRC := 	$(TINYUSB_DIR)/src/tusb.c \
				$(TINYUSB_DIR)/src/device/usbd.c \
				$(TINYUSB_DIR)/src/device/usbd_control.c \
				$(TINYUSB_DIR)/src/common/tusb_fifo.c \
				$(TINYUSB_DIR)/src/class/dfu/dfu_device.c 

TINYUSB_OBJ := $(notdir $(TINYUSB_SRC:.c=.o))

CFLAGS += 	-I$(TINYUSB_DIR)/src \
			-DCFG_TUSB_MCU=OPT_MCU_LUNA_EPTRI \
			-fdata-sections -ffunction-sections -fsingle-precision-constant -fno-strict-aliasing \
			-DCFG_TUSB_DEBUG=1

vpath %.c $(dir $(TINYUSB_SRC)) $(FW_DIRECTORY) $(FW_DIRECTORY)/luna/eptri
vpath %.S $(FW_DIRECTORY)

OBJECTS =  	crt0.o \
			main.o	\
			sleep.o \
			dcd_eptri.o \
			usb_descriptors.o 		   		

OBJECTS += $(TINYUSB_OBJ)

all: oc-fw.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

oc-fw.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -T $(FW_DIRECTORY)/linker.ld -N -o $@ \
		$(OBJECTS) \
		-L$(BUILD_DIR)/libc \
		-lc \
		-L$(BUILD_DIR)/libbase \
		-lbase \
		-L$(BUILD_DIR)/liblitespi \
		-lcompiler_rt \
		-L$(BUILD_DIR)/libcompiler_rt \
		-llitespi \
		-Wl,--whole-archive \
		-Wl,--gc-sections

# pull in dependency info for *existing* .o files
-include $(OBJECTS:.o=.d)

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) oc-fw.elf oc-fw.bin .*~ *~

.PHONY: all clean
