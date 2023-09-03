/* This file is part of OrangeCrab-test
 *
 * Copyright 2020 Gregory Davill <greg.davill@gmail.com> 
 * Copyright 2020 Michael Welling <mwelling@ieee.org>
 */

#include <stdlib.h>
#include <stdio.h>

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/git.h>
#include <generated/luna_usb.h>
#include <liblitespi/spiflash.h>

#include <irq.h>
#include <uart.h>

#include <sleep.h>

#include "tusb.h"


#define SPI_FLASH_PAGE_SZ (64*1024)

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+
typedef struct
{
	uint32_t address;
	uint32_t length;
} memory_offest;

memory_offest const alt_offsets[] = {
	{.address = 0x100000, .length = 0x100000}, /* Main Gateware */
	{.address = 0x1E0000, .length = 0x010000}, /* Main Firmawre */
	{.address = 0x800000, .length = 0x000000}, /* Extra */
	{.address = 0x800000, .length = 0x000000}  /* Bootloader */
};

static bool flash_command_seen = false;
static bool bus_reset_received = false;

/* Blink pattern
 * - 1000 ms : device should reboot
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
	BLINK_DFU_IDLE,
	BLINK_DFU_IDLE_BOOTLOADER,
	BLINK_DFU_DOWNLOAD,
	BLINK_DFU_ERROR,
	BLINK_DFU_SLEEP,
};

static uint32_t blink_interval_ms = BLINK_DFU_IDLE;

// Current system tick timer.
volatile uint32_t system_ticks = 0;
#if CFG_TUSB_OS == OPT_OS_NONE
uint32_t board_millis(void)
{
	return system_ticks;
}
#endif

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

static void timer_init(void)
{
	// Set up our timer to generate an interrupt every millisecond.
	timer0_reload_write(60 * 1000);
	timer0_en_write(1);
	timer0_ev_enable_write(1);

	// Enable our timer's interrupt.
	irq_setie(1);
	irq_setmask((1 << TIMER0_INTERRUPT) | irq_getmask());
}

static void timer_isr(void)
{
	// Increment our total millisecond count.
	++system_ticks;
}

void app_isr(void)
{
	unsigned int irqs;
	irqs = irq_pending() & irq_getmask();

	// Dispatch UART events.
	if (irqs & (1 << UART_INTERRUPT))
	{
		uart_isr();
	}

	// Dispatch timer events.
	if (irqs & (1 << TIMER0_INTERRUPT))
	{
		timer0_ev_pending_write(timer0_ev_pending_read());
		timer_isr();
	}

	// Dispatch USB events.
	if (irqs & (1 << USB_DEVICE_CONTROLLER_INTERRUPT | 1 << USB_IN_EP_INTERRUPT | 1 << USB_OUT_EP_INTERRUPT | 1 << USB_SETUP_INTERRUPT))
	{
		tud_int_handler(0);
	}

	/* Monitor bus resets */
	if(irqs & (1 << USB_DEVICE_CONTROLLER_INTERRUPT))
	{
		bus_reset_received = true;
	}
}

int main(int i, char **c)
{

	/* Setup IRQ, needed for UART */
	irq_setmask(0);
	irq_setie(1);
	uart_init();

    printf("Eurolut DFU bootloader\n");

	usb_device_controller_reset_write(1);
	msleep(100);
	usb_device_controller_reset_write(0);
	msleep(100);

    if (encoder_button_in_read() & 0x1)
    //if(true)
    {
        printf("Entering USB DFU mode until reset commanded...\n");

		timer_init();
		tusb_init();

		while (1)
		{
			tud_task(); // tinyusb device task

			if (bus_reset_received)
			{
				bus_reset_received = false;
				if(flash_command_seen)
				{
					break;
				}
			}
		}
	} else {
        printf("Bootloader not requested. Skipping.\n");
    }

	printf("Pull down PROGRAMN, reboot to user bitstream\n");

	/* Reboot to our user bitstream */
	irq_setie(0);
	usb_device_controller_connect_write(0);

	msleep(50);

	while (1)
	{
		programn_out_write(1);
	}

	return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
	blink_interval_ms = BLINK_DFU_IDLE;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	blink_interval_ms = BLINK_DFU_IDLE;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void)remote_wakeup_en;
	blink_interval_ms = BLINK_DFU_SLEEP;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	blink_interval_ms = BLINK_DFU_IDLE;
}

//--------------------------------------------------------------------+
// DFU callbacks
// Note: alt is used as the partition number, in order to support multiple partitions like FLASH, EEPROM, etc.
//--------------------------------------------------------------------+

// Invoked right before tud_dfu_download_cb() (state=DFU_DNBUSY) or tud_dfu_manifest_cb() (state=DFU_MANIFEST)
// Application return timeout in milliseconds (bwPollTimeout) for the next download/manifest operation.
// During this period, USB host won't try to communicate with us.
uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state)
{
	if (state == DFU_DNBUSY)
	{
		return 1; /* Request we are polled in 1ms */
	}
	else if (state == DFU_MANIFEST)
	{
		// since we don't buffer entire image and do any flashing in manifest stage
		return 0;
	}

	return 0;
}

// Invoked when received DFU_DNLOAD (wLength>0) following by DFU_GETSTATUS (state=DFU_DNBUSY) requests
// This callback could be returned before flashing op is complete (async).
// Once finished flashing, application must call tud_dfu_finish_flashing()
void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const *data, uint16_t length)
{
	(void)alt;
	(void)block_num;

	blink_interval_ms = BLINK_DFU_DOWNLOAD;
	flash_command_seen = true;

	if ((block_num * CFG_TUD_DFU_XFER_BUFSIZE) >= alt_offsets[alt].length)
	{
		// flashing op for download length error
		tud_dfu_finish_flashing(DFU_STATUS_ERR_ADDRESS);

		blink_interval_ms = BLINK_DFU_ERROR;

		return;
	}

	uint32_t flash_address = alt_offsets[alt].address + block_num * CFG_TUD_DFU_XFER_BUFSIZE;

	if ((flash_address & (SPI_FLASH_PAGE_SZ - 1)) == 0)
	{
        spiflash_erase_range(flash_address, SPI_FLASH_PAGE_SZ);
    }

    spiflash_write_stream(flash_address, data, CFG_TUD_DFU_XFER_BUFSIZE);

	// flashing op for download complete without error
	tud_dfu_finish_flashing(DFU_STATUS_OK);
}

// Invoked when download process is complete, received DFU_DNLOAD (wLength=0) following by DFU_GETSTATUS (state=Manifest)
// Application can do checksum, or actual flashing if buffered entire image previously.
// Once finished flashing, application must call tud_dfu_finish_flashing()
void tud_dfu_manifest_cb(uint8_t alt)
{
	(void)alt;
	blink_interval_ms = BLINK_DFU_DOWNLOAD;

	// flashing op for manifest is complete without error
	// Application can perform checksum, should it fail, use appropriate status such as errVERIFY.
	tud_dfu_finish_flashing(DFU_STATUS_OK);
}

// Invoked when the Host has terminated a download or upload transfer
void tud_dfu_abort_cb(uint8_t alt)
{
	(void)alt;
	blink_interval_ms = BLINK_DFU_ERROR;
}

// Invoked when a DFU_DETACH request is received
void tud_dfu_detach_cb(void)
{
	blink_interval_ms = BLINK_DFU_SLEEP;
}
