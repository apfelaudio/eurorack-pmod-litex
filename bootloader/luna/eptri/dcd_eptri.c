/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Ha Thach (tinyusb.org)
 * Copyright (c) 2021 Great Scott Gadgets <info@greatscottgadgets.com>
 * Copyright (c) 2021 Katherine J. Temkin <k@ktemk.in>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"
#include "board.h"

#if TUSB_OPT_DEVICE_ENABLED && (CFG_TUSB_MCU == OPT_MCU_LUNA_EPTRI)

#include "device/dcd.h"
#include "dcd_eptri.h"
#include "generated/luna_usb.h"

//--------------------------------------------------------------------+
// SIE Command
//--------------------------------------------------------------------+

#define EP_SIZE 64
#define EP_COUNT 16


uint16_t volatile rx_buffer_offset[EP_COUNT];
uint8_t* volatile rx_buffer[EP_COUNT];
uint16_t volatile rx_buffer_max[EP_COUNT];

volatile uint8_t tx_ep;
volatile bool tx_active;
volatile uint16_t tx_buffer_offset[EP_COUNT];
uint8_t* volatile tx_buffer[EP_COUNT];
volatile uint16_t tx_buffer_max[EP_COUNT];
volatile uint8_t reset_count;

//--------------------------------------------------------------------+
// PIPE HELPER
//--------------------------------------------------------------------+

static bool advance_tx_ep(void) {

	// Move on to the next transmit buffer in a round-robin manner
	uint8_t prev_tx_ep = tx_ep;
	for (tx_ep = (tx_ep + 1) & 0xf; tx_ep != prev_tx_ep; tx_ep = ((tx_ep + 1) & 0xf)) {
		if (tx_buffer[tx_ep])
			return true;
	}
	if (!tx_buffer[tx_ep])
		return false;
	return true;
}

static void tx_more_data(void) {
	// Send more data
	uint8_t added_bytes;
	for (added_bytes = 0; (added_bytes < EP_SIZE) && (tx_buffer_offset[tx_ep] < tx_buffer_max[tx_ep]); added_bytes++) {
		usb_in_ep_data_write(tx_buffer[tx_ep][tx_buffer_offset[tx_ep]++]);
	}

	// Updating the epno queues the data
	usb_in_ep_epno_write(tx_ep & 0xf);
}

static void process_tx(void) {

	// If the buffer is now empty, search for the next buffer to fill.
	if (!tx_buffer[tx_ep]) {
		if (advance_tx_ep())
			tx_more_data();
		else
			tx_active = false;
		return;
	}

	if (tx_buffer_offset[tx_ep] >= tx_buffer_max[tx_ep]) {
		tx_buffer[tx_ep] = NULL;
		uint16_t xferred_bytes = tx_buffer_max[tx_ep];
		uint8_t xferred_ep = tx_ep;

		if (!advance_tx_ep())
			tx_active = false;
		dcd_event_xfer_complete(0, tu_edpt_addr(xferred_ep, TUSB_DIR_IN), xferred_bytes, XFER_RESULT_SUCCESS, true);
		if (!tx_active)
			return;
	}

	tx_more_data();
	return;
}

static void process_rx(void) {
	uint8_t rx_ep = usb_out_ep_data_ep_read();

	// Drain the FIFO into the destination buffer
	uint32_t total_read = 0;
	uint32_t current_offset = rx_buffer_offset[rx_ep];
	while (usb_out_ep_have_read()) {
		uint8_t c = usb_out_ep_data_read();
		total_read++;
		if (current_offset < rx_buffer_max[rx_ep]) {
			if (rx_buffer[rx_ep] != (volatile uint8_t *)0xffffffff)
				rx_buffer[rx_ep][current_offset++] = c;
		}
	}

	// Adjust the Rx buffer offset.
	rx_buffer_offset[rx_ep] += total_read;
	if (rx_buffer_offset[rx_ep] > rx_buffer_max[rx_ep])
		rx_buffer_offset[rx_ep] = rx_buffer_max[rx_ep];

	// If there's no more data, complete the transfer to tinyusb
	if ((rx_buffer_max[rx_ep] == rx_buffer_offset[rx_ep])
	// ZLP with less than the total amount of data
	|| ((total_read == 0) && ((rx_buffer_offset[rx_ep] & 63) == 0))
	// Short read, but not a full packet
	|| (((rx_buffer_offset[rx_ep] & 63) != 0) && (total_read < 66))) {

		// Free up this buffer.
		rx_buffer[rx_ep] = NULL;
		uint16_t len = rx_buffer_offset[rx_ep];

		dcd_event_xfer_complete(0, tu_edpt_addr(rx_ep, TUSB_DIR_OUT), len, XFER_RESULT_SUCCESS, true);
	}
	else {
		// If there's more data, re-enable data reception.
		usb_out_ep_enable_write(1);
	}

	// Now that the buffer is drained, clear the pending IRQ.
	usb_out_ep_ev_pending_write(usb_out_ep_ev_pending_read());
}

//--------------------------------------------------------------------+
// CONTROLLER API
//--------------------------------------------------------------------+

static void dcd_reset(void)
{
	reset_count++;
	usb_setup_ev_enable_write(0);
	usb_in_ep_ev_enable_write(0);
	usb_out_ep_ev_enable_write(0);

	// Reset the device address to 0.
	usb_setup_address_write(0);

	// Reset all three FIFO handlers
	usb_setup_reset_write(1);
	usb_in_ep_reset_write(1);
	usb_out_ep_reset_write(1);

	memset((void *)rx_buffer, 0, sizeof(rx_buffer));
	memset((void *)rx_buffer_max, 0, sizeof(rx_buffer_max));
	memset((void *)rx_buffer_offset, 0, sizeof(rx_buffer_offset));

	memset((void *)tx_buffer, 0, sizeof(tx_buffer));
	memset((void *)tx_buffer_max, 0, sizeof(tx_buffer_max));
	memset((void *)tx_buffer_offset, 0, sizeof(tx_buffer_offset));
	tx_ep = 0;
	tx_active = false;

	// Enable all event handlers and clear their contents
	usb_device_controller_ev_pending_write(0xff);
	usb_setup_ev_pending_write(usb_setup_ev_pending_read());
	usb_in_ep_ev_pending_write(usb_in_ep_ev_pending_read());
	usb_out_ep_ev_pending_write(usb_out_ep_ev_pending_read());
	usb_in_ep_ev_enable_write(1);
	usb_out_ep_ev_enable_write(1);
	usb_setup_ev_enable_write(1);
	usb_device_controller_ev_enable_write(1);

	if (usb_device_controller_speed_read())	 {
		dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
	} else {
		dcd_event_bus_reset(0, TUSB_SPEED_HIGH, true);
	}

}

static void clear_endpoints(void)
{
	tx_active = false;

	for (int i = 0; i < EP_COUNT; ++i) {
		rx_buffer[i] = NULL;
		tx_buffer[i] = NULL;
	}
}


// Initializes the USB peripheral for device mode and enables it.
void dcd_init(uint8_t rhport)
{
	(void) rhport;

	usb_device_controller_connect_write(0);

	usb_setup_reset_write(1);
	usb_in_ep_reset_write(1);
	usb_out_ep_reset_write(1);

	clear_endpoints();

	// Enable all event handlers and clear their contents
	usb_device_controller_ev_pending_write(usb_device_controller_ev_pending_read());
	usb_setup_ev_pending_write(usb_setup_ev_pending_read());
	usb_in_ep_ev_pending_write(usb_in_ep_ev_pending_read());
	usb_out_ep_ev_pending_write(usb_out_ep_ev_pending_read());
	usb_device_controller_ev_enable_write(1);
	usb_in_ep_ev_enable_write(1);
	usb_out_ep_ev_enable_write(1);
	usb_setup_ev_enable_write(1);

	// Turn on the external pullup
	usb_device_controller_connect_write(1);
}

// Enables or disables the USB device interrupt(s). May be used to
// prevent concurrency issues when mutating data structures shared
// between main code and the interrupt handler.
void dcd_int_enable(uint8_t rhport)
{
	(void) rhport;
	usb_device_controller_interrupt_enable();
	usb_setup_interrupt_enable();
	usb_in_ep_interrupt_enable();
	usb_out_ep_interrupt_enable();
}

void dcd_int_disable(uint8_t rhport)
{
	(void) rhport;
	usb_device_controller_interrupt_disable();
	usb_setup_interrupt_disable();
	usb_in_ep_interrupt_disable();
	usb_out_ep_interrupt_disable();
}

// Called when the device is given a new bus address.
void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
	// Respond with ACK status first before changing device address
	dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);

	// Wait for the response packet to get sent
	while (tx_active);

	// Activate the new address
	usb_setup_address_write(dev_addr);
}

// Called to remote wake up host when suspended (e.g hid keyboard)
void dcd_remote_wakeup(uint8_t rhport)
{
	(void) rhport;
}

void dcd_connect(uint8_t rhport)
{
	(void) rhport;
	usb_device_controller_connect_write(1);
}

void dcd_disconnect(uint8_t rhport)
{
	(void) rhport;
	usb_device_controller_connect_write(0);
}


//--------------------------------------------------------------------+
// DCD Endpoint Port
//--------------------------------------------------------------------+
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * p_endpoint_desc)
{
	(void) rhport;
	uint8_t ep_num = tu_edpt_number(p_endpoint_desc->bEndpointAddress);
	uint8_t ep_dir = tu_edpt_dir(p_endpoint_desc->bEndpointAddress);

	if (p_endpoint_desc->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS)
		return false; // Not supported

	if (ep_dir == TUSB_DIR_OUT) {
		rx_buffer_offset[ep_num] = 0;
		rx_buffer_max[ep_num] = 0;
		rx_buffer[ep_num] = NULL;
	}

	else if (ep_dir == TUSB_DIR_IN) {
		tx_buffer_offset[ep_num] = 0;
		tx_buffer_max[ep_num] = 0;
		tx_buffer[ep_num] = NULL;
	}

	return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  // TODO implement dcd_edpt_close_all()
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
	(void) rhport;

	if (tu_edpt_dir(ep_addr) == TUSB_DIR_OUT) {
		uint8_t enable = 0;
		if (rx_buffer[ep_addr])
			enable = 1;
		usb_out_ep_epno_write(tu_edpt_number(ep_addr));
		usb_out_ep_stall_write(1);
		usb_out_ep_enable_write(enable);
	}
	else {
		usb_in_ep_stall_write(1);
		usb_in_ep_epno_write(tu_edpt_number(ep_addr));
	}
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
	(void) rhport;
	if (tu_edpt_dir(ep_addr) == TUSB_DIR_OUT) {
		uint8_t enable = 0;
		if (rx_buffer[ep_addr])
			enable = 1;
		usb_out_ep_epno_write(tu_edpt_number(ep_addr));
		usb_out_ep_stall_write(0);
		usb_out_ep_enable_write(enable);
	}
	// IN endpoints will get unstalled when more data is written.
}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes)
{
	(void)rhport;
	uint8_t ep_num = tu_edpt_number(ep_addr);
	uint8_t ep_dir = tu_edpt_dir(ep_addr);
	TU_ASSERT(ep_num < 16);

	// Give a nonzero buffer when we transmit 0 bytes, so that the
	// system doesn't think the endpoint is idle.
	if ((buffer == NULL) && (total_bytes == 0)) {
		buffer = (uint8_t *)0xffffffff;
	}

	TU_ASSERT(buffer != NULL);

	if (ep_dir == TUSB_DIR_IN) {
		// Wait for the tx pipe to free up
		uint8_t previous_reset_count = reset_count;

		// Continue until the buffer is empty, the system is idle, and the fifo is empty.
		while (tx_buffer[ep_num] != NULL)
			;

		dcd_int_disable(0);
		// If a reset happens while we're waiting, abort the transfer
		if (previous_reset_count != reset_count)
			return true;

		TU_ASSERT(tx_buffer[ep_num] == NULL);
		tx_buffer_offset[ep_num] = 0;
		tx_buffer_max[ep_num] = total_bytes;
		tx_buffer[ep_num] = buffer;

		// If the current buffer is NULL, then that means the tx logic is idle.
		// Update the tx_ep to point to our endpoint number and queue the data.
		// Otherwise, let it be and it'll get picked up after the next transfer
		// finishes.
		if (!tx_active) {
			tx_ep = ep_num;
			tx_active = true;
			tx_more_data();
		}
		dcd_int_enable(0);
	}

	else if (ep_dir == TUSB_DIR_OUT) {

		while (rx_buffer[ep_num] != NULL)
			;

		TU_ASSERT(rx_buffer[ep_num] == NULL);

		dcd_int_disable(0);
		rx_buffer[ep_num] = buffer;
		rx_buffer_offset[ep_num] = 0;
		rx_buffer_max[ep_num] = total_bytes;

		// Enable receiving on this particular endpoint, if it hasn't been already.
		usb_out_ep_epno_write(ep_num);
		usb_out_ep_prime_write(1);
		usb_out_ep_enable_write(1);

		dcd_int_enable(0);
	}
	return true;
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+

static void handle_out(void)
{
	// An "OUT" transaction just completed so we have new data.
	// (But only if we can accept the data)
	process_rx();
}

static void handle_in(void)
{
	usb_in_ep_ev_pending_write(usb_in_ep_ev_pending_read());
	process_tx();
}

static void handle_reset(void)
{
	usb_device_controller_ev_pending_write(usb_device_controller_ev_pending_read());

	// This event means a bus reset occurred.  Reset everything, and
	// abandon any further processing.
	dcd_reset();
}

static void handle_setup(void)
{
	uint8_t setup_packet_bfr[8];

	// We got a SETUP packet.  Copy it to the setup buffer and clear
	// the "pending" bit.
	// Setup packets are always 8 bytes, plus two bytes of crc16.
	uint32_t setup_length = 0;

	while (usb_setup_have_read()) {
		uint8_t c = usb_setup_data_read();
		if (setup_length < sizeof(setup_packet_bfr))
			setup_packet_bfr[setup_length] = c;
		setup_length++;
	}

	// If we have 8 bytes, that's a full SETUP packet
	// Otherwise, it was an RX error.
	if (setup_length == 8) {
		dcd_event_setup_received(0, setup_packet_bfr, true);
	}

	usb_setup_ev_pending_write(usb_setup_ev_pending_read());
}

void dcd_int_handler(uint8_t rhport)
{
	(void)rhport;

	// Handle USB interrupts for as long as any are pending.
	while(1) {
		if (usb_device_controller_ev_pending_read()) {
			handle_reset();
		}
		else if (usb_setup_ev_pending_read()) {
			handle_setup();
		}
		else if (usb_in_ep_ev_pending_read()) {
			handle_in();
		} 
		else if (usb_out_ep_ev_pending_read()) {
			handle_out();
		} 
		else {
			// No interrupts are pending -- we're done!
			return;
			
		}
	}
}

#endif
