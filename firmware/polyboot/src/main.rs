#![cfg(not(test))]
#![allow(unused_imports)]

#![no_std]
#![no_main]

use litex_hal::prelude::*;
use litex_pac as pac;
use riscv_rt::entry;
use litex_hal::hal::digital::v2::OutputPin;
use heapless::String;
use heapless::Vec;
use core::arch::asm;
use core::slice;

use ssd1322 as oled;

use tinyusb_sys::{tusb_init, dcd_int_handler, tud_task_ext,
                  tud_dfu_finish_flashing, dfu_state_t, dfu_status_t,
                  CFG_TUD_DFU_XFER_BUFSIZE};

use micromath::F32Ext;

use polyvec_lib::voice::*;
use polyvec_lib::draw;
use polyvec_lib::opt;

use polyvec_hal::gw::*;
use polyvec_hal::log::*;
use polyvec_hal::*;

static mut BUS_RESET: bool = false;
static mut FLASH_CMD: bool = false;

#[export_name = "DefaultHandler"]
unsafe fn irq_handler() {

    let pending_irq = vexriscv::register::vmip::read();

    // TODO: grab these correctly from PAC!
    // These are already in the SVD as constants, but not as
    // sub-nodes of the peripheral parents, so they aren't
    // rendered in the svd2rust bindings.
    //
    let irq_usb_device = 2usize;
    let irq_usb_setup  = 3usize;
    let irq_usb_in_ep  = 4usize;
    let irq_usb_out_ep = 5usize;

    if (pending_irq & (1 << irq_usb_device)) != 0 ||
       (pending_irq & (1 <<  irq_usb_setup)) != 0 ||
       (pending_irq & (1 <<  irq_usb_in_ep)) != 0 ||
       (pending_irq & (1 << irq_usb_out_ep)) != 0 {
        dcd_int_handler(0);
    }

    if (pending_irq & (1 << irq_usb_device)) != 0 {
        BUS_RESET = true;
    }
}

fn fence() {
    #[cfg(not(test))]
    {
        unsafe {
            asm!("fence iorw, iorw");
            asm!(".word(0x500F)");
        }
    }
}


fn oled_init(timer: &mut Timer, oled_spi: pac::OLED_SPI)
    -> ssd1322::Display<ssd1322::SpiInterface<OledSpi, OledGpio>> {

    let dc = OledGpio::new(0);
    let mut rstn = OledGpio::new(1);
    let mut csn = OledGpio::new(2);
    let spi = OledSpi::new(oled_spi);

    // Create the SpiInterface and Display.
    let mut disp = oled::Display::new(
        oled::SpiInterface::new(spi, dc),
        oled::PixelCoord(256, 64),
        oled::PixelCoord(112, 0),
    );

    csn.set_low().unwrap();

    // Assert the display's /RESET for 10ms.
    timer.delay_ms(10u32);
    rstn.set_low().unwrap();
    timer.delay_ms(10u32);
    rstn.set_high().unwrap();

    timer.delay_ms(1u16);

    disp.init(
           oled::Config::new(
               oled::ComScanDirection::RowZeroLast,
               oled::ComLayout::DualProgressive,
           ).clock_fosc_divset(9, 1)
               .display_enhancements(true, true)
               .contrast_current(159)
               .phase_lengths(5, 14)
               .precharge_voltage(31)
               .second_precharge_period(8)
               .com_deselect_voltage(7),
       ).unwrap();

    disp
}


const USB_DEVICE_CONTROLLER_CONNECT_ADDRESS: *mut u32 = 0xF0010000 as *mut u32;
const USB_DEVICE_CONTROLLER_RESET_ADDRESS: *mut u32 = 0xF0010004 as *mut u32;

unsafe fn usb_device_controller_reset_write(value: u32) {
    core::ptr::write_volatile(USB_DEVICE_CONTROLLER_RESET_ADDRESS, value);
}

unsafe fn usb_device_controller_connect_write(value: u32) {
    core::ptr::write_volatile(USB_DEVICE_CONTROLLER_CONNECT_ADDRESS, value);
}

pub trait SpiFlash {
    unsafe fn transfer_byte(&self, b: u8) -> u8;
    unsafe fn cmd(&self, header: &[u8], data: &[u8]);
    unsafe fn write_enable(&self);
    unsafe fn sector_erase(&self, addr: usize);
    unsafe fn page_program(&self, addr: usize, data: &[u8]);
    unsafe fn status_busy(&self) -> bool;
    unsafe fn erase_range(&self, addr: usize, len: usize);
    unsafe fn peek(&self, addr: usize) -> u8;
    unsafe fn write_stream(&self, addr: usize, data: &[u8]);
}

const SPI_FLASH_BLOCK_SIZE: usize = 256;
const SPI_FLASH_PAGE_SIZE: usize = 64*1024;
const SPIFLASH_BASE: *mut u8 = 0x00800000 as *mut u8;

macro_rules! spi_flash {
    ($($t:ty),+ $(,)?) => {
        $(impl SpiFlash for $t {
            unsafe fn transfer_byte(&self, b: u8) -> u8 {
                while self.master_status().read().tx_ready() == false { }
                self.master_rxtx().write(|w| w.bits(b as u32));
                while self.master_status().read().rx_ready() == false { }
                self.master_rxtx().read().bits() as u8
            }

            unsafe fn cmd(&self, header: &[u8], data: &[u8]) {

                self.master_phyconfig().write( |w| w
                    .len().bits(8)
                    .width().bits(1)
                    .mask().bits(1)
                );

                self.master_cs().write(|w| w.bits(1));

                fence();

                for i in 0..header.len() {
                    self.transfer_byte(header[i]);
                }

                for i in 0..data.len() {
                    self.transfer_byte(data[i]);
                }

                self.master_cs().write(|w| w.bits(0));

                fence();
            }

            unsafe fn write_enable(&self) {
                self.cmd(&[0x06], &[]);
            }

            unsafe fn sector_erase(&self, addr: usize) {
                let header = [
                    0xd8u8,
                    (addr>>16) as u8,
                    (addr>>8)  as u8,
                    (addr>>0)  as u8,
                ];
                self.cmd(&header, &[]);
            }

            unsafe fn page_program(&self, addr: usize, data: &[u8]) {
                let header = [
                    0x02u8,
                    (addr>>16) as u8,
                    (addr>>8)  as u8,
                    (addr>>0)  as u8,
                ];
                self.cmd(&header, data);
            }

            unsafe fn status_busy(&self) -> bool {
                self.master_phyconfig().write( |w| w
                    .len().bits(8)
                    .width().bits(1)
                    .mask().bits(1)
                );
                self.master_cs().write(|w| w.bits(1));

                fence();

                self.transfer_byte(0x05u8);
                self.transfer_byte(0x00u8);
                let status2 = self.transfer_byte(0x00u8);
                self.transfer_byte(0x00u8);

                self.master_cs().write(|w| w.bits(0));

                fence();

                (status2& 1) != 0
            }

            unsafe fn peek(&self, addr: usize) -> u8 {
                core::ptr::read_volatile(SPIFLASH_BASE.offset(addr as isize))
            }

            unsafe fn erase_range(&self, addr: usize, len: usize) {
                let mut i: usize = 0;
                while i < len {
                    info!("erase @ {:#x}", addr + i);
                    self.write_enable();
                    self.sector_erase(addr + i);

                    while self.status_busy() { }

                    for j in 0..SPI_FLASH_PAGE_SIZE {
                        let peek = self.peek(addr+i+j);
                        if peek != 0xff {
                            info!("error: erase failed at {:#x} (got {:#x}, want {:#x})",
                                  addr+i+j, peek, 0xff);
                        }
                    }
                    i += SPI_FLASH_PAGE_SIZE;
                }
            }

            unsafe fn write_stream(&self, addr: usize, data: &[u8]) {
                let mut w_len: usize = usize::min(data.len(), SPI_FLASH_BLOCK_SIZE);
                let mut offset: usize = 0;
                while w_len > 0 {
                    //info!("write {} bytes @ {:#x}", w_len, addr+offset);
                    self.write_enable();
                    self.page_program(addr+offset, &data[offset..offset+w_len]);
                    while self.status_busy() { }
                    for j in 0..w_len {
                        let peek = self.peek(addr+offset+j);
                        if peek != data[offset+j] {
                            info!("error: verify failed at {:#x} (got {:#x}, want {:#x})",
                                  addr+offset+j, peek, data[offset+j]);
                        }
                    }
                    offset += w_len;
                    w_len = usize::min(data.len()-offset, SPI_FLASH_BLOCK_SIZE);
                }
            }
        })+
    };
}

spi_flash!(pac::SPIFLASH_CORE);

#[no_mangle]
pub extern "C" fn tud_dfu_get_timeout_cb(_alt: u8, state: u8) -> u32 {
    match state {
        state if state == dfu_state_t::DFU_DNBUSY as u8 => 1,
        state if state == dfu_state_t::DFU_MANIFEST as u8 => 0,
        _ => 0
    }
}

#[no_mangle]
pub unsafe extern "C" fn tud_dfu_download_cb(alt: u8, block_num: u16, data: *const u8, length: u16)  {
    //info!("DOWNLOAD alt={} block_num={} len={}", alt, block_num, length);

    FLASH_CMD = true;

    let alt_to_base: [usize; 2] = [
        0x100000, // User Gateware
        0x1E0000, // User Firmware
    ];

    let alt_to_len: [usize; 2] = [
        0x100000, // User Gateware
        0x010000, // User Firmware
    ];

    let alt = alt as usize;
    let block_num = block_num as usize;
    let length = length as usize;

    if alt >= alt_to_base.len() {
		tud_dfu_finish_flashing(dfu_status_t::DFU_STATUS_ERR_ADDRESS as u8);
        return;
    }

    if (block_num * (CFG_TUD_DFU_XFER_BUFSIZE as usize)) >= alt_to_len[alt] {
		tud_dfu_finish_flashing(dfu_status_t::DFU_STATUS_ERR_ADDRESS as u8);
        return;
    }

    let peripherals = pac::Peripherals::steal();

	let flash_address: usize = alt_to_base[alt] + (block_num as usize) * (CFG_TUD_DFU_XFER_BUFSIZE as usize);

	if (flash_address & (SPI_FLASH_PAGE_SIZE - 1)) == 0 {
        peripherals.SPIFLASH_CORE.erase_range(flash_address, SPI_FLASH_PAGE_SIZE);
    }

    peripherals.SPIFLASH_CORE.write_stream(flash_address, slice::from_raw_parts(data, length));

    tud_dfu_finish_flashing(dfu_status_t::DFU_STATUS_OK as u8);
}

#[no_mangle]
pub extern "C" fn tud_dfu_manifest_cb(_alt: u8)  {
    // TODO
    unsafe {
        tud_dfu_finish_flashing(dfu_status_t::DFU_STATUS_OK as u8);
    }
}

#[no_mangle]
pub extern "C" fn _putchar(c: u8)  {
    log::_logger_write(&[c]);
}

#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    info!("hello from litex-fw!");

    /*
    unsafe {
    let data: [u8; 512] = [42u8; 512];
    tud_dfu_download_cb(0, 0, data.as_ptr(), data.len() as u16);
    }
    */

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    let pmod0 = peripherals.EURORACK_PMOD0;
    pmod0.reset(&mut timer);
    let pmod1 = peripherals.EURORACK_PMOD1;
    let pmod2 = peripherals.EURORACK_PMOD2;
    let pmod3 = peripherals.EURORACK_PMOD3;

    for i in 0..8 {
        pmod0.led_set(i, 0i8);
        pmod1.led_set(i, 0i8);
        pmod2.led_set(i, 0i8);
        pmod3.led_set(i, 0i8);
    }

    let mut disp = oled_init(&mut timer, peripherals.OLED_SPI);
    let mut spi_dma = SpiDma::new(peripherals.SPI_DMA, pac::OLED_SPI::PTR);

    let mut skip_dfu: bool = true;

    if peripherals.ENCODER_BUTTON.in_().read().bits() == 0 {
        draw::draw_boot_splash(&mut disp, "Booting ...").ok();
    } else {
        draw::draw_boot_splash(&mut disp, "Waiting for DFU ...").ok();
        skip_dfu = false;
    }

    // WARN: Don't update the screen while USB is being serviced. It seems screen
    // DMA is enough to cause the USB stack to spuriously fail internal assertions.
    let fb = disp.swap_clear();
    fence();
    spi_dma.transfer(fb.as_ptr(), fb.len());
    spi_dma.block();

    // LED pattern to test all the LEDs for a couple secs before we boot or DFU
    loop {
        let uptime_ms = (timer.uptime() / ((SYSTEM_CLOCK_FREQUENCY as u64)/1000u64)) as u32;

        for i in 0..8 {
            pmod0.led_set(i, (f32::sin((uptime_ms as f32)/100.0f32+(i as f32)) * 32.0f32) as i8);
            pmod1.led_set(i, (f32::sin((uptime_ms as f32)/100.0f32+((i+8) as f32)) * 32.0f32) as i8);
            pmod2.led_set(i, (f32::sin((uptime_ms as f32)/100.0f32+((i+16) as f32)) * 32.0f32) as i8);
            pmod3.led_set(i, (f32::sin((uptime_ms as f32)/100.0f32+((i+24) as f32)) * 32.0f32) as i8);
        }

        if uptime_ms > 3500 {
            break;
        }
    }

    for i in 0..8 {
        pmod0.led_set(i, 0i8);
        pmod1.led_set(i, 0i8);
        pmod2.led_set(i, 0i8);
        pmod3.led_set(i, 0i8);
    }

    if !skip_dfu {

        unsafe {
            // Warn: init USB AFTER first framebuffer xfer done.
            usb_device_controller_reset_write(1);
            timer.delay_ms(100u32);
            usb_device_controller_reset_write(0);
            timer.delay_ms(100u32);
            tusb_init();

            // Enable machine external interrupts (basically everything added on by LiteX).
            riscv::register::mie::set_mext();

            // WARN: delay before interrupt enable after tusb_init(), the USB core takes a while
            // to spin up after tusb_init() if nothing is connected, apparently.
            timer.delay_ms(100u32);

            // WARN: Don't do this before IRQs are registered for this scope,
            // otherwise you'll hang forever :)
            // Finally enable interrupts
            riscv::interrupt::enable();
        }

        loop {
            unsafe {
                tud_task_ext(u32::MAX, false);
                if BUS_RESET {
                    BUS_RESET = false;
                    if FLASH_CMD {
                        break;
                    }
                }
            }
        }

    }


    // Disconnect from USB and boot firmware.
    unsafe {
        riscv::interrupt::disable();
        usb_device_controller_connect_write(0);
        timer.delay_ms(50u32);
        peripherals.PROGRAMN.out().write(|w| w.bits(1));
    }

    loop {
    }
}
