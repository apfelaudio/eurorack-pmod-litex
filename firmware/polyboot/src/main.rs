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

use ssd1322 as oled;

use tinyusb_sys::{tusb_init, dcd_int_handler, tud_task_ext,
                  tud_dfu_finish_flashing, dfu_state_t, dfu_status_t};

use polyvec_lib::voice::*;
use polyvec_lib::draw;
use polyvec_lib::opt;

use polyvec_hal::gw::*;
use polyvec_hal::log::*;
use polyvec_hal::*;

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

#[no_mangle]
pub extern "C" fn tud_dfu_get_timeout_cb(_alt: u8, state: u8) -> u32 {
    match state {
        state if state == dfu_state_t::DFU_DNBUSY as u8 => 10,
        state if state == dfu_state_t::DFU_MANIFEST as u8 => 0,
        _ => 0
    }
}

#[no_mangle]
pub extern "C" fn tud_dfu_download_cb(alt: u8, block_num: u16, data: *const u8, length: u16)  {
    info!("DOWNLOAD alt={} block_num={} len={}", alt, block_num, length);
    unsafe {
        tud_dfu_finish_flashing(dfu_status_t::DFU_STATUS_OK as u8);
    }
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

const USB_DEVICE_CONTROLLER_RESET_ADDRESS: *mut u32 = 0xF0010004 as *mut u32;

unsafe fn usb_device_controller_reset_write(value: u32) {
    core::ptr::write_volatile(USB_DEVICE_CONTROLLER_RESET_ADDRESS, value);
}

#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART_MIDI);
    info!("hello from litex-fw!");


    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    let pmod0 = peripherals.EURORACK_PMOD0;
    pmod0.reset(&mut timer);

    let mut disp = oled_init(&mut timer, peripherals.OLED_SPI);
    let mut spi_dma = SpiDma::new(peripherals.SPI_DMA, pac::OLED_SPI::PTR);
    let encoder = Encoder::new(peripherals.ROTARY_ENCODER, peripherals.ENCODER_BUTTON);

    draw::draw_boot_splash(&mut disp).ok();

    let fb = disp.swap_clear();
    fence();
    spi_dma.transfer(fb.as_ptr(), fb.len());
    spi_dma.block();

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
        }
    }
}
