#![no_std]


use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use litex_pac as pac;
use litex_hal::hal::digital::v2::OutputPin;
use core::arch::asm;
use core::ffi::c_char;
use core::panic::PanicInfo;

use embedded_graphics::{
    pixelcolor::{Gray4, GrayColor},
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, Line},
    mono_font::{ascii::FONT_4X6, ascii::FONT_5X7, MonoTextStyle},
    prelude::*,
    text::{Alignment, Text},
};

use ssd1322 as oled;
use ssd1322::interface::DisplayInterface;

litex_hal::gpio! {
    CTL: litex_pac::OLED_CTL,
}

litex_hal::spi! {
    SPI: (litex_pac::OLED_SPI, u8),
}

fn fence() {
    unsafe {
        asm!("fence iorw, iorw");
        asm!(".word(0x500F)");
    }
}

#[panic_handler]
fn panic(_panic_info: &PanicInfo) -> ! {
    loop {}
}

#[no_mangle]
pub extern "C" fn oled_stub_write(text: *const c_char) {
    let peripherals = unsafe { pac::Peripherals::steal() };

    let dc = CTL { index: 0 };
    let mut rstn = CTL { index: 1 };
    let mut csn = CTL { index: 2 };
    let spi = SPI {
        registers: peripherals.OLED_SPI
    };

    unsafe {
        peripherals.SPI_DMA.spi_control_reg_address.write(
            |w| w.bits(litex_pac::OLED_SPI::PTR as u32));
        peripherals.SPI_DMA.spi_status_reg_address.write(
            |w| w.bits(litex_pac::OLED_SPI::PTR as u32 + 0x04));
        peripherals.SPI_DMA.spi_mosi_reg_address.write(
            |w| w.bits(litex_pac::OLED_SPI::PTR as u32 + 0x08));
    }

    let mut spi_interface = oled::SpiInterface::new(spi, dc);
    // Create the SpiInterface and Display.
    let mut disp = oled::Display::new(
        spi_interface,
        oled::PixelCoord(256, 64),
        oled::PixelCoord(112, 0),
    );

    csn.set_low().unwrap();

    // Assert the display's /RESET for 10ms.
    rstn.set_low().unwrap();
    for _ in 0..1000 { fence(); }
    rstn.set_high().unwrap();
    for _ in 0..1000 { fence(); }

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

    // This section breaks things (halt forever)
    let character_style = MonoTextStyle::new(&FONT_5X7, Gray4::WHITE);
    Text::with_alignment(
        "BOOTLOADER",
        Point::new(disp.bounding_box().center().x, 10),
        character_style,
        Alignment::Center,
    )
    .draw(&mut disp).ok();
    let fb = disp.swap_clear();

    // This section does not
    unsafe {
        fence();
        peripherals.SPI_DMA.read_base.write(|w| w.bits(fb.as_ptr() as u32));
        peripherals.SPI_DMA.read_length.write(|w| w.bits(fb.len() as u32));
        peripherals.SPI_DMA.start.write(|w| w.start().bit(true));
        peripherals.SPI_DMA.start.write(|w| w.start().bit(false));
        while peripherals.SPI_DMA.done.read().bits() == 0 {
        }
    }
}
