#![no_std]
#![no_main]
#![allow(dead_code)]

use core::panic::PanicInfo;
use defmt;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use litex_pac as pac;
use riscv;
use riscv_rt::entry;
use litex_hal::hal::digital::v2::OutputPin;
use heapless::String;
use core::fmt::Write;

use embedded_graphics::{
    pixelcolor::{Gray4, GrayColor},
    primitives::{Circle, PrimitiveStyle, PrimitiveStyleBuilder},
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    prelude::*,
    text::Text,
};

use ssd1322 as oled;

const SYSTEM_CLOCK_FREQUENCY: u32 = 12_000_000;

// Globals used by `defmt` logger such that we can log to UART from anywhere.
static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static mut UART_WRITER: Option<Uart> = None;

litex_hal::uart! {
    Uart: litex_pac::UART,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}

litex_hal::gpio! {
    CTL: litex_pac::OLED_CTL,
}

litex_hal::spi! {
    SPI: (litex_pac::OLED_SPI, u8),
}

#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        unsafe {
            riscv::interrupt::disable();
            ENCODER.start_frame(do_write);
        }
    }
    unsafe fn flush() {}
    unsafe fn release() {
        ENCODER.end_frame(do_write);
        unsafe {
            riscv::interrupt::enable();
        }
    }
    unsafe fn write(bytes: &[u8]) {
        ENCODER.write(bytes, do_write);
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    loop {}
}

fn do_write(bytes: &[u8]) {
    unsafe {
        if let Some(writer) = &mut UART_WRITER {
            writer.bwrite_all(bytes).ok();
        }
    }
}

#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    unsafe {
        UART_WRITER = Some(Uart::new(peripherals.UART));
        if let Some(writer) = &mut UART_WRITER {
            writer
                .bwrite_all(b"hello from litex-fw! dropping to defmt logging --\n")
                .ok();
        }
    }

    let mut elapsed: f32 = 0.0f32;

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    timer.delay_ms(100u32);
    peripherals.EURORACK_PMOD0.csr_reset.write(|w| unsafe { w.bits(1) });
    peripherals.EURORACK_PMOD1.csr_reset.write(|w| unsafe { w.bits(1) });
    timer.delay_ms(100u32);
    peripherals.EURORACK_PMOD0.csr_reset.write(|w| unsafe { w.bits(0) });
    peripherals.EURORACK_PMOD1.csr_reset.write(|w| unsafe { w.bits(0) });

    let dc = CTL { index: 0 };
    let mut rstn = CTL { index: 1 };
    let mut csn = CTL { index: 2 };
    let spi = SPI {
        registers: peripherals.OLED_SPI
    };

    // Create the SpiInterface and Display.
    let mut disp = oled::Display::new(
        oled::SpiInterface::new(spi, dc),
        oled::PixelCoord(256, 64),
        oled::PixelCoord(112, 0),
    );

    csn.set_low().unwrap();

    // Assert the display's /RESET for 10ms.
    timer.delay_ms(10_u16);
    rstn.set_low().unwrap();
    timer.delay_ms(10_u16);
    rstn.set_high().unwrap();

    timer.delay_ms(1_u16);

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


    defmt::info!("Starting main loop --");

    loop {
        /*

        defmt::info!("tick - elapsed {} sec", elapsed);
        elapsed += 1.0f32;

        defmt::info!("PMOD0");
        defmt::info!("jack_detect {=u8:x}", peripherals.EURORACK_PMOD0.csr_jack.read().bits() as u8);
        defmt::info!("input0 {}", peripherals.EURORACK_PMOD0.csr_cal_in0.read().bits() as i16);
        defmt::info!("input1 {}", peripherals.EURORACK_PMOD0.csr_cal_in1.read().bits() as i16);
        defmt::info!("input2 {}", peripherals.EURORACK_PMOD0.csr_cal_in2.read().bits() as i16);
        defmt::info!("input3 {}", peripherals.EURORACK_PMOD0.csr_cal_in3.read().bits() as i16);
        defmt::info!("serial {=u32:x}", peripherals.EURORACK_PMOD0.csr_eeprom_serial.read().bits() as u32);
        timer.delay_ms(1000u32);
        defmt::info!("PMOD1");
        defmt::info!("jack_detect {=u8:x}", peripherals.EURORACK_PMOD1.csr_jack.read().bits() as u8);
        defmt::info!("input0 {}", peripherals.EURORACK_PMOD1.csr_cal_in0.read().bits() as i16);
        defmt::info!("input1 {}", peripherals.EURORACK_PMOD1.csr_cal_in1.read().bits() as i16);
        defmt::info!("input2 {}", peripherals.EURORACK_PMOD1.csr_cal_in2.read().bits() as i16);
        defmt::info!("input3 {}", peripherals.EURORACK_PMOD1.csr_cal_in3.read().bits() as i16);
        defmt::info!("serial {=u32:x}", peripherals.EURORACK_PMOD1.csr_eeprom_serial.read().bits() as u32);
        timer.delay_ms(1000u32);
        */

        let rect_style = PrimitiveStyleBuilder::new()
            .stroke_color(Gray4::new(0x5))
            .stroke_width(1)
            .fill_color(Gray4::BLACK)
            .build();

        disp
            .bounding_box()
            .into_styled(rect_style)
            .draw(&mut disp).ok();

        let circle = Circle::new(Point::new(22, 22), 20)
            .into_styled(PrimitiveStyle::with_stroke(Gray4::WHITE, 1));

        circle.draw(&mut disp).unwrap();

        let style = MonoTextStyle::new(&FONT_6X10, Gray4::WHITE);

        Text::new("Hello,\nRust!", Point::new(0, 12), style).draw(&mut disp).ok();

        let mut s: String<128> = String::new();
        write!(&mut s, "serial0 {:#06x}", peripherals.EURORACK_PMOD0.csr_eeprom_serial.read().bits() as u32).ok();

        Text::new(&s, Point::new(128, 12), style).draw(&mut disp).ok();

        s.clear();

        write!(&mut s, "serial1 {:#06x}", peripherals.EURORACK_PMOD1.csr_eeprom_serial.read().bits() as u32).ok();

        Text::new(&s, Point::new(128, 24), style).draw(&mut disp).ok();

        s.clear();

        write!(&mut s, "jack0 {:#06x}", peripherals.EURORACK_PMOD0.csr_jack.read().bits() as u32).ok();

        Text::new(&s, Point::new(128, 36), style).draw(&mut disp).ok();

        s.clear();

        write!(&mut s, "jack1 {:#06x}", peripherals.EURORACK_PMOD1.csr_jack.read().bits() as u32).ok();

        Text::new(&s, Point::new(128, 48), style).draw(&mut disp).ok();

        disp.flush();

    }
}
