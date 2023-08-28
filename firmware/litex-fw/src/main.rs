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

    defmt::info!("Starting main loop --");

    loop {
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
        defmt::info!("tick - elapsed {} sec", elapsed);
        elapsed += 1.0f32;
    }
}
