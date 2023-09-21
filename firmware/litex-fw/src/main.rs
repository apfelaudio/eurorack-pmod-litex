#![no_std]
#![no_main]
#![allow(dead_code)]

use heapless::String;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use litex_pac as pac;
use riscv_rt::entry;

mod log;
use log::*;

const SYSTEM_CLOCK_FREQUENCY: u32 = 50_000_000;

litex_hal::uart! {
    Uart: litex_pac::UART,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}

#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let mut elapsed: f32 = 0.0f32;

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    loop {
        log::info!("jack_detect {:x}", peripherals.EURORACK_PMOD0.csr_jack.read().bits() as u8);
        log::info!("input0 {}", peripherals.EURORACK_PMOD0.csr_cal_in0.read().bits() as i16);
        log::info!("input1 {}", peripherals.EURORACK_PMOD0.csr_cal_in1.read().bits() as i16);
        log::info!("input2 {}", peripherals.EURORACK_PMOD0.csr_cal_in2.read().bits() as i16);
        log::info!("input3 {}", peripherals.EURORACK_PMOD0.csr_cal_in3.read().bits() as i16);
        log::info!("serial {:x}", peripherals.EURORACK_PMOD0.csr_eeprom_serial.read().bits() as u32);

        unsafe {
            peripherals.EURORACK_PMOD0.csr_cal_out0.write(|w| w.bits(((elapsed * 1000.0) as i32) as u32));
            peripherals.EURORACK_PMOD0.csr_cal_out1.write(|w| w.bits(((-elapsed * 1000.0) as i16) as u32));
        }

        timer.delay_ms(10u32);
        log::info!("tick - elapsed {} msec", (elapsed * 1000.0) as u32);
        elapsed += 0.01f32;
    }
}
