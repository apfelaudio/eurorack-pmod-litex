#![no_std]
#![no_main]
#![allow(dead_code)]

use heapless::*;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use litex_pac as pac;
use riscv_rt::entry;
use riscv;
use core::arch::asm;
use micromath::F32Ext;

mod log;
use log::*;

const SYSTEM_CLOCK_FREQUENCY: u32 = 50_000_000;

litex_hal::uart! {
    Uart: litex_pac::UART,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}

const BUF_SZ_WORDS: usize = 128;
const BUF_SZ_BYTES: usize = BUF_SZ_WORDS * 4;

static mut BUF_OUT: [u32; BUF_SZ_WORDS] = [0; BUF_SZ_WORDS];
static mut BUF_IN: [u32; BUF_SZ_WORDS] = [0; BUF_SZ_WORDS];

static mut BUF_IN_CP: [i16; BUF_SZ_WORDS] = [0; BUF_SZ_WORDS];
static mut BUF_OUT_CP: [i16; BUF_SZ_WORDS] = [0; BUF_SZ_WORDS];

#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let mut elapsed: f32 = 0.0f32;

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    for i in 0..BUF_SZ_WORDS {
        unsafe {
            BUF_OUT_CP[i] = (16000.0f32*f32::sin(2.0f32*3.141f32*i as f32 / BUF_SZ_WORDS as f32)) as i16;
        }
    }

    unsafe {
        peripherals.DMA_READER0.base0.write(|w| w.bits(BUF_OUT.as_ptr() as u32));
        peripherals.DMA_READER0.length.write(|w| w.bits(BUF_SZ_BYTES as u32));
        peripherals.DMA_READER0.loop_.write(|w| w.bits(1u32));
        peripherals.DMA_READER0.enable.write(|w| w.bits(1u32));

        peripherals.DMA_WRITER0.base0.write(|w| w.bits(BUF_IN.as_mut_ptr() as u32));
        peripherals.DMA_WRITER0.length.write(|w| w.bits(BUF_SZ_BYTES as u32));
        peripherals.DMA_WRITER0.loop_.write(|w| w.bits(1u32));
        peripherals.DMA_WRITER0.enable.write(|w| w.bits(1u32));

        peripherals.DMA_READER0.ev_enable.write(|w| w.half().bit(true));
        peripherals.DMA_WRITER0.ev_enable.write(|w| w.half().bit(true));

        asm!(
            "li {x}, 0xfff",
            "csrw mie, {x}",
            x = out(reg) _,
            );

        // VexRiscv // 0xBC0 == machineMaskCsrId // GenCoreDefault.scala
        // 0x4 + 0x8 = 0xC => IRQ 2 & IRQ 3 (dmaw.half, dmar.half)
        asm!(
            "li {x}, 0xC",
            "csrw 0xBC0, {x}",
            x = out(reg) _,
            );

        riscv::interrupt::enable();
    }

    loop {
        log::info!("READ");
        unsafe {
            asm!("fence iorw, iorw");
            for i in 0..BUF_SZ_WORDS {
                log::info!("{:x}@{:x}", i, BUF_IN_CP[i]);

            }
        }
        timer.delay_ms(500u32);
        /*
        log::info!("jack_detect {:x}", peripherals.EURORACK_PMOD0.csr_jack.read().bits() as u8);
        log::info!("input0 {}", peripherals.EURORACK_PMOD0.csr_cal_in0.read().bits() as i16);
        log::info!("input1 {}", peripherals.EURORACK_PMOD0.csr_cal_in1.read().bits() as i16);
        log::info!("input2 {}", peripherals.EURORACK_PMOD0.csr_cal_in2.read().bits() as i16);
        log::info!("input3 {}", peripherals.EURORACK_PMOD0.csr_cal_in3.read().bits() as i16);
        log::info!("serial {:x}", peripherals.EURORACK_PMOD0.csr_eeprom_serial.read().bits() as u32);

        log::info!("tick - elapsed {} msec", (elapsed * 1000.0) as u32);
        elapsed += 0.01f32;
        */
    }
}
