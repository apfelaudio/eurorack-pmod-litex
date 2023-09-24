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
use aligned_array::{Aligned, A4};

mod log;
use log::*;

const SYSTEM_CLOCK_FREQUENCY: u32 = 50_000_000;

litex_hal::uart! {
    Uart: litex_pac::UART,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}

const N_CHANNELS: usize = 4;
const BUF_SZ_WORDS: usize = 64;
const BUF_SZ_SAMPLES: usize = BUF_SZ_WORDS * 2;

// MUST be aligned to 4-byte (word) boundary for RV32. These buffers are directly
// accessed by DMA that iterates across words!.
static mut BUF_IN: Aligned<A4, [i16; BUF_SZ_SAMPLES]> = Aligned([0i16; BUF_SZ_SAMPLES]);
static mut BUF_OUT: Aligned<A4, [i16; BUF_SZ_SAMPLES]> = Aligned([0i16; BUF_SZ_SAMPLES]);

#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let mut elapsed: f32 = 0.0f32;

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    for i in 0..BUF_SZ_SAMPLES {
        unsafe {
            //BUF_OUT_CP[i] = (16000.0f32*f32::sin(2.0f32*3.141f32*i as f32 / BUF_SZ_WORDS as f32)) as i16;
            BUF_OUT[i] = (i*256) as i16;
        }
    }

    unsafe {
        peripherals.DMA_ROUTER0.base_writer.write(|w| w.bits(BUF_IN.as_mut_ptr() as u32));
        peripherals.DMA_ROUTER0.base_reader.write(|w| w.bits(BUF_OUT.as_ptr() as u32));
        peripherals.DMA_ROUTER0.length_words.write(|w| w.bits(BUF_SZ_WORDS as u32));
        peripherals.DMA_ROUTER0.enable.write(|w| w.bits(1u32));
        peripherals.DMA_ROUTER0.ev_enable.write(|w| w.half().bit(true));

        asm!(
            "li {x}, 0xfff",
            "csrw mie, {x}",
            x = out(reg) _,
            );

        // VexRiscv // 0xBC0 == machineMaskCsrId // GenCoreDefault.scala
        // 0x4 + 0x8 = 0xC => IRQ 2 & IRQ 3 (dmaw.half, dmar.half)
        asm!(
            "li {x}, 0x4",
            "csrw 0xBC0, {x}",
            x = out(reg) _,
            );

        riscv::interrupt::enable();
    }

    loop {
        /*
        let proc = |s| {
            if (s > 8000) {
                8000
            } else if (s < -8000) {
                -8000
            } else {
                s
            }
        };
        unsafe {
            asm!("fence iorw, iorw");
            if BUF_OUT_SENT_LO && BUF_IN_NEW_LO {
                for i in 0..(BUF_SZ_WORDS/2) {
                    BUF_OUT_CP[i] = proc(BUF_IN_CP[i]);
                }
                BUF_IN_NEW_LO = false;
                BUF_OUT_SENT_LO = false;
            }
            if BUF_OUT_SENT_HI && BUF_IN_NEW_HI {
                for i in (BUF_SZ_WORDS/2)..(BUF_SZ_WORDS) {
                    BUF_OUT_CP[i] = proc(BUF_IN_CP[i]);
                }
                BUF_IN_NEW_HI = false;
                BUF_OUT_SENT_HI = false;
            }
        }
        */
        /*
        log::info!("READ");
        unsafe {
            asm!("fence iorw, iorw");
            for i in 0..BUF_SZ_SAMPLES {
                log::info!("{:x}@{:x}", i, BUF_IN[i]);

            }
        }
        timer.delay_ms(10u32);
        */
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
