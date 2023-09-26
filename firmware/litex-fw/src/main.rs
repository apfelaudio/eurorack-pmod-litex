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
use aligned_array::{Aligned, A4};
use vexriscv;

mod log;
mod libvult;
use log::*;

const SYSTEM_CLOCK_FREQUENCY: u32 = 50_000_000;

litex_hal::uart! {
    Uart: litex_pac::UART,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}

const N_CHANNELS: usize = 4;
const BUF_SZ_WORDS: usize = 512;
const BUF_SZ_SAMPLES: usize = BUF_SZ_WORDS * 2;

// MUST be aligned to 4-byte (word) boundary for RV32. These buffers are directly
// accessed by DMA that iterates across words!.
static mut BUF_IN: Aligned<A4, [i16; BUF_SZ_SAMPLES]> = Aligned([0i16; BUF_SZ_SAMPLES]);
static mut BUF_OUT: Aligned<A4, [i16; BUF_SZ_SAMPLES]> = Aligned([0i16; BUF_SZ_SAMPLES]);

static mut VULTA: Option<libvult::DspProcessType> = None;
static mut VULTB: Option<libvult::DspProcessType> = None;

#[export_name = "DefaultHandler"]
unsafe fn irq_handler() {

    let pending_irq = vexriscv::register::vmip::read();
    let peripherals = pac::Peripherals::steal();

    if (pending_irq & (1 << pac::Interrupt::DMA_ROUTER0 as usize)) != 0 {
        let offset = peripherals.DMA_ROUTER0.offset_words.read().bits();
        let pending_subtype = peripherals.DMA_ROUTER0.ev_pending.read().bits();

        if let Some(ref mut vulta) = VULTA {
        if let Some(ref mut vultb) = VULTB {

            if offset as usize == ((BUF_SZ_WORDS/2)+1) {
                for i in 0..(BUF_SZ_SAMPLES/2) {
                    BUF_OUT[i] = match i % 4 {
                        0 => vulta.process(BUF_IN[i]),
                        1 => vultb.process(BUF_IN[i]),
                        _ => 0
                    }
                }
            }

            /*
            peripherals.TIMER0.uptime_latch.write(|w| w.bits(1));
            let trace_start = peripherals.TIMER0.uptime_cycles0.read().bits();
            */

            if offset as usize == (BUF_SZ_WORDS-1) {
                for i in (BUF_SZ_SAMPLES/2)..(BUF_SZ_SAMPLES) {
                    BUF_OUT[i] = match i % 4 {
                        0 => vulta.process(BUF_IN[i]),
                        1 => vultb.process(BUF_IN[i]),
                        _ => 0
                    }
                }
            }

            /*
            peripherals.TIMER0.uptime_latch.write(|w| w.bits(1));
            let trace_end = peripherals.TIMER0.uptime_cycles0.read().bits();
            let trace_diff = trace_end - trace_start;
            // cycles for half the samples
            info!("{} {}", trace_diff, trace_diff >> 8);
            */
        }
        }

        peripherals.DMA_ROUTER0.ev_pending.write(|w| w.bits(pending_subtype));
    }
}


#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    for i in 0..BUF_SZ_SAMPLES {
        unsafe {
            BUF_OUT[i] = (i*256) as i16;
        }
    }

    unsafe {
        VULTA = Some(libvult::DspProcessType::new());
        VULTB = Some(libvult::DspProcessType::new());

        peripherals.DMA_ROUTER0.base_writer.write(|w| w.bits(BUF_IN.as_mut_ptr() as u32));
        peripherals.DMA_ROUTER0.base_reader.write(|w| w.bits(BUF_OUT.as_ptr() as u32));
        peripherals.DMA_ROUTER0.length_words.write(|w| w.bits(BUF_SZ_WORDS as u32));
        peripherals.DMA_ROUTER0.enable.write(|w| w.bits(1u32));
        peripherals.DMA_ROUTER0.ev_enable.write(|w| w.half().bit(true));

        // Enable interrupts from DMA router (vexriscv specific register)
        vexriscv::register::vmim::write(1 << (pac::Interrupt::DMA_ROUTER0 as usize));

        // Enable machine external interrupts (basically everything added on by LiteX).
        riscv::register::mie::set_mext();

        // Finally enable interrupts.
        riscv::interrupt::enable();
    }

    loop {
        /*
        log::info!("READ");
        unsafe {
            asm!("fence iorw, iorw");
            for i in 0..4 {
                log::info!("{:x}@{:x}", i, BUF_IN[i]);

            }
        }
        timer.delay_ms(500u32);
        */
    }
}
