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

static mut VULTA: Option<libvult::VultDsp> = None;
static mut VULTB: Option<libvult::VultDsp> = None;
static mut VULTC: Option<libvult::VultDsp> = None;
static mut VULTD: Option<libvult::VultDsp> = None;

static mut LAST_IRQ: u32 = 0;
static mut LAST_IRQ_LEN: u32 = 0;
static mut LAST_IRQ_PERIOD: u32 = 0;

#[export_name = "DefaultHandler"]
unsafe fn irq_handler() {


    let pending_irq = vexriscv::register::vmip::read();
    let peripherals = pac::Peripherals::steal();

    peripherals.TIMER0.uptime_latch.write(|w| w.bits(1));
    let trace = peripherals.TIMER0.uptime_cycles0.read().bits();
    LAST_IRQ_PERIOD = trace - LAST_IRQ;
    LAST_IRQ = trace;

    if (pending_irq & (1 << pac::Interrupt::DMA_ROUTER0 as usize)) != 0 {
        let offset = peripherals.DMA_ROUTER0.offset_words.read().bits();
        let pending_subtype = peripherals.DMA_ROUTER0.ev_pending.read().bits();

        if let Some(ref mut vulta) = VULTA {
        if let Some(ref mut vultb) = VULTB {
        if let Some(ref mut vultc) = VULTC {
        if let Some(ref mut vultd) = VULTD {

            if offset as usize == ((BUF_SZ_WORDS/2)+1) {
                for i in 0..(BUF_SZ_SAMPLES/2) {
                    BUF_OUT[i] = match i % 4 {
                        0 => vulta.process(BUF_IN[i]),
                        //1 => vultb.process(BUF_IN[i]),
                        2 => vultc.process(BUF_IN[i]),
                        //3 => vultd.process(BUF_IN[i]),
                        _ => 0
                    }
                }
            }

            if offset as usize == (BUF_SZ_WORDS-1) {
                for i in (BUF_SZ_SAMPLES/2)..(BUF_SZ_SAMPLES) {
                    BUF_OUT[i] = match i % 4 {
                        0 => vulta.process(BUF_IN[i]),
                        //1 => vultb.process(BUF_IN[i]),
                        2 => vultc.process(BUF_IN[i]),
                        //3 => vultd.process(BUF_IN[i]),
                        _ => 0
                    }
                }
            }

        }
        }
        }
        }

        peripherals.DMA_ROUTER0.ev_pending.write(|w| w.bits(pending_subtype));

        asm!("fence iorw, iorw");
        asm!(".word(0x500F)");
    }

    peripherals.TIMER0.uptime_latch.write(|w| w.bits(1));
    let trace_end = peripherals.TIMER0.uptime_cycles0.read().bits();
    LAST_IRQ_LEN = trace_end - trace;
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
        VULTA = Some(libvult::VultDsp::new());
        VULTB = Some(libvult::VultDsp::new());
        VULTC = Some(libvult::VultDsp::new());
        VULTD = Some(libvult::VultDsp::new());

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
        unsafe {
            asm!("fence iorw, iorw");
            for i in 0..4 {
                log::info!("{:x}@{:x}", i, BUF_IN[i]);
            }
            log::info!("irq_period: {}", LAST_IRQ_PERIOD);
            log::info!("irq_len: {}", LAST_IRQ_LEN);
            if LAST_IRQ_PERIOD != 0 {
                log::info!("irq_load_percent: {}", (LAST_IRQ_LEN * 100) / LAST_IRQ_PERIOD);
            }
        }
        timer.delay_ms(500u32);
    }
}
