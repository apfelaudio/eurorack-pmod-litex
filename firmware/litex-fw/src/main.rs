#![no_std]
#![no_main]
#![allow(dead_code)]

/// Simple example of Rust firmware to do real-time audio DSP on
/// a LiteX-created VexRiscV SMP SoC. Assumes a single core.
///
/// This example implements a fixed-point low-pass filter of which
/// you can see the implementation in `dsp.rs`.
///
/// - Uses RISCV-PLIC (platform-level interrupt controller) to
///   claim and handle interrupts. This makes it easy to scale to
///   more cores as SMP VexRISCV only supports PLIC by default.
/// - Static buffers are managed by the 'DMA router' which is a separate
///   LiteX peripheral shuffling data back/forth to eurorack-pmod.
/// - IRQs fire when the buffer is half-full AND full such that the
///   ISR can process audio samples in real-time without glitches.
///

use heapless::*;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use litex_pac as pac;
use riscv_rt::entry;
use riscv;
use core::arch::asm;
use aligned_array::{Aligned, A4};

mod log;
mod plic;
mod dsp;
use log::*;
use plic::*;
use dsp::*;

/// TODO: Modify `svd2rust` so this can be automatically forwarded?
const SYSTEM_CLOCK_FREQUENCY: u32 = 75_000_000;

/// Number of channels per section (4x input, 4x output)
const N_CHANNELS: usize = 4;

/// Size of DMA buffers in 32-bit words.
const BUF_SZ_WORDS: usize = 512;

/// Each sample is 16-bits wide, so each buffer can store twice as many samples.
const BUF_SZ_SAMPLES: usize = BUF_SZ_WORDS * 2;

/// Each IRQ shall process half of the buffers as they are circularly serviced.
const BUF_SZ_SAMPLES_PER_IRQ: usize = BUF_SZ_SAMPLES / 2;

// Note: these buffers MUST be word-aligned because the DMA engine iterates at word granularity.

/// Static DMA buffer, circularly written to by the DMA engine to pipe samples IN from the eurorack-pmod.
static mut BUF_IN: Aligned<A4, [i16; BUF_SZ_SAMPLES]> = Aligned([0i16; BUF_SZ_SAMPLES]);

/// Static DMA buffer, circularly read by the DMA engine to pipe samples OUT of the eurorack-pmod.
static mut BUF_OUT: Aligned<A4, [i16; BUF_SZ_SAMPLES]> = Aligned([0i16; BUF_SZ_SAMPLES]);

/// Some global state to track how long IRQs are taking to service.
static mut LAST_IRQ: u32 = 0;
static mut LAST_IRQ_LEN: u32 = 0;
static mut LAST_IRQ_PERIOD: u32 = 0;

/// Persistent state required for LPF DSP operations.
static mut KARLSEN_LPF: Option<KarlsenLpf> = None;

// Map the RISCV IRQ PLIC onto the fixed address present in the VexRISCV implementation.
// TODO: ideally fetch this from the svf, its currently not exported by `svd2rust`!
riscv::plic_context!(PLIC0, 0xf0c0_0000, 0, VexInterrupt, VexPriority);

// Create the HAL bindings for the remaining LiteX peripherals.

litex_hal::uart! {
    Uart: litex_pac::UART,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}


/// Called once per IRQ to service the appropriate half of the DMA buffer.
/// Warn: this is run in IRQ context and blocks all IRQs while it is running.
/// You will want to re-think if this lives in a world with other IRQs.
fn process(dsp: &mut KarlsenLpf, buf_out: &mut [i16], buf_in: &[i16]) {
    for i in 0..(buf_in.len()/N_CHANNELS) {
        // Convert all input channels to an approprate fixed-point representation
        let x_in: [Fix; N_CHANNELS] = [
            Fix::from_bits(buf_in[N_CHANNELS*i+0] as i32),
            Fix::from_bits(buf_in[N_CHANNELS*i+1] as i32),
            Fix::from_bits(buf_in[N_CHANNELS*i+2] as i32),
            Fix::from_bits(buf_in[N_CHANNELS*i+3] as i32),
        ];
        // Feed them into our DSP function
        let y: Fix = dsp.proc(x_in[0], x_in[1], x_in[2]);

        // We only have 1 output, so use that for output 1 and just
        // mirror inputs straight to outputs for the rest.
        buf_out[N_CHANNELS*i+0] = y.to_bits() as i16;
        buf_out[N_CHANNELS*i+1] = x_in[1].to_bits() as i16;
        buf_out[N_CHANNELS*i+2] = x_in[2].to_bits() as i16;
        buf_out[N_CHANNELS*i+3] = x_in[3].to_bits() as i16;
    }
}

/// Flush all VexRiscv caches, this is necessary for when we are writing to
/// a buffer which will be read by the DMA engine soon.
unsafe fn fence() {
    asm!("fence iorw, iorw");
    // This magic instruction was just found in the LiteX source tree in
    // the C implementation which is copied over for CSR operations.
    // Without it, the caches aren't completely flushed.
    asm!(".word(0x500F)");
}

/// Handler for ALL IRQs.
#[export_name = "DefaultHandler"]
unsafe fn irq_handler() {

    // First, claim the IRQ for this core.
    // This should only ever fail if we have multiple cores claiming irqs
    // in an SMP environment (which is not the case for this simple example).
    let pending_irq = PLIC0::claim().unwrap();
    let peripherals = pac::Peripherals::steal();

    // Keep track of how long we spend in IRQs
    peripherals.TIMER0.uptime_latch().write(|w| w.bits(1));
    let trace = peripherals.TIMER0.uptime_cycles0().read().bits();
    LAST_IRQ_PERIOD = trace - LAST_IRQ;
    LAST_IRQ = trace;

    match pending_irq.pac_irq {
        pac::Interrupt::DMA_ROUTER0 => {

            // Read the current position in the circular DMA buffer to determine whether
            // we need to service the first or last half of the DMA buffer.

            let offset = peripherals.DMA_ROUTER0.offset_words().read().bits();
            let pending_subtype = peripherals.DMA_ROUTER0.ev_pending().read().bits();

            if let Some(ref mut dsp) = KARLSEN_LPF {
                if offset as usize == ((BUF_SZ_WORDS/2)+1) {
                    fence();
                    process(dsp,
                            &mut BUF_OUT[0..(BUF_SZ_SAMPLES/2)],
                            &BUF_IN[0..(BUF_SZ_SAMPLES/2)]);
                }

                if offset as usize == (BUF_SZ_WORDS-1) {
                    fence();
                    process(dsp,
                            &mut BUF_OUT[(BUF_SZ_SAMPLES/2)..(BUF_SZ_SAMPLES)],
                            &BUF_IN[(BUF_SZ_SAMPLES/2)..(BUF_SZ_SAMPLES)]);
                }
            }

            peripherals.DMA_ROUTER0.ev_pending().write(|w| w.bits(pending_subtype));

            fence();
        },
        _ => {
            // We shouldn't have any other types of IRQ...
        }
    }


    PLIC0::complete(pending_irq);

    peripherals.TIMER0.uptime_latch().write(|w| w.bits(1));
    let trace_end = peripherals.TIMER0.uptime_cycles0().read().bits();
    LAST_IRQ_LEN = trace_end - trace;
}


#[entry]
fn main() -> ! {

    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    unsafe {
        // Create an instance of our DSP function which has some internal state.
        // Rust irq / scoped irq crates are neater for this kind of thing but
        // using statics to reduce dependencies / make it obvious what this is doing.
        KARLSEN_LPF = Some(KarlsenLpf::new());

        // Configure the DMA engine such that it uses our static buffers, and start it.
        peripherals.DMA_ROUTER0.base_writer().write(|w| w.bits(BUF_IN.as_mut_ptr() as u32));
        peripherals.DMA_ROUTER0.base_reader().write(|w| w.bits(BUF_OUT.as_ptr() as u32));
        peripherals.DMA_ROUTER0.length_words().write(|w| w.bits(BUF_SZ_WORDS as u32));
        peripherals.DMA_ROUTER0.enable().write(|w| w.bits(1u32));
        peripherals.DMA_ROUTER0.ev_enable().write(|w| w.half().bit(true));

        // RISC-V PLIC configuration.
        let mut plic = PLIC0::new();
        let dma_irq = VexInterrupt::from(pac::Interrupt::DMA_ROUTER0);
        plic.set_threshold(VexPriority::from(0));
        plic.set_priority(dma_irq, VexPriority::from(1));
        plic.enable_interrupt(dma_irq);

        // Enable machine external interrupts (basically everything added on by LiteX).
        riscv::register::mie::set_mext();

        // Finally enable interrupts.
        riscv::interrupt::enable();
    }

    loop {
        unsafe {
            fence();

            // Print the current value of every input and output channel.
            for i in 0..4 {
                log::info!("{:x}@{:x},{:x}", i, BUF_IN[i], BUF_OUT[i]);
            }

            // Print out some metrics as to how long our DSP operations are taking.
            log::info!("irq_period: {}", LAST_IRQ_PERIOD);
            log::info!("irq_len: {}", LAST_IRQ_LEN);
            if LAST_IRQ_PERIOD != 0 {
                log::info!("irq_load_percent: {}", (LAST_IRQ_LEN * 100) / LAST_IRQ_PERIOD);
            }
        }

        timer.delay_ms(500u32);
    }
}
