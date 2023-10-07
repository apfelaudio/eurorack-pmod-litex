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

use fixed::{FixedI32, types::extra::U16};

mod log;
use log::*;

const SYSTEM_CLOCK_FREQUENCY: u32 = 60_000_000;

litex_hal::uart! {
    Uart: litex_pac::UART,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}

const N_CHANNELS: usize = 4;
const BUF_SZ_WORDS: usize = 512;
const BUF_SZ_SAMPLES: usize = BUF_SZ_WORDS * 2;
const BUF_SZ_SAMPLES_PER_IRQ: usize = BUF_SZ_SAMPLES / 2;

// MUST be aligned to 4-byte (word) boundary for RV32. These buffers are directly
// accessed by DMA that iterates across words!.
static mut BUF_IN: Aligned<A4, [i16; BUF_SZ_SAMPLES]> = Aligned([0i16; BUF_SZ_SAMPLES]);
static mut BUF_OUT: Aligned<A4, [i16; BUF_SZ_SAMPLES]> = Aligned([0i16; BUF_SZ_SAMPLES]);

static mut LAST_IRQ: u32 = 0;
static mut LAST_IRQ_LEN: u32 = 0;
static mut LAST_IRQ_PERIOD: u32 = 0;

#[derive(Copy, Clone)]
pub struct VexInterrupt {
    pac_irq: pac::Interrupt,
}

unsafe impl riscv::peripheral::plic::InterruptNumber for VexInterrupt {
    // TODO: this is PLIC max -- fetch this from PAC?
    const MAX_INTERRUPT_NUMBER: u16 = 32u16;

    fn number(self) -> u16 {
        self.pac_irq as u16
    }

    fn try_from(value: u16) -> Result<Self, u16> {
        if let Ok(value_u8) = value.try_into() {
            match pac::Interrupt::try_from(value_u8) {
                Ok(result) => Ok(VexInterrupt { pac_irq: result } ),
                Err(_) => Err(value)
            }
        } else {
            Err(value)
        }
    }
}

impl From<pac::Interrupt> for VexInterrupt {
    fn from(pac_irq: pac::Interrupt) -> Self {
        VexInterrupt { pac_irq }
    }
}

#[derive(Copy, Clone)]
pub struct VexPriority {
    prio: u8,
}

unsafe impl riscv::peripheral::plic::PriorityNumber for VexPriority {
    // TODO: Customize Vex for more priority numbers?
    const MAX_PRIORITY_NUMBER: u8 = 0x3u8;
    fn number(self) -> u8 { self.prio }
    fn try_from(value: u8) -> Result<Self, u8> { Ok(VexPriority{prio: value}) }
}

impl From<u8> for VexPriority {
    fn from(prio: u8) -> Self {
        VexPriority { prio }
    }
}

riscv::plic_context!(PLIC0, 0xf0c0_0000, 0, VexInterrupt, VexPriority);
static mut PLIC: Option<PLIC0> = None;


static mut PITCH: Option<PitchShift> = None;

type Fix = FixedI32<U16>;

struct KarlsenLpf {
    rezz: Fix,
    sat : Fix,
    a1  : Fix,
    a2  : Fix,
    a3  : Fix,
    a4  : Fix,
    dc_block : DcBlock,
}

struct DcBlock {
    x_k1: Fix,
    y_k1: Fix,
}

const DELAY_MAX: usize = 2048;

struct DelayLine {
    idx: usize,
    x: [Fix; DELAY_MAX],
}

impl DelayLine {
    fn new() -> Self {
        Self {
            idx: 0,
            x: [Fix::from_num(0); DELAY_MAX],
        }
    }
    fn add(&mut self, x: Fix) {
        self.x[self.idx] = x;
        self.idx = (self.idx + 1) % DELAY_MAX;
    }
    fn delayed(&self, delay: usize) -> Fix {
        self.x[(DELAY_MAX + (self.idx - delay - 1)) % DELAY_MAX]
    }
}

const WINDOW: usize = 512;
const XFADE: usize = 128;

struct PitchShift {
    delayline: DelayLine,
    delay: Fix,
}

impl PitchShift {
    fn new() -> Self {
        Self {
            delayline: DelayLine::new(),
            delay: Fix::from_num(0),
        }
    }
    fn proc(&mut self, x: Fix, pitch: Fix) -> Fix {
        let delay_int = self.delay.to_num::<usize>();
        let delay0 = self.delayline.delayed(delay_int);
        let delay1 = self.delayline.delayed(delay_int + WINDOW);
        let env0 = Fix::min(Fix::ONE, self.delay / Fix::from_num(XFADE));
        let env1 = Fix::max(Fix::ZERO, Fix::ONE - env0);
        self.delay += pitch;
        if self.delay >= WINDOW*2 {
            self.delay = Fix::ZERO;
        } else if self.delay <= Fix::ZERO {
            self.delay = Fix::from_num(WINDOW*2);
        }
        self.delayline.add(x);
        env0 * delay0 + env1 * delay1
    }
}

impl DcBlock {
    fn new() -> Self {
        DcBlock {
            x_k1: Fix::from_num(0),
            y_k1: Fix::from_num(0),
        }
    }

    /// -1 <= x <= 1 (audio sample)
    fn proc(&mut self, x_k: Fix) -> Fix {
        self.y_k1 = (x_k - self.x_k1) + Fix::from_num(0.99f32) * self.y_k1;
        self.x_k1 = x_k;
        self.y_k1
    }
}

impl KarlsenLpf {
    fn new() -> Self {
        KarlsenLpf {
            rezz: Fix::from_num(0),
            sat: Fix::from_num(0),
            a1: Fix::from_num(0),
            a2: Fix::from_num(0),
            a3: Fix::from_num(0),
            a4: Fix::from_num(0),
            dc_block: DcBlock::new(),
        }
    }

    /// -1 <= x <= 1 (audio sample)
    /// 0 <= g <= 1 (cutoff)
    /// 0 <= res <= 1 (resonance)
    fn proc(&mut self, x: Fix, g: Fix, res: Fix) -> Fix {
        let gmax = Fix::max(Fix::from_num(0), g);
        let res_scaled = Fix::max(Fix::from_num(0), res * 4);
        self.rezz = x - ((self.a4 - x) * res_scaled);
        self.sat = Fix::min(Fix::ONE, self.rezz);
        self.sat = Fix::max(-Fix::ONE, self.rezz);
        self.a1 = self.a1 + ((-self.a1 + self.sat) * gmax);
        self.a2 = self.a2 + ((-self.a2 + self.a1) * gmax);
        self.a3 = self.a3 + ((-self.a3 + self.a2) * gmax);
        self.a4 = self.a4 + ((-self.a4 + self.a3) * gmax);
        self.dc_block.proc(self.a4)
    }
}

fn process(dsp: &mut PitchShift, buf_out: &mut [i16], buf_in: &[i16]) {
    for i in 0..(buf_in.len()/N_CHANNELS) {
        let x_in: [Fix; N_CHANNELS] = [
            Fix::from_bits(buf_in[N_CHANNELS*i+0] as i32),
            Fix::from_bits(buf_in[N_CHANNELS*i+1] as i32),
            Fix::from_bits(buf_in[N_CHANNELS*i+2] as i32),
            Fix::from_bits(buf_in[N_CHANNELS*i+3] as i32),
        ];
        let y: Fix = dsp.proc(x_in[0], x_in[1]);
        buf_out[N_CHANNELS*i+0] = y.to_bits() as i16;
        buf_out[N_CHANNELS*i+1] = x_in[1].to_bits() as i16;
        buf_out[N_CHANNELS*i+2] = x_in[2].to_bits() as i16;
        buf_out[N_CHANNELS*i+3] = x_in[3].to_bits() as i16;
    }
}

unsafe fn fence() {
    asm!("fence iorw, iorw");
    asm!(".word(0x500F)");
}

#[export_name = "DefaultHandler"]
unsafe fn irq_handler() {

    // This should only fail if we have multiple cores claiming things
    let pending_irq = PLIC0::claim().unwrap();
    let peripherals = pac::Peripherals::steal();

    peripherals.TIMER0.uptime_latch.write(|w| w.bits(1));
    let trace = peripherals.TIMER0.uptime_cycles0.read().bits();
    LAST_IRQ_PERIOD = trace - LAST_IRQ;
    LAST_IRQ = trace;


    match pending_irq.pac_irq {
        pac::Interrupt::DMA_ROUTER0 => {
            let offset = peripherals.DMA_ROUTER0.offset_words.read().bits();
            let pending_subtype = peripherals.DMA_ROUTER0.ev_pending.read().bits();

            if let Some(ref mut dsp) = PITCH {
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

            peripherals.DMA_ROUTER0.ev_pending.write(|w| w.bits(pending_subtype));

            fence();
        },
        _ => {
        }
    }


    PLIC0::complete(pending_irq);

    peripherals.TIMER0.uptime_latch.write(|w| w.bits(1));
    let trace_end = peripherals.TIMER0.uptime_cycles0.read().bits();
    LAST_IRQ_LEN = trace_end - trace;
}


#[entry]
fn main() -> ! {

    if riscv::register::mhartid::read() != 0 {
        // If we are SMP, block all cores except core 0.
        loop {};
    }

    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    for i in 0..BUF_SZ_SAMPLES {
        unsafe {
            BUF_OUT[i] = (i*32) as i16;
        }
    }

    unsafe {
        PITCH = Some(PitchShift::new());

        peripherals.DMA_ROUTER0.base_writer.write(|w| w.bits(BUF_IN.as_mut_ptr() as u32));
        peripherals.DMA_ROUTER0.base_reader.write(|w| w.bits(BUF_OUT.as_ptr() as u32));
        peripherals.DMA_ROUTER0.length_words.write(|w| w.bits(BUF_SZ_WORDS as u32));
        peripherals.DMA_ROUTER0.enable.write(|w| w.bits(1u32));
        peripherals.DMA_ROUTER0.ev_enable.write(|w| w.half().bit(true));

        // PLIC configuration
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
            asm!("fence iorw, iorw");
            for i in 0..4 {
                log::info!("{:x}@{:x},{:x}", i, BUF_IN[i], BUF_OUT[i]);
            }
            log::info!("{}", riscv::register::mhartid::read());
            log::info!("irq_period: {}", LAST_IRQ_PERIOD);
            log::info!("irq_len: {}", LAST_IRQ_LEN);
            if LAST_IRQ_PERIOD != 0 {
                log::info!("irq_load_percent: {}", (LAST_IRQ_LEN * 100) / LAST_IRQ_PERIOD);
            }
        }
        timer.delay_ms(500u32);
    }
}
