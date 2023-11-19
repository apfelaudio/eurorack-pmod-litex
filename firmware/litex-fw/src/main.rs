#![cfg(not(test))]
#![allow(unused_imports)]

#![no_std]
#![no_main]

use litex_hal::prelude::*;
use litex_pac as pac;
use riscv_rt::entry;
use litex_hal::hal::digital::v2::OutputPin;
use heapless::String;
use embedded_midi::MidiIn;
use core::arch::asm;
use aligned_array::{Aligned, A4};
use core::cell::RefCell;
use critical_section::Mutex;
use irq::{handler, scope, scoped_interrupts};
use litex_interrupt::return_as_is;

use tinyusb_sys::{tusb_init, dcd_int_handler, tud_task};

use ssd1322 as oled;

use polyvec_lib::voice::*;
use polyvec_lib::draw;
use polyvec_lib::opt;

mod gw;
mod log;
use crate::gw::*;
use crate::log::*;

const N_VOICES: usize = 4;
const SCOPE_SAMPLES: usize = 256;
const N_CHANNELS: usize = 4;
const BUF_SZ_WORDS: usize = 512;
const BUF_SZ_SAMPLES: usize = BUF_SZ_WORDS * 2;
const TICK_MS: u32 = 5;

// MUST be aligned to 4-byte (word) boundary for RV32. These buffers are directly
// accessed by DMA that iterates across words!.
static mut BUF_IN: Aligned<A4, [i16; BUF_SZ_SAMPLES]> = Aligned([0i16; BUF_SZ_SAMPLES]);

scoped_interrupts! {

    #[allow(non_camel_case_types)]
    enum Interrupt {
        DMA_ROUTER0,
        TIMER0,
    }

    use #[return_as_is];
}

struct OScope {
    samples: [i16; SCOPE_SAMPLES],
    samples_dbl: [i16; SCOPE_SAMPLES],
    n_samples: usize,
    trig_lo: bool,
}

impl OScope {
    fn new() -> Self {
        OScope {
            samples: [0i16; SCOPE_SAMPLES],
            samples_dbl: [0i16; SCOPE_SAMPLES],
            n_samples: 0,
            trig_lo: false
        }
    }

    fn feed(&mut self, buf_in: &[i16]) {
        for i in 0..(buf_in.len()/N_CHANNELS) {
            let x_in: [i16; N_CHANNELS] = [
                buf_in[N_CHANNELS*i],
                buf_in[N_CHANNELS*i+1],
                buf_in[N_CHANNELS*i+2],
                buf_in[N_CHANNELS*i+3],
            ];

            self.trig_lo = self.trig_lo || x_in[0] < -4000;
            let trigger = x_in[0] > 4000 && self.trig_lo;

            if  (self.n_samples > 0 && self.n_samples != SCOPE_SAMPLES) ||
                (self.n_samples == 0 && trigger) {
                self.samples[self.n_samples] = x_in[0];
                self.n_samples += 1
            }
        }
    }

    fn full(&mut self) -> bool {
        self.n_samples == SCOPE_SAMPLES
    }

    fn reset(&mut self) {
        core::mem::swap(&mut self.samples, &mut self.samples_dbl);
        self.trig_lo = false;
        self.n_samples = 0;
    }
}

struct Trace {
    last_timestamp_cycles: u64,
    last_len_cycles: u64,
    last_period_cycles: u64,
}

impl Trace {
    fn new() -> Self {
        Self {
            last_timestamp_cycles: 0,
            last_len_cycles: 0,
            last_period_cycles: 0,
        }
    }

    fn start(&mut self, timer: &Timer) {
        let uptime = timer.uptime();
        self.last_period_cycles = uptime - self.last_timestamp_cycles;
        self.last_timestamp_cycles = uptime;
    }

    fn end(&mut self, timer: &Timer) {
        let uptime = timer.uptime();
        self.last_len_cycles = uptime - self.last_timestamp_cycles;
    }

    fn len_us(&self) -> u32 {
        (self.last_len_cycles / ((SYSTEM_CLOCK_FREQUENCY / 1_000_000u32) as u64)) as u32
    }
}

fn dma_router0_handler(dma_router: &Mutex<DmaRouter>, scope: &Mutex<RefCell<OScope>>) {
    unsafe {
        critical_section::with(|cs| {
            let offset = dma_router.borrow(cs).offset();
            let scope = &mut scope.borrow_ref_mut(cs);
            let mid = (BUF_SZ_WORDS/2)+1;
            let end = BUF_SZ_WORDS-1;
            if mid <= offset && offset <= mid + BUF_SZ_SAMPLES/4 {
                scope.feed(&BUF_IN[0..(BUF_SZ_SAMPLES/2)]);
            } else if end == offset || offset < BUF_SZ_SAMPLES/4 {
                scope.feed(&BUF_IN[(BUF_SZ_SAMPLES/2)..(BUF_SZ_SAMPLES)]);
            } else {
                panic!("latency too high into dma_router0?");
            }
        });
    }
}

fn timer0_handler(state: &Mutex<RefCell<State>>, opts: &Mutex<RefCell<opt::Options>>) {
    // WARN: Timer borrowed in multiple contexts!
    // Maybe should lock on this
    let peripherals = unsafe { pac::Peripherals::steal() };
    let timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);
    let uptime_ms = (timer.uptime() / ((SYSTEM_CLOCK_FREQUENCY as u64)/1000u64)) as u32;

    critical_section::with(|cs| {
        let state = &mut state.borrow_ref_mut(cs);
        let opts = &mut opts.borrow_ref_mut(cs);
        state.trace.start(&timer);
        state.tick(opts, uptime_ms);
        unsafe {
            tud_task();
        }
        state.trace.end(&timer);
    });
}

#[export_name = "DefaultHandler"]
unsafe fn irq_handler() {
    let pending_irq = vexriscv::register::vmip::read();
    let peripherals = pac::Peripherals::steal();

    if (pending_irq & (1 << pac::Interrupt::DMA_ROUTER0 as usize)) != 0 {
        let pending_subtype = peripherals.DMA_ROUTER0.ev_pending.read().bits();
        DMA_ROUTER0();
        peripherals.DMA_ROUTER0.ev_pending.write(|w| w.bits(pending_subtype));
        fence();
    }

    if (pending_irq & (1 << pac::Interrupt::TIMER0 as usize)) != 0 {
        let pending_subtype = peripherals.TIMER0.ev_pending.read().bits();
        TIMER0();
        peripherals.TIMER0.ev_pending.write(|w| w.bits(pending_subtype));
        fence();
    }

    // TODO: grab these correctly from PAC!
    let irq_usb_device = 2usize;
    let irq_usb_setup  = 3usize;
    let irq_usb_in_ep  = 4usize;
    let irq_usb_out_ep = 5usize;

    if ((pending_irq & (1 << irq_usb_device)) != 0 ||
        (pending_irq & (1 << irq_usb_setup)) != 0 ||
        (pending_irq & (1 << irq_usb_in_ep)) != 0 ||
        (pending_irq & (1 << irq_usb_out_ep)) != 0) {
        dcd_int_handler(0);
    }

}

struct State {
    trace: Trace,
    midi_in: MidiIn<UartMidi>,
    voice_manager: VoiceManager,
    encoder: Encoder,
    breathe: LedBreathe,
}

impl State {
    fn new(midi_in: MidiIn<UartMidi>, encoder: Encoder, breathe: LedBreathe) -> Self {
        State {
            trace: Trace::new(),
            midi_in,
            voice_manager: VoiceManager::new(),
            encoder,
            breathe,
        }
    }

    fn tick(&mut self, opts: &mut opt::Options, uptime_ms: u32) {

        let peripherals = unsafe { pac::Peripherals::steal() };

        self.breathe.tick();

        self.encoder.update_ticks(TICK_MS);

        if self.encoder.pending_short_press() {
            opts.toggle_modify();
        }

        if self.encoder.pending_long_press() {
            unsafe { reset_soc(&peripherals.CTRL); }
        }

        let encoder_ticks = self.encoder.pending_ticks();
        if encoder_ticks > 0 {
            for _ in 0..encoder_ticks {
                opts.tick_up();
            }
        }
        if encoder_ticks < 0 {
            for _ in 0..(-encoder_ticks) {
                opts.tick_down();
            }
        }

        let shifter = get_shifters(&peripherals);

        let lpf = get_lpfs(&peripherals);

        while let Ok(event) = self.midi_in.read() {
            self.voice_manager.event(event, uptime_ms);
        }

        self.voice_manager.tick(uptime_ms, opts);

        for n_voice in 0..N_VOICES {
            let voice = &self.voice_manager.voices[n_voice];
            shifter[n_voice].set_pitch(voice.pitch);
            lpf[n_voice].set_cutoff((voice.amplitude * 8000f32) as i16);
            lpf[n_voice].set_resonance(opts.adsr.resonance.value);
        }
    }
}

fn fence() {
    #[cfg(not(test))]
    {
        unsafe {
            asm!("fence iorw, iorw");
            asm!(".word(0x500F)");
        }
    }
}


fn oled_init(timer: &mut Timer, oled_spi: pac::OLED_SPI)
    -> ssd1322::Display<ssd1322::SpiInterface<OledSpi, OledGpio>> {

    let dc = OledGpio::new(0);
    let mut rstn = OledGpio::new(1);
    let mut csn = OledGpio::new(2);
    let spi = OledSpi::new(oled_spi);

    // Create the SpiInterface and Display.
    let mut disp = oled::Display::new(
        oled::SpiInterface::new(spi, dc),
        oled::PixelCoord(256, 64),
        oled::PixelCoord(112, 0),
    );

    csn.set_low().unwrap();

    // Assert the display's /RESET for 10ms.
    timer.delay_ms(10u32);
    rstn.set_low().unwrap();
    timer.delay_ms(10u32);
    rstn.set_high().unwrap();

    timer.delay_ms(1u16);

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

    disp
}

#[no_mangle]
pub extern "C" fn tud_dfu_get_timeout_cb(alt: u8, state: u8) -> u32 {
    // TODO
    0u32
}

#[no_mangle]
pub extern "C" fn tud_dfu_download_cb(alt: u8, block_num: u16, data: *const u8, length: u16)  {
    // TODO
}

#[no_mangle]
pub extern "C" fn tud_dfu_manifest_cb(alt: u8)  {
    // TODO
}

#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    peripherals.EURORACK_PMOD0.reset(&mut timer);
    peripherals.PCA9635.reset(&mut timer);

    let mut disp = oled_init(&mut timer, peripherals.OLED_SPI);
    let mut spi_dma = SpiDma::new(peripherals.SPI_DMA, pac::OLED_SPI::PTR);
    let dma_router = Mutex::new(
        DmaRouter::new(peripherals.DMA_ROUTER0,
                       unsafe { BUF_IN.as_mut_ptr() as u32 },
                       BUF_SZ_WORDS as u32));
    let uart_midi = UartMidi::new(peripherals.UART_MIDI);
    let midi_in =  MidiIn::new(uart_midi);
    let breathe = LedBreathe::new(peripherals.PCA9635);
    let encoder = Encoder::new(peripherals.ROTARY_ENCODER, peripherals.ENCODER_BUTTON);

    let opts = Mutex::new(RefCell::new(opt::Options::new()));
    let state = Mutex::new(RefCell::new(State::new(midi_in, encoder, breathe)));
    let oscope = Mutex::new(RefCell::new(OScope::new()));

    let mut trace_main = Trace::new();

    unsafe {
        // TODO: Issue device reset before init!
        tusb_init();
    }

    handler!(dma_router0 = || dma_router0_handler(&dma_router, &oscope));
    handler!(timer0 = || timer0_handler(&state, &opts));

    scope(|scope| {

        scope.register(Interrupt::DMA_ROUTER0, dma_router0);
        scope.register(Interrupt::TIMER0, timer0);

        unsafe {
            // Enable interrupts from DMA router (vexriscv specific register)
            vexriscv::register::vmim::write((1 << (pac::Interrupt::DMA_ROUTER0 as usize)) |
                                            (1 << (pac::Interrupt::TIMER0 as usize)) );

            // Enable machine external interrupts (basically everything added on by LiteX).
            riscv::register::mie::set_mext();

            // WARN: Don't do this before IRQs are registered for this scope,
            // otherwise you'll hang forever :)
            // Finally enable interrupts
            riscv::interrupt::enable();
        }

        timer.set_periodic_event(TICK_MS);

        loop {

            trace_main.start(&timer);

            let scope_samples = critical_section::with(|cs| {
                let scope = &mut oscope.borrow_ref_mut(cs);
                if scope.full() {
                    // samples_dbl can only ever update here
                    scope.reset();
                }
                scope.samples_dbl
            });

            let (opts, voices, irq0_len_us) = critical_section::with(|cs| {
                let state = state.borrow_ref(cs);
                (opts.borrow_ref(cs).clone(),
                 state.voice_manager.voices.clone(),
                 state.trace.len_us())
            });

            draw::draw_main(&mut disp, opts, voices, &scope_samples,
                            irq0_len_us, trace_main.len_us()).ok();

            let fb = disp.swap_clear();
            fence();
            spi_dma.block();
            spi_dma.transfer(fb.as_ptr(), fb.len());

            trace_main.end(&timer);
        }
    })

}
