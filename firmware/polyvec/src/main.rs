#![cfg(not(test))]
#![allow(unused_imports)]

#![no_std]
#![no_main]

use litex_hal::prelude::*;
use litex_pac as pac;
use riscv_rt::entry;
use litex_hal::hal::digital::v2::OutputPin;
use heapless::String;
use heapless::Vec;
use embedded_midi::MidiIn;
use core::arch::asm;
use aligned_array::{Aligned, A4};
use core::cell::RefCell;
use critical_section::Mutex;
use irq::{handler, scope, scoped_interrupts};
use litex_interrupt::return_as_is;

use ssd1322 as oled;

use polyvec_lib::voice::*;
use polyvec_lib::draw;
use polyvec_lib::opt;

use polyvec_hal::gw::*;
use polyvec_hal::log::*;
use polyvec_hal::*;

const N_VOICES: usize = 4;
const SCOPE_SAMPLES: usize = 128;
const N_CHANNELS: usize = 4;
const BUF_SZ_WORDS: usize = 1024;
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
    n_samples: usize,
    trig_lo: bool,
    subsample: usize,
    n_subsample: usize,
    trig_lvl_mv: i16,
    trig_sns_mv: i16,
}

impl OScope {
    fn new() -> Self {
        OScope {
            samples: [0i16; SCOPE_SAMPLES],
            n_samples: 0,
            trig_lo: false,
            subsample: 4,
            n_subsample: 0,
            trig_lvl_mv: 0,
            trig_sns_mv: 1000,
        }
    }

    fn feed(&mut self, buf_in: &[i16]) {

        for i in 0..(buf_in.len()/N_CHANNELS) {
            if self.n_subsample % self.subsample == 0 {
                let x_in: [i16; N_CHANNELS] = [
                    buf_in[N_CHANNELS*i],
                    buf_in[N_CHANNELS*i+1],
                    buf_in[N_CHANNELS*i+2],
                    buf_in[N_CHANNELS*i+3],
                ];

                let trig_lo_raw = 4*(self.trig_lvl_mv - self.trig_sns_mv);
                let trig_hi_raw = 4*(self.trig_lvl_mv + self.trig_sns_mv);

                self.trig_lo = self.trig_lo || x_in[0] < trig_lo_raw;
                let trigger = x_in[0] > trig_hi_raw && self.trig_lo;

                if  (self.n_samples > 0 && self.n_samples != SCOPE_SAMPLES) ||
                    (self.n_samples == 0 && trigger) {
                    self.samples[self.n_samples] = x_in[0];
                    self.n_samples += 1
                }
            }
            self.n_subsample += 1;
        }

    }

    fn full(&mut self) -> bool {
        self.n_samples == SCOPE_SAMPLES
    }

    fn reset(&mut self) {
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
            if mid <= offset && offset <= mid + BUF_SZ_WORDS/4 {
                scope.feed(&BUF_IN[0..(BUF_SZ_SAMPLES/2)]);
            } else if end == offset || offset < BUF_SZ_WORDS/4 {
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
        state.trace.end(&timer);
    });
}

#[export_name = "DefaultHandler"]
unsafe fn irq_handler() {

    let pending_irq = vexriscv::register::vmip::read();

    let peripherals = pac::Peripherals::steal();
    if (pending_irq & (1 << pac::Interrupt::DMA_ROUTER0 as usize)) != 0 {
        let pending_subtype = peripherals.DMA_ROUTER0.ev_pending().read().bits();
        DMA_ROUTER0();
        peripherals.DMA_ROUTER0.ev_pending().write(|w| w.bits(pending_subtype));
        fence();
    }

    if (pending_irq & (1 << pac::Interrupt::TIMER0 as usize)) != 0 {
        let pending_subtype = peripherals.TIMER0.ev_pending().read().bits();
        TIMER0();
        peripherals.TIMER0.ev_pending().write(|w| w.bits(pending_subtype));
        fence();
    }
}

struct State {
    trace: Trace,
    midi_in: MidiIn<UartMidi>,
    voice_manager: VoiceManager,
    encoder: Encoder,
    last_control_type: Option<opt::NoteControl>,
}

impl State {
    fn new(midi_in: MidiIn<UartMidi>, encoder: Encoder) -> Self {
        State {
            trace: Trace::new(),
            midi_in,
            voice_manager: VoiceManager::new(),
            encoder,
            last_control_type: None,
        }
    }

    fn tick(&mut self, opts: &mut opt::Options, uptime_ms: u32) {

        let peripherals = unsafe { pac::Peripherals::steal() };

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

        if let Some(cv) = self.last_control_type {
            if cv != opts.touch.note_control.value {
                self.voice_manager = VoiceManager::new();
            }
        }

        if opts.touch.note_control.value == opt::NoteControl::Midi {
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
        } else {
            let pmod1 = &peripherals.EURORACK_PMOD1;
            let pmod2 = &peripherals.EURORACK_PMOD2;
            let pmod3 = &peripherals.EURORACK_PMOD3;

            let touch1 = pmod1.touch();
            let touch2 = pmod2.touch();
            let touch3 = pmod3.touch();

            let mut touch_concat: [u8; 8*3] = [0u8; 8*3];
            touch_concat[0..8].copy_from_slice(&touch3);
            touch_concat[8..16].copy_from_slice(&touch2);
            touch_concat[16..24].copy_from_slice(&touch1);

            let minor_map: [usize; 24] =  [
                   0,    2,    3,    5,    7,    8,    10, 13,
                  12, 12+2, 12+3, 12+5, 12+7, 12+8, 12+10, 25,
                  24, 24+2, 24+3, 24+5, 24+7, 24+8, 24+10, 37,
            ];

            let index_to_note = |idx| (minor_map[idx] + 36) as u8;

            // Create a vector of tuples where each tuple consists of the note and ampl_raw.
            let mut touch: Vec<(u8, u8), 24> =
                touch_concat.iter().enumerate()
                .map(|(idx, &touch_raw)| (index_to_note(idx), touch_raw))
                .filter(|(_note, touch_raw)| *touch_raw > 0)
                .collect();
            touch.sort_unstable_by(|a, b| b.1.cmp(&a.1));

            let voices_old = self.voice_manager.voices.clone();

            let mut update_hw_voice = |n_voice: usize, midi_note: u8, touch_raw: u8| {
                let ampl = (touch_raw as f32) / 256.0f32;
                let pitch = note_to_pitch(midi_note);
                shifter[n_voice].set_pitch(pitch);

                // Low-pass filter to smooth touch on/off
                let ampl_old = (lpf[n_voice].cutoff() as f32) / 8000f32;
                let ampl_new = ampl*0.05 + ampl_old*0.95;
                lpf[n_voice].set_cutoff((ampl_new * 8000f32) as i16);
                lpf[n_voice].set_resonance(opts.adsr.resonance.value);

                // Push to voice manager to visualizations work
                self.voice_manager.voices[n_voice].amplitude = ampl_new;
                self.voice_manager.voices[n_voice].note = midi_note;
                self.voice_manager.voices[n_voice].state = VoiceState::Sustain;

                if touch_raw == 0 {
                    self.voice_manager.voices[n_voice].state = VoiceState::Idle;
                }
            };

            let mut updated_voices: Vec<usize, N_VOICES> = Vec::new();
            let mut remaining_touches: Vec<(u8, u8), N_VOICES> = Vec::new();
            for (note, touch_raw) in touch.iter().take(N_VOICES) {
                let mut updated = false;
                for n_voice in 0..N_VOICES {
                    if *note == voices_old[n_voice].note {
                        update_hw_voice(n_voice, *note, *touch_raw);
                        updated_voices.push(n_voice).unwrap();
                        updated = true;
                        break;
                    }
                }
                if !updated {
                    remaining_touches.push((*note, *touch_raw)).unwrap();
                }
            }

            for (note, touch_raw) in remaining_touches.iter() {
                for n_voice in 0..N_VOICES {
                    if !updated_voices.contains(&n_voice) {
                        update_hw_voice(n_voice, *note, *touch_raw);
                        updated_voices.push(n_voice).unwrap();
                        break;
                    }
                }
            }

            for n_voice in 0..N_VOICES {
                if !updated_voices.contains(&n_voice) {
                    update_hw_voice(n_voice, voices_old[n_voice].note, 0);
                }
            }

        }

        self.last_control_type = Some(opts.touch.note_control.value);
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
pub extern "C" fn _putchar(c: u8)  {
    log::_logger_write(&[c]);
}

const USB_DEVICE_CONTROLLER_RESET_ADDRESS: *mut u32 = 0xF0010004 as *mut u32;

unsafe fn usb_device_controller_reset_write(value: u32) {
    core::ptr::write_volatile(USB_DEVICE_CONTROLLER_RESET_ADDRESS, value);
}

#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    info!("hello from litex-fw!");


    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    unsafe {
        usb_device_controller_reset_write(1);
        timer.delay_ms(100u32);
        usb_device_controller_reset_write(0);
        timer.delay_ms(100u32);
    }

    let pmod0 = peripherals.EURORACK_PMOD0;
    pmod0.reset(&mut timer);

    let pmod1 = peripherals.EURORACK_PMOD1;
    let pmod2 = peripherals.EURORACK_PMOD2;
    let pmod3 = peripherals.EURORACK_PMOD3;

    let mut disp = oled_init(&mut timer, peripherals.OLED_SPI);
    let mut spi_dma = SpiDma::new(peripherals.SPI_DMA, pac::OLED_SPI::PTR);
    let dma_router = Mutex::new(
        DmaRouter::new(peripherals.DMA_ROUTER0,
                       unsafe { BUF_IN.as_mut_ptr() as u32 },
                       BUF_SZ_WORDS as u32));
    let uart_midi = UartMidi::new(peripherals.UART_MIDI);
    let midi_in =  MidiIn::new(uart_midi);
    let encoder = Encoder::new(peripherals.ROTARY_ENCODER, peripherals.ENCODER_BUTTON);

    let opts = Mutex::new(RefCell::new(opt::Options::new()));
    let state = Mutex::new(RefCell::new(State::new(midi_in, encoder)));
    let oscope = Mutex::new(RefCell::new(OScope::new()));

    let mut trace_main = Trace::new();

    handler!(dma_router0 = || dma_router0_handler(&dma_router, &oscope));
    handler!(timer0 = || timer0_handler(&state, &opts));

    scope(|scope| {

        scope.register(Interrupt::DMA_ROUTER0, dma_router0);
        scope.register(Interrupt::TIMER0, timer0);

        timer.set_periodic_event(TICK_MS);

        unsafe {

            // Note: USB interrupts are enabled later by dcd_eptri.c initialization (tusb_init)
            vexriscv::register::vmim::write((1 << (pac::Interrupt::DMA_ROUTER0 as usize)) |
                                            (1 << (pac::Interrupt::TIMER0 as usize)));

            // Enable machine external interrupts (basically everything added on by LiteX).
            riscv::register::mie::set_mext();

            // WARN: delay before interrupt enable after tusb_init(), the USB core takes a while
            // to spin up after tusb_init() if nothing is connected, apparently.
            timer.delay_ms(100u32);

            // WARN: Don't do this before IRQs are registered for this scope,
            // otherwise you'll hang forever :)
            // Finally enable interrupts
            riscv::interrupt::enable();
        }

        loop {

            trace_main.start(&timer);

            let scope_samples = critical_section::with(|cs| {
                let scope = &mut oscope.borrow_ref_mut(cs);
                if scope.full() {
                    scope.reset();
                    scope.trig_lvl_mv = opts.borrow_ref(cs).scope.trig_lvl.value as i16;
                    scope.trig_sns_mv = opts.borrow_ref(cs).scope.trig_sns.value as i16;
                }
                scope.samples
            });

            let (opts, voices, irq0_len_us) = critical_section::with(|cs| {
                let state = state.borrow_ref(cs);
                (opts.borrow_ref(cs).clone(),
                 state.voice_manager.voices.clone(),
                 state.trace.len_us())
            });

            let touch0 = pmod0.touch();
            let touch1 = pmod1.touch();
            let touch2 = pmod2.touch();
            let touch3 = pmod3.touch();

            let mut touch_concat: [u8; 8*4] = [0u8; 8*4];
            touch_concat[0..8].copy_from_slice(&touch0);
            touch_concat[8..16].copy_from_slice(&touch1);
            touch_concat[16..24].copy_from_slice(&touch2);
            touch_concat[24..32].copy_from_slice(&touch3);

            draw::draw_main(&mut disp, opts.clone(), voices, &scope_samples, &touch_concat,
                            irq0_len_us, trace_main.len_us()).ok();

            for (n, v) in touch0.iter().enumerate() {
                if opts.touch.led_mirror.value == opt::TouchLedMirror::MirrorOn {
                    pmod0.led_set(n, (v >> 1) as i8);
                } else {
                    pmod0.led_auto(n);
                }
            }

            for (n, v) in touch1.iter().enumerate() {
                if opts.touch.led_mirror.value == opt::TouchLedMirror::MirrorOn {
                    pmod1.led_set(n, (v >> 1) as i8);
                } else {
                    pmod1.led_auto(n);
                }
            }

            for (n, v) in touch2.iter().enumerate() {
                if opts.touch.led_mirror.value == opt::TouchLedMirror::MirrorOn {
                    pmod2.led_set(n, (v >> 1) as i8);
                } else {
                    pmod2.led_auto(n);
                }
            }

            for (n, v) in touch3.iter().enumerate() {
                if opts.touch.led_mirror.value == opt::TouchLedMirror::MirrorOn {
                    pmod3.led_set(n, (v >> 1) as i8);
                } else {
                    pmod3.led_auto(n);
                }
            }

            let fb = disp.swap_clear();
            fence();
            spi_dma.transfer(fb.as_ptr(), fb.len());
            spi_dma.block();

            trace_main.end(&timer);
        }
    })

}
