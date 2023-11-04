#![allow(unused_imports)]

#![no_std]
#![no_main]

use litex_hal::prelude::*;
use litex_hal::uart::UartError;
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

use embedded_graphics::{
    pixelcolor::{Gray4, GrayColor},
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, Line, Polyline},
    mono_font::{ascii::FONT_4X6, ascii::FONT_5X7, MonoTextStyle},
    prelude::*,
    text::{Alignment, Text},
};

use ssd1322 as oled;

mod log;
mod voice;
mod gw;
mod opt;

use voice::*;
use gw::*;
use log::*;

eurorack_pmod!(pac::EURORACK_PMOD0);
pitch_shift!(pac::PITCH_SHIFT0);
pitch_shift!(pac::PITCH_SHIFT1);
pitch_shift!(pac::PITCH_SHIFT2);
pitch_shift!(pac::PITCH_SHIFT3);
karlsen_lpf!(pac::KARLSEN_LPF0);
karlsen_lpf!(pac::KARLSEN_LPF1);
karlsen_lpf!(pac::KARLSEN_LPF2);
karlsen_lpf!(pac::KARLSEN_LPF3);
pwm_led!(pac::PCA9635);

const SYSTEM_CLOCK_FREQUENCY: u32 = 60_000_000;

litex_hal::uart! {
    UartMidi: litex_pac::UART_MIDI,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}

litex_hal::gpio! {
    OledGpio: litex_pac::OLED_CTL,
}

litex_hal::spi! {
    OledSpi: (litex_pac::OLED_SPI, u8),
}

const N_CHANNELS: usize = 4;
const BUF_SZ_WORDS: usize = 512;
const BUF_SZ_SAMPLES: usize = BUF_SZ_WORDS * 2;

// MUST be aligned to 4-byte (word) boundary for RV32. These buffers are directly
// accessed by DMA that iterates across words!.
static mut BUF_IN: Aligned<A4, [i16; BUF_SZ_SAMPLES]> = Aligned([0i16; BUF_SZ_SAMPLES]);

static mut LAST_IRQ: u32 = 0;
static mut LAST_IRQ_LEN: u32 = 0;
static mut LAST_IRQ_PERIOD: u32 = 0;

const SCOPE_SAMPLES: usize = 256;

const N_VOICES: usize = 4;

fn get_shifters(p: &pac::Peripherals) -> [&dyn gw::PitchShift; 4] {
    [
        &p.PITCH_SHIFT0,
        &p.PITCH_SHIFT1,
        &p.PITCH_SHIFT2,
        &p.PITCH_SHIFT3,
    ]
}

fn get_lpfs(p: &pac::Peripherals) -> [&dyn gw::KarlsenLpf; 4] {
    [
        &p.KARLSEN_LPF0,
        &p.KARLSEN_LPF1,
        &p.KARLSEN_LPF2,
        &p.KARLSEN_LPF3,
    ]
}

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
    trig_lo: bool
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

fn dma_router0_handler(scope: &Mutex<RefCell<OScope>>) {
    unsafe {
        let peripherals = pac::Peripherals::steal();
        let offset = peripherals.DMA_ROUTER0.offset_words.read().bits() as usize;

        critical_section::with(|cs| {
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
        state.tick(opts, uptime_ms);
    });
}

#[export_name = "DefaultHandler"]
unsafe fn irq_handler() {
    let pending_irq = vexriscv::register::vmip::read();
    let peripherals = pac::Peripherals::steal();

    peripherals.TIMER0.uptime_latch.write(|w| w.bits(1));
    let trace = peripherals.TIMER0.uptime_cycles0.read().bits();
    LAST_IRQ_PERIOD = trace - LAST_IRQ;
    LAST_IRQ = trace;

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

    peripherals.TIMER0.uptime_latch.write(|w| w.bits(1));
    let trace_end = peripherals.TIMER0.uptime_cycles0.read().bits();
    LAST_IRQ_LEN = trace_end - trace;
}

struct State {
    midi_in: MidiIn<UartMidi>,
    voice_manager: VoiceManager,
    encoder: Encoder,
    breathe: LedBreathe,
}

impl State {
    fn new(midi_in: MidiIn<UartMidi>, encoder: Encoder, breathe: LedBreathe) -> Self {
        State {
            midi_in,
            voice_manager: VoiceManager::new(),
            encoder,
            breathe,
        }
    }

    fn tick(&mut self, opts: &mut opt::Options, uptime_ms: u32) {

        let peripherals = unsafe { pac::Peripherals::steal() };

        self.breathe.tick();

        self.encoder.update_ticks(uptime_ms);

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
            lpf[n_voice].set_resonance(opts.resonance.value);
        }
    }
}

fn draw_voice<D>(d: &mut D, sy: u32, ix: u32, voice: &Voice) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{

    let thin_stroke = PrimitiveStyle::with_stroke(Gray4::WHITE, 1);
    let thin_stroke_grey = PrimitiveStyleBuilder::new()
        .stroke_color(Gray4::new(0x3))
        .stroke_width(1)
        .build();
    let character_style_h = MonoTextStyle::new(&FONT_5X7, Gray4::WHITE);
    let title_y = 10u32;
    let box_h = 20u32;
    let box_y = title_y + 3u32 + box_h;

    let mut s: String<32> = String::new();

    Rectangle::new(Point::new(2, sy as i32), Size::new(60, box_y))
        .into_styled(thin_stroke_grey)
        .draw(d)?;

    Rectangle::new(Point::new(2, sy as i32), Size::new(60, title_y))
        .into_styled(thin_stroke)
        .draw(d)?;

    ufmt::uwrite!(&mut s, "CH {}", ix).ok();

    Text::with_alignment(
        &s,
        Point::new(d.bounding_box().center().x, (sy as i32)+7),
        character_style_h,
        Alignment::Center,
    )
    .draw(d)?;


    let mut stroke_gain = PrimitiveStyleBuilder::new()
        .stroke_color(Gray4::new(0x1))
        .stroke_width(1)
        .build();

    let mut stroke_idle = PrimitiveStyleBuilder::new()
        .stroke_color(Gray4::new(0x1))
        .stroke_width(1)
        .build();

    s.clear();
    if voice.state != VoiceState::Idle {
        let semitones = voice.note as i32 - 60i32;
        if semitones > 0 {
            ufmt::uwrite!(&mut s, "+").ok();
        }
        ufmt::uwrite!(&mut s, "{}", semitones).ok();

        stroke_gain = PrimitiveStyleBuilder::new()
            .stroke_color(Gray4::new((15f32 * voice.amplitude) as u8))
            .stroke_width(1)
            .build();

        stroke_idle = PrimitiveStyleBuilder::new()
            .stroke_color(Gray4::WHITE)
            .stroke_width(1)
            .build();
    }

    let filter_pos: i32 = (20f32 * voice.amplitude) as i32;

    Line::new(Point::new(38, sy as i32 + 16),
              Point::new(40+filter_pos-2, sy as i32 + 16))
              .into_styled(stroke_gain)
              .draw(d)?;

    Line::new(Point::new(40+filter_pos, sy as i32 + 24),
              Point::new(55, sy as i32 + 24))
              .into_styled(stroke_gain)
              .draw(d)?;

    Line::new(Point::new(40+filter_pos-2, sy as i32 + 16),
              Point::new(40+filter_pos, sy as i32 + 24))
              .into_styled(stroke_gain)
              .draw(d)?;

    Rectangle::new(Point::new(7, sy as i32 + 15), Size::new(19, 11))
        .into_styled(stroke_idle)
        .draw(d)?;

    Text::new(
        &s,
        Point::new(9, sy as i32 + 22),
        character_style_h,
    )
    .draw(d)?;


    Ok(())
}

fn draw_options<D>(d: &mut D, opts: &opt::Options) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{
    let font_small_white = MonoTextStyle::new(&FONT_4X6, Gray4::WHITE);
    let font_small_grey = MonoTextStyle::new(&FONT_4X6, Gray4::new(0x4));

    // Should always succeed if the above CS runs.
    let opts_view = opts.view();

    let vy: usize = 205;
    for (n, opt) in opts_view.iter().enumerate() {
        let mut font = font_small_grey;
        if opts.selected == n {
            font = font_small_white;
            if opts.modify {
                Text::with_alignment(
                    "-",
                    Point::new(62, (vy+10*n) as i32),
                    font,
                    Alignment::Left,
                ).draw(d)?;
            }
        }
        Text::with_alignment(
            opt.name(),
            Point::new(5, (vy+10*n) as i32),
            font,
            Alignment::Left,
        ).draw(d)?;
        Text::with_alignment(
            &opt.value(),
            Point::new(60, (vy+10*n) as i32),
            font,
            Alignment::Right,
        ).draw(d)?;
    }

    Ok(())
}

fn fence() {
    unsafe {
        asm!("fence iorw, iorw");
        asm!(".word(0x500F)");
    }
}

struct LedBreathe {
    pca9635: pac::PCA9635,
    v: u8,
}

impl LedBreathe {
    fn new(pca9635: pac::PCA9635) -> Self {
        Self {
            pca9635,
            v: 0u8,
        }
    }

    fn tick(&mut self) {
        self.v += 3;
        for i in 0..=15 {
            let this_v = self.v+i*16;
            if this_v < 128 {
                self.pca9635.led(i.into(), this_v);
            } else {
                self.pca9635.led(i.into(), 128-this_v);
            }
        }
    }
}

struct Encoder {
    encoder: pac::ROTARY_ENCODER,
    button: pac::ENCODER_BUTTON,
    enc_last: u32,
    btn_held_ms: u32,
    short_press: bool,
    long_press: bool,
    ticks_since_last_read: i32,
}

impl Encoder {
    // NOTE: not easily reuseable pecause of pac naming?
    fn new(encoder: pac::ROTARY_ENCODER, button: pac::ENCODER_BUTTON) -> Self {
        let enc_last: u32 = encoder.csr_state.read().bits() >> 2;
        Self {
            encoder,
            button,
            enc_last,
            btn_held_ms: 0,
            short_press: false,
            long_press: false,
            ticks_since_last_read: 0,
        }
    }

    fn update_ticks(&mut self, uptime_ms: u32) {
        let enc_now: u32 = self.encoder.csr_state.read().bits() >> 2;
        let mut enc_delta: i32 = (enc_now as i32) - (self.enc_last as i32);
        if enc_delta > 10 {
            enc_delta = -1;
        }
        if enc_delta < -10 {
            enc_delta = 1;
        }
        self.enc_last = enc_now;

        if self.button.in_.read().bits() != 0 {
            self.btn_held_ms += uptime_ms;
        } else {
            if self.btn_held_ms > 0 {
                self.short_press = true;
            }
            self.btn_held_ms = 0;
        }

        if self.btn_held_ms > 3000 {
            self.long_press = true;
        }

        self.ticks_since_last_read += enc_delta;
    }

    fn pending_short_press(&mut self) -> bool {
        let result = self.short_press;
        self.short_press = false;
        result
    }

    fn pending_long_press(&mut self) -> bool {
        let result = self.long_press;
        self.short_press = false;
        result
    }

    fn pending_ticks(&mut self) -> i32 {
        let result = self.ticks_since_last_read;
        self.ticks_since_last_read = 0;
        result
    }
}

struct SpiDma {
    spi_dma: pac::SPI_DMA,
}

impl SpiDma {
    fn new(spi_dma: pac::SPI_DMA,
           target: *const pac::oled_spi::RegisterBlock) -> Self {
        unsafe {
            // TODO: Any way to get RegisterBlock sub-addresses automagically?
            spi_dma.spi_control_reg_address.write(
                |w| w.bits(target as u32));
            spi_dma.spi_status_reg_address.write(
                |w| w.bits(target as u32 + 0x04));
            spi_dma.spi_mosi_reg_address.write(
                |w| w.bits(target as u32 + 0x08));
        }
        Self {
            spi_dma
        }
    }

    fn block(&self) {
        while self.spi_dma.done.read().bits() == 0 {
            // Wait for an existing transfer to complete.
        }
    }

    fn transfer(&mut self, data_ptr: *const u8, data_len: usize) {
        unsafe {
            self.spi_dma.read_base.write(|w| w.bits(data_ptr as u32));
            self.spi_dma.read_length.write(|w| w.bits(data_len as u32));
            self.spi_dma.start.write(|w| w.start().bit(true));
            self.spi_dma.start.write(|w| w.start().bit(false));
        }
    }
}

unsafe fn reset_soc(ctrl: &pac::CTRL) {
    ctrl.reset.write(|w| w.soc_rst().bit(true));
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

#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let pmod0 = peripherals.EURORACK_PMOD0;

    let pca9635 = peripherals.PCA9635;


    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    pca9635.reset_line(true);
    pmod0.reset_line(true);
    timer.delay_ms(10u32);
    pca9635.reset_line(false);
    pmod0.reset_line(false);

    let mut disp = oled_init(&mut timer, peripherals.OLED_SPI);

    let character_style = MonoTextStyle::new(&FONT_5X7, Gray4::WHITE);
    let mut cycle_cnt = timer.uptime();
    let mut td_us: Option<u32> = None;

    timer.set_periodic_event(5); // 5ms tick

    let mut spi_dma = SpiDma::new(peripherals.SPI_DMA, pac::OLED_SPI::PTR);

    unsafe {

        peripherals.DMA_ROUTER0.base_writer.write(|w| w.bits(BUF_IN.as_mut_ptr() as u32));
        peripherals.DMA_ROUTER0.length_words.write(|w| w.bits(BUF_SZ_WORDS as u32));
        peripherals.DMA_ROUTER0.enable.write(|w| w.bits(1u32));
        peripherals.DMA_ROUTER0.ev_enable.write(|w| w.half().bit(true));

        // Enable interrupts from DMA router (vexriscv specific register)
        vexriscv::register::vmim::write((1 << (pac::Interrupt::DMA_ROUTER0 as usize)) |
                                        (1 << (pac::Interrupt::TIMER0 as usize)) );

        // Enable machine external interrupts (basically everything added on by LiteX).
        riscv::register::mie::set_mext();

        // Finally enable interrupts
        riscv::interrupt::enable();
    }


    let thin_stroke = PrimitiveStyle::with_stroke(Gray4::WHITE, 1);

    let uart_midi = UartMidi::new(peripherals.UART_MIDI);
    let midi_in =  MidiIn::new(uart_midi);
    let breathe = LedBreathe::new(pca9635);
    let encoder = Encoder::new(peripherals.ROTARY_ENCODER, peripherals.ENCODER_BUTTON);

    let opts = Mutex::new(RefCell::new(opt::Options::new()));
    let state = Mutex::new(RefCell::new(State::new(midi_in, encoder, breathe)));
    let oscope = Mutex::new(RefCell::new(OScope::new()));

    handler!(dma_router0 = || dma_router0_handler(&oscope));
    handler!(timer0 = || timer0_handler(&state, &opts));

    scope(|scope| {

        scope.register(Interrupt::DMA_ROUTER0, dma_router0);
        scope.register(Interrupt::TIMER0, timer0);

        loop {

            Text::with_alignment(
                "POLYPHONIZER",
                Point::new(disp.bounding_box().center().x, 10),
                character_style,
                Alignment::Center,
            )
            .draw(&mut disp).ok();

            // These should move to TIMER0 interrupt?

            let (opts, voices) = critical_section::with(|cs| {
                (opts.borrow_ref(cs).clone(),
                 state.borrow_ref(cs).voice_manager.voices.clone())
            });

            for (n_voice, voice) in voices.iter().enumerate() {
                draw_voice(&mut disp, (55+37*n_voice) as u32,
                           n_voice as u32, voice).ok();
            }

            draw_options(&mut disp, &opts).ok();

            if let Some(value) = td_us {
                let mut s: String<64> = String::new();
                ufmt::uwrite!(&mut s, "{}.", value / 1_000u32).ok();
                ufmt::uwrite!(&mut s, "{}ms\n", value % 1_000u32).ok();
                Text::with_alignment(
                    &s,
                    Point::new(5, 255),
                    character_style,
                    Alignment::Left,
                )
                .draw(&mut disp).ok();
            }

            {

                let samples = critical_section::with(|cs| {
                    let scope = &mut oscope.borrow_ref_mut(cs);
                    if scope.full() {
                        // samples_dbl can only ever update here
                        scope.reset();
                    }
                    scope.samples_dbl
                });
                let mut points: [Point; 64] = [Point::new(0, 0); 64];
                for (n, point) in points.iter_mut().enumerate() {
                    point.x = n as i32;
                    point.y = 30 + (samples[n] >> 10) as i32;
                }
                Polyline::new(&points)
                    .into_styled(thin_stroke)
                    .draw(&mut disp).ok();
            }

            let fb = disp.swap_clear();
            fence();
            spi_dma.block();
            spi_dma.transfer(fb.as_ptr(), fb.len());

            let cycle_cnt_now = timer.uptime();
            let cycle_cnt_last = cycle_cnt;
            cycle_cnt = cycle_cnt_now;
            let delta = (cycle_cnt_now - cycle_cnt_last) as u32;
            td_us = Some(delta / (SYSTEM_CLOCK_FREQUENCY / 1_000_000u32));

            /*
            unsafe {
                for i in 0..4 {
                    log::info!("{:x}@{:x}", i, BUF_IN[i]);
                }
                log::info!("irq_period: {}", LAST_IRQ_PERIOD);
                log::info!("irq_len: {}", LAST_IRQ_LEN);
                if LAST_IRQ_PERIOD != 0 {
                    log::info!("irq_load_percent: {}", (LAST_IRQ_LEN * 100) / LAST_IRQ_PERIOD);
                }
            }
            */
        }
    })

}
