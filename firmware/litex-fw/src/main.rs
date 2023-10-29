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
    CTL: litex_pac::OLED_CTL,
}

litex_hal::spi! {
    SPI: (litex_pac::OLED_SPI, u8),
}

const N_CHANNELS: usize = 4;
const BUF_SZ_WORDS: usize = 512;
const BUF_SZ_SAMPLES: usize = BUF_SZ_WORDS * 2;
const BUF_SZ_SAMPLES_PER_IRQ: usize = BUF_SZ_SAMPLES / 2;

static mut SCOPE: Option<Scope> = None;

// MUST be aligned to 4-byte (word) boundary for RV32. These buffers are directly
// accessed by DMA that iterates across words!.
static mut BUF_IN: Aligned<A4, [i16; BUF_SZ_SAMPLES]> = Aligned([0i16; BUF_SZ_SAMPLES]);

static mut LAST_IRQ: u32 = 0;
static mut LAST_IRQ_LEN: u32 = 0;
static mut LAST_IRQ_PERIOD: u32 = 0;

const SCOPE_SAMPLES: usize = 1024;
const SCOPE_SUBSAMPLE: usize = 2;

struct Scope {
    samples: [i16; SCOPE_SAMPLES],
    samples_dbl: [i16; SCOPE_SAMPLES],
    n_samples: usize,
}

impl Scope {
    fn new() -> Self {
        Scope {
            samples: [0i16; SCOPE_SAMPLES],
            samples_dbl: [0i16; SCOPE_SAMPLES],
            n_samples: 0,
        }
    }

    fn feed(&mut self, buf_in: &[i16]) {
        for i in 0..(buf_in.len()/N_CHANNELS) {
            let x_in: [i16; N_CHANNELS] = [
                buf_in[N_CHANNELS*i+0],
                buf_in[N_CHANNELS*i+1],
                buf_in[N_CHANNELS*i+2],
                buf_in[N_CHANNELS*i+3],
            ];

            if  self.n_samples != SCOPE_SAMPLES {
                self.samples[self.n_samples] = x_in[0];
                self.n_samples += 1
            }
        }
    }

    fn full(&mut self) -> bool {
        self.n_samples == SCOPE_SAMPLES
    }

    fn reset(&mut self) {
        self.n_samples = 0;
        core::mem::swap(&mut self.samples, &mut self.samples_dbl);
    }
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
        let offset = peripherals.DMA_ROUTER0.offset_words.read().bits();
        let pending_subtype = peripherals.DMA_ROUTER0.ev_pending.read().bits();

        if let Some(ref mut scope) = SCOPE {
            if offset as usize == ((BUF_SZ_WORDS/2)+1) {
                scope.feed(&BUF_IN[0..(BUF_SZ_SAMPLES/2)])
            }
            if offset as usize == (BUF_SZ_WORDS-1) {
                scope.feed(&BUF_IN[(BUF_SZ_SAMPLES/2)..(BUF_SZ_SAMPLES)])
            }
        }

        peripherals.DMA_ROUTER0.ev_pending.write(|w| w.bits(pending_subtype));

        fence();
    }

    peripherals.TIMER0.uptime_latch.write(|w| w.bits(1));
    let trace_end = peripherals.TIMER0.uptime_cycles0.read().bits();
    LAST_IRQ_LEN = trace_end - trace;
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

fn fence() {
    unsafe {
        asm!("fence iorw, iorw");
        asm!(".word(0x500F)");
    }
}

#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let pmod0 = peripherals.EURORACK_PMOD0;

    let shifter: [&dyn gw::PitchShift; 4] = [
        &peripherals.PITCH_SHIFT0,
        &peripherals.PITCH_SHIFT1,
        &peripherals.PITCH_SHIFT2,
        &peripherals.PITCH_SHIFT3,
    ];

    let lpf: [&dyn gw::KarlsenLpf; 4] = [
        &peripherals.KARLSEN_LPF0,
        &peripherals.KARLSEN_LPF1,
        &peripherals.KARLSEN_LPF2,
        &peripherals.KARLSEN_LPF3,
    ];

    let mut opts = opt::Options::new();

    let pca9635 = peripherals.PCA9635;

    let uart_midi = UartMidi::new(peripherals.UART_MIDI);

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    pca9635.reset_line(true);
    pmod0.reset_line(true);
    timer.delay_ms(10u32);
    pca9635.reset_line(false);
    pmod0.reset_line(false);

    let dc = CTL { index: 0 };
    let mut rstn = CTL { index: 1 };
    let mut csn = CTL { index: 2 };
    let spi = SPI {
        registers: peripherals.OLED_SPI
    };

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

    let mut midi_in = MidiIn::new(uart_midi);

    let mut voice_manager = VoiceManager::new();

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


    let character_style = MonoTextStyle::new(&FONT_5X7, Gray4::WHITE);
    let font_small_white = MonoTextStyle::new(&FONT_4X6, Gray4::WHITE);
    let font_small_grey = MonoTextStyle::new(&FONT_4X6, Gray4::new(0x4));

    let mut cycle_cnt = timer.uptime();
    let mut td_us: Option<u32> = None;
    let mut v = 0u8;

    let mut cur_opt: usize = 0;

    let mut enc_last: u32 = peripherals.ROTARY_ENCODER.csr_state.read().bits() >> 2;

    let mut modif: bool = false;
    let mut btn_held_ms: u32 = 0;

    unsafe {
        peripherals.SPI_DMA.spi_control_reg_address.write(
            |w| w.bits(litex_pac::OLED_SPI::PTR as u32));
        peripherals.SPI_DMA.spi_status_reg_address.write(
            |w| w.bits(litex_pac::OLED_SPI::PTR as u32 + 0x04));
        peripherals.SPI_DMA.spi_mosi_reg_address.write(
            |w| w.bits(litex_pac::OLED_SPI::PTR as u32 + 0x08));
    }

    unsafe {
        SCOPE = Some(Scope::new());

        peripherals.DMA_ROUTER0.base_writer.write(|w| w.bits(BUF_IN.as_mut_ptr() as u32));
        peripherals.DMA_ROUTER0.length_words.write(|w| w.bits(BUF_SZ_WORDS as u32));
        peripherals.DMA_ROUTER0.enable.write(|w| w.bits(1u32));
        peripherals.DMA_ROUTER0.ev_enable.write(|w| w.half().bit(true));

        // Enable interrupts from DMA router (vexriscv specific register)
        vexriscv::register::vmim::write(1 << (pac::Interrupt::DMA_ROUTER0 as usize));

        // Enable machine external interrupts (basically everything added on by LiteX).
        riscv::register::mie::set_mext();

        // Finally enable interrupts
        riscv::interrupt::enable();
    }

    let thin_stroke = PrimitiveStyle::with_stroke(Gray4::WHITE, 1);

    loop {

        Text::with_alignment(
            "POLYPHONIZER\nDEMO",
            Point::new(disp.bounding_box().center().x, 10),
            character_style,
            Alignment::Center,
        )
        .draw(&mut disp).ok();

        v += 3;

        for i in 0..=15 {
            let this_v = v+i*16;
            if this_v < 128 {
                pca9635.led(i.into(), this_v);
            } else {
                pca9635.led(i.into(), 128-this_v);
            }
        }

        let enc_now: u32 = peripherals.ROTARY_ENCODER.csr_state.read().bits() >> 2;
        let mut enc_delta: i32 = (enc_now as i32) - (enc_last as i32);
        if enc_delta > 10 {
            enc_delta = -1;
        }
        if enc_delta < -10 {
            enc_delta = 1;
        }
        enc_last = enc_now;

        let time_adsr = (cycle_cnt / 60_000u64) as u32;

        while let Ok(event) = midi_in.read() {
            voice_manager.event(event, time_adsr);
        }

        voice_manager.tick(time_adsr, &opts);

        let mut v = [0u8; 4];
        for n_voice in 0..=3 {
            let voice = &voice_manager.voices[n_voice];
            v[n_voice] = voice.note;
            shifter[n_voice].set_pitch(voice.pitch);
            lpf[n_voice].set_cutoff((voice.amplitude * 8000f32) as i16);
            lpf[n_voice].set_resonance(opts.resonance.value);
            draw_voice(&mut disp, (30+37*n_voice) as u32, n_voice as u32, voice).ok();
        }


        if peripherals.ENCODER_BUTTON.in_.read().bits() != 0 {
            btn_held_ms += td_us.unwrap() / 1000;
        } else {
            if btn_held_ms > 0 {
                modif = !modif;
            }
            btn_held_ms = 0;
        }

        if btn_held_ms > 3000 {
            peripherals.CTRL.reset.write(|w| w.soc_rst().bit(true));
        }



        {
            let opts_view: [&mut dyn opt::OptionTrait; 5] = [
                &mut opts.attack_ms,
                &mut opts.decay_ms,
                &mut opts.release_ms,
                &mut opts.resonance,
                &mut opts.delay_len,
            ];


            if !modif {
                if enc_delta > 0 && cur_opt < opts_view.len() - 1 {
                    cur_opt += 1;
                }

                if enc_delta < 0 && cur_opt > 0{
                    cur_opt -= 1;
                }
            }

            let vy: usize = 190;
            for n in 0..opts_view.len() {
                let mut font = font_small_grey;
                if cur_opt == n {
                    font = font_small_white;
                    if modif {
                        Text::with_alignment(
                            "-",
                            Point::new(62, (vy+10*n) as i32),
                            font,
                            Alignment::Left,
                        ).draw(&mut disp).ok();
                        if enc_delta > 0 {
                            opts_view[n].tick_up();
                        }
                        if enc_delta < 0 {
                            opts_view[n].tick_down();
                        }
                    }
                }
                Text::with_alignment(
                    opts_view[n].name(),
                    Point::new(5, (vy+10*n) as i32),
                    font,
                    Alignment::Left,
                ).draw(&mut disp).ok();
                Text::with_alignment(
                    &opts_view[n].value(),
                    Point::new(60, (vy+10*n) as i32),
                    font,
                    Alignment::Right,
                ).draw(&mut disp).ok();
            }
        }

        if let Some(value) = td_us {
            let mut s: String<64> = String::new();
            ufmt::uwrite!(&mut s, "{}.", value / 1_000u32).ok();
            ufmt::uwrite!(&mut s, "{}ms\n", value % 1_000u32).ok();
            ufmt::uwrite!(&mut s, "-apfelaudio-\n").ok();
            Text::with_alignment(
                &s,
                Point::new(5, 245),
                character_style,
                Alignment::Left,
            )
            .draw(&mut disp).ok();
        }

        unsafe {
            if let Some(ref mut scope) = SCOPE {
                if scope.full() {
                    scope.reset();
                    let mut points: [Point; 64] = [Point::new(0, 0); 64];
                    for n in 0..points.len() {
                        points[n].x = n as i32;
                        points[n].y = 100 + (scope.samples_dbl[n] >> 10) as i32;
                    }
                    Polyline::new(&points)
                        .into_styled(thin_stroke)
                        .draw(&mut disp).ok();
                }
            }
        }

        let fb = disp.swap_clear();
        unsafe {
            fence();
            while peripherals.SPI_DMA.done.read().bits() == 0 {
                // Don't start to DMA a new framebuffer if we're still
                // pushing through the last one.
            }
            peripherals.SPI_DMA.read_base.write(|w| w.bits(fb.as_ptr() as u32));
            peripherals.SPI_DMA.read_length.write(|w| w.bits(fb.len() as u32));
            peripherals.SPI_DMA.start.write(|w| w.start().bit(true));
            peripherals.SPI_DMA.start.write(|w| w.start().bit(false));
        }

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
}
