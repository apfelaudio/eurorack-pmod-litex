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

use embedded_graphics::{
    pixelcolor::{Gray4, GrayColor},
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, Line},
    mono_font::{ascii::FONT_4X6, ascii::FONT_5X7, MonoTextStyle},
    prelude::*,
    text::{Alignment, Text},
};

use ssd1322 as oled;

mod log;
mod voice;
mod gw;

use voice::*;
use gw::*;
use log::*;

eurorack_pmod!(pac::EURORACK_PMOD0);
eurorack_pmod!(pac::EURORACK_PMOD1);
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

#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let pmod0 = peripherals.EURORACK_PMOD0;
    let pmod1 = peripherals.EURORACK_PMOD1;

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

    let pca9635 = peripherals.PCA9635;

    let uart_midi = UartMidi::new(peripherals.UART_MIDI);

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    pca9635.reset_line(true);
    pmod0.reset_line(true);
    pmod1.reset_line(true);
    timer.delay_ms(10u32);
    pca9635.reset_line(false);
    pmod0.reset_line(false);
    pmod1.reset_line(false);

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

    let mut cycle_cnt = timer.uptime();
    let mut td_us: Option<u32> = None;
    let mut v = 0u8;

    loop {

        Text::with_alignment(
            "POLYPHONIZER\nDEMO\n-apfelaudio-",
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

        let time_adsr = (cycle_cnt / 60_000u64) as u32;

        while let Ok(event) = midi_in.read() {
            voice_manager.event(event, time_adsr);
        }

        voice_manager.tick(time_adsr);

        let mut v = [0u8; 4];
        for n_voice in 0..=3 {
            let voice = &voice_manager.voices[n_voice];
            v[n_voice] = voice.note;
            shifter[n_voice].set_pitch(voice.pitch);
            lpf[n_voice].set_cutoff((voice.amplitude * 8000f32) as i16);
            lpf[n_voice].set_resonance(0i16);
            draw_voice(&mut disp, (35+35*n_voice) as u32, n_voice as u32, voice).ok();
        }


        if peripherals.ENCODER_BUTTON.in_.read().bits() != 0 {
            peripherals.CTRL.reset.write(|w| w.soc_rst().bit(true));
        }

        /*
        draw_titlebox(&mut disp, 74, "PMOD2", &[
          "ser:",
          "jck:",
          "in0:",
          "in1:",
          "in2:",
          "in3:",
        ], &[
            pmod1.eeprom_serial(),
            pmod1.jack() as u32,
            pmod1.input(0) as u32,
            pmod1.input(1) as u32,
            pmod1.input(2) as u32,
            pmod1.input(3) as u32,
        ]).ok();

        draw_titlebox(&mut disp, 132, "ENCODER", &[
          "tick:",
          "btn:",
        ], &[
          peripherals.ROTARY_ENCODER.csr_state.read().bits() >> 2,
          peripherals.ENCODER_BUTTON.in_.read().bits(),
        ]).ok();


        draw_titlebox(&mut disp, 16, "PMOD1", &[
          "ser:",
          "jck:",
          "in0:",
          "in1:",
          "in2:",
          "in3:",
        ], &[
            pmod0.eeprom_serial(),
            pmod0.jack() as u32,
            pmod0.input(0) as u32,
            pmod0.input(1) as u32,
            pmod0.input(2) as u32,
            pmod0.input(3) as u32,
        ]).ok();

        draw_titlebox(&mut disp, 190, "MIDI", &[
          "v0",
          "v1",
          "v2",
          "v3",
        ], &[
          v[0].into(),
          v[1].into(),
          v[2].into(),
          v[3].into()
        ]).ok();
        */

        if let Some(value) = td_us {
            let mut s: String<32> = String::new();
            ufmt::uwrite!(&mut s, "{}.", value / 1_000u32).ok();
            ufmt::uwrite!(&mut s, "{}ms", value % 1_000u32).ok();
            Text::with_alignment(
                &s,
                Point::new(5, 250),
                character_style,
                Alignment::Left,
            )
            .draw(&mut disp).ok();
        }

        disp.swap_clear();

        let cycle_cnt_now = timer.uptime();
        let cycle_cnt_last = cycle_cnt;
        cycle_cnt = cycle_cnt_now;
        let delta = (cycle_cnt_now - cycle_cnt_last) as u32;
        td_us = Some(delta / (SYSTEM_CLOCK_FREQUENCY / 1_000_000u32));
    }
}
