#![no_std]
#![no_main]
#![allow(dead_code)]

use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use litex_pac as pac;
use riscv;
use riscv_rt::entry;
use litex_hal::hal::digital::v2::OutputPin;
use heapless::String;
use embedded_midi::MidiIn;
use midi_types::*;
use ufmt;

use embedded_graphics::{
    pixelcolor::{Gray4, GrayColor},
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
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
wavetable_oscillator!(pac::WAVETABLE_OSCILLATOR0);
wavetable_oscillator!(pac::WAVETABLE_OSCILLATOR1);
wavetable_oscillator!(pac::WAVETABLE_OSCILLATOR2);
wavetable_oscillator!(pac::WAVETABLE_OSCILLATOR3);
karlsen_lpf!(pac::KARLSEN_LPF0);
karlsen_lpf!(pac::KARLSEN_LPF1);
karlsen_lpf!(pac::KARLSEN_LPF2);
karlsen_lpf!(pac::KARLSEN_LPF3);

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

fn draw_titlebox<D>(d: &mut D, sy: u32, title: &str, fields: &[&str], values: &[u32]) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{

    let thin_stroke = PrimitiveStyle::with_stroke(Gray4::WHITE, 1);
    let thin_stroke_grey = PrimitiveStyleBuilder::new()
        .stroke_color(Gray4::new(0x3))
        .stroke_width(1)
        .fill_color(Gray4::BLACK)
        .build();
    let character_style = MonoTextStyle::new(&FONT_4X6, Gray4::WHITE);
    let character_style_h = MonoTextStyle::new(&FONT_5X7, Gray4::WHITE);
    let dy = 7u32;
    let title_y = 10u32;
    let box_y = title_y + 3u32 + (fields.len() as u32) * dy;

    Rectangle::new(Point::new(2, sy as i32), Size::new(60, box_y))
        .into_styled(thin_stroke_grey)
        .draw(d)?;

    Rectangle::new(Point::new(2, sy as i32), Size::new(60, title_y))
        .into_styled(thin_stroke)
        .draw(d)?;

    Text::with_alignment(
        title,
        Point::new(d.bounding_box().center().x, (sy as i32)+7),
        character_style_h,
        Alignment::Center,
    )
    .draw(d)?;

    let mut sy = sy + title_y + 6;

    for (f, v) in fields.iter().zip(values) {

        let mut s: String<32> = String::new();
        ufmt::uwrite!(&mut s, "{:#06x}", *v).ok();

        Text::with_alignment(
            f,
            Point::new(5, sy as i32),
            character_style,
            Alignment::Left,
        )
        .draw(d)?;

        if *v != 0 {
            Text::with_alignment(
                &s,
                Point::new(60, sy as i32),
                character_style,
                Alignment::Right,
            )
            .draw(d)?;
        }
        sy += dy;
    }

    Ok(())
}


#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let pmod0 = peripherals.EURORACK_PMOD0;
    let pmod1 = peripherals.EURORACK_PMOD1;

    let osc: [&dyn gw::WavetableOscillator; 4] = [
        &peripherals.WAVETABLE_OSCILLATOR0,
        &peripherals.WAVETABLE_OSCILLATOR1,
        &peripherals.WAVETABLE_OSCILLATOR2,
        &peripherals.WAVETABLE_OSCILLATOR3,
    ];

    let lpf: [&dyn gw::KarlsenLpf; 4] = [
        &peripherals.KARLSEN_LPF0,
        &peripherals.KARLSEN_LPF1,
        &peripherals.KARLSEN_LPF2,
        &peripherals.KARLSEN_LPF3,
    ];


    let uart_midi = UartMidi::new(peripherals.UART_MIDI);

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    pmod0.reset_line(true);
    pmod1.reset_line(true);
    timer.delay_ms(10u32);
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

    let mut voice_manager = VoiceManager {
        voices: [VoiceState::Off; N_VOICES],
    };

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

        let rect_style = PrimitiveStyleBuilder::new()
            .stroke_color(Gray4::new(0x0))
            .stroke_width(1)
            .fill_color(Gray4::BLACK)
            .build();


    disp
        .bounding_box()
        .into_styled(rect_style)
        .draw(&mut disp).ok();

    Text::with_alignment(
        "<TEST UTIL>",
        Point::new(disp.bounding_box().center().x, 10),
        character_style,
        Alignment::Center,
    )
    .draw(&mut disp).ok();


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
    ], &[0, 0]).ok();

    loop {

        while let Ok(event) = midi_in.read() {
            match event {
                MidiMessage::NoteOn(_, note, velocity) => {
                    log::info!(
                        "note on: note={} vel={}",
                        u8::from(note),
                        u8::from(velocity)
                    );
                    voice_manager.note_on(note);
                }
                MidiMessage::NoteOff(_, note, velocity) => {
                    log::info!(
                        "note off: note={} vel={}",
                        u8::from(note),
                        u8::from(velocity)
                    );
                    voice_manager.note_off(note);
                }
                _ => {}
            }
        }

        for n_voice in 0..=3 {
            let mut write_skip = 0u32;
            if let VoiceState::On(_, skip) = voice_manager.voices[n_voice] {
                write_skip = skip;
            }
            osc[n_voice].set_skip(write_skip);
            lpf[n_voice].set_cutoff(pmod0.input(1));
            lpf[n_voice].set_resonance(pmod0.input(2));
        }


        let mut v = [0u8; 4];
        for (i, voice) in v.iter_mut().enumerate() {
            if let VoiceState::On(note, _) = voice_manager.voices[i] {
                *voice = u8::from(note);
            }
        }

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


        draw_titlebox(&mut disp, 213, "MIDI", &[
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

        disp.flush();
    }
}
