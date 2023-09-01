#![no_std]
#![no_main]
#![allow(dead_code)]

use core::panic::PanicInfo;
use defmt;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use litex_pac as pac;
use riscv;
use riscv_rt::entry;
use litex_hal::hal::digital::v2::OutputPin;
use heapless::String;
use core::fmt::Write;
use embedded_midi::MidiIn;
use midi_types::*;
use micromath::F32Ext;

use embedded_graphics::{
    pixelcolor::{Gray4, GrayColor},
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
    mono_font::{ascii::FONT_4X6, ascii::FONT_5X7, MonoTextStyle},
    prelude::*,
    text::{Alignment, Text},
};

use ssd1322 as oled;

pub trait EurorackPmodTrait {
    fn reset_line(&self, set_high: bool);
    fn eeprom_serial(&self) -> u32;
    fn jack(&self) -> u8;
    fn input(&self, index: usize) -> i16;
}

pub trait WavetableOscillator {
    fn set_skip(&self, value: u32);
}

pub trait KarlsenLpf {
    fn set_cutoff(&self, value: i16);
    fn set_resonance(&self, value: i16);
}

macro_rules! impl_eurorack_pmod_trait {
    ($($t:ty),+ $(,)?) => {
        $(impl EurorackPmodTrait for $t {
            fn reset_line(&self, set_high: bool) {
                self.csr_reset.write(|w| unsafe { w.bits(set_high as u32) });
            }

            fn eeprom_serial(&self) -> u32 {
                self.csr_eeprom_serial.read().bits().into()
            }

            fn jack(&self) -> u8 {
                self.csr_jack.read().bits() as u8
            }

            fn input(&self, index: usize) -> i16 {
                (match index {
                    0 => self.csr_cal_in0.read().bits(),
                    1 => self.csr_cal_in1.read().bits(),
                    2 => self.csr_cal_in2.read().bits(),
                    3 => self.csr_cal_in3.read().bits(),
                    _ => panic!("bad index"),
                }) as i16
            }
        })+
    };
}

macro_rules! impl_wavetable_oscillator {
    ($($t:ty),+ $(,)?) => {
        $(impl WavetableOscillator for $t {
            fn set_skip(&self, value: u32) {
                unsafe {
                    self.csr_wavetable_inc.write(|w| w.csr_wavetable_inc().bits(value));
                }
            }
        })+
    };
}


macro_rules! impl_karlsen_lpf {
    ($($t:ty),+ $(,)?) => {
        $(impl KarlsenLpf for $t {
            fn set_cutoff(&self, value: i16) {
                unsafe {
                    self.csr_g.write(|w| w.csr_g().bits(value as u16));
                }
            }
            fn set_resonance(&self, value: i16) {
                unsafe {
                    self.csr_resonance.write(|w| w.csr_resonance().bits(value as u16));
                }
            }
        })+
    };
}

impl_eurorack_pmod_trait!(pac::EURORACK_PMOD0);
impl_eurorack_pmod_trait!(pac::EURORACK_PMOD1);
impl_wavetable_oscillator!(pac::WAVETABLE_OSCILLATOR0);
impl_wavetable_oscillator!(pac::WAVETABLE_OSCILLATOR1);
impl_wavetable_oscillator!(pac::WAVETABLE_OSCILLATOR2);
impl_wavetable_oscillator!(pac::WAVETABLE_OSCILLATOR3);
impl_karlsen_lpf!(pac::KARLSEN_LPF0);
impl_karlsen_lpf!(pac::KARLSEN_LPF1);
impl_karlsen_lpf!(pac::KARLSEN_LPF2);
impl_karlsen_lpf!(pac::KARLSEN_LPF3);

const SYSTEM_CLOCK_FREQUENCY: u32 = 60_000_000;

// Globals used by `defmt` logger such that we can log to UART from anywhere.
static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static mut UART_WRITER: Option<Uart> = None;

litex_hal::uart! {
    Uart: litex_pac::UART,
}

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

#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        unsafe {
            riscv::interrupt::disable();
            ENCODER.start_frame(do_write);
        }
    }
    unsafe fn flush() {}
    unsafe fn release() {
        ENCODER.end_frame(do_write);
        unsafe {
            riscv::interrupt::enable();
        }
    }
    unsafe fn write(bytes: &[u8]) {
        ENCODER.write(bytes, do_write);
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    loop {}
}

fn do_write(bytes: &[u8]) {
    unsafe {
        if let Some(writer) = &mut UART_WRITER {
            writer.bwrite_all(bytes).ok();
        }
    }
}

const N_WAVETABLE: u32 = 256;
const F_A3: f32 = 440f32;
const F_S: f32 = 93.75f32;

fn volt_to_skip(v: f32) -> u32 {
    (((N_WAVETABLE as f32 * F_A3) / F_S) * 2.0f32.powf(v - 3.75f32)) as u32
}

fn note_to_volt(midi_note: Note) -> f32 {
    let result = (((3 * 12) + u8::from(midi_note)) - 69) as f32 / 12.0f32;
    if result > 0.0f32 {
        result
    } else {
        0.0f32
    }
}

const N_VOICES: usize = 4;

#[derive(Copy, Clone, Debug, PartialEq)]
enum VoiceState {
    Off,
    On(Note, u32),
}

struct VoiceManager {
    voices: [VoiceState; N_VOICES],
}

impl VoiceManager {
    // For now we don't cull the oldest notes...
    fn note_on(&mut self, note: Note) {
        for v in self.voices.iter_mut() {
            if *v == VoiceState::Off {
                let skip = volt_to_skip(note_to_volt(note)) as u32;
                *v = VoiceState::On(note, skip);
                return;
            }
        }
    }

    fn note_off(&mut self, note: Note) {
        for v in self.voices.iter_mut() {
            if let VoiceState::On(v_note, _) = *v {
                if note == v_note {
                    *v = VoiceState::Off;
                }
            }
        }
    }
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
        write!(&mut s, "{:#06x}", v).ok();

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
    let pmod0 = peripherals.EURORACK_PMOD0;
    let pmod1 = peripherals.EURORACK_PMOD1;

    let osc: [&dyn WavetableOscillator; 4] = [
        &peripherals.WAVETABLE_OSCILLATOR0,
        &peripherals.WAVETABLE_OSCILLATOR1,
        &peripherals.WAVETABLE_OSCILLATOR2,
        &peripherals.WAVETABLE_OSCILLATOR3,
    ];

    let lpf: [&dyn KarlsenLpf; 4] = [
        &peripherals.KARLSEN_LPF0,
        &peripherals.KARLSEN_LPF1,
        &peripherals.KARLSEN_LPF2,
        &peripherals.KARLSEN_LPF3,
    ];

    unsafe {
        UART_WRITER = Some(Uart::new(peripherals.UART));
        if let Some(writer) = &mut UART_WRITER {
            writer
                .bwrite_all(b"hello from litex-fw! dropping to defmt logging --\n")
                .ok();
        }
    }

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


    defmt::info!("Starting main loop --");

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

        while let Ok(event) = midi_in.read() {
            //defmt::info!("MIDI event: {:?}", defmt::Debug2Format(&event));
            //update_skip = Some(volt_to_skip(note_to_volt(note)) as u32)
            match event {
                MidiMessage::NoteOn(_, note, velocity) => {
                    defmt::info!(
                        "note on: note={} vel={}",
                        u8::from(note),
                        u8::from(velocity)
                    );
                    voice_manager.note_on(note);
                }
                MidiMessage::NoteOff(_, note, velocity) => {
                    defmt::info!(
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


        let mut v0 = 0u8;
        if let VoiceState::On(note, _) = voice_manager.voices[0] {
            v0 = u8::from(note);
        }
        let mut v1 = 0u8;
        if let VoiceState::On(note, _) = voice_manager.voices[1] {
            v1 = u8::from(note);
        }
        let mut v2 = 0u8;
        if let VoiceState::On(note, _) = voice_manager.voices[2] {
            v2 = u8::from(note);
        }
        let mut v3 = 0u8;
        if let VoiceState::On(note, _) = voice_manager.voices[3] {
            v3 = u8::from(note);
        }

        draw_titlebox(&mut disp, 213, "MIDI", &[
          "v0",
          "v1",
          "v2",
          "v3",
        ], &[
          v0.into(),
          v1.into(),
          v2.into(),
          v3.into()
        ]).ok();

        disp.flush();

    }
}
