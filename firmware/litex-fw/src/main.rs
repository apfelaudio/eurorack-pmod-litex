#![no_std]
#![no_main]
#![allow(dead_code)]

use core::panic::PanicInfo;

use riscv;
use riscv_rt::entry;

use litex_pac as pac;
use litex_hal::prelude::*;

use litex_hal::uart::UartError;

use defmt;

use embedded_midi::MidiIn;

use midi_types::*;

use micromath::F32Ext;

use paste::paste;

litex_hal::uart! {
    Uart: litex_pac::UART,
}
litex_hal::uart! {
    UartMidi: litex_pac::UART_MIDI,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}

const SYSTEM_CLOCK_FREQUENCY: u32 = 12_000_000;

static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static mut UART_WRITER: Option<UartWriter> = None;

#[defmt::global_logger]
struct Logger;

struct UartWriter {
    uart: Uart
}

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        unsafe {
            riscv::interrupt::disable();
            ENCODER.start_frame(do_write);
        }
    }
    unsafe fn flush() {
    }
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
            writer.uart.bwrite_all(bytes).ok();
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


macro_rules! csr_read_n {
    ($periph:ident, $module:ident, $field:ident) => {
        paste!{
            [
                $periph.$module.[<$field 0>].read().bits(),
                $periph.$module.[<$field 1>].read().bits(),
                $periph.$module.[<$field 2>].read().bits(),
                $periph.$module.[<$field 3>].read().bits(),
            ]
        }
    };
}

macro_rules! csr_write_n {
    ($periph:ident, $module:ident, $index:expr, $field:ident, $value:expr) => {
        paste!{
            unsafe {
                match $index {
                    0 => $periph.[<$module 0>].$field.write(|w| w.$field().bits($value)),
                    1 => $periph.[<$module 1>].$field.write(|w| w.$field().bits($value)),
                    2 => $periph.[<$module 2>].$field.write(|w| w.$field().bits($value)),
                    3 => $periph.[<$module 3>].$field.write(|w| w.$field().bits($value)),
                    _ => ()
                }
            }
        }
    };
}

#[entry]
fn main() -> ! {

    let peripherals = unsafe { pac::Peripherals::steal() };

    let uart_midi = UartMidi::new(peripherals.UART_MIDI);

    unsafe {
        UART_WRITER = Some(UartWriter{uart: Uart::new(peripherals.UART)});
        if let Some(writer) = &mut UART_WRITER {
            writer.uart.bwrite_all(b"hello from litex-fw! dropping to defmt logging --\n").ok();
        }
    }

    let mut midi_in = MidiIn::new(uart_midi);

    let mut voice_manager = VoiceManager {
        voices: [VoiceState::Off; N_VOICES]
    };

    defmt::info!("Peripherals initialized");
    loop {

        let event = midi_in.read();

        //defmt::info!("MIDI event: {:?}", defmt::Debug2Format(&event));
        //update_skip = Some(volt_to_skip(note_to_volt(note)) as u32)
        match event {
            Ok(MidiMessage::NoteOn(_, note, velocity)) => {
                defmt::info!("note on: note={} vel={}",
                             u8::from(note), u8::from(velocity));
                voice_manager.note_on(note);
            }
            Ok(MidiMessage::NoteOff(_, note, velocity)) => {
                defmt::info!("note off: note={} vel={}",
                             u8::from(note), u8::from(velocity));
                voice_manager.note_off(note);
            }
            _ => {
            }
        }

        let ins = csr_read_n!(peripherals, EURORACK_PMOD0, csr_cal_in).map(|x| x as i16);

        for n_voice in 0..=3 {
            let mut write_skip = 0u32;
            if let VoiceState::On(_, skip) = voice_manager.voices[n_voice] {
                write_skip = skip;
            }
            csr_write_n!(peripherals, WAVETABLE_OSCILLATOR, n_voice, csr_wavetable_inc, write_skip);
            csr_write_n!(peripherals, KARLSEN_LPF, n_voice, csr_g, ins[1] as u16);
            csr_write_n!(peripherals, KARLSEN_LPF, n_voice, csr_resonance, ins[2] as u16);
        }
    }
}
