use micromath::F32Ext;
use midi_types::Note;

pub const N_VOICES: usize = 4;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum VoiceState {
    Off,
    On(Note, u32),
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

pub struct VoiceManager {
    pub voices: [VoiceState; N_VOICES],
}

impl VoiceManager {
    // For now we don't cull the oldest notes...
    pub fn note_on(&mut self, note: Note) {
        for v in self.voices.iter_mut() {
            if *v == VoiceState::Off {
                let skip = volt_to_skip(note_to_volt(note)) as u32;
                *v = VoiceState::On(note, skip);
                return;
            }
        }
    }

    pub fn note_off(&mut self, note: Note) {
        for v in self.voices.iter_mut() {
            if let VoiceState::On(v_note, _) = *v {
                if note == v_note {
                    *v = VoiceState::Off;
                }
            }
        }
    }
}
