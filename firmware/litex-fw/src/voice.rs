use micromath::F32Ext;
use midi_types::Note;

pub const N_VOICES: usize = 4;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum VoiceState {
    Off,
    On(Note, i16),
}

pub struct VoiceManager {
    pub voices: [VoiceState; N_VOICES],
}

fn note_to_pitch(note: u8) -> i16 {
    let scale = 2.0f32.powf(((note as i16) - 60) as f32 / 12.0f32);
    ((1.0f32 - scale) * 32768.0f32) as i16
}

impl VoiceManager {
    // For now we don't cull the oldest notes...
    pub fn note_on(&mut self, note: Note) {
        for v in self.voices.iter_mut() {
            if *v == VoiceState::Off {
                *v = VoiceState::On(note, note_to_pitch(note.into()));
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
