use micromath::F32Ext;
use midi_types::Note;
use ufmt::derive::uDebug;

pub const N_VOICES: usize = 4;

#[derive(Copy, Clone, Debug, PartialEq, uDebug)]
pub enum VoiceState {
    Idle,
    Attack,
    Decay,
    Sustain,
    Release(u32),
}

pub struct Voice {
    pub note: u8,
    pub start_time_ms: u32,
    pub state: VoiceState,
    pub pitch: i16,
    pub amplitude: f32,
}

impl Voice {
    fn new(note: Note, start_time_ms: u32, state: VoiceState) -> Voice {
        Voice {
            note: note.into(),
            start_time_ms,
            state,
            pitch: note_to_pitch(note.into()),
            amplitude: 0.0f32,
        }
    }
}

pub struct VoiceManager {
    pub voices: [Voice; N_VOICES],
}

impl VoiceManager {
    pub fn new() -> VoiceManager {
        VoiceManager {
            voices: [
                Voice::new(0.into(), 0, VoiceState::Idle),
                Voice::new(0.into(), 0, VoiceState::Idle),
                Voice::new(0.into(), 0, VoiceState::Idle),
                Voice::new(0.into(), 0, VoiceState::Idle),
            ]
        }
    }
}

fn note_to_pitch(note: u8) -> i16 {
    let scale = 2.0f32.powf(((note as i16) - 60) as f32 / 12.0f32);
    ((1.0f32 - scale) * 32768.0f32) as i16
}


const ATTACK_MS: u32 = 100u32;
const DECAY_MS: u32 = 200u32;
const RELEASE_MS: u32 = 500u32;

impl VoiceManager {
    pub fn note_on(&mut self, note: Note, time_ms: u32) {
        for v in self.voices.iter_mut() {
            if v.state == VoiceState::Idle {
                *v = Voice::new(note, time_ms, VoiceState::Attack);
                return;
            }
            if u8::from(note) == v.note && v.state != VoiceState::Idle {
                // Retrigger if same note already in a voice.
                v.start_time_ms = time_ms;
                v.state = VoiceState::Attack;
                return;
            }
        }
    }

    pub fn note_off(&mut self, note: Note, time_ms: u32) {
        for v in self.voices.iter_mut() {
            if u8::from(note) == v.note && v.state != VoiceState::Idle {
                v.state = VoiceState::Release(time_ms);
            }
        }
    }

    pub fn tick(&mut self, time_ms: u32) {
        for v in self.voices.iter_mut() {
            v.state = match v.state {
                VoiceState::Attack => {
                    v.amplitude = (time_ms - v.start_time_ms) as f32 / ATTACK_MS as f32;
                    if time_ms > ATTACK_MS + v.start_time_ms {
                        VoiceState::Decay
                    } else {
                        VoiceState::Attack
                    }
                },
                VoiceState::Decay => {
                    v.amplitude = 1.0f32 - 0.2f32 * (time_ms - ATTACK_MS - v.start_time_ms) as f32 / DECAY_MS as f32;
                    if time_ms > ATTACK_MS + DECAY_MS + v.start_time_ms {
                        VoiceState::Sustain
                    } else {
                        VoiceState::Decay
                    }
                },
                VoiceState::Sustain => {
                    v.amplitude = 0.8f32;
                    VoiceState::Sustain
                }
                VoiceState::Release(release_time_ms) => {
                    v.amplitude = 0.8f32 * (1.0f32 - (time_ms - release_time_ms) as f32 / RELEASE_MS as f32);
                    if time_ms > release_time_ms + RELEASE_MS {
                        VoiceState::Idle
                    } else {
                        VoiceState::Release(release_time_ms)
                    }
                }
                _ => {
                    v.note = 0u8;
                    v.amplitude = 0.0f32;
                    VoiceState::Idle
                },
            }
        }
    }
}
