use micromath::F32Ext;
use midi_types::*;
use ufmt::derive::uDebug;

use crate::opt::Options;

pub const N_VOICES: usize = 8;

#[derive(Copy, Clone, Debug, PartialEq, uDebug)]
pub enum VoiceState {
    Idle,
    Attack,
    Decay,
    Sustain,
    Release(u32),
}

#[derive(Clone)]
pub struct AdsrParams {
    attack_ms: u32,
    decay_ms: u32,
    release_ms: u32,
    attack_amplitude: f32,
    sustain_amplitude: f32,
}

#[derive(Clone)]
pub struct Voice {
    pub note: u8,
    pub start_time_ms: u32,
    pub state: VoiceState,
    pub pitch: f32,
    pub amplitude: f32,
    pub velocity: u8,
    pub adsr: Option<AdsrParams>,
}

impl Voice {
    fn new(note: u8, start_time_ms: u32, state: VoiceState, velocity: u8) -> Voice {
        Voice {
            note,
            start_time_ms,
            state,
            pitch: note_to_pitch(note),
            amplitude: 0.0f32,
            velocity,
            adsr: None,
            /*
            AdsrParams {
                attack_ms: 100u32,
                decay_ms: 100u32,
                release_ms: 300u32,
                attack_amplitude: 1.0f32 * (velocity as f32 / 128.0f32),
                sustain_amplitude: 0.8f32 * (velocity as f32 / 128.0f32),
            }
            */
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
                Voice::new(0, 0, VoiceState::Idle, 0),
                Voice::new(0, 0, VoiceState::Idle, 0),
                Voice::new(0, 0, VoiceState::Idle, 0),
                Voice::new(0, 0, VoiceState::Idle, 0),
                Voice::new(0, 0, VoiceState::Idle, 0),
                Voice::new(0, 0, VoiceState::Idle, 0),
                Voice::new(0, 0, VoiceState::Idle, 0),
                Voice::new(0, 0, VoiceState::Idle, 0),
            ]
        }
    }
}

pub fn note_to_pitch(note: u8) -> f32 {
    let scale = 2.0f32.powf(((note as i16) - 60) as f32 / 12.0f32);
    1.0f32 - scale
}


impl VoiceManager {
    pub fn event(&mut self, message: MidiMessage, time_ms: u32) {
        match message {
            MidiMessage::NoteOn(_, note, velocity) => {
                self.note_on(u8::from(note), u8::from(velocity), time_ms);
            }
            MidiMessage::NoteOff(_, note, _velocity) => {
                self.note_off(u8::from(note), time_ms);
            }
            _ => {}
        }
    }

    fn note_on(&mut self, note: u8, velocity: u8, time_ms: u32) {
        let mut oldest_note_releasing_ts = u32::MAX;
        for v in self.voices.iter_mut() {
            if let VoiceState::Release(ts) = v.state {
                if ts < oldest_note_releasing_ts {
                    oldest_note_releasing_ts = ts;
                }
            }
            if let VoiceState::Idle = v.state {
                *v = Voice::new(note, time_ms, VoiceState::Attack, velocity);
                return;
            }
            // Retrigger if same note already in a voice.
            if note == v.note && v.state != VoiceState::Idle {
                *v = Voice::new(note, time_ms, VoiceState::Attack, velocity);
                return;
            }
        }

        for v in self.voices.iter_mut() {
            if let VoiceState::Release(ts) = v.state {
                if ts == oldest_note_releasing_ts {
                    *v = Voice::new(note, time_ms, VoiceState::Attack, velocity);
                    return;
                }
            }
        }
    }

    fn note_off(&mut self, note: u8, time_ms: u32) {
        for v in self.voices.iter_mut() {
            if note == v.note && v.state != VoiceState::Idle {
                v.state = VoiceState::Release(time_ms);
            }
        }
    }

    pub fn tick(&mut self, time_ms: u32, opts: &Options) {
        for v in self.voices.iter_mut() {

            if v.adsr.is_none() {
                v.adsr = Some( AdsrParams {
                    attack_ms: opts.adsr.attack_ms.value,
                    decay_ms: opts.adsr.decay_ms.value,
                    release_ms: opts.adsr.release_ms.value,
                    attack_amplitude: 1.0f32 * (v.velocity as f32 / 128.0f32),
                    sustain_amplitude: 0.8f32 * (v.velocity as f32 / 128.0f32),
                });
            }

            if let Some(adsr) = &v.adsr {
                v.state = match v.state {
                    VoiceState::Attack => {
                        v.amplitude = adsr.attack_amplitude * (time_ms - v.start_time_ms) as f32 / adsr.attack_ms as f32;
                        if time_ms > adsr.attack_ms + v.start_time_ms {
                            VoiceState::Decay
                        } else {
                            VoiceState::Attack
                        }
                    },
                    VoiceState::Decay => {
                        v.amplitude = adsr.attack_amplitude - (adsr.attack_amplitude - adsr.sustain_amplitude) * (time_ms - adsr.attack_ms - v.start_time_ms) as f32 / adsr.decay_ms as f32;
                        if time_ms > adsr.attack_ms + adsr.decay_ms + v.start_time_ms {
                            VoiceState::Sustain
                        } else {
                            VoiceState::Decay
                        }
                    },
                    VoiceState::Sustain => {
                        v.amplitude = adsr.sustain_amplitude;
                        VoiceState::Sustain
                    }
                    VoiceState::Release(release_time_ms) => {
                        v.amplitude = adsr.sustain_amplitude * (1.0f32 - (time_ms - release_time_ms) as f32 / adsr.release_ms as f32);
                        if time_ms > release_time_ms + adsr.release_ms {
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
            if v.amplitude > 1.0f32 {
                v.amplitude = 1.0f32;
            }
            if v.amplitude < 0.0f32 {
                v.amplitude = 0.0f32;
            }
        }
    }
}
