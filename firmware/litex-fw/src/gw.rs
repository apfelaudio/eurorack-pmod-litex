pub trait EurorackPmod {
    fn reset_line(&self, set_high: bool);
    fn eeprom_serial(&self) -> u32;
    fn jack(&self) -> u8;
    fn input(&self, index: usize) -> i16;
}

pub trait WavetableOscillator {
    fn set_skip(&self, value: u32);
}

pub trait PitchShift {
    fn set_pitch(&self, value: i16);
}

pub trait KarlsenLpf {
    fn set_cutoff(&self, value: i16);
    fn set_resonance(&self, value: i16);
}

pub trait PwmLed {
    fn reset_line(&self, set_high: bool);
    fn led(&self, ix: usize, value: u8);
}

macro_rules! eurorack_pmod {
    ($($t:ty),+ $(,)?) => {
        $(impl EurorackPmod for $t {
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


macro_rules! wavetable_oscillator {
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

macro_rules! pitch_shift {
    ($($t:ty),+ $(,)?) => {
        $(impl PitchShift for $t {
            fn set_pitch(&self, value: i16) {
                unsafe {
                    self.csr_pitch.write(|w| w.csr_pitch().bits(value as u16));
                }
            }
        })+
    };
}

macro_rules! karlsen_lpf {
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

macro_rules! pwm_led {
    ($($t:ty),+ $(,)?) => {
        $(impl PwmLed for $t {
            fn reset_line(&self, set_high: bool) {
                self.csr_reset.write(|w| unsafe { w.bits(set_high as u32) });
            }
            fn led(&self, ix: usize, value: u8) {
                unsafe {
                    let v = value as u32;
                    match ix {
                        0 => self.led0.write(|w| w.bits(v)),
                        1 => self.led1.write(|w| w.bits(v)),
                        2 => self.led2.write(|w| w.bits(v)),
                        3 => self.led3.write(|w| w.bits(v)),
                        4 => self.led4.write(|w| w.bits(v)),
                        5 => self.led5.write(|w| w.bits(v)),
                        6 => self.led6.write(|w| w.bits(v)),
                        7 => self.led7.write(|w| w.bits(v)),
                        8 => self.led8.write(|w| w.bits(v)),
                        9 => self.led9.write(|w| w.bits(v)),
                        10 => self.led10.write(|w| w.bits(v)),
                        11 => self.led11.write(|w| w.bits(v)),
                        12 => self.led12.write(|w| w.bits(v)),
                        13 => self.led13.write(|w| w.bits(v)),
                        14 => self.led14.write(|w| w.bits(v)),
                        15 => self.led15.write(|w| w.bits(v)),
                        _ => panic!("bad index")
                    }
                }
            }
        })+
    };
}

pub(crate) use eurorack_pmod;
pub(crate) use wavetable_oscillator;
pub(crate) use pitch_shift;
pub(crate) use karlsen_lpf;
pub(crate) use pwm_led;
