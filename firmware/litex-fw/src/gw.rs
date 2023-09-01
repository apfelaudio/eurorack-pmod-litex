pub trait EurorackPmod {
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

pub(crate) use eurorack_pmod;
pub(crate) use wavetable_oscillator;
pub(crate) use karlsen_lpf;
