#![allow(unused_macros)]

use litex_hal::prelude::*;
use litex_pac as pac;
use litex_hal::uart::UartError;

pub const SYSTEM_CLOCK_FREQUENCY: u32 = 60_000_000;

pub trait EurorackPmod {
    fn reset_line(&self, set_high: bool);
    fn reset(&self, timer: &mut Timer);
    fn eeprom_serial(&self) -> u32;
    fn jack(&self) -> u8;
    fn touch(&self) -> [u8; 8];
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

macro_rules! eurorack_pmod {
    ($($t:ty),+ $(,)?) => {
        $(impl EurorackPmod for $t {
            fn reset_line(&self, set_high: bool) {
                self.csr_reset().write(|w| unsafe { w.bits(set_high as u32) });
            }

            fn reset(&self, timer: &mut Timer) {
                self.reset_line(true);
                timer.delay_ms(10u32);
                self.reset_line(false);
            }

            fn eeprom_serial(&self) -> u32 {
                self.csr_eeprom_serial().read().bits().into()
            }

            fn jack(&self) -> u8 {
                self.csr_jack().read().bits() as u8
            }

            fn touch(&self) -> [u8; 8] {
                [
                    self.csr_touch0().read().bits() as u8,
                    self.csr_touch1().read().bits() as u8,
                    self.csr_touch2().read().bits() as u8,
                    self.csr_touch3().read().bits() as u8,
                    self.csr_touch4().read().bits() as u8,
                    self.csr_touch5().read().bits() as u8,
                    self.csr_touch6().read().bits() as u8,
                    self.csr_touch7().read().bits() as u8,
                ]
            }

            fn input(&self, index: usize) -> i16 {
                (match index {
                    0 => self.csr_cal_in0().read().bits(),
                    1 => self.csr_cal_in1().read().bits(),
                    2 => self.csr_cal_in2().read().bits(),
                    3 => self.csr_cal_in3().read().bits(),
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
                    self.csr_wavetable_inc().write(|w| w.csr_wavetable_inc().bits(value));
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
                    self.csr_pitch().write(|w| w.csr_pitch().bits(value as u16));
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
                    self.csr_g().write(|w| w.csr_g().bits(value as u16));
                }
            }
            fn set_resonance(&self, value: i16) {
                unsafe {
                    self.csr_resonance().write(|w| w.csr_resonance().bits(value as u16));
                }
            }
        })+
    };
}

eurorack_pmod!(pac::EURORACK_PMOD0);
pitch_shift!(pac::PITCH_SHIFT0);
pitch_shift!(pac::PITCH_SHIFT1);
pitch_shift!(pac::PITCH_SHIFT2);
pitch_shift!(pac::PITCH_SHIFT3);
karlsen_lpf!(pac::KARLSEN_LPF0);
karlsen_lpf!(pac::KARLSEN_LPF1);
karlsen_lpf!(pac::KARLSEN_LPF2);
karlsen_lpf!(pac::KARLSEN_LPF3);

litex_hal::uart! {
    UartMidi: litex_pac::UART,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}

litex_hal::gpio! {
    OledGpio: litex_pac::OLED_CTL,
}

litex_hal::spi! {
    OledSpi: (litex_pac::OLED_SPI, u8),
}

pub fn get_shifters(p: &pac::Peripherals) -> [&dyn PitchShift; 4] {
    [
        &p.PITCH_SHIFT0,
        &p.PITCH_SHIFT1,
        &p.PITCH_SHIFT2,
        &p.PITCH_SHIFT3,
    ]
}

pub fn get_lpfs(p: &pac::Peripherals) -> [&dyn KarlsenLpf; 4] {
    [
        &p.KARLSEN_LPF0,
        &p.KARLSEN_LPF1,
        &p.KARLSEN_LPF2,
        &p.KARLSEN_LPF3,
    ]
}

pub struct Encoder {
    encoder: pac::ROTARY_ENCODER,
    button: pac::ENCODER_BUTTON,
    enc_last: u8,
    btn_held_ms: u32,
    short_press: bool,
    long_press: bool,
    ticks_since_last_read: i32,
}

impl Encoder {
    // NOTE: not easily reuseable pecause of pac naming?
    pub fn new(encoder: pac::ROTARY_ENCODER, button: pac::ENCODER_BUTTON) -> Self {
        let enc_last: u8 = encoder.csr_state().read().bits() as u8;
        Self {
            encoder,
            button,
            enc_last,
            btn_held_ms: 0,
            short_press: false,
            long_press: false,
            ticks_since_last_read: 0,
        }
    }

    pub fn update_ticks(&mut self, ms_per_tick: u32) {
        let enc_now: u8 = self.encoder.csr_state().read().bits() as u8;
        let mut enc_delta: i32 = (enc_now as i32) - (self.enc_last as i32);

        // encoder tick over/underflow: source is an 8-bit counter.
        let m = u8::MAX as i32;
        if enc_delta > m/2 {
            enc_delta = m - enc_delta;
        } else if enc_delta < -m/2 {
            enc_delta += m;
        }

        self.enc_last = enc_now;

        if self.button.in_().read().bits() != 0 {
            self.btn_held_ms += ms_per_tick;
        } else {
            if self.btn_held_ms > 0 {
                self.short_press = true;
            }
            self.btn_held_ms = 0;
        }

        if self.btn_held_ms > 3000 {
            self.long_press = true;
        }

        self.ticks_since_last_read += enc_delta;
    }

    pub fn pending_short_press(&mut self) -> bool {
        let result = self.short_press;
        self.short_press = false;
        result
    }

    pub fn pending_long_press(&mut self) -> bool {
        let result = self.long_press;
        self.long_press = false;
        result
    }

    pub fn pending_ticks(&mut self) -> i32 {
        let result = self.ticks_since_last_read >> 2;
        self.ticks_since_last_read -= result << 2;
        result
    }
}

pub struct SpiDma {
    spi_dma: pac::SPI_DMA,
}

impl SpiDma {
    pub fn new(spi_dma: pac::SPI_DMA,
           target: *const pac::oled_spi::RegisterBlock) -> Self {
        unsafe {
            // TODO: Any way to get RegisterBlock sub-addresses automagically?
            spi_dma.spi_control_reg_address().write(
                |w| w.bits(target as u32));
            spi_dma.spi_status_reg_address().write(
                |w| w.bits(target as u32 + 0x04));
            spi_dma.spi_mosi_reg_address().write(
                |w| w.bits(target as u32 + 0x08));
        }
        Self {
            spi_dma
        }
    }

    pub fn block(&self) {
        while self.spi_dma.done().read().bits() == 0 {
            // Wait for an existing transfer to complete.
        }
    }

    pub fn transfer(&mut self, data_ptr: *const u8, data_len: usize) {
        unsafe {
            self.spi_dma.read_base().write(|w| w.bits(data_ptr as u32));
            self.spi_dma.read_length().write(|w| w.bits(data_len as u32));
            self.spi_dma.start().write(|w| w.start().bit(true));
            self.spi_dma.start().write(|w| w.start().bit(false));
        }
    }
}

pub struct DmaRouter {
    reg: pac::DMA_ROUTER0,
}

impl DmaRouter {
    pub fn new(reg: pac::DMA_ROUTER0, buf_in_mut_ptr: u32, buf_sz_words: u32) -> Self {
        unsafe {
            reg.base_writer().write(|w| w.bits(buf_in_mut_ptr));
            reg.length_words().write(|w| w.bits(buf_sz_words));
            reg.enable().write(|w| w.bits(1u32));
            reg.ev_enable().write(|w| w.half().bit(true));
        }
        Self {
            reg
        }
    }

    pub fn offset(&self) -> usize {
        self.reg.offset_words().read().bits() as usize
    }
}

pub unsafe fn reset_soc(ctrl: &pac::CTRL) {
    ctrl.reset().write(|w| w.soc_rst().bit(true));
}
