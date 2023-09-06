use heapless::String;
use lazy_static::lazy_static;

pub type OptionString = String<32>;

pub trait OptionTrait {
    fn name(&self) -> &OptionString;
    fn value(&self) -> OptionString;
    fn tick_up(&mut self);
    fn tick_down(&mut self);
}

pub struct Option<T> {
    pub name: OptionString,
    pub value: T,
    step: T,
    min: T,
    max: T,
}

pub struct Options {
    pub delay_len: Option<u32>,
    pub attack_ms: Option<u32>,
    pub decay_ms: Option<u32>,
    pub release_ms: Option<u32>,
    pub resonance: Option<i16>,
}

impl Options {
    pub fn new() -> Options {
        Options {
            delay_len: Option {
                name: "delay_len".into(),
                value: 511,
                step: 1,
                min: 128,
                max: 511,
            },
            attack_ms: Option {
                name: "attack_ms".into(),
                value: 100,
                step: 10,
                min: 10,
                max: 1000,
            },
            decay_ms: Option {
                name: "decay_ms".into(),
                value: 100,
                step: 10,
                min: 10,
                max: 1000,
            },
            release_ms: Option {
                name: "release_ms".into(),
                value: 100,
                step: 10,
                min: 10,
                max: 1000,
            },
            resonance: Option {
                name: "resonance".into(),
                value: 10000,
                step: 1000,
                min: 0,
                max: i16::MAX,
            },
        }
    }
}

impl<T: Copy +
        core::ops::Add<Output = T> +
        core::ops::Sub<Output = T> +
        core::cmp::PartialOrd +
        ufmt::uDisplay>
    OptionTrait for Option<T> {

    fn name(&self) -> &OptionString {
        &self.name
    }

    fn value(&self) -> OptionString {
        let mut s: OptionString = String::new();
        ufmt::uwrite!(&mut s, "{}", self.value).ok();
        s
    }

    fn tick_up(&mut self) {
        self.value = self.value + self.step;
        if self.value > self.max {
            self.value = self.max;
        }
    }

    fn tick_down(&mut self) {
        self.value = self.value - self.step;
        if self.value < self.min {
            self.value = self.min;
        }
    }
}
