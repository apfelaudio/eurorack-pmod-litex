use heapless::String;

pub type OptionString = String<32>;

pub trait OptionTrait {
    fn name(&self) -> &OptionString;
    fn value(&self) -> OptionString;
    fn tick_up(&mut self);
    fn tick_down(&mut self);
}

#[derive(Clone)]
pub struct Option<T> {
    pub name: OptionString,
    pub value: T,
    step: T,
    min: T,
    max: T,
}

#[derive(Clone)]
pub struct Options {
    pub attack_ms: Option<u32>,
    pub decay_ms: Option<u32>,
    pub release_ms: Option<u32>,
    pub resonance: Option<i16>,
    pub delay_len: Option<u32>,
}

impl Options {
    pub fn new() -> Options {
        Options {
            delay_len: Option {
                name: "delayln".into(),
                value: 511,
                step: 1,
                min: 128,
                max: 511,
            },
            attack_ms: Option {
                name: "attack".into(),
                value: 100,
                step: 50,
                min: 0,
                max: 5000,
            },
            decay_ms: Option {
                name: "decay".into(),
                value: 100,
                step: 50,
                min: 0,
                max: 5000,
            },
            release_ms: Option {
                name: "release".into(),
                value: 300,
                step: 50,
                min: 0,
                max: 5000,
            },
            resonance: Option {
                name: "reso".into(),
                value: 10000,
                step: 1000,
                min: 0,
                max: 20000,
            },
        }
    }

    #[allow(dead_code)]
    pub fn view(&self) -> [& dyn OptionTrait; 5] {
        [
            &self.attack_ms,
            &self.decay_ms,
            &self.release_ms,
            &self.resonance,
            &self.delay_len,
        ]
    }

    #[allow(dead_code)]
    pub fn view_mut(&mut self) -> [&mut dyn OptionTrait; 5] {
        [
            &mut self.attack_ms,
            &mut self.decay_ms,
            &mut self.release_ms,
            &mut self.resonance,
            &mut self.delay_len,
        ]
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
        if self.value >= self.max {
            self.value = self.max;
            return
        }
        if self.value + self.step >= self.max {
            self.value = self.max;
        } else {
            self.value = self.value + self.step;
        }
    }

    fn tick_down(&mut self) {
        if self.value <= self.min {
            self.value = self.min;
            return
        }
        if self.value - self.step <= self.min {
            self.value = self.min;
        } else {
            self.value = self.value - self.step;
        }
    }
}
