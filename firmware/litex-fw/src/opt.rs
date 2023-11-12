use heapless::String;

use strum_macros::{EnumIter, IntoStaticStr};

pub type OptionString = String<32>;

pub trait OptionTrait {
    fn name(&self) -> &OptionString;
    fn value(&self) -> OptionString;
    fn tick_up(&mut self);
    fn tick_down(&mut self);
}

#[derive(Clone)]
pub struct NumOption<T> {
    pub name: OptionString,
    pub value: T,
    step: T,
    min: T,
    max: T,
}

#[derive(Clone)]
pub struct EnumOption<T> {
    pub name: OptionString,
    pub value: T,
}

#[derive(Clone, Copy, PartialEq, EnumIter, IntoStaticStr)]
#[strum(serialize_all = "kebab-case")]
pub enum EnumTest {
    ValueA,
    ValueB,
    ValueC
}

#[derive(Clone)]
pub struct Options {
    pub modify: bool,
    pub selected: usize,
    pub attack_ms: NumOption<u32>,
    pub decay_ms: NumOption<u32>,
    pub release_ms: NumOption<u32>,
    pub resonance: NumOption<i16>,
    pub delay_len: NumOption<u32>,
    pub enum_test: EnumOption<EnumTest>,
}

impl Options {
    pub fn new() -> Options {
        Options {
            modify: false,
            selected: 0,
            delay_len: NumOption{
                name: "delayln".into(),
                value: 511,
                step: 1,
                min: 128,
                max: 511,
            },
            attack_ms: NumOption{
                name: "attack".into(),
                value: 100,
                step: 50,
                min: 0,
                max: 5000,
            },
            decay_ms: NumOption{
                name: "decay".into(),
                value: 100,
                step: 50,
                min: 0,
                max: 5000,
            },
            release_ms: NumOption{
                name: "release".into(),
                value: 300,
                step: 50,
                min: 0,
                max: 5000,
            },
            resonance: NumOption{
                name: "reso".into(),
                value: 10000,
                step: 1000,
                min: 0,
                max: 20000,
            },
            enum_test: EnumOption{
                name: "enumt".into(),
                value: EnumTest::ValueA,
            },
        }
    }

    pub fn toggle_modify(&mut self) {
        self.modify = !self.modify;
    }

    pub fn tick_up(&mut self) {
        let selected = self.selected;
        if self.modify {
            self.view_mut()[selected].tick_up();
        } else if selected < self.view().len()-1 {
            self.selected = selected + 1;
        }
    }

    pub fn tick_down(&mut self) {
        let selected = self.selected;
        if self.modify {
            self.view_mut()[selected].tick_down();
        } else if selected != 0 {
            self.selected = selected - 1;
        }
    }

    #[allow(dead_code)]
    pub fn view(&self) -> [& dyn OptionTrait; 6] {
        [
            &self.enum_test,
            &self.attack_ms,
            &self.decay_ms,
            &self.release_ms,
            &self.resonance,
            &self.delay_len,
        ]
    }

    #[allow(dead_code)]
    pub fn view_mut(&mut self) -> [&mut dyn OptionTrait; 6] {
        [
            &mut self.enum_test,
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
    OptionTrait for NumOption<T> {

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

impl<T: Copy + strum::IntoEnumIterator + PartialEq + Into<&'static str>>
    OptionTrait for EnumOption<T> {

    fn name(&self) -> &OptionString {
        &self.name
    }

    fn value(&self) -> OptionString {
        String::from(self.value.into())
    }

    fn tick_up(&mut self) {
        let mut it = T::iter();
        for v in it.by_ref() {
            if v == self.value {
                break;
            }
        }
        if let Some(v) = it.next() {
            self.value = v;
        }
    }

    fn tick_down(&mut self) {
        let it = T::iter();
        let mut last_value: Option<T> = None;
        for v in it {
            if v == self.value {
                if let Some(lv) = last_value {
                    self.value = lv;
                    return;
                }
            }
            last_value = Some(v);
        }
    }
}
