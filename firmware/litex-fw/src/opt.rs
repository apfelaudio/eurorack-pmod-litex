use heapless::String;
use heapless::Vec;

use strum_macros::{EnumIter, IntoStaticStr};

pub type OptionString = String<32>;
pub type OptionView<'a> = Vec<&'a dyn OptionTrait, 10>;
pub type OptionViewMut<'a> = Vec<&'a mut dyn OptionTrait, 10>;

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

#[derive(Clone, Copy, PartialEq, EnumIter, IntoStaticStr)]
#[strum(serialize_all = "SCREAMING-KEBAB-CASE")]
pub enum Screen {
    Adsr,
    Scope,
}


#[derive(Clone)]
pub struct AdsrOptions {
    pub selected: Option<usize>,
    pub attack_ms: NumOption<u32>,
    pub decay_ms: NumOption<u32>,
    pub release_ms: NumOption<u32>,
    pub resonance: NumOption<i16>,
}

#[derive(Clone)]
pub struct ScopeOptions {
    pub selected: Option<usize>,
    pub delay_len: NumOption<u32>,
    pub enum_test: EnumOption<EnumTest>,
}

#[derive(Clone)]
pub struct Options {
    pub modify: bool,
    pub screen: EnumOption<Screen>,

    pub adsr: AdsrOptions,
    pub scope: ScopeOptions,
}

impl Options {
    pub fn new() -> Options {
        Options {
            modify: true,
            screen: EnumOption {
                name: "screen".into(),
                value: Screen::Adsr,
            },
            adsr: AdsrOptions {
                selected: None,
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
            },
            scope: ScopeOptions {
                selected: None,
                delay_len: NumOption{
                    name: "delayln".into(),
                    value: 511,
                    step: 1,
                    min: 128,
                    max: 511,
                },
                enum_test: EnumOption{
                    name: "enumt".into(),
                    value: EnumTest::ValueA,
                },
            }
        }
    }

    pub fn toggle_modify(&mut self) {
        self.modify = !self.modify;
    }

    pub fn tick_up(&mut self) {
        if let Some(n_selected) = self.selected() {
            if self.modify {
                self.view_mut()[n_selected].tick_up();
            } else if n_selected < self.view().len()-1 {
                *self.selected_mut() = Some(n_selected + 1);
            }
        } else if self.modify {
            self.screen.tick_up();
        } else if !self.view().is_empty() {
            *self.selected_mut() = Some(0);
        }
    }

    pub fn tick_down(&mut self) {
        if let Some(n_selected) = self.selected() {
            if self.modify {
                self.view_mut()[n_selected].tick_down();
            } else if n_selected != 0 {
                *self.selected_mut() = Some(n_selected - 1);
            } else {
                *self.selected_mut() = None;
            }
        } else if self.modify {
            self.screen.tick_down();
        }
    }

    pub fn selected(&self) -> Option<usize> {
        match self.screen.value {
            Screen::Adsr => self.adsr.selected,
            Screen::Scope => self.scope.selected,
        }
    }

    pub fn selected_mut(&mut self) -> &mut Option<usize> {
        match self.screen.value {
            Screen::Adsr => &mut self.adsr.selected,
            Screen::Scope => &mut self.scope.selected,
        }
    }

    #[allow(dead_code)]
    pub fn view(&self) -> OptionView {
        match self.screen.value {
            Screen::Adsr => OptionView::from_slice(&[
                &self.adsr.attack_ms,
                &self.adsr.decay_ms,
                &self.adsr.release_ms,
                &self.adsr.resonance,
            ]),
            Screen::Scope => OptionView::from_slice(&[
                &self.scope.delay_len,
                &self.scope.enum_test,
            ]),
        }.unwrap()
    }

    #[allow(dead_code)]
    fn view_mut(&mut self) -> OptionViewMut {
        let mut r = OptionViewMut::new();
        match self.screen.value {
            Screen::Adsr => {
                r.push(&mut self.adsr.attack_ms).ok();
                r.push(&mut self.adsr.decay_ms).ok();
                r.push(&mut self.adsr.release_ms).ok();
                r.push(&mut self.adsr.resonance).ok();
            },
            Screen::Scope => {
                r.push(&mut self.scope.delay_len).ok();
                r.push(&mut self.scope.enum_test).ok();
            },
        }
        r
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
