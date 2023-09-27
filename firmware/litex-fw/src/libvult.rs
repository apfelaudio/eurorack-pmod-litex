#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/libvult_bindings.rs"));

type WrappedContext = Dsp_process_type;

pub struct VultDsp {
    _ctx: WrappedContext,
    _time: fix16_t,
    _feedback: fix16_t,
}

impl VultDsp {
    pub unsafe fn new() -> VultDsp {
        let mut ctx: WrappedContext = unsafe { core::mem::zeroed() };
        Dsp_process_init(&mut ctx);

        VultDsp {
            _ctx: ctx,
            _time: short_to_fix(0x7FFF),
            _feedback: short_to_fix(0x4FFF),
        }
    }

    pub unsafe fn process(&mut self, input: i16) -> i16 {
        Dsp_process(&mut self._ctx, short_to_fix(input), self._time, self._feedback) as i16
    }
}
