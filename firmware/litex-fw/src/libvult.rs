#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/libvult_bindings.rs"));

pub type DspProcessType = Dsp_process_type;

impl DspProcessType {
    pub unsafe fn new() -> DspProcessType {
        let mut ctx: DspProcessType = unsafe { core::mem::zeroed() };
        Dsp_process_init(&mut ctx);
        ctx
    }

    pub unsafe fn process(&mut self, input: i16) -> i16 {
        Dsp_process(self, input as i32, 0, 0) as i16
    }
}
