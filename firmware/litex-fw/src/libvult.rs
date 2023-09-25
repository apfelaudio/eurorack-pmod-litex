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
        // TODO: take slow param conversions out of here.
        let time: fix16_t = short_to_fix(0x7FFF);
        let feedback: fix16_t = short_to_fix(0x4FFF);
        Dsp_process(self, short_to_fix(input), time, feedback) as i16
    }
}
