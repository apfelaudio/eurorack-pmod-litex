#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/libvult_bindings.rs"));

pub fn process(input: i16) -> i16 {
    unsafe {
        Dsp_process(input as i32, 0x8000) as i16
    }
}
