#![allow(clippy::empty_loop)]

use core::panic::PanicInfo;
use heapless::String;
use litex_pac as pac;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;

litex_hal::uart! {
    Uart: pac::UART_MIDI,
}

static mut UART_WRITER: Option<Uart> = None;

#[cfg(not(test))]
#[panic_handler]
fn panic(panic_info: &PanicInfo) -> ! {
    if let Some(location) = panic_info.location() {
        info!("panic_handler: file '{}' at line {}",
            location.file(),
            location.line(),
        );
    } else {
        info!("panic_handler: no location information");
    }
    loop {}
}

#[export_name = "ExceptionHandler"]
fn exception_handler(trap_frame: &riscv_rt::TrapFrame) -> ! {
    _logger_write(b"exception_handler\n");
    info!("mcause: {:#x}", riscv::register::mcause::read().bits());
    info!("mtval: {:#x}", riscv::register::mtval::read());
    info!("mepc: {:#x}", riscv::register::mepc::read());
    info!("ra: {:#x}", trap_frame.ra);
    info!("t0: {:#x}", trap_frame.t0);
    info!("t1: {:#x}", trap_frame.t1);
    info!("t2: {:#x}", trap_frame.t2);
    info!("t3: {:#x}", trap_frame.t3);
    info!("t4: {:#x}", trap_frame.t4);
    info!("t5: {:#x}", trap_frame.t5);
    info!("t6: {:#x}", trap_frame.t6);
    info!("a0: {:#x}", trap_frame.a0);
    info!("a1: {:#x}", trap_frame.a1);
    info!("a2: {:#x}", trap_frame.a2);
    info!("a3: {:#x}", trap_frame.a3);
    info!("a4: {:#x}", trap_frame.a4);
    info!("a5: {:#x}", trap_frame.a5);
    info!("a6: {:#x}", trap_frame.a6);
    info!("a7: {:#x}", trap_frame.a7);
    loop {}
}

pub fn _logger_write(bytes: &[u8]) {
    unsafe {
        if let Some(writer) = &mut UART_WRITER {
            writer.bwrite_all(bytes).ok();
        }
    }
}

pub fn init(uart: pac::UART_MIDI) {
    unsafe {
        UART_WRITER = Some(Uart::new(uart));
        if let Some(writer) = &mut UART_WRITER {
            writer
                .bwrite_all(b"UART logger up!\n")
                .ok();
        }
    }
}

macro_rules! info {
    () => {
        _logger_write(b"\n");
    };
    ($($arg:tt)*) => {{
        let mut s: String<256> = String::new();
        ufmt::uwrite!(&mut s, $($arg)*).ok();
        _logger_write(&s.into_bytes());
        _logger_write(b"\n");
    }};
}

pub(crate) use info;
