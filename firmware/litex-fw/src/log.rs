#![allow(clippy::empty_loop)]

/// Somewhat fragile implementation of a UART logger so you can
/// do a `log::info("format {} string {}", a, b)` elsewhere in
/// the codebase.
///
/// Also (ab)uses this for some handlers for panic/trap/exceptions.
///

use core::panic::PanicInfo;
use heapless::String;
use litex_pac as pac;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;

litex_hal::uart! {
    Uart: pac::UART,
}

static mut UART_WRITER: Option<Uart> = None;

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
    info!("exception_handler: TrapFrame from {:x}", trap_frame.ra);
    loop {}
}

pub fn _logger_write(bytes: &[u8]) {
    unsafe {
        if let Some(writer) = &mut UART_WRITER {
            writer.bwrite_all(bytes).ok();
        }
    }
}

pub fn init(uart: pac::UART) {
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
        let mut s: String<128> = String::new();
        ufmt::uwrite!(&mut s, $($arg)*).ok();
        _logger_write(&s.into_bytes());
        _logger_write(b"\n");
    }};
}

pub(crate) use info;
