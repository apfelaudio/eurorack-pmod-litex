use core::panic::PanicInfo;
use heapless::String;
use litex_pac as pac;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use ufmt;

litex_hal::uart! {
    Uart: pac::UART,
}

static mut UART_WRITER: Option<Uart> = None;

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    info!("PANIC");
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
        let mut s: String<256> = String::new();
        ufmt::uwrite!(&mut s, $($arg)*).ok();
        _logger_write(&s.into_bytes());
        _logger_write(b"\n");
    }};
}

pub(crate) use info;
