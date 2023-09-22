#![allow(clippy::empty_loop)]

use core::panic::PanicInfo;
use heapless::String;
use litex_pac as pac;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use core::arch::asm;

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
fn exception_handler(_trap_frame: &riscv_rt::TrapFrame) -> ! {
    _logger_write(b"exception_handler\n");
    loop {}
}

#[export_name = "DefaultHandler"]
fn default_handler() {
    _logger_write(b"irq\n");

    let mut pending: u32 = 0;
    unsafe {
    asm!(
        "csrr {x}, 0xFC0",
        x = inout(reg) pending,
        );
    }

    if (pending & (1u32 << pac::Interrupt::DMA_WRITER0 as u32)) != 0 {
        let peripherals = unsafe { pac::Peripherals::steal() };
        let pending_type = peripherals.DMA_WRITER0.ev_pending.read().bits();
        info!("dmaw0 {:x}", pending_type);
        unsafe {
            peripherals.DMA_WRITER0.ev_pending.write(|w| w.bits(pending_type));
        }
    } else {
        info!("unknown irq!!");
    }
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
