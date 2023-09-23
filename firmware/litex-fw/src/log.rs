#![allow(clippy::empty_loop)]

use core::panic::PanicInfo;
use heapless::String;
use litex_pac as pac;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use core::arch::asm;
use crate::{BUF_IN, BUF_IN_CP};
use core::sync::atomic::fence;
use core::sync::atomic::compiler_fence;
use core::sync::atomic::Ordering;

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
    //_logger_write(b"irq\n");

    let mut pending: u32 = 0;
    unsafe {
    // VexRiscv // 0xFC0 == machinePendingsCsrId // GenCoreDefault.scala
    asm!(
        "csrr {x}, 0xFC0",
        x = inout(reg) pending,
        );
    }

    let peripherals = unsafe { pac::Peripherals::steal() };
    if (pending & (1u32 << pac::Interrupt::DMA_WRITER0 as u32)) != 0 {
        let offset = peripherals.DMA_WRITER0.offset.read().bits();
        //info!("dmaw0 {:x} {:x}", pending_type, offset);

        if offset == 0x10 {
            for i in 0..0x10 {
                unsafe {
                    BUF_IN_CP[i] = BUF_IN[i];
                }
            }
        }

        if offset == 0x1f {
            for i in 0x10..0x20 {
                unsafe {
                    BUF_IN_CP[i] = BUF_IN[i];
                }
            }
        }

        let pending_type = peripherals.DMA_WRITER0.ev_pending.read().bits();
        unsafe {
            peripherals.DMA_WRITER0.ev_pending.write(|w| w.bits(pending_type));
        }
    }

    if (pending & (1u32 << pac::Interrupt::DMA_READER0 as u32)) != 0 {
        let pending_type = peripherals.DMA_READER0.ev_pending.read().bits();
        let offset = peripherals.DMA_READER0.offset.read().bits();
        //info!("dmar0 {:x} {:x}", pending_type, offset);
        unsafe {
            peripherals.DMA_READER0.ev_pending.write(|w| w.bits(pending_type));
        }
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
