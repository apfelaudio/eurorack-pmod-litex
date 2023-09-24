#![allow(clippy::empty_loop)]

use core::panic::PanicInfo;
use heapless::String;
use litex_pac as pac;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use core::arch::asm;
use crate::{BUF_IN, BUF_OUT, BUF_SZ_WORDS, BUF_SZ_SAMPLES};
use crate::*;
use core::sync::atomic::fence;
use core::sync::atomic::compiler_fence;
use core::sync::atomic::Ordering;
use crate::libvult;

litex_hal::uart! {
    Uart: pac::UART,
}

static mut UART_WRITER: Option<Uart> = None;

static mut VULT: Option<libvult::DspProcessType> = None;

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
    if (pending & (1u32 << pac::Interrupt::DMA_ROUTER0 as u32)) != 0 {
        let offset = peripherals.DMA_ROUTER0.offset_words.read().bits();

        unsafe {
            asm!("fence iorw, iorw");
        }

        unsafe {

            if let Some(ref mut vult) = VULT {

                if offset as usize == ((BUF_SZ_WORDS/2)+1) {
                    for i in 0..(BUF_SZ_SAMPLES/2) {
                        if i % 4 == 0 {
                            BUF_OUT[i] = vult.process(BUF_IN[i]);
                        } else {
                            BUF_OUT[i] = 0;
                        }
                    }
                }

                peripherals.TIMER0.uptime_latch.write(|w| w.bits(1));
                let trace_start = peripherals.TIMER0.uptime_cycles0.read().bits();

                if offset as usize == (BUF_SZ_WORDS-1) {
                    for i in (BUF_SZ_SAMPLES/2)..(BUF_SZ_SAMPLES) {
                        if i % 4 == 0 {
                            BUF_OUT[i] = vult.process(BUF_IN[i]);
                        } else {
                            BUF_OUT[i] = 0;
                        }
                    }
                }

                peripherals.TIMER0.uptime_latch.write(|w| w.bits(1));
                let trace_end = peripherals.TIMER0.uptime_cycles0.read().bits();
                let trace_diff = trace_end - trace_start;
                // cycles for all samples
                info!("{} {}", trace_diff, trace_diff >> 8);
            }

        }




        let pending_type = peripherals.DMA_ROUTER0.ev_pending.read().bits();
        unsafe {
            peripherals.DMA_ROUTER0.ev_pending.write(|w| w.bits(pending_type));
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
        VULT = Some(libvult::DspProcessType::new());
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
