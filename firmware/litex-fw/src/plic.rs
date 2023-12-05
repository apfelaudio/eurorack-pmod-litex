/// Various structs / implementations required for us to use `riscv::peripheral::plic`.

use riscv;
use litex_pac as pac;

#[derive(Copy, Clone)]
pub struct VexInterrupt {
    pub pac_irq: pac::Interrupt,
}

unsafe impl riscv::peripheral::plic::InterruptNumber for VexInterrupt {

    // TODO: we should be able to fetch this from the PAC?
    const MAX_INTERRUPT_NUMBER: u16 = 32u16;

    fn number(self) -> u16 {
        self.pac_irq as u16
    }

    fn try_from(value: u16) -> Result<Self, u16> {
        if let Ok(value_u8) = value.try_into() {
            match pac::Interrupt::try_from(value_u8) {
                Ok(result) => Ok(VexInterrupt { pac_irq: result } ),
                Err(_) => Err(value)
            }
        } else {
            Err(value)
        }
    }
}

impl From<pac::Interrupt> for VexInterrupt {
    fn from(pac_irq: pac::Interrupt) -> Self {
        VexInterrupt { pac_irq }
    }
}

#[derive(Copy, Clone)]
pub struct VexPriority {
    prio: u8,
}

unsafe impl riscv::peripheral::plic::PriorityNumber for VexPriority {
    // TODO: This is the default VexRiscv max priority. Fetch from PAC?
    const MAX_PRIORITY_NUMBER: u8 = 0x3u8;
    fn number(self) -> u8 { self.prio }
    fn try_from(value: u8) -> Result<Self, u8> { Ok(VexPriority{prio: value}) }
}

impl From<u8> for VexPriority {
    fn from(prio: u8) -> Self {
        VexPriority { prio }
    }
}
