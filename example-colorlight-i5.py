#!/usr/bin/env python3

# This variable defines all the external programs that this module
# relies on.  lxbuildenv reads this variable in order to ensure
# the build will finish without exiting due to missing third-party
# programs.
LX_DEPENDENCIES = ["riscv", "nextpnr-ecp5", "yosys"]

# Import lxbuildenv to integrate the deps/ directory
import lxbuildenv

from litex.build.generic_platform import *
from litex.soc.cores.clock import *
from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.gpio import GPIOIn, GPIOOut
from litex.soc.cores.uart import UARTPHY, UART
from litex.soc.integration.builder import *

from litex_boards.targets.colorlight_i5 import *

from usbrtl.eptri import LunaEpTriWrapper

from rtl.eurorack_pmod_wrapper import *
from rtl.dsp_wrapper import *
from rtl.spi_dma import Wishbone2SPIDMA
from rtl.dma_router import *
from rtl.dsp import create_voices

_io_eurolut_proto1 = [
    ("eurorack_pmod_clk0", 0,
        # Global clock buffer through 74HC245PW,118
        Subsignal("mclk",    Pins("J16")),
        Subsignal("bick",    Pins("L5")),
        Subsignal("lrck",    Pins("M4")),
        Subsignal("pdn",     Pins("R3")),
        IOStandard("LVCMOS33")
    ),
    ("eurorack_pmod_aux0", 0,
        Subsignal("i2c_sda", Pins("L4")),
        Subsignal("i2c_scl", Pins("N4")),
        Subsignal("sdin1",   Pins("J18")),
        Subsignal("sdout1",  Pins("P16")),
        IOStandard("LVCMOS33")
    ),
    ("eurorack_pmod_aux1", 0,
        Subsignal("i2c_sda", Pins("K20")),
        Subsignal("i2c_scl", Pins("G20")),
        Subsignal("sdin1",   Pins("N18")),
        Subsignal("sdout1",  Pins("L20")),
        IOStandard("LVCMOS33")
    ),
    ("eurorack_pmod_aux2", 0,
        Subsignal("i2c_sda", Pins("L18")),
        Subsignal("i2c_scl", Pins("J20")),
        Subsignal("sdin1",   Pins("N17")),
        Subsignal("sdout1",  Pins("M18")),
        IOStandard("LVCMOS33")
    ),
    ("eurorack_pmod_aux3", 0,
        Subsignal("i2c_sda", Pins("D2")),
        Subsignal("i2c_scl", Pins("E2")),
        Subsignal("sdin1",   Pins("A3")),
        Subsignal("sdout1",  Pins("B1")),
        IOStandard("LVCMOS33")
    ),
    ("ulpi", 0,
        Subsignal("data",  Pins("D18 G5 F5 E5 D17 D16 E6 F4")),
        Subsignal("clk",   Pins("W1")),
        Subsignal("dir",   Pins("E16")),
        Subsignal("nxt",   Pins("E17")),
        Subsignal("stp",   Pins("R1")),
        Subsignal("rst",   Pins("U1")),
        IOStandard("LVCMOS33"),Misc("SLEWRATE=FAST")
    ),
    ("oled_spi", 0,
        Subsignal("clk",  Pins("Y2")),
        Subsignal("mosi", Pins("N2")),
        IOStandard("LVCMOS33"),
    ),
    ("oled_ctl", 0,
        Subsignal("dc",   Pins("T1")),
        Subsignal("resn", Pins("V1")),
        Subsignal("csn",  Pins("M1")),
        IOStandard("LVCMOS33"),
    ),
    ("uart_midi", 0,
        Subsignal("tx", Pins("B3")),
        Subsignal("rx", Pins("K5")),
        IOStandard("LVCMOS33")
    ),
    ("encoder", 0,
        Subsignal("a", Pins("K4"), Misc("PULLMODE=NONE")),
        Subsignal("b", Pins("B2"), Misc("PULLMODE=NONE")),
        Subsignal("sw_n", Pins("E19"), Misc("PULLMODE=NONE")),
        IOStandard("LVCMOS33")
    ),
    ("sdcard", 0,
        Subsignal("data", Pins("J4 H4 E1 E4"), Misc("PULLMODE=UP")),
        Subsignal("cmd",  Pins("F3"), Misc("PULLMODE=UP")),
        Subsignal("clk",  Pins("H3")),
        Subsignal("cd",   Pins("H5")),
        IOStandard("LVCMOS33"),
    ),
    # This is pin 27 of the B50612D (top ethernet PHY), which is pretty easy to
    # bodge over to the side of R106 closest to the ECP5 so we get multiboot.
    # I removed R86 so the PHY is disconnected.
    ("programn", 0, Pins("P1"), IOStandard("LVCMOS33")),
]

def add_audio_clocks(soc, sample_rate=46875):
    # Create 256*Fs clock domain
    soc.crg.cd_clk_256fs = ClockDomain()
    soc.crg.pll.create_clkout(soc.crg.cd_clk_256fs, sample_rate * 256)
    # Create 1*Fs clock domain using a division register
    soc.crg.cd_clk_fs = ClockDomain()
    clkdiv_fs = Signal(8)
    soc.sync.clk_256fs += clkdiv_fs.eq(clkdiv_fs+1)
    soc.comb += soc.crg.cd_clk_fs.clk.eq(clkdiv_fs[-1])

def into_mirror(soc, eurorack_pmod):
    # Pipe inputs straight to outputs.
    soc.comb += [
        eurorack_pmod.cal_out0.eq(eurorack_pmod.cal_in0),
        eurorack_pmod.cal_out1.eq(eurorack_pmod.cal_in1),
        eurorack_pmod.cal_out2.eq(eurorack_pmod.cal_in2),
        eurorack_pmod.cal_out3.eq(eurorack_pmod.cal_in3),
    ]

def into_shifter(soc, eurorack_pmod):
    N_VOICES = 8

    create_voices(soc, eurorack_pmod, n_voices=4, start=0, prefix="group0")
    create_voices(soc, eurorack_pmod, n_voices=4, start=4, prefix="group1")

    # Hacky 8 -> stereo
    soc.sync.clk_fs += [
            eurorack_pmod.cal_out2.eq((soc.group0cdc_vout0.source.out0 >> 2) +
                                      (soc.group0cdc_vout0.source.out2 >> 2) +
                                      (soc.group1cdc_vout0.source.out0 >> 2) +
                                      (soc.group1cdc_vout0.source.out2 >> 2)),
            eurorack_pmod.cal_out3.eq((soc.group0cdc_vout0.source.out1 >> 2) +
                                      (soc.group0cdc_vout0.source.out3 >> 2) +
                                      (soc.group1cdc_vout0.source.out1 >> 2) +
                                      (soc.group1cdc_vout0.source.out3 >> 2)),
    ]

    add_dma_router(soc, eurorack_pmod, output_capable=False)

def add_oled(soc):
    pads = soc.platform.request("oled_spi")
    pads.miso = Signal()
    spi_master = SPIMaster(pads, 8, sys_clk_freq=soc.sys_clk_freq, spi_clk_freq=10e6)
    soc.submodules.oled_spi = spi_master
    spi_master.add_clk_divider()
    soc.submodules.oled_ctl = GPIOOut(soc.platform.request("oled_ctl"))

    # SPI DMA for framebuffer dumping
    soc.submodules.spi_dma = Wishbone2SPIDMA()
    soc.bus.add_master(master=soc.spi_dma.bus)

class RotaryEncoder(Module, AutoCSR):
    def __init__(self, in_i, in_q):
        # two incoming bits for in-phase and quadrature (A and B) inputs
        self.in_i = in_i
        self.in_q = in_q
        # create storage for inputs
        self.iq_history = Array(Signal(2) for _ in range(2))
        # outgoing signals
        self.step = Signal(1)
        self.direction = Signal(1)
        self.csr_state = CSRStatus(8)

        self.comb += [
            # a step is only taken when either I or Q flip.
            # if none flip, no step is taken
            # if both flip, an error happend
            self.step.eq(self.iq_history[0][0] ^
                         self.iq_history[0][1] ^
                         self.iq_history[1][0] ^
                         self.iq_history[1][1]),
            # if the former value of I is the current value of Q, we move counter clockwise
            self.direction.eq(self.iq_history[1][0] ^ self.iq_history[0][1]),
        ]

        self.sync += [
            # store the current and former state
            self.iq_history[1].eq(self.iq_history[0]),
            self.iq_history[0].eq(Cat(self.in_i, self.in_q)),
        ]

        self.sync += [
            If(self.step,
               If(self.direction == 1,
                   self.csr_state.status.eq(self.csr_state.status + 1)
               ).Else(
                   self.csr_state.status.eq(self.csr_state.status - 1)
               )
            )
        ]

def add_encoder(soc):
    pads = soc.platform.request("encoder")
    # Assign the button to a GPIOIn
    encoder_s = Signal()
    soc.comb += encoder_s.eq(~pads.sw_n)
    soc.submodules.encoder_button = GPIOIn(encoder_s)
    # Create logic for decoding IQ rotation
    rotary_encoder = RotaryEncoder(pads.a, pads.b)
    soc.add_module("rotary_encoder", rotary_encoder)

def add_programn_gpio(soc):
    programp = Signal()
    soc.submodules.programn = GPIOOut(programp)
    soc.comb += soc.platform.request("programn").eq(~programp)

def add_usb(soc, base_addr=0xf0010000):
    soc.crg.cd_usb = ClockDomain()
    usb_por_done = Signal()
    usb_por_count = Signal(24, reset=int(60e6 * 10e-3))
    soc.comb += soc.crg.cd_usb.clk.eq(soc.crg.cd_sys.clk)
    soc.comb += usb_por_done.eq(usb_por_count == 0)
    soc.comb += soc.crg.cd_usb.rst.eq(~usb_por_done)
    soc.sync.sys += If(~usb_por_done, usb_por_count.eq(usb_por_count - 1))

    soc.submodules.usb = LunaEpTriWrapper(soc.platform, base_addr=base_addr)
    region = soc.add_memory_region("usb", base_addr, 0x10000, type="");
    soc.bus.add_slave("usb", soc.usb.bus, region)
    for name, irq in soc.usb.irqs.items():
        name = 'usb_{}'.format(name)
        class DummyIRQ(Module):
            def __init__(soc, irq):
                class DummyEV(Module):
                    def __init__(soc, irq):
                        soc.irq = irq
                soc.submodules.ev = DummyEV(irq)
        setattr(soc.submodules, name, DummyIRQ(irq))
        soc.irq.add(name=name)

def add_uart_midi(soc):
    # Extra UART
    uart_name = "uart_midi"
    uart_pads      = soc.platform.request(uart_name, loose=True)
    if uart_pads is None:
        raise ValueError(f"pads for '{uart_name}' does not exist in platform")
    uart_phy  = UARTPHY(uart_pads, clk_freq=soc.sys_clk_freq, baudrate=32500)
    uart      = UART(uart_phy, tx_fifo_depth=16, rx_fifo_depth=16)
    soc.add_module(name=f"{uart_name}_phy", module=uart_phy)
    soc.add_module(name=uart_name, module=uart)
    soc.irq.add(uart_name, use_loc_if_exists=True)

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=colorlight_i5.Platform, description="LiteX SoC on Colorlight I9.")
    parser.add_target_argument("--board",            default="i9",             help="Board type (i9).")
    parser.add_target_argument("--revision",         default="7.2",            help="Board revision (7.2).")
    parser.add_target_argument("--sys-clk-freq",     default=60e6, type=float, help="System clock frequency.")
    # This argument is 0x200000 + (address in flash of firmware image to boot from LiteX BIOS)
    parser.add_target_argument("--flash-boot",       default=0x3E0000, type=lambda x: int(x,0), help="Flash boot address.")
    viopts = parser.target_group.add_mutually_exclusive_group()
    args = parser.parse_args()
    print(args)

    soc = BaseSoC(board=args.board, revision=args.revision,
        toolchain              = args.toolchain,
        sys_clk_freq           = args.sys_clk_freq,
        with_led_chaser        = False,
        **parser.soc_argdict
    )

    soc.platform.add_extension(_io_eurolut_proto1)

    add_programn_gpio(soc)

    #add_usb(soc)
    """

    add_audio_clocks(soc)

    shared_pads = soc.platform.request("eurorack_pmod_clk0")

    pmod0_pads = soc.platform.request("eurorack_pmod_aux2")
    pmod0 = EurorackPmod(soc.platform, pmod0_pads, drive_shared_pads=shared_pads)
    soc.add_module("eurorack_pmod0", pmod0)

    into_shifter(soc, pmod0)

    pmod1_pads = soc.platform.request("eurorack_pmod_aux0")
    pmod1 = EurorackPmod(soc.platform, pmod1_pads, drive_shared_pads=None, external_reset=pmod0.rst)
    soc.add_module("eurorack_pmod1", pmod1)

    into_mirror(soc, pmod1)

    pmod2_pads = soc.platform.request("eurorack_pmod_aux1")
    pmod2 = EurorackPmod(soc.platform, pmod2_pads, drive_shared_pads=None, external_reset=pmod0.rst)
    soc.add_module("eurorack_pmod2", pmod2)

    into_mirror(soc, pmod2)

    pmod3_pads = soc.platform.request("eurorack_pmod_aux3")
    pmod3 = EurorackPmod(soc.platform, pmod3_pads, drive_shared_pads=None, external_reset=pmod0.rst)
    soc.add_module("eurorack_pmod3", pmod3)

    into_mirror(soc, pmod3)

    add_oled(soc)

    add_uart_midi(soc)

    add_encoder(soc)

    soc.add_constant("FLASH_BOOT_ADDRESS", args.flash_boot)

    """

    soc.add_sdcard()

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

if __name__ == "__main__":
    main()
