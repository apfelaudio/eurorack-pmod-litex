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

def into_shifter(soc, eurorack_pmod):
    N_VOICES = 4
    create_voices(soc, eurorack_pmod, N_VOICES)

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

    soc = BaseSoC(board=args.board, revision=args.revision,
        toolchain              = args.toolchain,
        sys_clk_freq           = args.sys_clk_freq,
        with_led_chaser        = False,
        **parser.soc_argdict
    )

    soc.platform.add_extension(_io_eurolut_proto1)

    add_audio_clocks(soc)

    shared_pads = soc.platform.request("eurorack_pmod_clk0")
    pmod0_pads = soc.platform.request("eurorack_pmod_aux2")
    pmod0 = EurorackPmod(soc.platform, pmod0_pads, drive_shared_pads=shared_pads)
    soc.add_module("eurorack_pmod0", pmod0)

    into_shifter(soc, pmod0)

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

if __name__ == "__main__":
    main()
