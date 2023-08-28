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
from litex.soc.cores.gpio import GPIOOut, GPIOIn
from litex.soc.integration.builder import *

from litex_boards.targets.colorlight_i5 import *

from rtl.eptri import LunaEpTriWrapper

from eurorack_pmod_migen.core import *
from eurorack_pmod_migen.blocks import *

_io_eurolut_proto1 = [
    ("eurorack_pmod_p3b", 0,
        Subsignal("mclk",    Pins("A3")),
        Subsignal("pdn",     Pins("B1")),
        Subsignal("i2c_sda", Pins("D2")),
        Subsignal("i2c_scl", Pins("E2")),
        Subsignal("sdin1",   Pins("D1")),
        Subsignal("sdout1",  Pins("C1")),
        Subsignal("lrck",    Pins("C2")),
        Subsignal("bick",    Pins("E3")),
        IOStandard("LVCMOS33")
    ),
    ("eurorack_pmod_p3a", 0,
        Subsignal("mclk",    Pins("D20")),
        Subsignal("pdn",     Pins("B19")),
        Subsignal("i2c_sda", Pins("A19")),
        Subsignal("i2c_scl", Pins("A18")),
        Subsignal("sdin1",   Pins("C17")),
        Subsignal("sdout1",  Pins("B18")),
        Subsignal("lrck",    Pins("B20")),
        Subsignal("bick",    Pins("F20")),
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
    ("programn", 0, Pins("L4"), IOStandard("LVCMOS33"), Misc("OPENDRAIN=ON")),
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

def add_eurorack_pmod(soc, pads, mod_name):
    # Instantiate a EurorackPmod.
    eurorack_pmod_pads = soc.platform.request(pads)
    eurorack_pmod = EurorackPmod(soc.platform, eurorack_pmod_pads)
    # Pipe inputs straight to outputs.
    soc.comb += [
        eurorack_pmod.cal_out0.eq(eurorack_pmod.cal_in0),
        eurorack_pmod.cal_out1.eq(eurorack_pmod.cal_in1),
        eurorack_pmod.cal_out2.eq(eurorack_pmod.cal_in2),
        eurorack_pmod.cal_out3.eq(eurorack_pmod.cal_in3),
    ]
    soc.add_module(mod_name, eurorack_pmod)

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


def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=colorlight_i5.Platform, description="LiteX SoC on Colorlight I5.")
    parser.add_target_argument("--board",            default="i5",             help="Board type (i5).")
    parser.add_target_argument("--revision",         default="7.0",            help="Board revision (7.0).")
    parser.add_target_argument("--sys-clk-freq",     default=60e6, type=float, help="System clock frequency.")
    viopts = parser.target_group.add_mutually_exclusive_group()
    args = parser.parse_args()

    soc = BaseSoC(board=args.board, revision=args.revision,
        toolchain              = args.toolchain,
        sys_clk_freq           = args.sys_clk_freq,
        **parser.soc_argdict
    )

    soc.platform.add_extension(_io_eurolut_proto1)

    add_programn_gpio(soc)

    #add_usb(soc)

    add_audio_clocks(soc)

    add_eurorack_pmod(soc, pads="eurorack_pmod_p3a", mod_name="eurorack_pmod0")
    add_eurorack_pmod(soc, pads="eurorack_pmod_p3b", mod_name="eurorack_pmod1")


    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

if __name__ == "__main__":
    main()
