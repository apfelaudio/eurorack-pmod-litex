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

from litex_boards.targets.lambdaconcept_ecpix5 import *

from eurorack_pmod_migen.core import *
from eurorack_pmod_migen.blocks import *

_io_eurorack_pmod = [
    ("eurorack_pmod_p0", 0,
        Subsignal("mclk",    Pins("W26")),
        Subsignal("pdn",     Pins("V26")),
        Subsignal("i2c_sda", Pins("U26")),
        Subsignal("i2c_scl", Pins("T26")),
        Subsignal("sdin1",   Pins("T25")),
        Subsignal("sdout1",  Pins("U25")),
        Subsignal("lrck",    Pins("U24")),
        Subsignal("bick",    Pins("V24")),
        IOStandard("LVCMOS33")
    ),
]

def add_eurorack_pmod(soc, sample_rate=48000):
    soc.platform.add_extension(_io_eurorack_pmod)

    # Create 256*Fs clock domain
    soc.crg.cd_clk_256fs = ClockDomain()
    soc.crg.pll.create_clkout(soc.crg.cd_clk_256fs, sample_rate * 256)

    # Create 1*Fs clock domain using a division register
    soc.crg.cd_clk_fs = ClockDomain()
    clkdiv_fs = Signal(8)
    soc.sync.clk_256fs += clkdiv_fs.eq(clkdiv_fs+1)
    soc.comb += soc.crg.cd_clk_fs.clk.eq(clkdiv_fs[-1])

    # Now instantiate a EurorackPmod.
    eurorack_pmod_pads = soc.platform.request("eurorack_pmod_p0")
    eurorack_pmod = EurorackPmod(soc.platform, eurorack_pmod_pads)

    # Pipe inputs straight to outputs.
    soc.comb += [
        eurorack_pmod.cal_out0.eq(eurorack_pmod.cal_in0),
        eurorack_pmod.cal_out1.eq(eurorack_pmod.cal_in1),
        eurorack_pmod.cal_out2.eq(eurorack_pmod.cal_in2),
        eurorack_pmod.cal_out3.eq(eurorack_pmod.cal_in3),
    ]

    soc.add_module("eurorack_pmod0", eurorack_pmod)

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=lambdaconcept_ecpix5.Platform, description="LiteX SoC on ECPIX-5.")
    parser.add_target_argument("--device",          default="85F",            help="ECP5 device (45F or 85F).")
    parser.add_target_argument("--sys-clk-freq",    default=75e6, type=float, help="System clock frequency.")

    args = parser.parse_args()

    soc = BaseSoC(
        device                 = args.device,
        sys_clk_freq           = args.sys_clk_freq,
        toolchain              = args.toolchain,
        **parser.soc_argdict
    )

    add_eurorack_pmod(soc)

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

if __name__ == "__main__":
    main()
