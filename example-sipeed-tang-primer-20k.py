#!/usr/bin/env python3

# This variable defines all the external programs that this module
# relies on.  lxbuildenv reads this variable in order to ensure
# the build will finish without exiting due to missing third-party
# programs.
LX_DEPENDENCIES = ["riscv", "nextpnr-ecp5", "yosys"]

# Import lxbuildenv to integrate the deps/ directory
import lxbuildenv

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.build.generic_platform import *

from litex.soc.cores.clock import *
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart import *
from litex.soc.interconnect.csr import *

from litex_boards.platforms import sipeed_tang_primer_20k

from eurorack_pmod_migen.core import *
from eurorack_pmod_migen.blocks import *


_io_eurorack_pmod = [
    ("eurorack_pmod", 0,
        Subsignal("mclk",    Pins("F10")),
        Subsignal("pdn",     Pins("D10")),
        Subsignal("i2c_sda", Pins("R7")),
        Subsignal("i2c_scl", Pins("M6")),
        Subsignal("sdin1",   Pins("L8")),
        Subsignal("sdout1",  Pins("P7")),
        Subsignal("lrck",    Pins("E10")),
        Subsignal("bick",    Pins("D11")),
        IOStandard("LVCMOS33")
    ),
]


class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst        = Signal()
        self.cd_sys     = ClockDomain()
        self.cd_por     = ClockDomain()
        self.cd_init    = ClockDomain()
        self.cd_sys2x   = ClockDomain()
        self.cd_sys2x_i = ClockDomain()

        # # #

        self.stop  = Signal()
        self.reset = Signal()

        # Clk
        clk27 = platform.request("clk27")

        # Power on reset (the onboard POR is not aware of reprogramming)
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += self.cd_por.clk.eq(clk27)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # PLL
        self.pll = pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        self.comb += pll.reset.eq(~por_done)
        pll.register_clkin(clk27, 27e6)
        pll.create_clkout(self.cd_sys2x_i, 2*sys_clk_freq)
        self.specials += [
            Instance("DHCEN",
                i_CLKIN  = self.cd_sys2x_i.clk,
                i_CE     = self.stop,
                o_CLKOUT = self.cd_sys2x.clk),
            Instance("CLKDIV",
                p_DIV_MODE = "2",
                i_CALIB    = 0,
                i_HCLKIN   = self.cd_sys2x.clk,
                i_RESETN   = ~self.reset,
                o_CLKOUT   = self.cd_sys.clk),
            AsyncResetSynchronizer(self.cd_sys, ~pll.locked | self.reset),
        ]

        # Init clock domain
        self.comb += self.cd_init.clk.eq(clk27)
        self.comb += self.cd_init.rst.eq(pll.reset)


class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=48e6,
        with_spi_flash      = False,
        dock                = "lite",
        **kwargs):

        platform = sipeed_tang_primer_20k.Platform(dock, toolchain="gowin")

        self.crg = _CRG(platform, sys_clk_freq)

        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Tang Primer 20K", **kwargs)

        if with_spi_flash:
            from litespi.modules import W25Q32JV as SpiFlashModule
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            self.add_spi_flash(mode="1x", module=SpiFlashModule(Codes.READ_1_1_1))

    def add_eurorack_pmod(self):
        eurorack_pmod_pads = self.platform.request("eurorack_pmod")
        eurorack_pmod = EurorackPmod(self.platform, eurorack_pmod_pads)
        # Pipe inputs straight to outputs.
        self.comb += [
            eurorack_pmod.cal_out0.eq(eurorack_pmod.cal_in0),
            eurorack_pmod.cal_out1.eq(eurorack_pmod.cal_in1),
            eurorack_pmod.cal_out2.eq(eurorack_pmod.cal_in2),
            eurorack_pmod.cal_out3.eq(eurorack_pmod.cal_in3),
        ]
        self.add_module("eurorack_pmod0", eurorack_pmod)



def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_primer_20k.Platform, description="LiteX SoC on Tang Primer 20K.")
    parser.add_target_argument("--dock",         default="lite",       help="Dock version (standard (default) or lite.")
    parser.add_target_argument("--flash",        action="store_true",      help="Flash Bitstream.")
    parser.add_target_argument("--sys-clk-freq", default=12e6, type=float, help="System clock frequency.")
    sdopts = parser.target_group.add_mutually_exclusive_group()
    args = parser.parse_args()

    # TODO: `eurorack-pmod` gateware currently assumes 12MHz system clock, this
    # should be automatically derived from its own PLL at some point..
    assert(args.sys_clk_freq == 12e6)

    soc = BaseSoC(
        sys_clk_freq        = args.sys_clk_freq,
        dock                = args.dock,
        **parser.soc_argdict
    )

    soc.platform.add_extension(_io_eurorack_pmod)
    soc.add_eurorack_pmod()

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash", ext=".fs"), external=True)

if __name__ == "__main__":
    main()
