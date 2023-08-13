#!/usr/bin/env python3

# This variable defines all the external programs that this module
# relies on.  lxbuildenv reads this variable in order to ensure
# the build will finish without exiting due to missing third-party
# programs.
LX_DEPENDENCIES = ["riscv", "nextpnr-ecp5", "yosys"]

# Import lxbuildenv to integrate the deps/ directory
import lxbuildenv

from migen import *

from litex.gen import *

from litex.build.io import DDROutput
from litex.build.generic_platform import *

from litex_boards.platforms import colorlight_i5

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart import *

from litex.soc.interconnect.csr import *

from litedram.modules import M12L64322A # Compatible with EM638325-6H.
from litedram.phy import GENSDRPHY, HalfRateGENSDRPHY

from eurorack_pmod_migen.core import *
from eurorack_pmod_migen.blocks import *


_io_eurorack_pmod = [
    ("uart_midi", 0,
        Subsignal("tx", Pins("T1")),
        Subsignal("rx", Pins("R1")),
        IOStandard("LVCMOS33")
    ),

    ("eurorack_pmod_p2b", 0,
        Subsignal("mclk",    Pins("N18")),
        Subsignal("pdn",     Pins("L20")),
        Subsignal("i2c_sda", Pins("K20")),
        Subsignal("i2c_scl", Pins("G20")),
        Subsignal("sdin1",   Pins("J20")),
        Subsignal("sdout1",  Pins("L18")),
        Subsignal("lrck",    Pins("M18")),
        Subsignal("bick",    Pins("N17")),
        IOStandard("LVCMOS33")
    ),
]


# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, use_internal_osc=False, sdram_rate="1:1"):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()
        if sdram_rate == "1:2":
            self.cd_sys2x    = ClockDomain()
            self.cd_sys2x_ps = ClockDomain()
        else:
            self.cd_sys_ps = ClockDomain()

        # # #

        # Clk / Rst
        if not use_internal_osc:
            clk = platform.request("clk25")
            clk_freq = 25e6
        else:
            clk = Signal()
            div = 5
            self.specials += Instance("OSCG",
                p_DIV = div,
                o_OSC = clk
            )
            clk_freq = 310e6/div

        self.rst_n = platform.request("cpu_reset_n")

        # PLL
        self.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(~self.rst_n | self.rst)
        pll.register_clkin(clk, clk_freq)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        if sdram_rate == "1:2":
            pll.create_clkout(self.cd_sys2x,    2*sys_clk_freq)
            pll.create_clkout(self.cd_sys2x_ps, 2*sys_clk_freq, phase=180) # Idealy 90° but needs to be increased.
        else:
           pll.create_clkout(self.cd_sys_ps, sys_clk_freq, phase=180) # Idealy 90° but needs to be increased.


        # SDRAM clock
        sdram_clk = ClockSignal("sys2x_ps" if sdram_rate == "1:2" else "sys_ps")
        self.specials += DDROutput(1, 0, platform.request("sdram_clock"), sdram_clk)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, board="i5", revision="7.0", toolchain="trellis", sys_clk_freq=60e6,
        use_internal_osc       = False,
        sdram_rate             = "1:1",
        **kwargs):
        board = board.lower()
        assert board in ["i5", "i9"]
        platform = colorlight_i5.Platform(board=board, revision=revision, toolchain=toolchain)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq,
            use_internal_osc = use_internal_osc,
            sdram_rate       = sdram_rate
        )

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, int(sys_clk_freq), ident = "LiteX SoC on Colorlight " + board.upper(), **kwargs)

        # SPI Flash --------------------------------------------------------------------------------
        if board == "i5":
            from litespi.modules import GD25Q16 as SpiFlashModule
        if board == "i9":
            from litespi.modules import W25Q64 as SpiFlashModule

        from litespi.opcodes import SpiNorFlashOpCodes as Codes
        self.add_spi_flash(mode="1x", module=SpiFlashModule(Codes.READ_1_1_1))

        # SDR SDRAM --------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            sdrphy_cls = HalfRateGENSDRPHY if sdram_rate == "1:2" else GENSDRPHY
            self.sdrphy = sdrphy_cls(platform.request("sdram"))
            self.add_sdram("sdram",
                phy           = self.sdrphy,
                module        = M12L64322A(sys_clk_freq, sdram_rate),
                l2_cache_size = kwargs.get("l2_size", 8192)
            )

    def add_uart_midi(self):

        # Extra UART
        uart_name = "uart_midi"
        uart_pads      = self.platform.request(uart_name, loose=True)
        if uart_pads is None:
            raise ValueError(f"pads for '{uart_name}' does not exist in platform")
        uart_phy  = UARTPHY(uart_pads, clk_freq=self.sys_clk_freq, baudrate=32500)
        uart      = UART(uart_phy, tx_fifo_depth=16, rx_fifo_depth=16)
        self.add_module(name=f"{uart_name}_phy", module=uart_phy)
        self.add_module(name=uart_name, module=uart)
        self.irq.add(uart_name, use_loc_if_exists=True)

    def add_eurorack_pmod(self):

        eurorack_pmod_pads = self.platform.request("eurorack_pmod_p2b")

        eurorack_pmod = EurorackPmod(self.platform, eurorack_pmod_pads)

        N_VOICES = 4

        for voice in range(N_VOICES):
            osc = WavetableOscillator(self.platform)
            lpf = KarlsenLowPass(self.platform)
            dc_block = DcBlock(self.platform)

            self.comb += [
                lpf.sample_in.eq(osc.out),
                dc_block.sample_in.eq(lpf.sample_out),
                getattr(eurorack_pmod, f"cal_out{voice}").eq(dc_block.sample_out),
            ]

            self.add_module(f"wavetable_oscillator{voice}", osc)
            self.add_module(f"karlsen_lpf{voice}", lpf)
            self.add_module(f"dc_block{voice}", dc_block)

        self.add_module("eurorack_pmod0", eurorack_pmod)


# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=colorlight_i5.Platform, description="LiteX SoC on Colorlight I5.")
    parser.add_target_argument("--board",            default="i5",             help="Board type (i5).")
    parser.add_target_argument("--revision",         default="7.0",            help="Board revision (7.0).")
    parser.add_target_argument("--sys-clk-freq",     default=12e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--use-internal-osc", action="store_true", help="Use internal oscillator.")
    parser.add_target_argument("--sdram-rate",       default="1:1",       help="SDRAM Rate (1:1 Full Rate or 1:2 Half Rate).")
    viopts = parser.target_group.add_mutually_exclusive_group()
    args = parser.parse_args()

    # TODO: `eurorack-pmod` gateware currently assumes 12MHz system clock, this
    # should be automatically derived from its own PLL at some point..
    assert(args.sys_clk_freq == 12e6)

    soc = BaseSoC(board=args.board, revision=args.revision,
        toolchain              = args.toolchain,
        sys_clk_freq           = args.sys_clk_freq,
        use_internal_osc       = args.use_internal_osc,
        sdram_rate             = args.sdram_rate,
        **parser.soc_argdict
    )
    soc.platform.add_extension(_io_eurorack_pmod)

    soc.add_uart_midi()
    soc.add_eurorack_pmod()

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
