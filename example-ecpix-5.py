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
from litex.soc.cores.dma import *
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.stream import ClockDomainCrossing
from litex.soc.interconnect.csr_eventmanager import *

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

    # CDC
    cdc_in0 = ClockDomainCrossing(layout=[("data", 32)], cd_from="clk_fs", cd_to="sys")
    cdc_out0 = ClockDomainCrossing(layout=[("data", 32)], cd_from="sys", cd_to="clk_fs")

    # CDC <-> I2S (clk_fs domain)
    soc.comb += [
        # ADC -> CDC
        cdc_in0.sink.valid.eq(1),
        cdc_in0.sink.payload.data.eq(eurorack_pmod.cal_in0),
        # CDC -> DAC
        cdc_out0.source.ready.eq(1),
        eurorack_pmod.cal_out0.eq(cdc_out0.source.payload.data)
    ]

    # DMA master (ADC -> CDC -> Wishbone)
    soc.submodules.dma_writer0 = WishboneDMAWriter(wishbone.Interface(), endianness="big", with_csr=True)
    soc.bus.add_master(master=soc.dma_writer0.bus)
    soc.comb += cdc_in0.source.connect(soc.dma_writer0.sink)
    soc.irq.add("dma_writer0", use_loc_if_exists=True)

    # DMA master (Wishbone -> CDC -> DAC)
    soc.submodules.dma_reader0 = WishboneDMAReader(wishbone.Interface(), endianness="big", with_csr=True)
    soc.bus.add_master(master=soc.dma_reader0.bus)
    soc.comb += soc.dma_reader0.source.connect(cdc_out0.sink)
    soc.irq.add("dma_reader0", use_loc_if_exists=True)

    soc.add_module("eurorack_pmod0", eurorack_pmod)
    soc.add_module("cdc_in0", cdc_in0)
    soc.add_module("cdc_out0", cdc_out0)

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
