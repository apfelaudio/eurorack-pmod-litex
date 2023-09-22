#!/usr/bin/env python3

import lxbuildenv

from litex.tools.litex_sim import main

from litex.build.generic_platform import *
from litex.soc.cores.clock import *
from litex.soc.cores.dma import *
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.stream import ClockDomainCrossing
from litex.soc.interconnect.csr_eventmanager import *

from eurorack_pmod_migen.core import *
from eurorack_pmod_migen.blocks import *

CLK_FREQ_SYS = 5e6
CLK_FREQ_256FS = 1e6
CLK_FREQ_FS = CLK_FREQ_256FS / 256

_io_extra_clockers = [
    ("clocker_256fs", 0, Pins(1)),
]

_io_eurorack_pmod = [
    ("eurorack_pmod_p0", 0,
        Subsignal("mclk",    Pins(1)),
        Subsignal("pdn",     Pins(1)),
        Subsignal("i2c_sda", Pins(1)),
        Subsignal("i2c_scl", Pins(1)),
        Subsignal("sdin1",   Pins(1)),
        Subsignal("sdout1",  Pins(1)),
        Subsignal("lrck",    Pins(1)),
        Subsignal("bick",    Pins(1)),
    ),
]

def add_eurorack_pmod(soc):
    soc.platform.add_extension(_io_eurorack_pmod)

    # Create 1*Fs clock domain using a division register
    soc.cd_clk_fs = ClockDomain()
    clkdiv_fs = Signal(8)
    soc.sync.clk_256fs += clkdiv_fs.eq(clkdiv_fs+1)
    soc.comb += soc.cd_clk_fs.clk.eq(clkdiv_fs[-1])

    # Now instantiate a EurorackPmod.
    eurorack_pmod_pads = soc.platform.request("eurorack_pmod_p0")
    eurorack_pmod = EurorackPmod(soc.platform, eurorack_pmod_pads, sim=True)

    # Simulate all outputs looped back to inputs on the PMOD I2S
    soc.comb += eurorack_pmod_pads.sdout1.eq(eurorack_pmod_pads.sdin1)

    # CDC
    cdc_in0 = ClockDomainCrossing(layout=[("data", 32)], cd_from="clk_fs", cd_to="sys")
    cdc_out0 = ClockDomainCrossing(layout=[("data", 32)], cd_from="sys", cd_to="clk_fs")

    # CDC <-> I2S (clk_fs domain)
    soc.comb += [
        # ADC -> CDC
        cdc_in0.sink.valid.eq(1),
        cdc_in0.sink.payload.data.eq(0xDEADBEEF),
        #cdc_in0.sink.payload.data.eq(eurorack_pmod.cal_in0),
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

    soc.add_module("eurorack_pmod0", eurorack_pmod)
    soc.add_module("cdc_in0", cdc_in0)
    soc.add_module("cdc_out0", cdc_out0)


def sim_soc_extension(sim_config, soc):
    soc.platform.add_extension(_io_extra_clockers)
    sim_config.add_clocker("clocker_256fs", freq_hz=int(CLK_FREQ_256FS))
    soc.cd_clk_256fs = ClockDomain()
    soc.comb += [
        soc.cd_clk_256fs.clk.eq(soc.platform.request("clocker_256fs")),
    ]
    add_eurorack_pmod(soc)

if __name__ == "__main__":
    main(sys_clk_freq=CLK_FREQ_SYS, soc_extension_hook=sim_soc_extension)
