#!/usr/bin/env python3

import lxbuildenv

from litex.tools.litex_sim import main

from litex.build.generic_platform import *
from litex.soc.cores.clock import *

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
    eurorack_pmod = EurorackPmod(soc.platform, eurorack_pmod_pads, output_csr_read_only=False, sim=True)

    # Simulate all outputs looped back to inputs on the PMOD I2S
    soc.comb += eurorack_pmod_pads.sdout1.eq(eurorack_pmod_pads.sdin1)

    soc.add_module("eurorack_pmod0", eurorack_pmod)


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
