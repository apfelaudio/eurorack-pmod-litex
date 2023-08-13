import os

from migen import *

from litex.soc.interconnect.csr import *

SOURCES_ROOT = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "../deps/eurorack-pmod/gateware"
        )

class WavetableOscillator(Module, AutoCSR):
    def __init__(self, platform, w=16):
        self.w = w
        self.wavetable_path = os.path.join(SOURCES_ROOT, "cores/util/vco/wavetable.hex")
        self.wavetable_size = 256

        # Exposed signals

        self.rst = ResetSignal()
        self.sample_clk = ClockSignal("sample_clk")

        self.wavetable_inc = Signal(32)
        self.out = Signal((w, True))

        platform.add_verilog_include_path(SOURCES_ROOT)
        platform.add_sources(SOURCES_ROOT, "cores/util/wavetable_osc.sv")

        self.specials += Instance("wavetable_osc",

            # Parameters
            p_W = self.w,
            p_WAVETABLE_PATH = self.wavetable_path,
            p_WAVETABLE_SIZE = self.wavetable_size,

            # Ports (clk + reset)
            i_rst = self.rst,
            i_sample_clk = self.sample_clk,

            # Ports (filter parameters)
            i_wavetable_inc = self.wavetable_inc,
            o_out = self.out,
        )

        # Exposed CSRs

        self.csr_wavetable_inc = CSRStorage(32)

        self.comb += [
                self.wavetable_inc.eq(self.csr_wavetable_inc.storage),
        ]

class KarlsenLowPass(Module, AutoCSR):
    def __init__(self, platform, w=16):
        self.w = w

        # Exposed signals

        self.rst = ResetSignal()
        self.clk_12mhz = ClockSignal("sys")
        self.sample_clk = ClockSignal("sample_clk")

        self.g = Signal((w, True))
        self.resonance = Signal((w, True))
        self.sample_in = Signal((w, True))
        self.sample_out = Signal((w, True))

        # Verilog sources

        platform.add_verilog_include_path(SOURCES_ROOT)
        platform.add_sources(SOURCES_ROOT, "cores/util/filter/karlsen_lpf_pipelined.sv")

        self.specials += Instance("karlsen_lpf_pipelined",

            # Parameters
            p_W = self.w,

            # Ports (clk + reset)
            i_rst = self.rst,
            i_clk = self.clk_12mhz,
            i_sample_clk = self.sample_clk,

            # Ports (filter parameters)
            i_g = self.g,
            i_resonance = self.resonance,

            # Ports (audio in/out)
            i_sample_in = self.sample_in,
            o_sample_out = self.sample_out,
        )


        # Exposed CSRs

        self.csr_g = CSRStorage(16)
        self.csr_resonance = CSRStorage(16)

        self.comb += [
                self.g.eq(self.csr_g.storage),
                self.resonance.eq(self.csr_resonance.storage),
        ]

class DcBlock(Module):
    def __init__(self, platform, w=16):
        self.w = w

        # Exposed signals

        self.sample_clk = ClockSignal("sample_clk")

        self.sample_in = Signal((w, True))
        self.sample_out = Signal((w, True))

        # Verilog sources

        platform.add_verilog_include_path(SOURCES_ROOT)
        platform.add_sources(SOURCES_ROOT, "cores/util/dc_block.sv")

        self.specials += Instance("dc_block",

            # Parameters
            p_W = self.w,

            # Ports (clk + reset)
            i_sample_clk = self.sample_clk,

            # Ports (audio in/out)
            i_sample_in = self.sample_in,
            o_sample_out = self.sample_out,
        )
