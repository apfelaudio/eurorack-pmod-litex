import os

from migen import *

from litex.soc.interconnect.csr import *

ROOT = os.path.dirname(os.path.realpath(__file__))
PMOD_ROOT = os.path.join(ROOT, "../deps/eurorack-pmod/gateware")

class PCA9635Master(Module, AutoCSR):
    def __init__(self, platform, pads):

        # Configuration .hex

        self.led_cfg_file = os.path.join(PMOD_ROOT, "drivers/pca9635-cfg.hex")

        # Exposed signals

        self.clk_256fs = ClockSignal("clk_256fs")
        self.rst = Signal()

        self.csr_reset = CSRStorage(1)

        self.led0 = CSRStorage(8)
        self.led1 = CSRStorage(8)
        self.led2 = CSRStorage(8)
        self.led3 = CSRStorage(8)
        self.led4 = CSRStorage(8)
        self.led5 = CSRStorage(8)
        self.led6 = CSRStorage(8)
        self.led7 = CSRStorage(8)
        self.led8 = CSRStorage(8)
        self.led9 = CSRStorage(8)
        self.led10 = CSRStorage(8)
        self.led11 = CSRStorage(8)
        self.led12 = CSRStorage(8)
        self.led13 = CSRStorage(8)
        self.led14 = CSRStorage(8)
        self.led15 = CSRStorage(8)

        # Internal signals

        self.i2c_scl_oe = Signal()
        self.i2c_scl_i = Signal()
        self.i2c_sda_oe = Signal()
        self.i2c_sda_i = Signal()

        # Verilog sources

        platform.add_verilog_include_path(ROOT)
        platform.add_verilog_include_path(PMOD_ROOT)
        platform.add_sources(ROOT, "pca9635_master.sv")
        platform.add_sources(PMOD_ROOT, "external/no2misc/rtl/i2c_master.v")

        self.specials += Instance("pca9635_master",
            # Parameters
            p_LED_CFG = self.led_cfg_file,

            # Ports (clk + reset)
            i_clk = self.clk_256fs,
            i_rst = self.rst,

            # Pads (tristate, require different logic to hook these
            # up to pads depending on the target platform).
            o_scl_oe = self.i2c_scl_oe,
            i_scl_i = self.i2c_scl_i,
            o_sda_oe = self.i2c_sda_oe,
            i_sda_i = self.i2c_sda_i,

            i_led0 = self.led0.storage,
            i_led1 = self.led1.storage,
            i_led2 = self.led2.storage,
            i_led3 = self.led3.storage,
            i_led4 = self.led4.storage,
            i_led5 = self.led5.storage,
            i_led6 = self.led6.storage,
            i_led7 = self.led7.storage,
            i_led8 = self.led8.storage,
            i_led9 = self.led9.storage,
            i_led10 = self.led10.storage,
            i_led11 = self.led11.storage,
            i_led12 = self.led12.storage,
            i_led13 = self.led13.storage,
            i_led14 = self.led14.storage,
            i_led15 = self.led15.storage,
        )


        # FIXME: For now these tristate implementations are ECP5 specific.

        self.specials += Instance("TRELLIS_IO",
            p_DIR = "BIDIR",
            i_B   = pads.i2c_scl,
            i_I   = 0,
            o_O   = self.i2c_scl_i,
            i_T   = ~self.i2c_scl_oe
        )

        self.specials += Instance("TRELLIS_IO",
            p_DIR = "BIDIR",
            i_B   = pads.i2c_sda,
            i_I   = 0,
            o_O   = self.i2c_sda_i,
            i_T   = ~self.i2c_sda_oe
        )

        self.comb += [
                self.rst.eq(self.csr_reset.storage),
        ]
