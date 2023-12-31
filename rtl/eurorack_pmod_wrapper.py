import os

from migen import *

from litex.soc.interconnect.csr import *

from litex.soc.cores.gpio import GPIOOut

SOURCES_ROOT = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "../deps/eurorack-pmod/gateware"
        )

class EurorackPmod(Module, AutoCSR):
    def __init__(self, platform, pads, w=16, output_csr_read_only=True, sim=False):
        self.w = w
        self.cal_mem_file = os.path.join(SOURCES_ROOT, "cal/cal_mem.hex")
        self.codec_cfg_file = os.path.join(SOURCES_ROOT, "drivers/ak4619-cfg.hex")
        self.led_cfg_file = os.path.join(SOURCES_ROOT, "drivers/pca9635-cfg.hex")

        # Exposed signals

        self.clk_256fs = ClockSignal("clk_256fs")
        self.clk_fs = ClockSignal("clk_fs")

        self.rst = ResetSignal("sys")

        self.cal_in0 = Signal((w, True))
        self.cal_in1 = Signal((w, True))
        self.cal_in2 = Signal((w, True))
        self.cal_in3 = Signal((w, True))
        self.cal_out0 = Signal((w, True))
        self.cal_out1 = Signal((w, True))
        self.cal_out2 = Signal((w, True))
        self.cal_out3 = Signal((w, True))

        self.eeprom_mfg = Signal(8)
        self.eeprom_dev = Signal(8)
        self.eeprom_serial = Signal(32)
        self.jack = Signal(8)

        # Exposed (for debugging)

        self.sample_adc0 = Signal((w, True))
        self.sample_adc1 = Signal((w, True))
        self.sample_adc2 = Signal((w, True))
        self.sample_adc3 = Signal((w, True))

        self.force_dac_output = Signal((w, True))

        # Internal signals

        self.i2c_scl_oe = Signal()
        self.i2c_scl_i = Signal()
        self.i2c_sda_oe = Signal()
        self.i2c_sda_i = Signal()

        # Verilog sources

        platform.add_verilog_include_path(SOURCES_ROOT)
        platform.add_sources(SOURCES_ROOT, "eurorack_pmod.sv")
        platform.add_sources(SOURCES_ROOT, "drivers/pmod_i2c_master.sv")
        platform.add_sources(SOURCES_ROOT, "drivers/ak4619.sv")
        platform.add_sources(SOURCES_ROOT, "cal/cal.sv")
        platform.add_sources(SOURCES_ROOT, "external/no2misc/rtl/i2c_master.v")

        self.specials += Instance("eurorack_pmod",
            # Parameters
            p_W = self.w,
            p_CAL_MEM_FILE = self.cal_mem_file,
            p_CODEC_CFG_FILE = self.codec_cfg_file,
            p_LED_CFG_FILE = self.led_cfg_file,

            # Ports (clk + reset)
            i_clk_256fs = self.clk_256fs,
            i_clk_fs = self.clk_fs,
            i_rst = self.rst,

            # Pads (tristate, require different logic to hook these
            # up to pads depending on the target platform).
            o_i2c_scl_oe = self.i2c_scl_oe,
            i_i2c_scl_i = self.i2c_scl_i,
            o_i2c_sda_oe = self.i2c_sda_oe,
            i_i2c_sda_i = self.i2c_sda_i,

            # Pads (directly hooked up to pads without extra logic required)
            o_pdn = pads.pdn,
            o_mclk = pads.mclk,
            o_sdin1 = pads.sdin1,
            i_sdout1 = pads.sdout1,
            o_lrck = pads.lrck,
            o_bick = pads.bick,

            # Ports (clock at clk_fs)
            o_cal_in0 = self.cal_in0,
            o_cal_in1 = self.cal_in1,
            o_cal_in2 = self.cal_in2,
            o_cal_in3 = self.cal_in3,
            i_cal_out0 = self.cal_out0,
            i_cal_out1 = self.cal_out1,
            i_cal_out2 = self.cal_out2,
            i_cal_out3 = self.cal_out3,

            # Ports (serialized data fetched over I2C)
            o_eeprom_mfg = self.eeprom_mfg,
            o_eeprom_dev = self.eeprom_dev,
            o_eeprom_serial = self.eeprom_serial,
            o_jack = self.jack,

            # Debug ports
            o_sample_adc0 = self.sample_adc0,
            o_sample_adc1 = self.sample_adc1,
            o_sample_adc2 = self.sample_adc2,
            o_sample_adc3 = self.sample_adc3,
            i_force_dac_output = self.force_dac_output,
        )


        if not sim:
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
        else:
            # No need for special IO buffers if in simulation.
            self.comb += [
                pads.i2c_sda.eq(~self.i2c_sda_oe),
                pads.i2c_scl.eq(~self.i2c_scl_oe),
            ]

        # Exposed CSRs

        self.csr_cal_in0 = CSRStatus(16)
        self.csr_cal_in1 = CSRStatus(16)
        self.csr_cal_in2 = CSRStatus(16)
        self.csr_cal_in3 = CSRStatus(16)

        if output_csr_read_only:
            self.csr_cal_out0 = CSRStatus(16)
            self.csr_cal_out1 = CSRStatus(16)
            self.csr_cal_out2 = CSRStatus(16)
            self.csr_cal_out3 = CSRStatus(16)
        else:
            self.csr_cal_out0 = CSRStorage(16)
            self.csr_cal_out1 = CSRStorage(16)
            self.csr_cal_out2 = CSRStorage(16)
            self.csr_cal_out3 = CSRStorage(16)

        self.csr_eeprom_mfg = CSRStatus(8)
        self.csr_eeprom_dev = CSRStatus(8)
        self.csr_eeprom_serial = CSRStatus(32)
        self.csr_jack = CSRStatus(8)

        # Connect CSRs directly to inputs and outputs

        self.comb += [
                self.csr_cal_in0.status.eq(self.cal_in0),
                self.csr_cal_in1.status.eq(self.cal_in1),
                self.csr_cal_in2.status.eq(self.cal_in2),
                self.csr_cal_in3.status.eq(self.cal_in3),
                self.csr_eeprom_mfg.status.eq(self.eeprom_mfg),
                self.csr_eeprom_dev.status.eq(self.eeprom_dev),
                self.csr_eeprom_serial.status.eq(self.eeprom_serial),
                self.csr_jack.status.eq(self.jack)
        ]

        if output_csr_read_only:
            self.comb += [
                    self.csr_cal_out0.status.eq(self.cal_out0),
                    self.csr_cal_out1.status.eq(self.cal_out1),
                    self.csr_cal_out2.status.eq(self.cal_out2),
                    self.csr_cal_out3.status.eq(self.cal_out3),
            ]
        else:
            self.comb += [
                    self.cal_out0.eq(self.csr_cal_out0.storage),
                    self.cal_out1.eq(self.csr_cal_out1.storage),
                    self.cal_out2.eq(self.csr_cal_out2.storage),
                    self.cal_out3.eq(self.csr_cal_out3.storage),
            ]
