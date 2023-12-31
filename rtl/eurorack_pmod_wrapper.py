import os

from migen import *

from litex.soc.interconnect.csr import *

from litex.soc.cores.gpio import GPIOOut

SOURCES_ROOT = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "../deps/eurorack-pmod/gateware"
        )

class EurorackPmod(Module, AutoCSR):
    def __init__(self, platform, pads, w=16, drive_shared_pads=None, external_reset=None, sim=False):
        self.w = w
        self.cal_mem_file = os.path.join(SOURCES_ROOT, "cal/cal_mem.hex")
        self.codec_cfg_file = os.path.join(SOURCES_ROOT, "drivers/ak4619-cfg.hex")
        self.led_cfg_file = os.path.join(SOURCES_ROOT, "drivers/pca9635-cfg.hex")
        self.touch_cfg_file = os.path.join(SOURCES_ROOT, "drivers/touch-cfg.hex")

        # Exposed signals

        self.clk_256fs = ClockSignal("clk_256fs")
        self.clk_fs = ClockSignal("clk_fs")

        self.rst = Signal()

        self.cal_in0 = Signal((w, True))
        self.cal_in1 = Signal((w, True))
        self.cal_in2 = Signal((w, True))
        self.cal_in3 = Signal((w, True))
        # Routed to MUX
        self.cal_out0 = Signal((w, True))
        self.cal_out1 = Signal((w, True))
        self.cal_out2 = Signal((w, True))
        self.cal_out3 = Signal((w, True))
        # Routed to CODEC
        self.cal_out_int0 = Signal((w, True))
        self.cal_out_int1 = Signal((w, True))
        self.cal_out_int2 = Signal((w, True))
        self.cal_out_int3 = Signal((w, True))

        self.eeprom_mfg = Signal(8)
        self.eeprom_dev = Signal(8)
        self.eeprom_serial = Signal(32)
        self.jack = Signal(8)

        # LED modes & values
        # for led_mode, for each bit 1==auto (audio), 0==manual
        self.led_mode = Signal(8)
        self.led0 = Signal((8, True))
        self.led1 = Signal((8, True))
        self.led2 = Signal((8, True))
        self.led3 = Signal((8, True))
        self.led4 = Signal((8, True))
        self.led5 = Signal((8, True))
        self.led6 = Signal((8, True))
        self.led7 = Signal((8, True))

        # Touchsense inputs (R3.3+)
        self.touch0 = Signal(8)
        self.touch1 = Signal(8)
        self.touch2 = Signal(8)
        self.touch3 = Signal(8)
        self.touch4 = Signal(8)
        self.touch5 = Signal(8)
        self.touch6 = Signal(8)
        self.touch7 = Signal(8)

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

        # Possibly shared signals
        if drive_shared_pads is not None:
            self.pdn = drive_shared_pads.pdn
            self.lrck = drive_shared_pads.lrck
            self.bick = drive_shared_pads.bick
            self.mclk = drive_shared_pads.mclk
        else:
            # Assume these are driven by another EurorackPmod instance
            self.pdn = Signal()
            self.lrck = Signal()
            self.bick = Signal()
            self.mclk = Signal()
            assert external_reset is not None
            self.rst = external_reset

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
            p_TOUCH_CFG_FILE = self.touch_cfg_file,

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

            # Possibly shared
            o_pdn = self.pdn,
            o_mclk = self.mclk,
            o_lrck = self.lrck,
            o_bick = self.bick,

            # Not shared
            o_sdin1 = pads.sdin1,
            i_sdout1 = pads.sdout1,

            # Ports (clock at clk_fs)
            o_cal_in0 = self.cal_in0,
            o_cal_in1 = self.cal_in1,
            o_cal_in2 = self.cal_in2,
            o_cal_in3 = self.cal_in3,
            i_cal_out0 = self.cal_out_int0,
            i_cal_out1 = self.cal_out_int1,
            i_cal_out2 = self.cal_out_int2,
            i_cal_out3 = self.cal_out_int3,

            # Ports (serialized data fetched over I2C)
            o_eeprom_mfg = self.eeprom_mfg,
            o_eeprom_dev = self.eeprom_dev,
            o_eeprom_serial = self.eeprom_serial,
            o_jack = self.jack,
            o_touch0 = self.touch0,
            o_touch1 = self.touch1,
            o_touch2 = self.touch2,
            o_touch3 = self.touch3,
            o_touch4 = self.touch4,
            o_touch5 = self.touch5,
            o_touch6 = self.touch6,
            o_touch7 = self.touch7,

            # LED signals
            i_led_mode = self.led_mode,
            i_led0 = self.led0,
            i_led1 = self.led1,
            i_led2 = self.led2,
            i_led3 = self.led3,
            i_led4 = self.led4,
            i_led5 = self.led5,
            i_led6 = self.led6,
            i_led7 = self.led7,

            # Debug ports
            o_sample_adc0 = self.sample_adc0,
            o_sample_adc1 = self.sample_adc1,
            o_sample_adc2 = self.sample_adc2,
            o_sample_adc3 = self.sample_adc3,
            i_force_dac_output = self.force_dac_output,
        )


        # FIXME: For now these tristate implementations are ECP5 specific.
        if not sim:
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
            self.comb += [
                pads.i2c_sda.eq(~self.i2c_sda_oe),
                pads.i2c_scl.eq(~self.i2c_scl_oe),
            ]

        # Exposed CSRs

        self.csr_cal_in0 = CSRStatus(16)
        self.csr_cal_in1 = CSRStatus(16)
        self.csr_cal_in2 = CSRStatus(16)
        self.csr_cal_in3 = CSRStatus(16)

        # 1 == manual CSR control, 0 == audio from cal_outX signals.
        self.csr_out_mode = CSRStorage(4, reset=0x0)
        self.csr_cal_out0 = CSRStorage(16)
        self.csr_cal_out1 = CSRStorage(16)
        self.csr_cal_out2 = CSRStorage(16)
        self.csr_cal_out3 = CSRStorage(16)

        self.csr_eeprom_mfg = CSRStatus(8)
        self.csr_eeprom_dev = CSRStatus(8)
        self.csr_eeprom_serial = CSRStatus(32)
        self.csr_jack = CSRStatus(8)

        self.csr_touch0 = CSRStatus(8)
        self.csr_touch1 = CSRStatus(8)
        self.csr_touch2 = CSRStatus(8)
        self.csr_touch3 = CSRStatus(8)
        self.csr_touch4 = CSRStatus(8)
        self.csr_touch5 = CSRStatus(8)
        self.csr_touch6 = CSRStatus(8)
        self.csr_touch7 = CSRStatus(8)

        # default LED mode to auto
        self.csr_led_mode = CSRStorage(8, reset=0xFF)
        self.csr_led0 = CSRStorage(8)
        self.csr_led1 = CSRStorage(8)
        self.csr_led2 = CSRStorage(8)
        self.csr_led3 = CSRStorage(8)
        self.csr_led4 = CSRStorage(8)
        self.csr_led5 = CSRStorage(8)
        self.csr_led6 = CSRStorage(8)
        self.csr_led7 = CSRStorage(8)

        # Connect CSRs directly to inputs and outputs

        if external_reset is None:
            self.csr_reset = CSRStorage(1, reset=0)
            self.comb += [
                    self.rst.eq(self.csr_reset.storage),
            ]

        self.comb += [
                self.csr_cal_in0.status.eq(self.cal_in0),
                self.csr_cal_in1.status.eq(self.cal_in1),
                self.csr_cal_in2.status.eq(self.cal_in2),
                self.csr_cal_in3.status.eq(self.cal_in3),
                self.csr_eeprom_mfg.status.eq(self.eeprom_mfg),
                self.csr_eeprom_dev.status.eq(self.eeprom_dev),
                self.csr_eeprom_serial.status.eq(self.eeprom_serial),
                self.csr_jack.status.eq(self.jack),

                self.csr_touch0.status.eq(self.touch0),
                self.csr_touch1.status.eq(self.touch1),
                self.csr_touch2.status.eq(self.touch2),
                self.csr_touch3.status.eq(self.touch3),
                self.csr_touch4.status.eq(self.touch4),
                self.csr_touch5.status.eq(self.touch5),
                self.csr_touch6.status.eq(self.touch6),
                self.csr_touch7.status.eq(self.touch7),

                self.led_mode.eq(self.csr_led_mode.storage),
                self.led0.eq(self.csr_led0.storage),
                self.led1.eq(self.csr_led1.storage),
                self.led2.eq(self.csr_led2.storage),
                self.led3.eq(self.csr_led3.storage),
                self.led4.eq(self.csr_led4.storage),
                self.led5.eq(self.csr_led5.storage),
                self.led6.eq(self.csr_led6.storage),
                self.led7.eq(self.csr_led7.storage),

                If(self.csr_out_mode.storage[0],
                    self.cal_out_int0.eq(self.csr_cal_out0.storage)
                ).Else(
                    self.cal_out_int0.eq(self.cal_out0)
                ),

                If(self.csr_out_mode.storage[1],
                    self.cal_out_int1.eq(self.csr_cal_out1.storage)
                ).Else(
                    self.cal_out_int1.eq(self.cal_out1)
                ),

                If(self.csr_out_mode.storage[2],
                    self.cal_out_int2.eq(self.csr_cal_out2.storage)
                ).Else(
                    self.cal_out_int2.eq(self.cal_out2)
                ),

                If(self.csr_out_mode.storage[3],
                    self.cal_out_int3.eq(self.csr_cal_out3.storage)
                ).Else(
                    self.cal_out_int3.eq(self.cal_out3)
                ),
        ]
