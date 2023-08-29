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
from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.gpio import GPIOOut
from litex.soc.cores.uart import UARTPHY, UART
from litex.soc.integration.builder import *

from litex_boards.targets.colorlight_i5 import *

from rtl.eptri import LunaEpTriWrapper

from eurorack_pmod_migen.core import *
from eurorack_pmod_migen.blocks import *

_io_eurolut_proto1 = [
    ("eurorack_pmod_p3b", 0,
        Subsignal("mclk",    Pins("A3")),
        Subsignal("pdn",     Pins("B1")),
        Subsignal("i2c_sda", Pins("D2")),
        Subsignal("i2c_scl", Pins("E2")),
        Subsignal("sdin1",   Pins("D1")),
        Subsignal("sdout1",  Pins("C1")),
        Subsignal("lrck",    Pins("C2")),
        Subsignal("bick",    Pins("E3")),
        IOStandard("LVCMOS33")
    ),
    ("eurorack_pmod_p3a", 0,
        Subsignal("mclk",    Pins("D20")),
        Subsignal("pdn",     Pins("B19")),
        Subsignal("i2c_sda", Pins("A19")),
        Subsignal("i2c_scl", Pins("A18")),
        Subsignal("sdin1",   Pins("C17")),
        Subsignal("sdout1",  Pins("B18")),
        Subsignal("lrck",    Pins("B20")),
        Subsignal("bick",    Pins("F20")),
        IOStandard("LVCMOS33")
    ),
    ("ulpi", 0,
        Subsignal("data",  Pins("D18 G5 F5 E5 D17 D16 E6 F4")),
        Subsignal("clk",   Pins("W1")),
        Subsignal("dir",   Pins("E16")),
        Subsignal("nxt",   Pins("E17")),
        Subsignal("stp",   Pins("R1")),
        Subsignal("rst",   Pins("U1")),
        IOStandard("LVCMOS33"),Misc("SLEWRATE=FAST")
    ),
    ("oled_spi", 0,
        Subsignal("clk",  Pins("Y2")),
        Subsignal("mosi", Pins("N2")),
        IOStandard("LVCMOS33"),
    ),
    ("oled_ctl", 0,
        Subsignal("dc",   Pins("T1")),
        Subsignal("resn", Pins("V1")),
        Subsignal("csn",  Pins("M1")),
        IOStandard("LVCMOS33"),
    ),
    ("uart_midi", 0,
        Subsignal("tx", Pins("B3")),
        Subsignal("rx", Pins("K5")),
        IOStandard("LVCMOS33")
    ),
    ("programn", 0, Pins("L4"), IOStandard("LVCMOS33"), Misc("OPENDRAIN=ON")),
]

def add_audio_clocks(soc, sample_rate=46875):
    # Create 256*Fs clock domain
    soc.crg.cd_clk_256fs = ClockDomain()
    soc.crg.pll.create_clkout(soc.crg.cd_clk_256fs, sample_rate * 256)
    # Create 1*Fs clock domain using a division register
    soc.crg.cd_clk_fs = ClockDomain()
    clkdiv_fs = Signal(8)
    soc.sync.clk_256fs += clkdiv_fs.eq(clkdiv_fs+1)
    soc.comb += soc.crg.cd_clk_fs.clk.eq(clkdiv_fs[-1])

def add_eurorack_pmod_mirror(soc, pads, mod_name):
    # Instantiate a EurorackPmod.
    eurorack_pmod_pads = soc.platform.request(pads)
    eurorack_pmod = EurorackPmod(soc.platform, eurorack_pmod_pads)
    # Pipe inputs straight to outputs.
    soc.comb += [
        eurorack_pmod.cal_out0.eq(eurorack_pmod.cal_in0),
        eurorack_pmod.cal_out1.eq(eurorack_pmod.cal_in1),
        eurorack_pmod.cal_out2.eq(eurorack_pmod.cal_in2),
        eurorack_pmod.cal_out3.eq(eurorack_pmod.cal_in3),
    ]
    soc.add_module(mod_name, eurorack_pmod)

def add_eurorack_pmod_wave(soc, pads, mod_name):
    # Instantiate a EurorackPmod.
    eurorack_pmod_pads = soc.platform.request(pads)
    eurorack_pmod = EurorackPmod(soc.platform, eurorack_pmod_pads)
    # Pipe inputs straight to outputs.
    soc.comb += [
        eurorack_pmod.cal_out0.eq(eurorack_pmod.cal_in0),
        eurorack_pmod.cal_out1.eq(eurorack_pmod.cal_in1),
        eurorack_pmod.cal_out2.eq(eurorack_pmod.cal_in2),
        eurorack_pmod.cal_out3.eq(eurorack_pmod.cal_in3),
    ]

    N_VOICES = 4

    for voice in range(N_VOICES):
        osc = WavetableOscillator(soc.platform)
        lpf = KarlsenLowPass(soc.platform)
        dc_block = DcBlock(soc.platform)

        soc.comb += [
            lpf.sample_in.eq(osc.out),
            dc_block.sample_in.eq(lpf.sample_out),
            getattr(eurorack_pmod, f"cal_out{voice}").eq(dc_block.sample_out),
        ]

        soc.add_module(f"wavetable_oscillator{voice}", osc)
        soc.add_module(f"karlsen_lpf{voice}", lpf)
        soc.add_module(f"dc_block{voice}", dc_block)

    soc.add_module(mod_name, eurorack_pmod)

def add_oled(soc):
    pads = soc.platform.request("oled_spi")
    pads.miso = Signal()
    spi_master = SPIMaster(pads, 8, sys_clk_freq=soc.sys_clk_freq, spi_clk_freq=10e6)
    soc.submodules.oled_spi = spi_master
    spi_master.add_clk_divider()
    soc.submodules.oled_ctl = GPIOOut(soc.platform.request("oled_ctl"))

def add_programn_gpio(soc):
    programp = Signal()
    soc.submodules.programn = GPIOOut(programp)
    soc.comb += soc.platform.request("programn").eq(~programp)

def add_usb(soc, base_addr=0xf0010000):
    soc.crg.cd_usb = ClockDomain()
    usb_por_done = Signal()
    usb_por_count = Signal(24, reset=int(60e6 * 10e-3))
    soc.comb += soc.crg.cd_usb.clk.eq(soc.crg.cd_sys.clk)
    soc.comb += usb_por_done.eq(usb_por_count == 0)
    soc.comb += soc.crg.cd_usb.rst.eq(~usb_por_done)
    soc.sync.sys += If(~usb_por_done, usb_por_count.eq(usb_por_count - 1))

    soc.submodules.usb = LunaEpTriWrapper(soc.platform, base_addr=base_addr)
    region = soc.add_memory_region("usb", base_addr, 0x10000, type="");
    soc.bus.add_slave("usb", soc.usb.bus, region)
    for name, irq in soc.usb.irqs.items():
        name = 'usb_{}'.format(name)
        class DummyIRQ(Module):
            def __init__(soc, irq):
                class DummyEV(Module):
                    def __init__(soc, irq):
                        soc.irq = irq
                soc.submodules.ev = DummyEV(irq)
        setattr(soc.submodules, name, DummyIRQ(irq))
        soc.irq.add(name=name)

def add_uart_midi(soc):
    # Extra UART
    uart_name = "uart_midi"
    uart_pads      = soc.platform.request(uart_name, loose=True)
    if uart_pads is None:
        raise ValueError(f"pads for '{uart_name}' does not exist in platform")
    uart_phy  = UARTPHY(uart_pads, clk_freq=soc.sys_clk_freq, baudrate=32500)
    uart      = UART(uart_phy, tx_fifo_depth=16, rx_fifo_depth=16)
    soc.add_module(name=f"{uart_name}_phy", module=uart_phy)
    soc.add_module(name=uart_name, module=uart)
    soc.irq.add(uart_name, use_loc_if_exists=True)

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=colorlight_i5.Platform, description="LiteX SoC on Colorlight I5.")
    parser.add_target_argument("--board",            default="i5",             help="Board type (i5).")
    parser.add_target_argument("--revision",         default="7.0",            help="Board revision (7.0).")
    parser.add_target_argument("--sys-clk-freq",     default=60e6, type=float, help="System clock frequency.")
    viopts = parser.target_group.add_mutually_exclusive_group()
    args = parser.parse_args()

    soc = BaseSoC(board=args.board, revision=args.revision,
        toolchain              = args.toolchain,
        sys_clk_freq           = args.sys_clk_freq,
        **parser.soc_argdict
    )

    soc.platform.add_extension(_io_eurolut_proto1)

    add_programn_gpio(soc)

    #add_usb(soc)

    add_audio_clocks(soc)

    add_eurorack_pmod_wave(soc, pads="eurorack_pmod_p3a", mod_name="eurorack_pmod0")
    add_eurorack_pmod_mirror(soc, pads="eurorack_pmod_p3b", mod_name="eurorack_pmod1")

    add_oled(soc)

    add_uart_midi(soc)

    """
    # Useful to double-check connectivity ...
    clkdiv_test = Signal(8)
    soc.sync.clk_fs += clkdiv_test.eq(clkdiv_test+1)
    oled = soc.platform.request("oled")
    soc.comb += [
        oled.clk.eq(clkdiv_test[-1]),
        oled.din.eq(clkdiv_test[-2]),
        oled.dc.eq(clkdiv_test[-3]),
        oled.res.eq(clkdiv_test[-4]),
        oled.cs.eq(clkdiv_test[-5]),
    ]
    """

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

if __name__ == "__main__":
    main()
