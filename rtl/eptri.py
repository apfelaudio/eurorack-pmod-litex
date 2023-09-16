# This file is under the BSD license.
# Adapted from 'Butterstick Bootloader' project.
# Copyright (c) 2021 Greg Davill <greg.davill@gmail.com>
# Copyright (c) 2023 Seb Holzapfel <me@sebholzapfel.com>

import os
from migen import *
from litex.soc.interconnect import wishbone
from litex.build.io import DDROutput
from rtl.amaranth_rtl.eptri import LunaEpTri
from litex.soc.interconnect.csr_eventmanager import *

PLATFORM_NAME = "EuroLUT Proto1"

class LunaEpTriWrapper(Module):

    WISHBONE_I = ['adr', 'stb', 'cyc', 'we', 'sel', 'dat_w']
    WISHBONE_O = ['dat_r', 'ack']
    IRQS = ["device_controller", "setup", "in_ep", "out_ep"]

    def _connect_wishbone(self):
        self.bus = wishbone.Interface()
        self.params.update({
            f"i__bus__{attr}": getattr(self.bus, attr) for attr in self.WISHBONE_I
        })
        self.params.update({
            f"o__bus__{attr}": getattr(self.bus, attr) for attr in self.WISHBONE_O
        })

    def _connect_irqs(self):
        self.irqs = {
            name: Signal() for name in self.IRQS
        }
        self.params.update({
            f"o_usb_{name}_ev_irq": irq for name, irq in self.irqs.items()
        })

    def _connect_ulpi(self, ulpi_pads):
        # Connect ULPI to platform
        ulpi_data = TSTriple(8)
        reset = Signal()
        if hasattr(ulpi_pads, "rst"):
            self.comb += ulpi_pads.rst.eq(~ResetSignal("usb"))
        self.specials += [
            DDROutput(~reset, 0, ulpi_pads.clk, ClockSignal("usb")),
            ulpi_data.get_tristate(ulpi_pads.data)
        ]
        # Connect ULPI TO LUNA
        self.params = {
            "i_usb_clk": ClockSignal("usb"),
            # o_usb_rst driven internally by PHYResetController
            "i_clk": ClockSignal("sys"),
            "i_rst": ResetSignal("sys"),
            "o_ulpi__data__o": ulpi_data.o,
            "o_ulpi__data__oe": ulpi_data.oe,
            "i_ulpi__data__i": ulpi_data.i,
            # o_ulpi__clk__o driven externally with DDROutput
            "o_ulpi__stp": ulpi_pads.stp,
            "i_ulpi__nxt__i": ulpi_pads.nxt,
            "i_ulpi__dir__i": ulpi_pads.dir,
            "o_ulpi__rst": reset
        }

    def _generate_path(self, base_name, subpath, ext):
        dir_path = os.path.join(os.getcwd(), "build", self.platform.name, subpath.replace("/", os.path.sep))
        os.makedirs(dir_path, exist_ok=True)
        return os.path.join(dir_path, f"{base_name}.{ext}")

    def _generate_wrapper(self, name, elaboratable):
        from amaranth import Record, Signal
        from amaranth.back import verilog

        # Patch through all Records/Ports
        ports = []
        for attr in dir(elaboratable):
            if not attr.startswith("_"):
                obj = getattr(elaboratable, attr)
                if isinstance(obj, (Signal, Record)):
                    ports += obj._lhs_signals()

        self.verilog = verilog.convert(elaboratable, name=name, ports=ports, strip_internal_attrs=False)
        self.verilog_name = name
        self.amaranth_module = elaboratable

    def __init__(self, platform, base_addr=0):
        self.platform = platform
        self._generate_wrapper("LunaEpTri", LunaEpTri(base_addr))
        self._connect_ulpi(self.platform.request('ulpi'))
        self._connect_wishbone()
        self._connect_irqs()
        self.specials += Instance("LunaEpTri", **self.params)

    def finalize(self):
        verilog_file = self._generate_path("wrapper", "gateware", "v")
        with open(verilog_file, "w") as f:
            f.write(self.verilog)
        self.platform.add_source(verilog_file)
        resource_file = self._generate_path("luna_usb", "software/include/generated", "h")
        with open(resource_file, 'w') as f:
            self.amaranth_module.soc.generate_c_header(
                    macro_name="LUNA_EPTRI", file=f, platform_name=PLATFORM_NAME)
