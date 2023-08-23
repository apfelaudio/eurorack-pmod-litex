# This file is Copyright (c) 2021 Greg Davill <greg.davill@gmail.com>
# License: BSD


from migen import *

from litex.soc.interconnect import wishbone

from litex.build.io import DDROutput
from rtl.amaranth_rtl.eptri import LunaEpTri


from litex.soc.interconnect.csr_eventmanager import *

class LunaEpTriWrapper(Module):

    def __init__(self, platform, base_addr=0):
        self.platform = platform
        
        ulpi_pads = platform.request('ulpi')
        ulpi_data = TSTriple(8)
        reset = Signal()
        if hasattr(ulpi_pads, "rst"):
            self.comb += ulpi_pads.rst.eq(~ResetSignal("usb"))

        self.specials += DDROutput(~reset ,0, ulpi_pads.clk, ClockSignal("usb"))
        self.specials += ulpi_data.get_tristate(ulpi_pads.data)
        
        self.wrapper("LunaEpTri", LunaEpTri(base_addr))

        self.params = dict(
            # Clock / Reset
            i_usb_clk   = ClockSignal("usb"),
            #o_usb_rst   = ResetSignal("usb"), # Driven internally by PHYResetController
            i_clk   = ClockSignal("sys"),
            i_rst   = ResetSignal("sys"),

            o_ulpi__data__o = ulpi_data.o,
            o_ulpi__data__oe = ulpi_data.oe,
            i_ulpi__data__i = ulpi_data.i,
            #o_ulpi__clk__o = clk, # Driven externally with DDROutput
            o_ulpi__stp = ulpi_pads.stp,
            i_ulpi__nxt__i = ulpi_pads.nxt,
            i_ulpi__dir__i = ulpi_pads.dir,
            o_ulpi__rst = reset,
        )

        self.bus = bus = wishbone.Interface()

        self.params.update( 
            i__bus__adr = bus.adr,
            i__bus__stb = bus.stb,
            i__bus__cyc = bus.cyc,
            i__bus__we = bus.we,
            i__bus__sel = bus.sel,
            i__bus__dat_w = bus.dat_w,
            o__bus__dat_r = bus.dat_r,
            o__bus__ack = bus.ack,
        )


        # Wire up IRQs 
        self.irqs = irqs = {}
        irqs['device_controller'] = Signal()
        irqs['setup'] = Signal()
        irqs['in_ep'] = Signal()
        irqs['out_ep'] = Signal()

        self.params.update( 
            o_usb_device_controller_ev_irq = irqs['device_controller'],
            o_usb_setup_ev_irq = irqs['setup'],
            o_usb_in_ep_ev_irq = irqs['in_ep'],
            o_usb_out_ep_ev_irq = irqs['out_ep'],
        )


        self.specials += Instance("LunaEpTri",
            **self.params
        )

    def finalize(self):
        import os
        verilog_file = f"wrapper_{self.verilog_name}.v"
    
        vdir = os.path.join(os.getcwd(), "build", self.platform.name, "gateware")
        os.makedirs(vdir, exist_ok=True)

        verilog_file = os.path.join(vdir, verilog_file)
        with open(verilog_file, "w") as f:
            f.write(self.verilog)

        self.platform.add_source(verilog_file)

        # Create resource.h file
        #elaboratable.soc.log_resources()
        resource_file = f"luna_usb.h"
    
        vdir = os.path.join(os.getcwd(), "build", self.platform.name, "software", "include", "generated")
        os.makedirs(vdir, exist_ok=True)

        resource_file = os.path.join(vdir, resource_file)
        with open(resource_file, 'w') as f:
            self.amaranth_module.soc.generate_c_header(macro_name="LUNA_EPTRI", file=f, platform_name="LiteX Butterstick Bootloader")





    ## Wrapper 
    def wrapper(self, name, elaboratable):
        from amaranth import Record, Signal
        from amaranth.back import verilog

        ports = []

        # Patch through all Records/Ports
        for attr in dir(elaboratable):
            if not attr.startswith("_"):
                obj = getattr(elaboratable, attr)
                if isinstance(obj, (Signal, Record)):
                    ports += obj._lhs_signals()

        self.verilog = verilog.convert(elaboratable, name=name, ports=ports, strip_internal_attrs=False)
        self.verilog_name = name

        self.amaranth_module = elaboratable
        