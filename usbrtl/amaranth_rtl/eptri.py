#!/usr/bin/env python3
#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import sys
import logging
import os.path

from amaranth import Elaboratable, Module, ResetSignal, Signal
from amaranth.hdl.rec import Record
from amaranth_soc import wishbone

from luna.gateware.usb.usb2.device import USBDevice, USBDeviceController
from luna.gateware.architecture.car import PHYResetController
from luna.gateware.usb.usb2.interfaces.eptri import SetupFIFOInterface, InFIFOInterface, OutFIFOInterface

from amaranth.hdl.rec import Direction


from .blanksoc import BlankSoC



class LunaEpTri(Elaboratable):
    """ Simple SoC for hosting TinyUSB. """

    USB_CORE_ADDRESS = 0x0000_0000
    USB_SETUP_ADDRESS = 0x0000_1000
    USB_IN_ADDRESS = 0x0000_2000
    USB_OUT_ADDRESS = 0x0000_3000

    def __init__(self, base_addr=0):

        # Create a stand-in for our ULPI.
        self.ulpi = Record(
            [
                ('data', [('i', 8, Direction.FANIN),
                 ('o', 8, Direction.FANOUT), ('oe', 1, Direction.FANOUT)]),
                ('clk', [('o', 1, Direction.FANOUT)]),
                ('stp', 1, Direction.FANOUT),
                ('nxt', [('i', 1, Direction.FANIN)]),
                ('dir', [('i', 1, Direction.FANIN)]),
                ('rst', 1, Direction.FANOUT)
            ]
        )

        self.soc = soc = BlankSoC()

        self.usb_holdoff = Signal()
 
        # ... a core USB controller ...
        self.usb_device_controller = USBDeviceController()
        self.add_peripheral(self.usb_device_controller, addr=self.USB_CORE_ADDRESS + base_addr)

        # ... our eptri peripherals.
        self.usb_setup = SetupFIFOInterface()
        self.add_peripheral(self.usb_setup, addr=self.USB_SETUP_ADDRESS + base_addr)

        self.usb_in_ep = InFIFOInterface()
        self.add_peripheral(self.usb_in_ep, addr=self.USB_IN_ADDRESS + base_addr)

        self.usb_out_ep = OutFIFOInterface()
        self.add_peripheral(self.usb_out_ep, addr=self.USB_OUT_ADDRESS + base_addr)

        # Pulling out the bus, freezes the decoder, so this needs to be done at the end.
        self.bus = self.soc.bus_decoder.bus

    def add_peripheral(self, p, **kwargs):
        """ Adds a peripheral to the SoC.

        For now, this is identical to adding a peripheral to the SoC's wishbone bus.
        """

        # Add the peripheral to our bus...
        interface = getattr(p, 'bus')
        self.soc.bus_decoder.add(interface, **kwargs)

        # ... add its IRQs to top level signals...
        try:
            irq_line = getattr(p, 'irq')
            setattr(self, irq_line.name, irq_line)

            self.soc._irqs[self.soc._next_irq_index] = p
            self.soc._next_irq_index += 1
        except (AttributeError, NotImplementedError):

            # If the object has no associated IRQs, continue anyway.
            # This allows us to add devices with only Wishbone interfaces to our SoC.
            pass

    def elaborate(self, platform):
        m = Module()
        m.submodules.bus_decoder = self.soc.bus_decoder

        # Dummy submodule to remove warning.
        # Probably a better way ta handle this
        m.submodules.soc = self.soc

        # Create our USB device.
        m.submodules.usb_controller = self.usb_device_controller
        m.submodules.usb = usb = USBDevice(bus=self.ulpi)

        
        # Generate our domain clocks/resets.
        m.submodules.usb_reset = controller = PHYResetController(clock_frequency=60e6, reset_length=10e-3, stop_length=2e-4, power_on_reset=True)
        m.d.comb += [
            ResetSignal("usb")  .eq(controller.phy_reset),
            self.usb_holdoff    .eq(controller.phy_stop),
            controller.trigger  .eq(self.usb_device_controller.reset)
        ]


        m.d.comb += usb.full_speed_only.eq(0)

        # Connect up our device controller.
        m.d.comb += self.usb_device_controller.attach(usb)

        # Add our eptri endpoint handlers.
        usb.add_endpoint(self.usb_setup)
        usb.add_endpoint(self.usb_in_ep)
        usb.add_endpoint(self.usb_out_ep)

        return m
