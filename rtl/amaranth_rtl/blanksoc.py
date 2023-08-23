# This file is Copyright (c) 2021 Greg Davill <greg.davill@gmail.com>
# License: BSD
# 

import os
import datetime
import logging

from amaranth                  import Elaboratable, Module
from amaranth_soc              import wishbone

from luna.gateware.soc                       import SimpleSoC

class BlankSoC(SimpleSoC):
    """ Class used for building simple, example system-on-a-chip architectures.

    Thi class builds on the SimpleSoC but remaves the CPU, and IRQ controller. 
    Instead simply exposing a wishbone bus. However the bundeled classes for creating resource.h file are used

    """

    def __init__(self, clock_frequency=int(60e6)):
        """
        Parameters:
            clock_frequency -- The frequency of our `sync` domain, in MHz.
        """

        self.clk_freq = clock_frequency

        self._main_rom  = None
        self._main_ram  = None
        self._uart_baud = None

        # Keep track of our created peripherals and interrupts.
        self._submodules     = []
        self._irqs           = {}
        self._next_irq_index = 2

        # By default, don't attach any debug hardware; or build a BIOS.
        self._auto_debug = False
        self._build_bios = False

        # Create our bus decoder and set up our memory map.
        self.bus_decoder = wishbone.Decoder(addr_width=30, data_width=32, granularity=8, features={"cti", "bte"})

    def _emit_minerva_basics(self, emit):
        # These are LiteX headers adapt pending_irqs
        emit("#include <irq.h>")
        emit("#define pending_irqs(a) irq_pending(a)")     
        pass    

    def elaborate(self, platform):
        return Module()

