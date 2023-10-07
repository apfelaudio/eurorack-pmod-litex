#!/bin/python3

from migen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import wishbone

SPI_START  = ((8<<8) | (1<<0))
SPI_LENGTH = (1<<8)
SPI_DONE   = (1<<0)

class Wishbone2SPIDMA(Module, AutoCSR):
    def __init__(self):
        # Wishbone
        self.bus = bus = wishbone.Interface()

        # Control
        self.start = CSR()
        self.done  = CSRStatus()

        # Read parameters: base and length of DMA
        self.read_base   = CSRStorage(32)
        self.read_length = CSRStorage(32)

        # SPI parameters: address of control/status/mosi registers
        self.spi_control_reg_address = CSRStorage(32)
        self.spi_status_reg_address  = CSRStorage(32)
        self.spi_mosi_reg_address    = CSRStorage(32)

        # # #

        # Shorten CSR's names
        start = self.start.re
        done  = self.done.status

        read_base   = self.read_base.storage[2:]
        read_length = self.read_length.storage

        spi_mosi_reg_address    = self.spi_mosi_reg_address.storage[2:]
        spi_control_reg_address = self.spi_control_reg_address.storage[2:]
        spi_status_reg_address  = self.spi_status_reg_address.storage[2:]

        # internals
        word_offset = Signal(32)
        byte_offset = Signal(3)
        byte_count  = Signal(32)
        data        = Signal(32)

        # fsm
        self.submodules.fsm = fsm = FSM()
        fsm.act("IDLE",
            If(start,
                NextValue(word_offset, 0),
                NextValue(byte_offset, 0),
                NextValue(byte_count, 0),
                NextState("WISHBONE-READ-DATA")
            ).Else(
                done.eq(1),
            )
        )
        fsm.act("WISHBONE-READ-DATA",
            bus.stb.eq(1),
            bus.cyc.eq(1),
            bus.sel.eq(2**(bus.data_width//8)-1),
            bus.adr.eq(read_base + word_offset),
            If(bus.ack,
                NextValue(data, bus.dat_r),
                NextState("SPI-WRITE-DATA")
            )
        )
        fsm.act("SPI-WRITE-DATA",
            bus.stb.eq(1),
            bus.cyc.eq(1),
            bus.we.eq(1),
            bus.sel.eq(2**(bus.data_width//8)-1),
            bus.adr.eq(spi_mosi_reg_address),
            bus.dat_w.eq(data),
            If(bus.ack,
                NextState("SPI-WRITE-START")
            )
        )
        fsm.act("SPI-WRITE-START",
            bus.stb.eq(1),
            bus.cyc.eq(1),
            bus.we.eq(1),
            bus.sel.eq(2**(bus.data_width//8)-1),
            bus.adr.eq(spi_control_reg_address),
            bus.dat_w.eq(SPI_START),
            If(bus.ack,
                NextState("SPI-WAIT-DONE")
            )
        )
        fsm.act("SPI-WAIT-DONE",
            bus.stb.eq(1),
            bus.cyc.eq(1),
            bus.sel.eq(2**(bus.data_width//8)-1),
            bus.adr.eq(spi_status_reg_address),
            If(bus.ack,
                If(bus.dat_r & SPI_DONE,
                    NextValue(byte_count, byte_count + 1),
                    NextValue(byte_offset, byte_offset + 1),
                    NextState("SHIFT-BYTE")
                )
            )
        )
        fsm.act("SHIFT-BYTE",
            If(byte_count >= read_length,
                NextState("IDLE")
            ).Elif(byte_offset >= 4,
                NextValue(byte_offset, 0),
                NextState("INC-WORD-OFFSET")
            ).Else(
                NextValue(data, data >> 8),
                NextState("SPI-WRITE-DATA")
            )
        )
        fsm.act("INC-WORD-OFFSET",
            NextValue(word_offset, word_offset + 1),
            NextState("WISHBONE-READ-DATA")
        )
