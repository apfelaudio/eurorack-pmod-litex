#!/usr/bin/env python3

import lxbuildenv

from litex.tools.litex_sim import main

from litex.build.generic_platform import *
from litex.soc.cores.clock import *
from litex.soc.cores.dma import *
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.stream import ClockDomainCrossing
from litex.soc.interconnect.csr_eventmanager import *

from eurorack_pmod_migen.core import *
from eurorack_pmod_migen.blocks import *

CLK_FREQ_SYS = 5e6
CLK_FREQ_256FS = 1e6
CLK_FREQ_FS = CLK_FREQ_256FS / 256

_io_extra_clockers = [
    ("clocker_256fs", 0, Pins(1)),
]

_io_eurorack_pmod = [
    ("eurorack_pmod_p0", 0,
        Subsignal("mclk",    Pins(1)),
        Subsignal("pdn",     Pins(1)),
        Subsignal("i2c_sda", Pins(1)),
        Subsignal("i2c_scl", Pins(1)),
        Subsignal("sdin1",   Pins(1)),
        Subsignal("sdout1",  Pins(1)),
        Subsignal("lrck",    Pins(1)),
        Subsignal("bick",    Pins(1)),
    ),
]

class DMARouter(LiteXModule):

    def __init__(self, soc):

        self.sink   = stream.Endpoint([("in0", 16),
                                       ("in1", 16),
                                       ("in2", 16),
                                       ("in3", 16)])

        self.source   = stream.Endpoint([("out0", 16),
                                         ("out1", 16),
                                         ("out2", 16),
                                         ("out3", 16)])

        self.writer_bus0 = wishbone.Interface()
        self.reader_bus0 = wishbone.Interface()
        self.submodules.dma_writer0 = WishboneDMAWriter(self.writer_bus0, endianness="big")
        self.submodules.dma_reader0 = WishboneDMAReader(self.reader_bus0, endianness="big")

    def add_csr(self):
        # CSR
        self._base_writer   = CSRStorage(32)
        self._base_reader   = CSRStorage(32)
        self._length_words  = CSRStorage(32, reset=0)
        self._offset_words  = CSRStatus(32)
        self._enable        = CSRStorage(reset=0)

        # Local signals
        shift         = log2_int(self.writer_bus0.data_width//8)
        base_writer   = Signal(self.writer_bus0.adr_width)
        base_reader   = Signal(self.writer_bus0.adr_width)
        offset_words  = Signal(self.writer_bus0.adr_width)
        length_words  = Signal(self.writer_bus0.adr_width)
        offset_words_r= Signal(self.writer_bus0.adr_width)

        self.comb += [
            base_writer.eq(self._base_writer.storage[shift:]),
            base_reader.eq(self._base_writer.storage[shift:]),
            length_words.eq(self._length_words.storage),
            self._offset_words.status.eq(offset_words),
        ]

        # IRQ logic
        self.ev = EventManager()
        self.ev.half = EventSourceProcess(edge="rising")
        self.comb += [
            self.ev.half.trigger.eq(
                (offset_words == (length_words >> 1)) |
                (offset_words == (length_words - 2))),
        ]
        self.ev.finalize()

        # DMA FSM (write side)

        fsm_write = FSM(reset_state="IDLE")
        fsm_write = ResetInserter()(fsm_write)
        self.submodules += fsm_write
        self.comb += fsm_write.reset.eq(~self._enable.storage)

        fsm_write.act("IDLE",
            NextValue(offset_words, 0),
            NextState("EVEN"),
        )

        fsm_write.act("EVEN",
            self.dma_writer0.sink.valid.eq(self.sink.valid),
            self.dma_writer0.sink.address.eq(base_writer + offset_words),
            self.dma_writer0.sink.data.eq((self.sink.in1 << 16) | self.sink.in0),
            self.sink.ready.eq(self.dma_writer0.sink.ready),
            If(self.sink.valid & self.sink.ready,
                NextValue(offset_words, offset_words + 1),
                NextState("ODD"),
            )
        )

        fsm_write.act("ODD",
            self.dma_writer0.sink.valid.eq(self.sink.valid),
            self.dma_writer0.sink.address.eq(base_writer + offset_words),
            self.dma_writer0.sink.data.eq((self.sink.in3 << 16) | self.sink.in2),
            self.sink.ready.eq(0),
            If(self.sink.valid & self.dma_writer0.sink.ready,
                NextValue(offset_words, offset_words + 1),
                If((offset_words + 1) == length_words,
                    NextValue(offset_words, 0)
                ),
                NextState("EVEN"),
            )
        )

        # DMA FSM (read side)

        fsm_read = FSM(reset_state="IDLE")
        fsm_read = ResetInserter()(fsm_read)
        self.submodules += fsm_read
        self.comb += fsm_read.reset.eq(~self._enable.storage)

        fsm_read.act("IDLE",
            NextValue(offset_words_r, 0),
            NextState("EVEN"),
        )

        fsm_read.act("EVEN",
            self.source.valid.eq(0),
            self.dma_reader0.sink.valid.eq(1),
            self.dma_reader0.sink.address.eq(base_reader + offset_words_r),
            If(self.dma_reader0.sink.ready,
                NextValue(offset_words_r, offset_words_r + 1),
                NextState("ODD"),
            ),
        )

        fsm_read.act("ODD",
            self.dma_reader0.sink.valid.eq(1),
            self.dma_reader0.sink.address.eq(base_reader + offset_words_r),
            If(self.dma_reader0.sink.ready,
                NextValue(offset_words_r, offset_words_r + 1),
                If((offset_words_r + 1) == length_words,
                    NextValue(offset_words_r, 0)
                ),
                NextState("WAIT1"),
            ),
        )

        fsm_read.act("WAIT1",
            NextValue(self.dma_reader0.source.ready, 1),
            # Not sure about this...
            If(self.dma_reader0.source.valid,
                NextValue(self.source.out0, self.dma_reader0.source.data[:16]),
                NextValue(self.source.out1, self.dma_reader0.source.data[16:32]),
                NextState("WAIT2"),
            )
         )

        fsm_read.act("WAIT2",
            # Not sure about this...
            If(self.source.ready,
                self.source.valid.eq(1),
                NextValue(self.dma_reader0.source.ready, 0),
                NextValue(self.source.out2, self.dma_reader0.source.data[:16]),
                NextValue(self.source.out3, self.dma_reader0.source.data[16:32]),
                NextState("EVEN"),
            )
         )

def add_eurorack_pmod(soc):
    soc.platform.add_extension(_io_eurorack_pmod)

    # Create 1*Fs clock domain using a division register
    soc.cd_clk_fs = ClockDomain()
    clkdiv_fs = Signal(8)
    soc.sync.clk_256fs += clkdiv_fs.eq(clkdiv_fs+1)
    soc.comb += soc.cd_clk_fs.clk.eq(clkdiv_fs[-1])

    # Now instantiate a EurorackPmod.
    eurorack_pmod_pads = soc.platform.request("eurorack_pmod_p0")
    eurorack_pmod = EurorackPmod(soc.platform, eurorack_pmod_pads, sim=True)

    # Simulate all outputs looped back to inputs on the PMOD I2S
    soc.comb += eurorack_pmod_pads.sdout1.eq(eurorack_pmod_pads.sdin1)

    # CDC
    cdc_in0 = ClockDomainCrossing(
            layout=[("in0", 16),
                    ("in1", 16),
                    ("in2", 16),
                    ("in3", 16)],
            cd_from="clk_fs",
            cd_to="sys"
        )
    cdc_out0 = ClockDomainCrossing(
            layout=[("out0", 16),
                    ("out1", 16),
                    ("out2", 16),
                    ("out3", 16)],
            cd_from="sys",
            cd_to="clk_fs"
        )

    # CDC <-> I2S (clk_fs domain)
    soc.comb += [
        # ADC -> CDC
        cdc_in0.sink.valid.eq(1),
        cdc_in0.sink.in0.eq(0xDEAD),
        cdc_in0.sink.in1.eq(0xBEEF),
        cdc_in0.sink.in2.eq(0xFEED),
        cdc_in0.sink.in3.eq(0x8008),
        #cdc_in0.sink.data.eq(eurorack_pmod.cal_in0),
        # CDC -> DAC
        cdc_out0.source.ready.eq(1),
        eurorack_pmod.cal_out0.eq(cdc_out0.source.out0),
        eurorack_pmod.cal_out1.eq(cdc_out0.source.out1),
        eurorack_pmod.cal_out2.eq(cdc_out0.source.out2),
        eurorack_pmod.cal_out3.eq(cdc_out0.source.out3),
    ]

    soc.submodules.dma_router0 = DMARouter(soc)
    soc.dma_router0.add_csr()
    soc.comb += [
        soc.dma_router0.source.connect(cdc_out0.sink),
        cdc_in0.source.connect(soc.dma_router0.sink),
    ]
    soc.bus.add_master(master=soc.dma_router0.dma_writer0.bus)
    soc.bus.add_master(master=soc.dma_router0.dma_reader0.bus)
    soc.irq.add("dma_router0", use_loc_if_exists=True)

    soc.add_module("eurorack_pmod0", eurorack_pmod)
    soc.add_module("cdc_in0", cdc_in0)
    soc.add_module("cdc_out0", cdc_out0)


def sim_soc_extension(sim_config, soc):
    soc.platform.add_extension(_io_extra_clockers)
    sim_config.add_clocker("clocker_256fs", freq_hz=int(CLK_FREQ_256FS))
    soc.cd_clk_256fs = ClockDomain()
    soc.comb += [
        soc.cd_clk_256fs.clk.eq(soc.platform.request("clocker_256fs")),
    ]
    add_eurorack_pmod(soc)

if __name__ == "__main__":
    main(sys_clk_freq=CLK_FREQ_SYS, soc_extension_hook=sim_soc_extension)
