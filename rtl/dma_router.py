#!/bin/python3

from migen import *

from litex.soc.cores.dma import *
from litex.soc.interconnect.stream import ClockDomainCrossing
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr_eventmanager import *

class DMARouter(LiteXModule):

    def __init__(self, soc, output_capable):

        self.output_capable = output_capable

        self.sink   = stream.Endpoint([("in0", 16),
                                       ("in1", 16),
                                       ("in2", 16),
                                       ("in3", 16)])
        self.writer_bus0 = wishbone.Interface()
        self.submodules.dma_writer0 = WishboneDMAWriter(self.writer_bus0, endianness="big")

        if self.output_capable:
            self.source   = stream.Endpoint([("out0", 16),
                                             ("out1", 16),
                                             ("out2", 16),
                                             ("out3", 16)])
            self.reader_bus0 = wishbone.Interface()
            self.submodules.dma_reader0 = WishboneDMAReader(self.reader_bus0, endianness="big", fifo_depth=1)

    def add_csr(self):
        # CSR
        self._base_writer   = CSRStorage(32)
        if self.output_capable:
            self._base_reader   = CSRStorage(32)
        self._length_words  = CSRStorage(32, reset=0)
        self._offset_words  = CSRStatus(32)
        self._enable        = CSRStorage(reset=0)

        # Local signals
        shift         = log2_int(self.writer_bus0.data_width//8)
        base_writer   = Signal(self.writer_bus0.adr_width)
        if self.output_capable:
            base_reader   = Signal(self.writer_bus0.adr_width)
        offset_words  = Signal(self.writer_bus0.adr_width)
        length_words  = Signal(self.writer_bus0.adr_width)
        offset_words_r= Signal(self.writer_bus0.adr_width)

        self.comb += [
            base_writer.eq(self._base_writer.storage[shift:]),
            length_words.eq(self._length_words.storage),
            self._offset_words.status.eq(offset_words),
        ]

        if self.output_capable:
            self.comb += [
                base_reader.eq(self._base_reader.storage[shift:]),
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
        if self.output_capable:
            fsm_read = FSM(reset_state="IDLE")
            fsm_read = ResetInserter()(fsm_read)
            self.submodules += fsm_read
            self.comb += fsm_read.reset.eq(~self._enable.storage)

            fsm_read.act("IDLE",
                NextValue(offset_words_r, 0),
                NextState("EVEN"),
            )

            fsm_read.act("EVEN",
                NextValue(self.dma_reader0.sink.valid, 1),
                NextValue(self.dma_reader0.sink.address, base_reader + offset_words_r),
                If(self.dma_reader0.sink.ready,
                    NextValue(self.dma_reader0.sink.valid, 0),
                    NextValue(self.dma_reader0.source.ready, 1),
                    NextValue(offset_words_r, offset_words_r + 1),
                    NextState("WAIT1"),
                ),
            )

            fsm_read.act("WAIT1",
                If(self.dma_reader0.source.valid,
                    NextValue(self.dma_reader0.source.ready, 0),
                    NextValue(self.source.out0, self.dma_reader0.source.data[:16]),
                    NextValue(self.source.out1, self.dma_reader0.source.data[16:32]),
                    NextState("ODD"),
                )
             )

            fsm_read.act("ODD",
                NextValue(self.dma_reader0.sink.valid, 1),
                NextValue(self.dma_reader0.sink.address, base_reader + offset_words_r),
                If(self.dma_reader0.sink.ready,
                    NextValue(self.dma_reader0.sink.valid, 0),
                    NextValue(self.dma_reader0.source.ready, 1),
                    NextValue(offset_words_r, offset_words_r + 1),
                    NextState("WAIT2"),
                ),
            )


            fsm_read.act("WAIT2",
                If(self.dma_reader0.source.valid,
                    NextValue(self.source.valid, 1),
                    NextValue(self.dma_reader0.source.ready, 0),
                    NextValue(self.source.out2, self.dma_reader0.source.data[:16]),
                    NextValue(self.source.out3, self.dma_reader0.source.data[16:32]),
                    NextState("WAIT3"),
                )
             )

            fsm_read.act("WAIT3",
                If(self.source.ready,
                    NextValue(self.source.valid, 0),
                    If(offset_words_r == length_words,
                        NextValue(offset_words_r, 0)
                    ),
                    NextState("EVEN"),
                )
             )


def add_dma_router(soc, eurorack_pmod, output_capable):

    soc.submodules.dma_router0 = DMARouter(soc, output_capable=output_capable)
    soc.dma_router0.add_csr()

    # CDC
    cdc_in0 = ClockDomainCrossing(
            layout=[("in0", 16),
                    ("in1", 16),
                    ("in2", 16),
                    ("in3", 16)],
            cd_from="clk_fs",
            cd_to="sys"
    )
    soc.add_module("cdc_in0", cdc_in0)

    soc.comb += [
        # CDC <-> I2S (clk_fs domain)
        # ADC -> CDC
        cdc_in0.sink.valid.eq(1),
        cdc_in0.sink.in0.eq(eurorack_pmod.cal_in0),
        cdc_in0.sink.in1.eq(eurorack_pmod.cal_in1),
        cdc_in0.sink.in2.eq(eurorack_pmod.cal_in2),
        cdc_in0.sink.in3.eq(eurorack_pmod.cal_in3),

        # ADC -> CDC -> Router -> DRAM
        cdc_in0.source.connect(soc.dma_router0.sink),
    ]

    soc.bus.add_master(master=soc.dma_router0.dma_writer0.bus)

    if output_capable:
        cdc_out0 = ClockDomainCrossing(
                layout=[("out0", 16),
                        ("out1", 16),
                        ("out2", 16),
                        ("out3", 16)],
                cd_from="sys",
                cd_to="clk_fs"
        )
        soc.add_module("cdc_out0", cdc_out0)

        soc.comb += [
            # CDC <-> I2S (clk_fs domain)
            # CDC -> DAC
            cdc_out0.source.ready.eq(1),
            eurorack_pmod.cal_out0.eq(cdc_out0.source.out0),
            eurorack_pmod.cal_out1.eq(cdc_out0.source.out1),
            eurorack_pmod.cal_out2.eq(cdc_out0.source.out2),
            eurorack_pmod.cal_out3.eq(cdc_out0.source.out3),

            # DRAM -> Router -> CDC -> DAC
            soc.dma_router0.source.connect(cdc_out0.sink),
        ]

        soc.bus.add_master(master=soc.dma_router0.dma_reader0.bus)

    soc.irq.add("dma_router0", use_loc_if_exists=True)

