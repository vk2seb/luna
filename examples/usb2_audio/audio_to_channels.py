# Copyright (c) 2024 Seb Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD--3-Clause

from amaranth              import *
from amaranth.lib.fifo     import AsyncFIFO

class AudioToChannels(Elaboratable):

    """
    Domain crossing logic to move samples from `eurorack-pmod` logic in the audio domain
    to `channels_to_usb_stream` and `usb_stream_to_channels` logic in the USB domain.
    """

    def __init__(self, eurorack_pmod, to_usb_stream, from_usb_stream):

        self.to_usb = to_usb_stream
        self.from_usb = from_usb_stream
        self.eurorack_pmod = eurorack_pmod

    def elaborate(self, platform) -> Module:

        m = Module()

        eurorack_pmod = self.eurorack_pmod

        # Sample width used in underlying I2S driver.
        SW=eurorack_pmod.width

        # adc_fifo contains one entry per sample strobe in the audio domain, where each entry contains
        # a concatenated binary string of all input channels in one entry.

        m.submodules.adc_fifo = adc_fifo = AsyncFIFO(width=SW*4, depth=64, w_domain="audio", r_domain="usb")

        m.d.audio += [
            # FIXME: ignoring rdy in write domain. Should be fine as write domain
            # will always be slower than the read domain, but should be fixed.
            adc_fifo.w_en.eq(eurorack_pmod.fs_strobe),
            adc_fifo.w_data[    :SW*1].eq(eurorack_pmod.cal_in0),
            adc_fifo.w_data[SW*1:SW*2].eq(eurorack_pmod.cal_in1),
            adc_fifo.w_data[SW*2:SW*3].eq(eurorack_pmod.cal_in2),
            adc_fifo.w_data[SW*3:SW*4].eq(eurorack_pmod.cal_in3),
        ]

        # In the USB domain, unpack samples from the adc_fifo (one big concatenated
        # entry with samples for all channels once per sample strobe) and feed them
        # into ChannelsToUSBStream with one entry per channel, i.e 1 -> 4 entries
        # per sample strobe in the audio domain.

        # Storage for samples in the USB domain as we send them to the channel stream.
        adc_latched = Signal(SW*4)

        with m.FSM(domain="usb") as fsm:

            with m.State('WAIT'):
                m.d.usb += self.to_usb.valid.eq(0),
                with m.If(adc_fifo.r_rdy):
                    m.d.usb += adc_fifo.r_en.eq(1)
                    m.next = 'LATCH'

            with m.State('LATCH'):
                m.d.usb += [
                    adc_fifo.r_en.eq(0),
                    adc_latched.eq(adc_fifo.r_data)
                ]
                m.next = 'CH0'

            def generate_channel_states(channel, next_state_name):
                with m.State(f'CH{channel}'):
                    m.d.usb += [
                        # FIXME: currently filling bottom 8 bits with zeroes for 16bit -> 24bit
                        # sample conversion. Better to just switch native rate of I2S driver.
                        self.to_usb.payload.eq(
                            Cat(Const(0, 8), adc_latched[channel*SW:(channel+1)*SW])),
                        self.to_usb.channel_no.eq(channel),
                        self.to_usb.valid.eq(1),
                    ]
                    m.next = f'CH{channel}-SEND'
                with m.State(f'CH{channel}-SEND'):
                    with m.If(self.to_usb.ready):
                        m.d.usb += self.to_usb.valid.eq(0)
                        m.next = next_state_name

            generate_channel_states(0, 'CH1')
            generate_channel_states(1, 'CH2')
            generate_channel_states(2, 'CH3')
            generate_channel_states(3, 'WAIT')

        m.submodules.dac_fifo0 = dac_fifo0 = AsyncFIFO(width=16, depth=64, w_domain="usb", r_domain="audio")
        m.submodules.dac_fifo1 = dac_fifo1 = AsyncFIFO(width=16, depth=64, w_domain="usb", r_domain="audio")
        m.submodules.dac_fifo2 = dac_fifo2 = AsyncFIFO(width=16, depth=64, w_domain="usb", r_domain="audio")
        m.submodules.dac_fifo3 = dac_fifo3 = AsyncFIFO(width=16, depth=64, w_domain="usb", r_domain="audio")

        m.d.comb += [
            dac_fifo0.w_data.eq(self.from_usb.payload[8:]),
            dac_fifo0.w_en.eq((self.from_usb.channel_no == 0) &
                              self.from_usb.valid),
            dac_fifo1.w_data.eq(self.from_usb.payload[8:]),
            dac_fifo1.w_en.eq((self.from_usb.channel_no == 1) &
                              self.from_usb.valid),
            dac_fifo2.w_data.eq(self.from_usb.payload[8:]),
            dac_fifo2.w_en.eq((self.from_usb.channel_no == 2) &
                              self.from_usb.valid),
            dac_fifo3.w_data.eq(self.from_usb.payload[8:]),
            dac_fifo3.w_en.eq((self.from_usb.channel_no == 3) &
                              self.from_usb.valid),
            self.from_usb.ready.eq(
                dac_fifo0.w_rdy | dac_fifo1.w_rdy | dac_fifo2.w_rdy | dac_fifo3.w_rdy),
        ]

        with m.FSM(domain="audio") as fsm:
            with m.State('READ'):
                with m.If(eurorack_pmod.fs_strobe & dac_fifo0.r_rdy):
                    m.d.audio += dac_fifo0.r_en.eq(1)
                    m.next = 'SEND'
            with m.State('SEND'):
                m.d.audio += [
                    dac_fifo0.r_en.eq(0),
                    eurorack_pmod.cal_out0.eq(dac_fifo0.r_data),
                ]
                m.next = 'READ'

        with m.FSM(domain="audio") as fsm:
            with m.State('READ'):
                with m.If(eurorack_pmod.fs_strobe & dac_fifo1.r_rdy):
                    m.d.audio += dac_fifo1.r_en.eq(1)
                    m.next = 'SEND'
            with m.State('SEND'):
                m.d.audio += [
                    dac_fifo1.r_en.eq(0),
                    eurorack_pmod.cal_out1.eq(dac_fifo1.r_data),
                ]
                m.next = 'READ'

        with m.FSM(domain="audio") as fsm:
            with m.State('READ'):
                with m.If(eurorack_pmod.fs_strobe & dac_fifo2.r_rdy):
                    m.d.audio += dac_fifo2.r_en.eq(1)
                    m.next = 'SEND'
            with m.State('SEND'):
                m.d.audio += [
                    dac_fifo2.r_en.eq(0),
                    eurorack_pmod.cal_out2.eq(dac_fifo2.r_data),
                ]
                m.next = 'READ'

        with m.FSM(domain="audio") as fsm:
            with m.State('READ'):
                with m.If(eurorack_pmod.fs_strobe & dac_fifo3.r_rdy):
                    m.d.audio += dac_fifo3.r_en.eq(1)
                    m.next = 'SEND'
            with m.State('SEND'):
                m.d.audio += [
                    dac_fifo3.r_en.eq(0),
                    eurorack_pmod.cal_out3.eq(dac_fifo3.r_data),
                ]
                m.next = 'READ'

        return m
