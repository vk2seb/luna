#
# This file is part of usb2-highspeed-core
#
# Copyright (c) 2021 Hans Baier <hansfbaier@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

""" Contains the gatware module necessary to interpret and generate low-level USB host packets. """


import operator
import unittest
import functools

from amaranth            import Signal, Module, Elaboratable, Cat, Array, Const
from amaranth.build      import Platform
from amaranth.hdl.rec    import Record, DIR_FANIN, DIR_FANOUT


from ..                 import USBSpeed, USBPacketID
from ...stream          import USBInStreamInterface, USBOutStreamInterface
from ....interface.utmi import UTMITransmitInterface
from ....test           import LunaGatewareTestCase, usb_domain_test_case

from ..packet           import USBTokenDetector



class TokenGeneratorInterface(Record):
    """ Record providing an interface to a USB token generator.

    Attributes
    ----------
    pid: Signal(4), input
        The Packet ID of the token.
    payload: Signal(11), input
        The payload of the token.

    new_token: Signal(), input
        Strobe asserted for a single cycle when a new token packet should be sent.
    """

    def __init__(self):
        super().__init__([
            ('pid',                4, DIR_FANIN),
            ('payload',           11, DIR_FANIN),
            ('new_token',          1, DIR_FANIN),
        ])

class USBTokenGenerator(Elaboratable):
    """ Gateware that generates token packets, including SOF

    Attributes
    ----------
    interface: TokenGeneratorInterface
        The interface that specifies the Token to be constructed
    speed: Signal(2), input
        Carries a ``USBSpeed`` constant identifying the device's current operating speed.

    Parameters
    ----------
        utmi: UTMIInterface
            The UTMI bus to transmit on.
    """

    def __init__(self, *, utmi, domain_clock=60e6):
        self.utmi = utmi
        self._domain_clock = domain_clock

        #
        # I/O port
        #
        self.interface = TokenGeneratorInterface()
        self.speed     = Signal(2)

    def elaborate(self, platform: Platform) -> Module:
        m = Module()
        interface = self.interface
        utmi      = self.utmi

        token_data    = Signal(24)
        token_crc     = Signal(5)

        m.d.comb += token_crc.eq(USBTokenDetector.generate_crc_for_token(interface.payload))

        m.d.usb += [
            token_data.eq(
                Cat(
                    interface.pid,
                    ~interface.pid,
                    interface.payload,
                    token_crc))
        ]

        with m.FSM(domain="usb"):

            # waiting for a new packet to be sent
            with m.State("IDLE"):
                with m.If(utmi.tx_ready & interface.new_token):
                    m.next = "PID"

            with m.State("PID"):
                with m.If(utmi.tx_ready):
                    m.d.comb += [
                        utmi.tx_data.eq(token_data[0:8]),
                        utmi.tx_valid.eq(1),
                    ]
                    m.next = "PAYLOAD0"

            with m.State("PAYLOAD0"):
                with m.If(utmi.tx_ready):
                    m.d.comb += [
                        utmi.tx_data.eq(token_data[8:16]),
                        utmi.tx_valid.eq(1),
                    ]
                    m.next = "PAYLOAD1"

            with m.State("PAYLOAD1"):
                with m.If(utmi.tx_ready):
                    m.d.comb += [
                        utmi.tx_data.eq(token_data[16:24]),
                        utmi.tx_valid.eq(1),
                    ]
                    m.next = "IDLE"

        return m