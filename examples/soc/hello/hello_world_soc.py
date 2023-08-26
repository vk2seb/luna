#!/usr/bin/env python3
#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from amaranth                     import Elaboratable, Module, Cat, ClockSignal, ResetSignal
from amaranth.hdl.rec             import Record
from amaranth_stdio.serial                   import AsyncSerial

from lambdasoc.periph             import Peripheral
from lambdasoc.periph.serial      import AsyncSerialPeripheral
from lambdasoc.periph.timer       import TimerPeripheral

from luna.gateware.interface.ulpi  import ULPIRegisterWindow

from luna                         import top_level_cli
from luna.gateware.soc            import SimpleSoC
from luna.gateware.interface.uart import UARTTransmitterPeripheral

from luna.full_devices   import USBSerialDevice

class LEDPeripheral(Peripheral, Elaboratable):
    """ Example peripheral that controls the board's LEDs. """

    def __init__(self):
        super().__init__()

        # Create our LED register.
        # Note that there's a bunch of 'magic' that goes on behind the scenes, here:
        # a memory address will automatically be reserved for this register in the address
        # space it's attached to; and the SoC utilities will automatically generate header
        # entires and stub functions for it.
        bank            = self.csr_bank()
        self._output    = bank.csr(6, "rw")

        # ... and convert our register into a Wishbone peripheral.
        self._bridge    = self.bridge(data_width=32, granularity=8, alignment=2)
        self.bus        = self._bridge.bus


    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge = self._bridge

        # Grab our LEDS...
        leds = Cat(platform.request("led", i) for i in range(4))

        # ... and update them on each register write.
        with m.If(self._output.w_stb):
            m.d.sync += [
                self._output.r_data  .eq(self._output.w_data),
                leds                 .eq(self._output.w_data),
            ]

        return m

class USBSerialDeviceExample(Elaboratable):
    """ Device that acts as a 'USB-to-serial' loopback using our premade gateware. """

    def elaborate(self, platform):
        m = Module()

        # Create our USB-to-serial converter.
        ulpi = platform.request(platform.default_usb_connection)
        m.submodules.usb_serial = usb_serial = \
                USBSerialDevice(bus=ulpi, idVendor=0x16d0, idProduct=0x0f3b)

        m.d.comb += [
            # Place the streams into a loopback configuration...
            usb_serial.tx.payload  .eq(usb_serial.rx.payload),
            usb_serial.tx.valid    .eq(usb_serial.rx.valid),
            usb_serial.tx.first    .eq(usb_serial.rx.first),
            usb_serial.tx.last     .eq(usb_serial.rx.last),
            usb_serial.rx.ready    .eq(usb_serial.tx.ready),

            # ... and always connect by default.
            usb_serial.connect     .eq(1)
        ]

        return m

class LunaCPUExample(Elaboratable):
    """ Simple example of building a simple SoC around LUNA. """

    def __init__(self):
        clock_freq = 60e6


        # Create our SoC...
        self.soc = soc = SimpleSoC()

        soc.add_rom('hello_world.bin', size=0x1000)
        soc.add_ram(0x1000)

        # ...  add our UART peripheral...
        self.uart_pins = Record([
            ('rx', [('i', 1)]),
            ('tx', [('o', 1)])
        ])

        uart_core = AsyncSerial(
            data_bits = 8,
            divisor   = int(100e6 // 115200),
            pins      = self.uart_pins,
        )
        self.uart = uart = AsyncSerialPeripheral(core=uart_core)
        soc.add_peripheral(uart)

        # ... add a timer, to control our LED blinkies...
        self.timer = timer = TimerPeripheral(24)
        soc.add_peripheral(timer)

        # ... and add our LED peripheral.
        leds = LEDPeripheral()
        soc.add_peripheral(leds)


    def elaborate(self, platform):
        m = Module()
        m.submodules.soc = self.soc

        # Generate our domain clocks/resets.
        m.submodules.car = platform.clock_domain_generator()

        m.submodules.usb_serial = USBSerialDeviceExample();

        # Connect up our UART.
        uart_io = platform.request("uart", 0)
        m.d.comb += [
            uart_io.tx         .eq(self.uart_pins.tx),
            self.uart_pins.rx  .eq(uart_io.rx)
        ]

        return m


if __name__ == "__main__":
    design = LunaCPUExample()
    top_level_cli(design, cli_soc=design.soc)
