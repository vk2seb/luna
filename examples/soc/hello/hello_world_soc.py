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


class ULPIRegisterPeripheral(Peripheral, Elaboratable):
    """ Peripheral that provides access to a ULPI PHY, and its registers. """

    def __init__(self, name="ulpi", io_resource_name="usb"):
        super().__init__(name=name)
        self._io_resource = io_resource_name

        # Create our registers...
        bank            = self.csr_bank()
        self._address   = bank.csr(8, "w")
        self._value     = bank.csr(8, "rw")
        self._busy      = bank.csr(1, "r")

        # ... and convert our register into a Wishbone peripheral.
        self._bridge    = self.bridge(data_width=32, granularity=8, alignment=2)
        self.bus        = self._bridge.bus


    def elaborate(self, platform):
        m = Module()
        m.submodules.bridge = self._bridge

        # Grab a connection to our ULPI PHY.
        target_ulpi = platform.request(self._io_resource)

        #
        # ULPI Register Window
        #
        ulpi_reg_window  = ULPIRegisterWindow()
        m.submodules  += ulpi_reg_window

        # Connect up the window.
        m.d.comb += [
            ulpi_reg_window.ulpi_data_in  .eq(target_ulpi.data.i),
            ulpi_reg_window.ulpi_dir      .eq(target_ulpi.dir.i),
            ulpi_reg_window.ulpi_next     .eq(target_ulpi.nxt.i),

            target_ulpi.clk               .eq(ClockSignal("usb")),
            target_ulpi.rst               .eq(ResetSignal("usb")),
            target_ulpi.stp               .eq(ulpi_reg_window.ulpi_stop),
            target_ulpi.data.o            .eq(ulpi_reg_window.ulpi_data_out),
            target_ulpi.data.oe           .eq(~target_ulpi.dir.i)
        ]

        #
        # Address register logic.
        #

        # Perform a read request whenever the user writes to ULPI address...
        m.d.sync += ulpi_reg_window.read_request.eq(self._address.w_stb)

        # And update the register address accordingly.
        with m.If(self._address.w_stb):
            m.d.sync += ulpi_reg_window.address.eq(self._address.w_data)


        #
        # Value register logic.
        #

        # Always report back the last read data.
        m.d.comb += self._value.r_data.eq(ulpi_reg_window.read_data)

        # Perform a write whenever the user writes to our ULPI value.
        m.d.sync += ulpi_reg_window.write_request.eq(self._value.w_stb)
        with m.If(self._address.w_stb):
            m.d.sync += ulpi_reg_window.write_data.eq(self._value.w_data)


        #
        # Busy register logic.
        #
        m.d.comb += self._busy.r_data.eq(ulpi_reg_window.busy)

        return m




class LunaCPUExample(Elaboratable):
    """ Simple example of building a simple SoC around LUNA. """

    def __init__(self):
        clock_freq = 100e6


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

        ulpi = ULPIRegisterPeripheral(name="ulpi", io_resource_name="ulpi")
        soc.add_peripheral(ulpi)


    def elaborate(self, platform):
        m = Module()
        m.submodules.soc = self.soc

        # Generate our domain clocks/resets.
        m.submodules.car = platform.clock_domain_generator()

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
