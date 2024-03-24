#!/usr/bin/env python3
#
# Copyright (c) 2021 Hans Baier <hansfbaier@gmail.com>
# SPDX-License-Identifier: BSD--3-Clause
import os

from amaranth              import *
from amaranth.build        import *
from amaranth.lib.cdc      import FFSynchronizer
from amaranth.lib.fifo     import SyncFIFO, AsyncFIFO

from luna                import top_level_cli
from luna.usb2           import USBDevice, USBIsochronousInMemoryEndpoint, USBIsochronousOutStreamEndpoint, USBIsochronousInStreamEndpoint

from usb_protocol.types                       import USBRequestType, USBRequestRecipient, USBTransferType, USBSynchronizationType, USBUsageType, USBDirection, USBStandardRequests
from usb_protocol.types.descriptors.uac2      import AudioClassSpecificRequestCodes
from usb_protocol.emitters                    import DeviceDescriptorCollection
from usb_protocol.emitters.descriptors        import uac2, standard

from luna.gateware.platform                   import NullPin
from luna.gateware.usb.usb2.device            import USBDevice
from luna.gateware.usb.usb2.request           import USBRequestHandler, StallOnlyRequestHandler
from luna.gateware.usb.stream                 import USBInStreamInterface
from luna.gateware.stream.generator           import StreamSerializer
from luna.gateware.stream                     import StreamInterface
from luna.gateware.architecture.car                    import PHYResetController

class EdgeToPulse(Elaboratable):
    """
        each rising edge of the signal edge_in will be
        converted to a single clock pulse on pulse_out
    """
    def __init__(self):
        self.edge_in          = Signal()
        self.pulse_out        = Signal()

    def elaborate(self, platform) -> Module:
        m = Module()

        edge_last = Signal()

        m.d.sync += edge_last.eq(self.edge_in)
        with m.If(self.edge_in & ~edge_last):
            m.d.comb += self.pulse_out.eq(1)
        with m.Else():
            m.d.comb += self.pulse_out.eq(0)

        return m

class USBStreamToChannels(Elaboratable):
    def __init__(self, max_nr_channels=2):
        # parameters
        self._max_nr_channels = max_nr_channels
        self._channel_bits    = Shape.cast(range(max_nr_channels)).width

        # ports
        self.usb_stream_in       = StreamInterface()
        self.channel_stream_out  = StreamInterface(payload_width=24, extra_fields=[("channel_no", self._channel_bits)])

    def elaborate(self, platform):
        m = Module()

        out_channel_no   = Signal(self._channel_bits)
        out_sample       = Signal(16)
        usb_valid        = Signal()
        usb_first        = Signal()
        usb_payload      = Signal(8)
        out_ready        = Signal()

        m.d.comb += [
            usb_first.eq(self.usb_stream_in.first),
            usb_valid.eq(self.usb_stream_in.valid),
            usb_payload.eq(self.usb_stream_in.payload),
            out_ready.eq(self.channel_stream_out.ready),
            self.usb_stream_in.ready.eq(out_ready),
        ]

        m.d.sync += [
            self.channel_stream_out.valid.eq(0),
            self.channel_stream_out.first.eq(0),
            self.channel_stream_out.last.eq(0),
        ]

        with m.If(usb_valid & out_ready):
            with m.FSM():
                with m.State("B0"):
                    with m.If(usb_first):
                        m.d.sync += out_channel_no.eq(0)
                    with m.Else():
                        m.d.sync += out_channel_no.eq(out_channel_no + 1)

                    m.next = "B1"

                with m.State("B1"):
                    m.d.sync += out_sample[:8].eq(usb_payload)
                    m.next = "B2"

                with m.State("B2"):
                    m.d.sync += out_sample[8:16].eq(usb_payload)
                    m.next = "B3"

                with m.State("B3"):
                    m.d.sync += [
                        self.channel_stream_out.payload.eq(Cat(out_sample, usb_payload)),
                        self.channel_stream_out.valid.eq(1),
                        self.channel_stream_out.channel_no.eq(out_channel_no),
                        self.channel_stream_out.first.eq(out_channel_no == 0),
                        self.channel_stream_out.last.eq(out_channel_no == (2**self._channel_bits - 1)),
                    ]
                    m.next = "B0"

        return m

def connect_fifo_to_stream(fifo, stream, firstBit: int=None, lastBit: int=None) -> None:
    """Connects the output of the FIFO to the of the stream. Data flows from the fifo the stream.
       It is assumed the payload occupies the lowest significant bits
       This function connects first/last signals if their bit numbers are given
    """

    result = [
        stream.valid.eq(fifo.r_rdy),
        fifo.r_en.eq(stream.ready),
        stream.payload.eq(fifo.r_data),
    ]

    if firstBit:
        result.append(stream.first.eq(fifo.r_data[firstBit]))
    if lastBit:
        result.append(stream.last.eq(fifo.r_data[lastBit]))

    return result

class ChannelsToUSBStream(Elaboratable):
    def __init__(self, max_nr_channels=2, sample_width=24, max_packet_size=512):
        assert sample_width in [16, 24, 32]

        # parameters
        self._max_nr_channels = max_nr_channels
        self._channel_bits    = Shape.cast(range(max_nr_channels)).width
        self._sample_width    = sample_width
        self._max_packet_size = max_packet_size

        # ports
        self.usb_stream_out      = StreamInterface()
        self.channel_stream_in   = StreamInterface(payload_width=self._sample_width, extra_fields=[("channel_no", self._channel_bits)])

    def elaborate(self, platform):
        m = Module()
        m.submodules.out_fifo = out_fifo = SyncFIFO(width=8, depth=self._max_packet_size)

        channel_stream  = self.channel_stream_in
        channel_payload = Signal(self._sample_width)
        channel_valid   = Signal()
        channel_ready   = Signal()

        m.d.comb += [
            *connect_fifo_to_stream(out_fifo, self.usb_stream_out),
            channel_payload.eq(channel_stream.payload),
            channel_valid.eq(channel_stream.valid),
            channel_stream.ready.eq(channel_ready),
        ]

        current_sample  = Signal(32 if self._sample_width > 16 else 16)
        current_channel = Signal(self._channel_bits)
        current_byte    = Signal(2 if self._sample_width > 16 else 1)

        last_channel    = self._max_nr_channels - 1
        num_bytes = 4
        last_byte = num_bytes - 1

        shift = 8 if self._sample_width == 24 else 0

        with m.If(out_fifo.w_rdy):
            with m.FSM() as fsm:
                current_channel_next = (current_channel + 1)[:self._channel_bits]

                with m.State("WAIT-FIRST"):
                    # we have to accept data until we find a first channel sample
                    m.d.comb += channel_ready.eq(1)
                    with m.If(channel_valid & (channel_stream.channel_no == 0)):
                        m.d.sync += [
                            current_sample.eq(channel_payload << shift),
                            current_channel.eq(0),
                        ]
                        m.next = "SEND"

                with m.State("SEND"):
                    m.d.comb += [
                        out_fifo.w_data.eq(current_sample[0:8]),
                        out_fifo.w_en.eq(1),
                    ]
                    m.d.sync += [
                        current_byte.eq(current_byte + 1),
                        current_sample.eq(current_sample >> 8),
                    ]

                    with m.If(current_byte == last_byte):
                        with m.If(channel_valid):
                            m.d.comb += channel_ready.eq(1)

                            m.d.sync += current_channel.eq(current_channel_next)

                            with m.If(current_channel_next == channel_stream.channel_no):
                                m.d.sync += current_sample.eq(channel_payload << shift)
                                m.next = "SEND"
                            with m.Else():
                                m.next = "FILL-ZEROS"

                        with m.Else():
                            m.next = "WAIT"

                with m.State("WAIT"):
                    with m.If(channel_valid):
                        m.d.comb += channel_ready.eq(1)
                        m.d.sync += [
                            current_sample.eq(channel_payload << shift),
                            current_channel.eq(current_channel_next),
                        ]
                        m.next = "SEND"

                with m.State("FILL-ZEROS"):
                    m.d.comb += [
                        out_fifo.w_data.eq(0),
                        out_fifo.w_en.eq(1),
                    ]
                    m.d.sync += current_byte.eq(current_byte + 1)

                    with m.If(current_byte == last_byte):
                        m.d.sync += current_channel.eq(current_channel + 1)
                        with m.If(current_channel == last_channel):
                            m.next = "WAIT-FIRST"
        return m

class USB2AudioInterface(Elaboratable):
    """ USB Audio Class v2 interface """
    NR_CHANNELS = 2
    MAX_PACKET_SIZE = 512 # NR_CHANNELS * 24 + 4
    USE_ILA = False
    ILA_MAX_PACKET_SIZE = 512

    def create_descriptors(self):
        """ Creates the descriptors that describe our audio topology. """

        descriptors = DeviceDescriptorCollection()

        with descriptors.DeviceDescriptor() as d:
            d.bcdUSB             = 2.00
            d.bDeviceClass       = 0xEF
            d.bDeviceSubclass    = 0x02
            d.bDeviceProtocol    = 0x01
            d.idVendor           = 0x1209
            d.idProduct          = 0x4711

            d.iManufacturer      = "OpenAudioGear"
            d.iProduct           = "DECAface"
            d.iSerialNumber      = "4711"
            d.bcdDevice          = 0.01

            d.bNumConfigurations = 1

        with descriptors.ConfigurationDescriptor() as configDescr:
            # Interface Association
            interfaceAssociationDescriptor                 = uac2.InterfaceAssociationDescriptorEmitter()
            interfaceAssociationDescriptor.bInterfaceCount = 3 # Audio Control + Inputs + Outputs
            configDescr.add_subordinate_descriptor(interfaceAssociationDescriptor)

            # Interface Descriptor (Control)
            interfaceDescriptor = uac2.StandardAudioControlInterfaceDescriptorEmitter()
            interfaceDescriptor.bInterfaceNumber = 0
            configDescr.add_subordinate_descriptor(interfaceDescriptor)

            # AudioControl Interface Descriptor
            audioControlInterface = self.create_audio_control_interface_descriptor()
            configDescr.add_subordinate_descriptor(audioControlInterface)

            self.create_output_channels_descriptor(configDescr)

            self.create_input_channels_descriptor(configDescr)

            if self.USE_ILA:
                with configDescr.InterfaceDescriptor() as i:
                    i.bInterfaceNumber = 3

                    with i.EndpointDescriptor() as e:
                        e.bEndpointAddress = USBDirection.IN.to_endpoint_address(3) # EP 3 IN
                        e.wMaxPacketSize   = self.ILA_MAX_PACKET_SIZE

        return descriptors


    def create_audio_control_interface_descriptor(self):
        audioControlInterface = uac2.ClassSpecificAudioControlInterfaceDescriptorEmitter()

        # AudioControl Interface Descriptor (ClockSource)
        clockSource = uac2.ClockSourceDescriptorEmitter()
        clockSource.bClockID     = 1
        clockSource.bmAttributes = uac2.ClockAttributes.INTERNAL_FIXED_CLOCK
        clockSource.bmControls   = uac2.ClockFrequencyControl.HOST_READ_ONLY
        audioControlInterface.add_subordinate_descriptor(clockSource)


        # streaming input port from the host to the USB interface
        inputTerminal               = uac2.InputTerminalDescriptorEmitter()
        inputTerminal.bTerminalID   = 2
        inputTerminal.wTerminalType = uac2.USBTerminalTypes.USB_STREAMING
        # The number of channels needs to be 2 here in order to be recognized
        # default audio out device by Windows. We provide an alternate
        # setting with the full channel count, which also references
        # this terminal ID
        inputTerminal.bNrChannels   = self.NR_CHANNELS
        inputTerminal.bCSourceID    = 1
        audioControlInterface.add_subordinate_descriptor(inputTerminal)

        # audio output port from the USB interface to the outside world
        outputTerminal               = uac2.OutputTerminalDescriptorEmitter()
        outputTerminal.bTerminalID   = 3
        outputTerminal.wTerminalType = uac2.OutputTerminalTypes.SPEAKER
        outputTerminal.bSourceID     = 2
        outputTerminal.bCSourceID    = 1
        audioControlInterface.add_subordinate_descriptor(outputTerminal)

        # audio input port from the outside world to the USB interface
        inputTerminal               = uac2.InputTerminalDescriptorEmitter()
        inputTerminal.bTerminalID   = 4
        inputTerminal.wTerminalType = uac2.InputTerminalTypes.MICROPHONE
        inputTerminal.bNrChannels   = self.NR_CHANNELS
        inputTerminal.bCSourceID    = 1
        audioControlInterface.add_subordinate_descriptor(inputTerminal)

        # audio output port from the USB interface to the host
        outputTerminal               = uac2.OutputTerminalDescriptorEmitter()
        outputTerminal.bTerminalID   = 5
        outputTerminal.wTerminalType = uac2.USBTerminalTypes.USB_STREAMING
        outputTerminal.bSourceID     = 4
        outputTerminal.bCSourceID    = 1
        audioControlInterface.add_subordinate_descriptor(outputTerminal)

        return audioControlInterface


    def create_output_streaming_interface(self, c, *, nr_channels, alt_setting_nr):
        # Interface Descriptor (Streaming, OUT, active setting)
        activeAudioStreamingInterface                   = uac2.AudioStreamingInterfaceDescriptorEmitter()
        activeAudioStreamingInterface.bInterfaceNumber  = 1
        activeAudioStreamingInterface.bAlternateSetting = alt_setting_nr
        activeAudioStreamingInterface.bNumEndpoints     = 2
        c.add_subordinate_descriptor(activeAudioStreamingInterface)

        # AudioStreaming Interface Descriptor (General)
        audioStreamingInterface               = uac2.ClassSpecificAudioStreamingInterfaceDescriptorEmitter()
        audioStreamingInterface.bTerminalLink = 2
        audioStreamingInterface.bFormatType   = uac2.FormatTypes.FORMAT_TYPE_I
        audioStreamingInterface.bmFormats     = uac2.TypeIFormats.PCM
        audioStreamingInterface.bNrChannels   = nr_channels
        c.add_subordinate_descriptor(audioStreamingInterface)

        # AudioStreaming Interface Descriptor (Type I)
        typeIStreamingInterface  = uac2.TypeIFormatTypeDescriptorEmitter()
        typeIStreamingInterface.bSubslotSize   = 4
        typeIStreamingInterface.bBitResolution = 24 # we use all 24 bits
        c.add_subordinate_descriptor(typeIStreamingInterface)

        # Endpoint Descriptor (Audio out)
        audioOutEndpoint = standard.EndpointDescriptorEmitter()
        audioOutEndpoint.bEndpointAddress     = USBDirection.OUT.to_endpoint_address(1) # EP 1 OUT
        audioOutEndpoint.bmAttributes         = USBTransferType.ISOCHRONOUS  | \
                                                (USBSynchronizationType.ASYNC << 2) | \
                                                (USBUsageType.DATA << 4)
        audioOutEndpoint.wMaxPacketSize = self.MAX_PACKET_SIZE
        audioOutEndpoint.bInterval       = 1
        c.add_subordinate_descriptor(audioOutEndpoint)

        # AudioControl Endpoint Descriptor
        audioControlEndpoint = uac2.ClassSpecificAudioStreamingIsochronousAudioDataEndpointDescriptorEmitter()
        c.add_subordinate_descriptor(audioControlEndpoint)

        # Endpoint Descriptor (Feedback IN)
        feedbackInEndpoint = standard.EndpointDescriptorEmitter()
        feedbackInEndpoint.bEndpointAddress  = USBDirection.IN.to_endpoint_address(1) # EP 1 IN
        feedbackInEndpoint.bmAttributes      = USBTransferType.ISOCHRONOUS  | \
                                               (USBSynchronizationType.NONE << 2)  | \
                                               (USBUsageType.FEEDBACK << 4)
        feedbackInEndpoint.wMaxPacketSize    = 4
        feedbackInEndpoint.bInterval         = 4
        c.add_subordinate_descriptor(feedbackInEndpoint)


    def create_output_channels_descriptor(self, c):
        #
        # Interface Descriptor (Streaming, OUT, quiet setting)
        #
        quietAudioStreamingInterface = uac2.AudioStreamingInterfaceDescriptorEmitter()
        quietAudioStreamingInterface.bInterfaceNumber  = 1
        quietAudioStreamingInterface.bAlternateSetting = 0
        c.add_subordinate_descriptor(quietAudioStreamingInterface)

        # we need the default alternate setting to be stereo
        # out for windows to automatically recognize
        # and use this audio interface
        self.create_output_streaming_interface(c, nr_channels=self.NR_CHANNELS, alt_setting_nr=1)


    def create_input_streaming_interface(self, c, *, nr_channels, alt_setting_nr, channel_config=0):
        # Interface Descriptor (Streaming, IN, active setting)
        activeAudioStreamingInterface = uac2.AudioStreamingInterfaceDescriptorEmitter()
        activeAudioStreamingInterface.bInterfaceNumber  = 2
        activeAudioStreamingInterface.bAlternateSetting = alt_setting_nr
        activeAudioStreamingInterface.bNumEndpoints     = 1
        c.add_subordinate_descriptor(activeAudioStreamingInterface)

        # AudioStreaming Interface Descriptor (General)
        audioStreamingInterface                 = uac2.ClassSpecificAudioStreamingInterfaceDescriptorEmitter()
        audioStreamingInterface.bTerminalLink   = 5
        audioStreamingInterface.bFormatType     = uac2.FormatTypes.FORMAT_TYPE_I
        audioStreamingInterface.bmFormats       = uac2.TypeIFormats.PCM
        audioStreamingInterface.bNrChannels     = nr_channels
        audioStreamingInterface.bmChannelConfig = channel_config
        c.add_subordinate_descriptor(audioStreamingInterface)

        # AudioStreaming Interface Descriptor (Type I)
        typeIStreamingInterface  = uac2.TypeIFormatTypeDescriptorEmitter()
        typeIStreamingInterface.bSubslotSize   = 4
        typeIStreamingInterface.bBitResolution = 24 # we use all 24 bits
        c.add_subordinate_descriptor(typeIStreamingInterface)

        # Endpoint Descriptor (Audio out)
        audioOutEndpoint = standard.EndpointDescriptorEmitter()
        audioOutEndpoint.bEndpointAddress     = USBDirection.IN.to_endpoint_address(2) # EP 2 IN
        audioOutEndpoint.bmAttributes         = USBTransferType.ISOCHRONOUS  | \
                                                (USBSynchronizationType.ASYNC << 2) | \
                                                (USBUsageType.DATA << 4)
        audioOutEndpoint.wMaxPacketSize = self.MAX_PACKET_SIZE
        audioOutEndpoint.bInterval      = 1
        c.add_subordinate_descriptor(audioOutEndpoint)

        # AudioControl Endpoint Descriptor
        audioControlEndpoint = uac2.ClassSpecificAudioStreamingIsochronousAudioDataEndpointDescriptorEmitter()
        c.add_subordinate_descriptor(audioControlEndpoint)


    def create_input_channels_descriptor(self, c):
        #
        # Interface Descriptor (Streaming, IN, quiet setting)
        #
        quietAudioStreamingInterface = uac2.AudioStreamingInterfaceDescriptorEmitter()
        quietAudioStreamingInterface.bInterfaceNumber  = 2
        quietAudioStreamingInterface.bAlternateSetting = 0
        c.add_subordinate_descriptor(quietAudioStreamingInterface)

        # Windows wants a stereo pair as default setting, so let's have it
        self.create_input_streaming_interface(c, nr_channels=self.NR_CHANNELS, alt_setting_nr=1, channel_config=0x3)

    def add_eurorack_pmod(self, m, platform):

        eurorack_pmod = [
            Resource("eurorack_pmod", 0,
                Subsignal("sdin1",   Pins("1",  conn=("pmod",0), dir='o')),
                Subsignal("sdout1",  Pins("2",  conn=("pmod",0), dir='i')),
                Subsignal("lrck",    Pins("3",  conn=("pmod",0), dir='o')),
                Subsignal("bick",    Pins("4",  conn=("pmod",0), dir='o')),
                Subsignal("mclk",    Pins("10", conn=("pmod",0), dir='o')),
                Subsignal("pdn",     Pins("9",  conn=("pmod",0), dir='o')),
                Subsignal("i2c_sda", Pins("8",  conn=("pmod",0), dir='io')),
                Subsignal("i2c_scl", Pins("7",  conn=("pmod",0), dir='io')),
                Attrs(IO_TYPE="LVCMOS33"),
            )
        ]

        platform.add_resources(eurorack_pmod)
        pmod0 = platform.request("eurorack_pmod")


        # Verilog sources
        vroot = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                             "../eurorack-pmod/gateware")

        platform.add_file("eurorack_pmod_defines.sv", "`define HW_R33")
        platform.add_file("eurorack_pmod.sv", open(os.path.join(vroot, "eurorack_pmod.sv")))
        platform.add_file("pmod_i2c_master.sv", open(os.path.join(vroot, "drivers/pmod_i2c_master.sv")))
        platform.add_file("ak4619.sv", open(os.path.join(vroot, "drivers/ak4619.sv")))
        platform.add_file("cal.sv", open(os.path.join(vroot, "cal/cal.sv")))
        platform.add_file("i2c_master.sv", open(os.path.join(vroot, "external/no2misc/rtl/i2c_master.v")))

        # .hex files
        platform.add_file("cal/cal_mem_default_r33.hex",
                          open(os.path.join(vroot, "cal/cal_mem_default_r33.hex")))
        platform.add_file("drivers/ak4619-cfg.hex",
                          open(os.path.join(vroot, "drivers/ak4619-cfg.hex")))
        platform.add_file("drivers/pca9635-cfg.hex",
                          open(os.path.join(vroot, "drivers/pca9635-cfg.hex")))

        # clk_fs divider.
        # not a true clock domain, don't create one.
        clkdiv_fs = Signal(8)
        self.clk_fs = clk_fs = Signal()
        m.d.audio += clkdiv_fs.eq(clkdiv_fs+1)
        m.d.comb += clk_fs.eq(clkdiv_fs[-1])

        # eurorack-pmod global pipeline sample width
        w = 16

        self.cal_in0 = Signal(signed(w))
        self.cal_in1 = Signal(signed(w))
        self.cal_in2 = Signal(signed(w))
        self.cal_in3 = Signal(signed(w))

        self.cal_out0 = Signal(signed(w))
        self.cal_out1 = Signal(signed(w))
        self.cal_out2 = Signal(signed(w))
        self.cal_out3 = Signal(signed(w))

        self.eeprom_mfg = Signal(8)
        self.eeprom_dev = Signal(8)
        self.eeprom_serial = Signal(32)
        self.jack = Signal(8)

        self.sample_adc0 = Signal(signed(w))
        self.sample_adc1 = Signal(signed(w))
        self.sample_adc2 = Signal(signed(w))
        self.sample_adc3 = Signal(signed(w))

        self.force_dac_output = Signal(signed(w))

        m.d.comb += [
            pmod0.i2c_scl.o.eq(0),
            pmod0.i2c_sda.o.eq(0),
        ]

        m.submodules.veurorack_pmod = Instance("eurorack_pmod",
            # Parameters
            p_W = w,

            # Ports (clk + reset)
            i_clk_256fs = ClockSignal("audio"),
            i_clk_fs = clk_fs,
            i_rst = ResetSignal("audio"),

            # Pads (tristate, require different logic to hook these
            # up to pads depending on the target platform).
            o_i2c_scl_oe = pmod0.i2c_scl.oe,
            i_i2c_scl_i = pmod0.i2c_scl.i,
            o_i2c_sda_oe = pmod0.i2c_sda.oe,
            i_i2c_sda_i = pmod0.i2c_sda.i,

            # Pads (directly hooked up to pads without extra logic required)
            o_pdn = pmod0.pdn.o,
            o_mclk = pmod0.mclk.o,
            o_sdin1 = pmod0.sdin1.o,
            i_sdout1 = pmod0.sdout1.i,
            o_lrck = pmod0.lrck.o,
            o_bick = pmod0.bick.o,

            # Ports (clock at clk_fs)
            o_cal_in0 = self.cal_in0,
            o_cal_in1 = self.cal_in1,
            o_cal_in2 = self.cal_in2,
            o_cal_in3 = self.cal_in3,
            i_cal_out0 = self.cal_out0,
            i_cal_out1 = self.cal_out1,
            i_cal_out2 = self.cal_out2,
            i_cal_out3 = self.cal_out3,

            # Ports (serialized data fetched over I2C)
            o_eeprom_mfg = self.eeprom_mfg,
            o_eeprom_dev = self.eeprom_dev,
            o_eeprom_serial = self.eeprom_serial,
            o_jack = self.jack,

            # Debug ports
            o_sample_adc0 = self.sample_adc0,
            o_sample_adc1 = self.sample_adc1,
            o_sample_adc2 = self.sample_adc2,
            o_sample_adc3 = self.sample_adc3,
            i_force_dac_output = self.force_dac_output,
        )

    def elaborate(self, platform):
        m = Module()

        m.submodules.car = platform.clock_domain_generator()

        self.add_eurorack_pmod(m, platform)

        ulpi = platform.request(platform.default_usb_connection)
        m.submodules.usb = usb = USBDevice(bus=ulpi)

        # Add our standard control endpoint to the device.
        descriptors = self.create_descriptors()
        control_ep = usb.add_control_endpoint()
        control_ep.add_standard_request_handlers(descriptors, blacklist=[
            lambda setup:   (setup.type    == USBRequestType.STANDARD)
                          & (setup.request == USBStandardRequests.SET_INTERFACE)
        ])

        # Attach our class request handlers.
        class_request_handler = UAC2RequestHandlers()
        control_ep.add_request_handler(class_request_handler)

        # Attach class-request handlers that stall any vendor or reserved requests,
        # as we don't have or need any.
        stall_condition = lambda setup : \
            (setup.type == USBRequestType.VENDOR) | \
            (setup.type == USBRequestType.RESERVED)
        control_ep.add_request_handler(StallOnlyRequestHandler(stall_condition))

        ep1_out = USBIsochronousOutStreamEndpoint(
            endpoint_number=1, # EP 1 OUT
            max_packet_size=self.MAX_PACKET_SIZE)
        usb.add_endpoint(ep1_out)

        ep1_in = USBIsochronousInMemoryEndpoint(
            endpoint_number=1, # EP 1 IN
            max_packet_size=4)
        usb.add_endpoint(ep1_in)

        ep2_in = USBIsochronousInStreamEndpoint(
            endpoint_number=2, # EP 2 IN
            max_packet_size=self.MAX_PACKET_SIZE)
        usb.add_endpoint(ep2_in)

        # calculate bytes in frame for audio in
        audio_in_frame_bytes = Signal(range(self.MAX_PACKET_SIZE), reset=24 * self.NR_CHANNELS)
        audio_in_frame_bytes_counting = Signal()

        with m.If(ep1_out.stream.valid & ep1_out.stream.ready):
            with m.If(audio_in_frame_bytes_counting):
                m.d.usb += audio_in_frame_bytes.eq(audio_in_frame_bytes + 1)

            with m.If(ep1_out.stream.first):
                m.d.usb += [
                    audio_in_frame_bytes.eq(1),
                    audio_in_frame_bytes_counting.eq(1),
                ]
            with m.Elif(ep1_out.stream.last):
                m.d.usb += audio_in_frame_bytes_counting.eq(0)

        # Connect our device as a high speed device
        m.d.comb += [
            ep1_in.bytes_in_frame.eq(4),
            ep2_in.bytes_in_frame.eq(audio_in_frame_bytes),
            usb.connect          .eq(1),
            usb.full_speed_only  .eq(0),
        ]

        # feedback endpoint
        feedbackValue      = Signal(32, reset=0x60000)
        bitPos             = Signal(5)

        # this tracks the number of audio frames since the last USB frame
        # 12.288MHz / 8kHz = 1536, so we need at least 11 bits = 2048
        # we need to capture 32 micro frames to get to the precision
        # required by the USB standard, so and that is 0xc000, so we
        # need 16 bits here
        audio_clock_counter = Signal(16)
        sof_counter         = Signal(5)

        audio_clock_usb = Signal()
        m.submodules.audio_clock_usb_sync = FFSynchronizer(ClockSignal("audio"), audio_clock_usb, o_domain="usb")
        m.submodules.audio_clock_usb_pulse = audio_clock_usb_pulse = DomainRenamer("usb")(EdgeToPulse())
        audio_clock_tick = Signal()
        m.d.usb += [
            audio_clock_usb_pulse.edge_in.eq(audio_clock_usb),
            audio_clock_tick.eq(audio_clock_usb_pulse.pulse_out),
        ]

        ## IDEAS
        ## Same EdgeToPulse pattern for clk_256fs -> clk_fs strobe (in audio domain)
        ## AsyncFIFO for clk_256fs (valid on strobe) -> usb domain stream
        ## Warn: channel_no synchronization: test with some assumptions before integrating?
        ## DAC side will probably be easier -- read implementation of USB -> Channel component
        ## >>> Output channel numbers are always alternating
        ## >>> But they still should go through some FIFOs? Origin is USB domain.
        ## AsyncFIFO with payload for all samples (cd_audio) -> (cd_usb)
        ## Then state machine in cd_usb to read the samples into a destination stream

        with m.If(audio_clock_tick):
            m.d.usb += audio_clock_counter.eq(audio_clock_counter + 1)

        with m.If(usb.sof_detected):
            m.d.usb += sof_counter.eq(sof_counter + 1)

            # according to USB2 standard chapter 5.12.4.2
            # we need 2**13 / 2**8 = 2**5 = 32 SOF-frames of
            # sample master frequency counter to get enough
            # precision for the sample frequency estimate
            # / 2**8 because the ADAT-clock = 256 times = 2**8
            # the sample frequency and sof_counter is 5 bits
            # so it wraps automatically every 32 SOFs
            with m.If(sof_counter == 0):
                m.d.usb += [
                    feedbackValue.eq(audio_clock_counter << 3),
                    audio_clock_counter.eq(0),
                ]

        m.d.comb += [
            bitPos.eq(ep1_in.address << 3),
            ep1_in.value.eq(0xff & (feedbackValue >> bitPos)),
        ]

        m.submodules.usb_to_channel_stream = usb_to_channel_stream = \
            DomainRenamer("usb")(USBStreamToChannels(self.NR_CHANNELS))

        m.submodules.channels_to_usb_stream = channels_to_usb_stream = \
            DomainRenamer("usb")(ChannelsToUSBStream(self.NR_CHANNELS))


        audio_big_counter = Signal(32)
        with m.If(audio_clock_tick):
            m.d.usb += audio_big_counter.eq(audio_big_counter + 1)

        m.submodules.adc_fifo0 = adc_fifo0 = AsyncFIFO(width=16, depth=64, w_domain="audio", r_domain="usb")
        m.submodules.adc_fifo1 = adc_fifo1 = AsyncFIFO(width=16, depth=64, w_domain="audio", r_domain="usb")
        fs_strobe = Signal()
        m.submodules.fs_edge = fs_edge = DomainRenamer("audio")(EdgeToPulse())
        m.d.audio += [
            fs_edge.edge_in.eq(self.clk_fs),

            # warn: ignoring rdy // should be fine
            adc_fifo0.w_data.eq(self.cal_in0),
            adc_fifo0.w_en.eq(fs_edge.pulse_out),

            adc_fifo1.w_data.eq(self.cal_in1),
            adc_fifo1.w_en.eq(fs_edge.pulse_out),
        ]

        # ADC FSM
        with m.FSM(domain="usb") as fsm:
            with m.State('CH0-WAIT'):
                m.d.usb += channels_to_usb_stream.channel_stream_in.valid.eq(0),
                with m.If(adc_fifo0.r_rdy):
                    m.d.usb += adc_fifo0.r_en.eq(1)
                    m.next = 'CH0-LATCH'
            with m.State('CH0-LATCH'):
                m.d.usb += [
                    adc_fifo0.r_en.eq(0),
                    channels_to_usb_stream.channel_stream_in.payload.eq(Cat(Const(0, 8), adc_fifo0.r_data)),
                    channels_to_usb_stream.channel_stream_in.channel_no.eq(0),
                    channels_to_usb_stream.channel_stream_in.valid.eq(1),
                ]
                m.next = 'CH0-SEND'
            with m.State('CH0-SEND'):
                with m.If(channels_to_usb_stream.channel_stream_in.ready):
                    m.d.usb += channels_to_usb_stream.channel_stream_in.valid.eq(0)
                    m.next = 'CH1-WAIT'
            with m.State('CH1-WAIT'):
                m.d.usb += channels_to_usb_stream.channel_stream_in.valid.eq(0),
                with m.If(adc_fifo1.r_rdy):
                    m.d.usb += adc_fifo1.r_en.eq(1)
                    m.next = 'CH1-LATCH'
            with m.State('CH1-LATCH'):
                m.d.usb += [
                    adc_fifo1.r_en.eq(0),
                    channels_to_usb_stream.channel_stream_in.payload.eq(Cat(Const(0, 8), adc_fifo1.r_data)),
                    channels_to_usb_stream.channel_stream_in.channel_no.eq(1),
                    channels_to_usb_stream.channel_stream_in.valid.eq(1),
                ]
                m.next = 'CH1-SEND'
            with m.State('CH1-SEND'):
                with m.If(channels_to_usb_stream.channel_stream_in.ready):
                    m.d.usb += channels_to_usb_stream.channel_stream_in.valid.eq(0)
                    m.next = 'CH0-WAIT'

        m.submodules.dac_fifo0 = dac_fifo0 = AsyncFIFO(width=16, depth=64, w_domain="usb", r_domain="audio")
        m.submodules.dac_fifo1 = dac_fifo1 = AsyncFIFO(width=16, depth=64, w_domain="usb", r_domain="audio")
        m.d.comb += [
            dac_fifo0.w_data.eq(usb_to_channel_stream.channel_stream_out.payload[8:]),
            dac_fifo0.w_en.eq((usb_to_channel_stream.channel_stream_out.channel_no == 0) &
                              usb_to_channel_stream.channel_stream_out.valid),
            dac_fifo1.w_data.eq(usb_to_channel_stream.channel_stream_out.payload[8:]),
            dac_fifo1.w_en.eq((usb_to_channel_stream.channel_stream_out.channel_no == 1) &
                              usb_to_channel_stream.channel_stream_out.valid),
            usb_to_channel_stream.channel_stream_out.ready.eq(dac_fifo0.w_rdy | dac_fifo1.w_rdy),
        ]

        with m.FSM(domain="audio") as fsm:
            with m.State('READ'):
                with m.If(fs_edge.pulse_out & dac_fifo0.r_rdy):
                    m.d.audio += dac_fifo0.r_en.eq(1)
                    m.next = 'SEND'
            with m.State('SEND'):
                m.d.audio += [
                    dac_fifo0.r_en.eq(0),
                    self.cal_out0.eq(dac_fifo0.r_data),
                ]
                m.next = 'READ'

        with m.FSM(domain="audio") as fsm:
            with m.State('READ'):
                with m.If(fs_edge.pulse_out & dac_fifo1.r_rdy):
                    m.d.audio += dac_fifo1.r_en.eq(1)
                    m.next = 'SEND'
            with m.State('SEND'):
                m.d.audio += [
                    dac_fifo1.r_en.eq(0),
                    self.cal_out1.eq(dac_fifo1.r_data),
                ]
                m.next = 'READ'

        m.d.comb += [
            # Wire USB <-> stream synchronizers
            usb_to_channel_stream.usb_stream_in.stream_eq(ep1_out.stream),
            ep2_in.stream.stream_eq(channels_to_usb_stream.usb_stream_out),

            # Wire stream synchronizers <-> fake data (for now)

            # TODO: replace
            # i2s_transmitter.stream_in.stream_eq(usb_to_channel_stream.channel_stream_out),
            # TODO: this is a HACK: Override stream IN, always ready
            usb_to_channel_stream.channel_stream_out.ready.eq(1),

            # wire I2S receiver to USB
            # TODO: replace
            #channels_to_usb_stream.channel_stream_in.stream_eq(i2s_receiver.stream_out),
            #channels_to_usb_stream.channel_stream_in.channel_no.eq(~i2s_receiver.stream_out.first),
            # TODO: this is a HACK: Override stream OUT, always valid, channel 0 bumps FSM
            #channels_to_usb_stream.channel_stream_in.valid.eq(1),
            #channels_to_usb_stream.channel_stream_in.payload.eq(audio_big_counter[16:]),
            #channels_to_usb_stream.channel_stream_in.channel_no.eq(0),

        ]

        return m

class UAC2RequestHandlers(USBRequestHandler):
    """ request handlers to implement UAC2 functionality. """
    def __init__(self):
        super().__init__()

        self.output_interface_altsetting_nr = Signal(3)
        self.input_interface_altsetting_nr  = Signal(3)
        self.interface_settings_changed     = Signal()

    def elaborate(self, platform):
        m = Module()

        interface         = self.interface
        setup             = self.interface.setup

        m.submodules.transmitter = transmitter = \
            StreamSerializer(data_length=14, domain="usb", stream_type=USBInStreamInterface, max_length_width=14)

        m.d.usb += self.interface_settings_changed.eq(0)

        #
        # Class request handlers.
        #
        with m.If(setup.type == USBRequestType.STANDARD):
            with m.If((setup.recipient == USBRequestRecipient.INTERFACE) &
                      (setup.request == USBStandardRequests.SET_INTERFACE)):

                interface_nr   = setup.index
                alt_setting_nr = setup.value

                m.d.usb += [
                    self.output_interface_altsetting_nr.eq(0),
                    self.input_interface_altsetting_nr.eq(0),
                    self.interface_settings_changed.eq(1),
                ]

                with m.Switch(interface_nr):
                    with m.Case(1):
                        m.d.usb += self.output_interface_altsetting_nr.eq(alt_setting_nr)
                    with m.Case(2):
                        m.d.usb += self.input_interface_altsetting_nr.eq(alt_setting_nr)

                # Always ACK the data out...
                with m.If(interface.rx_ready_for_response):
                    m.d.comb += interface.handshakes_out.ack.eq(1)

                # ... and accept whatever the request was.
                with m.If(interface.status_requested):
                    m.d.comb += self.send_zlp()

        request_clock_freq = (setup.value == 0x100) & (setup.index == 0x0100)
        with m.Elif(setup.type == USBRequestType.CLASS):
            with m.Switch(setup.request):
                with m.Case(AudioClassSpecificRequestCodes.RANGE):
                    m.d.comb += transmitter.stream.attach(self.interface.tx)

                    with m.If(request_clock_freq):
                        m.d.comb += [
                            Cat(transmitter.data).eq(
                                Cat(Const(0x1, 16), # no triples
                                    Const(48000, 32), # MIN
                                    Const(48000, 32), # MAX
                                    Const(0, 32))),   # RES
                            transmitter.max_length.eq(setup.length)
                        ]
                    with m.Else():
                        m.d.comb += interface.handshakes_out.stall.eq(1)

                    # ... trigger it to respond when data's requested...
                    with m.If(interface.data_requested):
                        m.d.comb += transmitter.start.eq(1)

                    # ... and ACK our status stage.
                    with m.If(interface.status_requested):
                        m.d.comb += interface.handshakes_out.ack.eq(1)

                with m.Case(AudioClassSpecificRequestCodes.CUR):
                    m.d.comb += transmitter.stream.attach(self.interface.tx)
                    with m.If(request_clock_freq & (setup.length == 4)):
                        m.d.comb += [
                            Cat(transmitter.data[0:4]).eq(Const(48000, 32)),
                            transmitter.max_length.eq(4)
                        ]
                    with m.Else():
                        m.d.comb += interface.handshakes_out.stall.eq(1)

                    # ... trigger it to respond when data's requested...
                    with m.If(interface.data_requested):
                        m.d.comb += transmitter.start.eq(1)

                    # ... and ACK our status stage.
                    with m.If(interface.status_requested):
                        m.d.comb += interface.handshakes_out.ack.eq(1)

                with m.Case():
                    #
                    # Stall unhandled requests.
                    #
                    with m.If(interface.status_requested | interface.data_requested):
                        m.d.comb += interface.handshakes_out.stall.eq(1)

                return m

if __name__ == "__main__":
    os.environ["AMARANTH_debug_verilog"] = "1"
    os.environ["LUNA_PLATFORM"] = "luna.gateware.platform.ecpix5:ECPIX5_85F_Platform"
    top_level_cli(USB2AudioInterface)
