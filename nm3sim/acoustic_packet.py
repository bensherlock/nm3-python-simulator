#!/usr/bin/env python
#
# Acoustic Packet
#
# This file is part of NM3 Python Simulator.
# https://github.com/bensherlock/nm3-python-simulator
#
#
# MIT License
#
# Copyright (c) 2023 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

class AcousticPacket:
    """Acoustic Packet class.
    PACKET MESSAGE HEADER STRUCTURE
    LFM (UP / DN)   Payload Length  CMD     Address (Src / Dest)    Packet Type
    ===============================================================================================
    UP              0               0       Dest                    Ping Request
    DN                              1       Src                     Ack / Ping Response
    UP                              2       Dest                    Test Message Request
    UP                              3       Dest                    VBatt Request
    UP              1-63            0       Dest                    Unicast Message
    UP                              1       Src                     Broadcast Message
    UP                              2       Dest                    Unicast Message With Ack Request
    UP                              3       Dest                    Echo Message Request
    """

    FRAMESYNCH_UP, FRAMESYNCH_DN = range(2)
    CMD_PING_REQ, CMD_PING_REP, CMD_TEST_REQ, CMD_VBATT_REQ = range(4)
    CMD_UNICAST_MSG, CMD_BROADCAST_MSG, CMD_UNICAST_ACK_MSG, CMD_ECHO_MSG = range(4)

    def __init__(self, frame_synch=FRAMESYNCH_UP, address=255, command=CMD_PING_REQ, payload_length=0, payload_bytes=None, hamr_timestamp=0.0,
                 source_level=168.0, band_start=24000.0, band_stop=32000.0, receive_snr=20.0, receive_sound_pressure_level=168.0):
        #AcousticPacket: { FrameSynch: Up/Dn, Address: 0-255, Command: 0-3, PayloadLength: 0-64, PayloadBytes: bytes(0-64) }
        self._frame_synch = frame_synch
        self._address = address
        self._command = command
        self._payload_length = payload_length
        self._payload_bytes = payload_bytes
        self._hamr_timestamp = hamr_timestamp

        # Source Level (dB re 1uPa @ 1m) is provided by the transmitting modem
        self._source_level = source_level

        # Band (Hz)
        self._band_start = band_start
        self._band_stop = band_stop

        # Receive SNR (dB) is updated by the controller based on transmission losses and ambient noise levels
        # Used by the receiving modem (hardware or software receiver) to determine probability of packet success.
        self._receive_snr = receive_snr

        # Receive Sound Pressure Level (SPL) (dB re 1uPa) this is the acoustic transmission after losses as arriving
        # at the receiver. This is isolated from ambient noises and other arrivals. Used by "Hydrophones" and more
        # complex simulations of the modem receiver.
        self._receive_sound_pressure_level = receive_sound_pressure_level

        # Transmit Duration is the acoustic duration of the packet.
        # This determines the time spent in Transmit state and in Receive state
        self._transmit_duration = self.calculate_transmit_duration()


    @property
    def frame_synch(self):
        return self._frame_synch

    @frame_synch.setter
    def frame_synch(self, frame_synch):
        self._frame_synch = frame_synch

    @property
    def address(self):
        return self._address

    @address.setter
    def address(self, address):
        self._address = address

    @property
    def command(self):
        return self._command

    @command.setter
    def command(self, command):
        self._command = command

    @property
    def payload_length(self):
        return self._payload_length

    @payload_length.setter
    def payload_length(self, payload_length):
        self._payload_length = payload_length
        self._transmit_duration = self.calculate_transmit_duration()

    @property
    def payload_bytes(self):
        return self._payload_bytes

    @payload_bytes.setter
    def payload_bytes(self, payload_bytes):
        self._payload_bytes = payload_bytes

    @property
    def hamr_timestamp(self):
        return self._hamr_timestamp

    @hamr_timestamp.setter
    def hamr_timestamp(self, hamr_timestamp):
        self._hamr_timestamp = hamr_timestamp

    @property
    def band_start(self) -> float:
        return self._band_start

    @band_start.setter
    def band_start(self, band_start: float):
        self._band_start = band_start

    @property
    def band_stop(self) -> float:
        return self._band_stop

    @band_stop.setter
    def band_stop(self, band_stop: float):
        self._band_stop = band_stop

    @property
    def band_centre(self) -> float:
        return (self._band_start + self._band_stop) / 2.0

    @property
    def band_width(self) -> float:
        return self._band_stop - self._band_start

    @property
    def source_level(self) -> float:
        return self._source_level

    @source_level.setter
    def source_level(self, source_level: float):
        self._source_level = source_level

    @property
    def receive_snr(self) -> float:
        return self._receive_snr

    @receive_snr.setter
    def receive_snr(self, receive_snr: float):
        self._receive_snr = receive_snr

    @property
    def receive_sound_pressure_level(self) -> float:
        return self._receive_sound_pressure_level

    @receive_sound_pressure_level.setter
    def receive_sound_pressure_level(self, receive_sound_pressure_level: float):
        self._receive_sound_pressure_level = receive_sound_pressure_level


    
    @property
    def transmit_duration(self) -> float:
        return self._transmit_duration

    def calculate_transmit_duration(self):
        """Calculate the packet transmit duration."""
        # 0.105 + ((payload_bytes + 16) * 2.0 * 50.0 / 8000.0)
        transmit_duration = 0.105
        if self._payload_length:
            transmit_duration = 0.105 + ((self._payload_length + 16.0) * 2.0 * 50.0 / 8000.0)

        return transmit_duration

    def json(self):
        """Returns a json dictionary representation."""
        jason = {"FrameSynch": self._frame_synch,
                 "Address": self._address,
                 "Command": self._command,
                 "PayloadLength": self._payload_length,
                 "PayloadBytes": self._payload_bytes,
                 "HamrTimestamp": self._hamr_timestamp,
                 "SourceLevel": self.source_level,
                 "BandStart": self.band_start,
                 "BandStop": self.band_stop,
                 "ReceiveSnr": self.receive_snr,
                 "ReceiveSpl": self.receive_sound_pressure_level}

        return jason

    @staticmethod
    def from_json(jason): # -> Union[AcousticPacket, None]:
        acoustic_packet = AcousticPacket(frame_synch=jason["FrameSynch"],
                                         address=jason["Address"],
                                         command=jason["Command"],
                                         payload_length=jason["PayloadLength"],
                                         payload_bytes=jason["PayloadBytes"],
                                         hamr_timestamp=jason["HamrTimestamp"],
                                         source_level=jason["SourceLevel"],
                                         band_start=jason["BandStart"],
                                         band_stop=jason["BandStop"],
                                         receive_snr=jason["ReceiveSnr"],
                                         receive_sound_pressure_level=jason["ReceiveSpl"])
        return acoustic_packet
