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

    def __init__(self, frame_synch=FRAMESYNCH_UP, address=255, command=CMD_PING_REQ, payload_length=0, payload_bytes=None, hamr_timestamp=0.0, source_level=168.0):
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
        self._band_start = 24000
        self._band_stop = 32000

        # Receive SNR (dB) is updated by the controller based on transmission losses and ambient noise levels
        # Used by the receiving modem (hardware or software receiver) to determine probability of packet success.
        self._receive_snr = 20.0

        # Transmit Duration is the acoustic duration of the packet.
        # This determines the time spent in Transmit state and in Receive state
        self._transmit_duration = self.calculate_transmit_duration()

        # Lookup tables here based on varying data payload lengths and varying multipath severity.
        # Check curves in paper - Fig. 12.
        # Tables generated for the paper: 20190827-nm3paperdata.git 03-nm3packetsim\results
        # Anything below -9dB is lost, anything above 9dB is 100% successful.
        # Everything else is within the lookup table.
        # Multipath configurations (1,1,0) is a harsh multipath scenario.
        # (1,0,0) is less so. (0,0,0) is just AWGN.
        # Multipath level - 0=(0,0,0)/AWGN, 1=(1,0,0), 2=(1,1,0)
        self._snr_vs_per_results = [
            (0, [
                (4, [(-9, 0.919), (-8, 0.736), (-7, 0.465), (-6, 0.236), (-5, 0.076),
                     (-4, 0.012), (-3, 0.000), (-2, 0.000), (-1, 0.000), (0, 0.000),
                     (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                     (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L4 (0,0,0)
                (8, [(-9, 0.946), (-8, 0.732), (-7, 0.484), (-6, 0.234), (-5, 0.074),
                     (-4, 0.013), (-3, 0.002), (-2, 0.000), (-1, 0.000), (0, 0.000),
                     (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                     (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L8 (0,0,0)
                (16, [(-9, 0.984), (-8, 0.777), (-7, 0.482), (-6, 0.225), (-5, 0.070),
                      (-4, 0.017), (-3, 0.001), (-2, 0.000), (-1, 0.000), (0, 0.000),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L16 (0,0,0)
                (32, [(-9, 1.000), (-8, 0.910), (-7, 0.469), (-6, 0.228), (-5, 0.074),
                      (-4, 0.011), (-3, 0.000), (-2, 0.000), (-1, 0.000), (0, 0.000),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L32 (0,0,0)
                (64, [(-9, 1.000), (-8, 0.997), (-7, 0.644), (-6, 0.221), (-5, 0.066),
                      (-4, 0.011), (-3, 0.001), (-2, 0.000), (-1, 0.000), (0, 0.000),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L64 (0,0,0)
            ]),  # Multpath Level 0 (0,0,0) AWGN
            (1, [
                (4, [(-9, 1.000), (-8, 1.000), (-7, 0.998), (-6, 0.960), (-5, 0.776),
                     (-4, 0.434), (-3, 0.200), (-2, 0.062), (-1, 0.013), (0, 0.001),
                     (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                     (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L4 (1,0,0)
                (8, [(-9, 1.000), (-8, 1.000), (-7, 0.999), (-6, 0.981), (-5, 0.816),
                     (-4, 0.456), (-3, 0.194), (-2, 0.062), (-1, 0.011), (0, 0.001),
                     (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                     (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L8 (1,0,0)
                (16, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 0.998), (-5, 0.944),
                      (-4, 0.605), (-3, 0.228), (-2, 0.061), (-1, 0.011), (0, 0.001),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L16 (1,0,0)
                (32, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 1.000), (-5, 0.998),
                      (-4, 0.897), (-3, 0.388), (-2, 0.080), (-1, 0.013), (0, 0.002),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L32 (1,0,0)
                (64, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 1.000), (-5, 1.000),
                      (-4, 0.998), (-3, 0.856), (-2, 0.270), (-1, 0.024), (0, 0.001),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L64 (1,0,0)
            ]),  # Multpath Level 1 (1,0,0)
            (2, [
                (16, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 1.000), (-5, 1.000),
                      (-4, 0.998), (-3, 0.944), (-2, 0.717), (-1, 0.412), (0, 0.161),
                      (1, 0.047), (2, 0.015), (3, 0.006), (4, 0.003), (5, 0.001),
                      (6, 0.001), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L16 (1,1,0)
                (32, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 1.000), (-5, 1.000),
                      (-4, 1.000), (-3, 0.997), (-2, 0.952), (-1, 0.804), (0, 0.553),
                      (1, 0.245), (2, 0.089), (3, 0.020), (4, 0.010), (5, 0.003),
                      (6, 0.001), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L32 (1,1,0)
                (64, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 1.000), (-5, 1.000),
                      (-4, 1.000), (-3, 1.000), (-2, 0.999), (-1, 0.984), (0, 0.932),
                      (1, 0.793), (2, 0.483), (3, 0.217), (4, 0.068), (5, 0.020),
                      (6, 0.007), (7, 0.002), (8, 0.002), (9, 0.000)])  # L64 (1,1,0)
            ])  # Multipath Level 2 (1,1,0)
        ]  # SNR vs PER Lookup Table

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
    def transmit_duration(self) -> float:
        return self._transmit_duration

    def calculate_transmit_duration(self):
        """Calculate the packet transmit duration."""
        # 0.105 + ((payload_bytes + 16) * 2.0 * 50.0 / 8000.0)
        transmit_duration = 0.105
        if self._payload_length:
            transmit_duration = 0.105 + ((self._payload_length + 16.0) * 2.0 * 50.0 / 8000.0)

        return transmit_duration

    def get_snr_to_per_table(self, multipath_level=2):
        """Get the lookup table of SNR and packet error rate"""
        # Find nearest data_bytes value equal or greater in the lookup tables for given multipath.
        look_up_table = []
        for lut in self._snr_vs_per_results[multipath_level][1]:
            if self._payload_length <= lut[0]:
                # This one.
                look_up_table = lut[1]
                break

        return look_up_table

    def calculate_probability_of_delivery(self, receive_snr, multipath_level=2):
        """Calculate the probability of delivery."""

        probability_of_delivery = 1.0

        # Find nearest data_bytes value equal or greater in the lookup tables for given multipath.
        look_up_table = self.get_snr_to_per_table(multipath_level)

        # Search for packet error rate based on received SNR
        #
        # round SNR to nearest integer
        rounded_snr = int(round(receive_snr))
        if rounded_snr < look_up_table[0][0]:  # out of range?
            probability_of_delivery = 0.0
        elif rounded_snr > look_up_table[-1][0]:
            probability_of_delivery = 1.0  # short range?
        else:
            # read PER from table and convert to Pd
            start_snr = look_up_table[0][0]
            per = look_up_table[rounded_snr - start_snr][1]
            probability_of_delivery = 1.0 - per

        return probability_of_delivery

    def json(self):
        """Returns a json dictionary representation."""
        jason = {"FrameSynch": self._frame_synch,
                 "Address": self._address,
                 "Command": self._command,
                 "PayloadLength": self._payload_length,
                 "PayloadBytes": self._payload_bytes,
                 "HamrTimestamp": self._hamr_timestamp}

        return jason

    @staticmethod
    def from_json(jason): # -> Union[AcousticPacket, None]:
        acoustic_packet = AcousticPacket(frame_synch=jason["FrameSynch"],
                                         address=jason["Address"],
                                         command=jason["Command"],
                                         payload_length=jason["PayloadLength"],
                                         payload_bytes=jason["PayloadBytes"],
                                         hamr_timestamp=jason["HamrTimestamp"])
        return acoustic_packet
