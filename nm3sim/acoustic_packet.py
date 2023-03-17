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

    def __init__(self, frame_synch=FRAMESYNCH_UP, address=255, command=CMD_PING_REQ, payload_length=0, payload_bytes=None, hamr_timestamp=0.0):
        #AcousticPacket: { FrameSynch: Up/Dn, Address: 0-255, Command: 0-3, PayloadLength: 0-64, PayloadBytes: bytes(0-64) }
        self._frame_synch = frame_synch
        self._address = address
        self._command = command
        self._payload_length = payload_length
        self._payload_bytes = payload_bytes
        self._hamr_timestamp = hamr_timestamp


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
