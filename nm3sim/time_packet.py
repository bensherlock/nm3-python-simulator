#!/usr/bin/env python
#
# Time Packet
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

class TimePacket:
    """Time Packet Class"""

    def __init__(self):
        # There...
        self._client_transmit_time = 0.0
        self._server_arrival_time = 0.0
        # ...and back again
        self._server_transmit_time = 0.0
        self._client_arrival_time = 0.0

    def __call__(self):
        return self

    @property
    def client_transmit_time(self):
        return self._client_transmit_time

    @client_transmit_time.setter
    def client_transmit_time(self, client_transmit_time):
        self._client_transmit_time = client_transmit_time

    @property
    def server_arrival_time(self):
        return self._server_arrival_time

    @server_arrival_time.setter
    def server_arrival_time(self, server_arrival_time):
        self._server_arrival_time = server_arrival_time

    @property
    def server_transmit_time(self):
        return self._server_transmit_time

    @server_transmit_time.setter
    def server_transmit_time(self, server_transmit_time):
        self._server_transmit_time = server_transmit_time

    @property
    def client_arrival_time(self):
        return self._client_arrival_time

    @client_arrival_time.setter
    def client_arrival_time(self, client_arrival_time):
        self._client_arrival_time = client_arrival_time


    @property
    def offset(self):
        time_offset = (self._server_arrival_time - self._client_transmit_time) \
                      + (self._server_transmit_time - self._client_arrival_time) / 2.0
        return time_offset

    @property
    def delay(self):
        round_trip_delay = (self._client_arrival_time - self._client_transmit_time) \
                           - (self._server_transmit_time - self._server_arrival_time)
        return round_trip_delay


    def calculate_offset(self):
        """Calculate the offset."""
        # Clock synchronization algorithm from
        # https://en.wikipedia.org/wiki/Network_Time_Protocol
        time_offset = (self._server_arrival_time - self._client_transmit_time) \
                      + (self._server_transmit_time - self._client_arrival_time) / 2.0
        round_trip_delay = (self._client_arrival_time - self._client_transmit_time) \
                           - (self._server_transmit_time - self._server_arrival_time)
        return time_offset

    def json(self):
        """Returns a json dictionary representation."""
        jason = {"ClientTransmitTime": self._client_transmit_time,
                 "ServerArrivalTime": self._server_arrival_time,
                 "ServerTransmitTime": self._server_transmit_time,
                 "ClientArrivalTime": self._client_arrival_time }
        return jason

    @staticmethod
    def from_json(jason): # -> Union[TimePacket, None]:
        time_packet = TimePacket()
        time_packet.client_transmit_time = jason["ClientTransmitTime"]
        time_packet.server_arrival_time = jason["ServerArrivalTime"]
        time_packet.server_transmit_time = jason["ServerTransmitTime"]
        time_packet.client_arrival_time = jason["ClientArrivalTime"]
        return time_packet

    def to_string(self):
        return "TimePacket:" \
               + "\n  client_transmit_time = " + str(self.client_transmit_time) \
               + "\n  server_arrival_time  = " + str(self.server_arrival_time) \
               + "\n  server_transmit_time = " + str(self.server_transmit_time) \
               + "\n  client_arrival_time  = " + str(self.client_arrival_time) \
               + "\n  offset               = " + str(self.offset) \
               + "\n  delay                = " + str(self.delay)

