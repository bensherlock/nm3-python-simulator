#!/usr/bin/env python
#
# Modem Packet
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

#from .modem import Modem


class ModemPacket:
    """Modem Packet class."""

    def __init__(self, modem_state=None, modem_event=None, label=None):
        self._modem_state = modem_state
        self._modem_event = modem_event
        self._label = label

    @property
    def modem_state(self):
        return self._modem_state

    @modem_state.setter
    def modem_state(self, modem_state):
        self._modem_state = modem_state

    @property
    def modem_event(self):
        return self._modem_event

    @modem_event.setter
    def modem_event(self, modem_event):
        self._modem_event = modem_event

    @property
    def label(self) -> str:
        return self._label

    @label.setter
    def label(self, label: str):
        self._label = label

    def json(self):
        """Returns a json dictionary representation."""
        jason = {"ModemState": self._modem_state,
                 "ModemEvent": self._modem_event, "Label": self._label}
        return jason

    @staticmethod
    def from_json(jason): # -> Union[ModemPacket, None]:
        modem_packet = ModemPacket(modem_state=jason["ModemState"], modem_event=jason["ModemEvent"],
                                  label=jason["Label"])

        return modem_packet

