#!/usr/bin/env python
#
# Node Packet
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

class NodePacket:
    """Node Packet class."""


    def __init__(self, position_xy=(0.0,0.0), depth=10.0, label=None):
        self._position_xy = position_xy
        self._depth = depth
        self._label = label

    @property
    def position_xy(self):
        return self._position_xy

    @position_xy.setter
    def position_xy(self, position_xy):
        self._position_xy = position_xy

    @property
    def depth(self):
        return self._depth

    @depth.setter
    def depth(self, depth):
        self._depth = depth

    @property
    def label(self) -> str:
        return self._label

    @label.setter
    def label(self, label: str):
        self._label = label


    def json(self):
        """Returns a json dictionary representation."""
        jason = {"PositionXY": { "x": self._position_xy[0], "y": self._position_xy[1] },
                 "Depth": self._depth, "Label": self._label}
        return jason

    @staticmethod
    def from_json(jason): # -> Union[NodePacket, None]:
        node_packet = NodePacket(position_xy=(jason["PositionXY"]["x"], jason["PositionXY"]["y"]),
                                 depth=jason["Depth"], label=jason["Label"])

        return node_packet

