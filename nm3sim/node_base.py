#!/usr/bin/env python
#
# Node Base
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

from typing import Tuple

class NodeBase:
    """Node Base."""

    def __init__(self, unique_id):
        self._unique_id = unique_id

        self._address = 255
        self._depth = 100.0
        self._position_xy = (0.0, 0.0)


    def __call__(self):
        return self

    @property
    def unique_id(self):
        return self._unique_id


    @property
    def address(self) -> int:
        return self._address

    @address.setter
    def address(self, address: int):
        self._address = address

    @property
    def depth(self) -> float:
        return self._depth

    @depth.setter
    def depth(self, depth: float):
        self._depth = depth

    @property
    def position_xy(self) -> Tuple[float, float]:
        return self._position_xy

    @position_xy.setter
    def position_xy(self, position_xy: Tuple[float, float]):
        self._position_xy = position_xy
