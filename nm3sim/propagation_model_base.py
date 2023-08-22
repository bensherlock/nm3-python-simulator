#!/usr/bin/env python
#
# Propagation Model Base
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

import math
from .node_base import NodeBase
from .acoustic_packet import AcousticPacket


class PropagationModelBase:
    """Propagation Model for packet transmission between two nodes.
    Extend this class and provide to the Controller."""

    def calculate_propagation(self, source_node: NodeBase,
                              destination_node: NodeBase,
                              acoustic_packet: AcousticPacket = None):
        """Calculate the propagation delay of acoustic packet from source to destination node.
        Returns propagation_delay and probability. (probability will always be 1.0)."""

        # TODO: This needs to change such that the probability is calculated by the receiving modem.
        #       Propagation model will provide propagation delay, and using the Source Level, band,
        #       and ambient noise will calculate the receive SNR based on transmission losses and noise.
        #       The controller will always forward on an acoustic packet to the modem/hydrophone node.
        #       The receive SNR will be updated in the provided AcousticPacket.

        x0 = source_node.position_xy[0]
        y0 = source_node.position_xy[1]
        z0 = source_node.depth

        x1 = destination_node.position_xy[0]
        y1 = destination_node.position_xy[1]
        z1 = destination_node.depth

        # Please note: This is a **very** simplistic model to just get us started.
        # Assuming no losses. And isovelocity. And no obstructions. And no multipath. And no noise.
        # The joy of simulation.

        straight_line_range = math.sqrt(((x1 - x0) * (x1 - x0))
                                        + ((y1 - y0) * (y1 - y0))
                                        + ((z1 - z0) * (z1 - z0)))
        speed_of_sound = 1500.0
        #print("straight_line_range=" + str(straight_line_range))
        propagation_delay = straight_line_range / speed_of_sound
        #print("propagation_delay=" + str(propagation_delay))

        return propagation_delay
