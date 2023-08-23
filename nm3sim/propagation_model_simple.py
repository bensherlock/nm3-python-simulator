#!/usr/bin/env python
#
# Propagation Model Simple
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
from .propagation_model_base import PropagationModelBase


class PropagationModelSimple(PropagationModelBase):
    """Propagation Model for packet transmission between two nodes.
        Transmission losses and ambient noise are applied to calculate a receive SNR."""

    def __init__(self, speed_of_sound: float = 1500.0, attenuation_alpha: float = 6.0, noise_spectral_density: float = 50.0 ):
        """Constructor."""

        self._speed_of_sound = speed_of_sound  # m/s

         # Alpha. Attenuation coefficient in dB/km at 28 kHz (e.g. 6 for North Sea Summer,
        # 0.3 for Loch Ness Winter)
        self._attenuation_alpha = attenuation_alpha

        #  NSL. ambient noise spectral density in dB re 1uPa^2/Hz at Sea State 6
        #  (see Wenz Curves at 28 kHz)
        self._noise_spectral_density = noise_spectral_density

    def __call__(self):
        return self

    def calculate_propagation(self, source_node: NodeBase,
                              destination_node: NodeBase,
                              acoustic_packet: AcousticPacket = None):
        """Calculate the propagation delay of acoustic packet from source to destination node.
            Returns propagation_delay and updates the provided acoustic packet."""

        propagation_delay = PropagationModelBase.calculate_straight_line_propagation_delay(source_node, destination_node, self._speed_of_sound)

        if acoustic_packet:
            straight_line_range = PropagationModelBase.calculate_straight_line_range(source_node, destination_node)

            # transmission loss from spreading and attenuation
            transmission_loss = self.calculate_transmission_loss(straight_line_range)

            # estimated noise power from ambient NL
            noise_loss = self._noise_spectral_density + 10.0 * math.log10(acoustic_packet.band_width)


            # calculate receive SNR based on source level and the transmission and noise losses
            receive_snr = acoustic_packet.source_level - (transmission_loss + noise_loss)
            receive_sound_pressure_level = acoustic_packet.source_level - transmission_loss

            # update the acoustic packet (this should be a copy rather than a single packet as it will be modified for each channel)
            acoustic_packet.receive_snr = receive_snr
            acoustic_packet.receive_sound_pressure_level = receive_sound_pressure_level

        return propagation_delay

    def calculate_transmission_loss(self, straight_line_range):
        """Calculate Transmissions loss (dB) from spreading and attenuation."""
        transmission_loss = 0.0

        if straight_line_range > 0.0:
            # calculate transmission loss (free field spreading) TL
            transmission_loss = 20.0 * math.log10(straight_line_range) \
                                + self._attenuation_alpha * straight_line_range * 0.001

        return transmission_loss
