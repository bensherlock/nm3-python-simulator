#!/usr/bin/env python
#
# Map Visualisation Client
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



import argparse

import copy
import json
import matplotlib.pyplot as plt
from matplotlib.patches import Annulus
import time
from typing import Union
import zmq

from .acoustic_packet import AcousticPacket
from .modem_packet import ModemPacket
from .modem import Modem
from .node_base import NodeBase
from .node_packet import NodePacket


def _debug_print(*args, **kwargs):
    """File local debug printing"""
    #print(*args, **kwargs)
    pass


class MapVisualisation:
    """Map Visualisation Client."""

    def __init__(self, network_address=None, network_port=None):
        self._network_address = network_address
        self._network_port = network_port
        self._socket = None
        self._socket_poller = None

        # Offset to synchronise times
        self._hamr_time_offset = 0.0

        self._speed_of_sound = 1500.0  # To be provided from elsewhere.
        self._max_acoustic_propagation_range = 4000.0

        self._nodes = {}  # Map unique_id to node
        self._node_acoustic_packets = {}  # Map unique id to list of acoustic packets
        self._node_modem_states = {}  # Map unique id to modem state

        self._display_to_be_updated = True

        self._figure = None
        self._map_ax = None


    def get_hamr_time(self, local_time=None):
        """Get Homogenous Acoustic Medium Relative time from either local_time or time.time()."""
        if local_time:
            hamr_time = self._hamr_time_offset + local_time
            return hamr_time
        else:
            hamr_time = self._hamr_time_offset + time.time()
            return hamr_time

    def get_local_time(self, hamr_time=None):
        """Get local time from Homogenous Acoustic Medium Relative time or time.time()."""
        if hamr_time:
            local_time = hamr_time - self._hamr_time_offset
            return local_time
        else:
            return time.time()

    def add_node(self, unique_id):
        """Add a new NodeBase to the controller.
        Return the unique id for this node."""
        self._nodes[unique_id] = NodeBase(unique_id)
        self._node_acoustic_packets[unique_id] = []

        return unique_id

    def has_node(self, unique_id):
        """Check if Node exists for this unique_id."""
        return unique_id in self._nodes

    def delete_node(self, unique_id):
        """Delete the Node from the controller using the id."""
        self._nodes.pop(unique_id, None)
        self._node_acoustic_packets.pop(unique_id, None)


    def get_node(self, unique_id) \
            -> Union[NodeBase, None]:
        """Get a copy of the Nm3SimulatorNode from the controller by id.
        Returns a copy of the Nm3SimulatorNode or None."""
        if unique_id in self._nodes:
            return copy.deepcopy(self._nodes[unique_id])
        else:
            return None

    def update_node(self, node: NodeBase):
        unique_id = node.unique_id
        if unique_id in self._nodes:
            self._nodes[unique_id] = node
            self._display_to_be_updated = True

    def process_node_packet(self, node_id, node_packet):
        """Process incoming new node packets."""

        if not self.has_node(node_id):
            self.add_node(node_id)

        node = self.get_node(node_id)
        if node:
            node.position_xy = node_packet.position_xy
            node.depth = node_packet.depth
            node.label = node_packet.label
            self.update_node(node)



    def process_acoustic_packet(self, node_id, acoustic_packet):
        """Process incoming new acoustic packets from transmissions."""

        if not self.has_node(node_id):
            self.add_node(node_id)

        self._node_acoustic_packets[node_id].append(acoustic_packet)

        self._display_to_be_updated = True


    def process_modem_packet(self, node_id, modem_packet):
        """Process incoming new modem packets."""

        if not self.has_node(node_id):
            self.add_node(node_id)

        self._node_modem_states[node_id] = modem_packet.modem_state

        self._display_to_be_updated = True



    def create_display(self):
        """Create the display window."""
        plt.ion()  # interactive mode on
        self._figure = plt.figure()

        self._map_ax = self._figure.add_subplot(1, 1, 1)

        self._figure.show()
        self._figure.canvas.draw()

        self.update_display() # Draw


    def update_display(self):
        """Update the visual display if information has changed since last render."""

        if self._display_to_be_updated:
            #print("updating scatter")
            self._display_to_be_updated = False  # Clear flag

            self._map_ax.clear()

            # Ordered by node_id keys
            x_positions = [float(self._nodes[k].position_xy[0]) for k in self._nodes.keys()]
            y_positions = [float(self._nodes[k].position_xy[1]) for k in self._nodes.keys()]

            #x_positions = [float(n.position_xy[0]) for n in self._nodes.values()]
            #y_positions = [float(n.position_xy[1]) for n in self._nodes.values()]

            modem_state_colours = {
                Modem.MODEM_STATE_LISTENING: '#00ff00',
                Modem.MODEM_STATE_RECEIVING: '#00ffff',
                Modem.MODEM_STATE_TRANSMITTING: '#ffff00',
                Modem.MODEM_STATE_UARTING: '#444444',
                Modem.MODEM_STATE_SLEEPING: '#000000'
            }

            #states =


            if x_positions and y_positions:
                margin = 100.0
                # Need square aspect

                #xlim = [min(x_positions)-margin, max(x_positions)+margin]
                #ylim = [min(y_positions)-margin, max(y_positions)+margin]
                lim = [min(min(x_positions), min(y_positions))-margin, max(max(x_positions), max(y_positions))+margin]

                #print(x_positions)
                #print(y_positions)

                # Colours based on current modem states
                node_colors = []




                self._map_ax.scatter(x_positions, y_positions)
                self._map_ax.set_xlim(lim)
                self._map_ax.set_ylim(lim)
                self._map_ax.set_aspect(1)  # square
                self._map_ax.set_xlabel("x (m)")
                self._map_ax.set_ylabel("y (m)")

                for node in self._nodes.values():
                    if node.label:
                        self._map_ax.text(x=node.position_xy[0], y=node.position_xy[1], s=node.label)


            current_hamr_time = self.get_hamr_time()

            for node_id in self._nodes.keys():
                node = self._nodes[node_id]

                for acoustic_packet in self._node_acoustic_packets[node_id]:
                    self._display_to_be_updated = True # Still have acoustic packets to display

                    outer_diameter = (current_hamr_time - acoustic_packet.hamr_timestamp) * self._speed_of_sound
                    inner_diameter = (current_hamr_time - acoustic_packet.hamr_timestamp - acoustic_packet.transmit_duration) * self._speed_of_sound

                    circle_color = '#F0F0F0A0'

                    if inner_diameter < 0.0:
                        # Still transmitting
                        circle = plt.Circle(node.position_xy, outer_diameter, fill=True, color=circle_color)
                        self._map_ax.add_artist(circle)
                    else:
                        # Finished transmitting
                        width = outer_diameter - inner_diameter
                        annulus = Annulus(node.position_xy, outer_diameter, width, 0.0, fill=True, color=circle_color )
                        self._map_ax.add_artist(annulus)


                    if outer_diameter > self._max_acoustic_propagation_range:
                        self._node_acoustic_packets[node_id].remove(acoustic_packet)



            # update visualization
            #self._figure.canvas.draw()
            self._figure.canvas.draw_idle()
            self._figure.canvas.flush_events()

        plt.pause(0.001)


    def run(self):
        """Run the client. Never returns."""
        # Connect to the controller
        if not self._network_address or not self._network_port:
            raise TypeError("Network Address/Port not set. Address("
                                    + self._network_address
                                    + ") Port( "
                                    + str(self._network_port) + ")" )
        if self._socket:
            # Already created?
            pass
        else:
            context = zmq.Context()
            self._socket = context.socket(zmq.SUB)
            self._socket.connect("tcp://" + self._network_address + ":"
                              + str(self._network_port))
            #self._socket.subscribe(b"NodePacket")
            #self._socket.subscribe(b"AcousticPacket")
            self._socket.setsockopt(zmq.SUBSCRIBE, b"NodePacket")
            self._socket.setsockopt(zmq.SUBSCRIBE, b"AcousticPacket")
            self._socket.setsockopt(zmq.SUBSCRIBE, b"ModemPacket")

            self._socket_poller = zmq.Poller()
            self._socket_poller.register(self._socket, zmq.POLLIN)


        # Create the display
        self.create_display()

        while True:
            # Poll the socket for incoming "acoustic" messages
            #_debug_print("Checking socket poller")
            sockets = dict(self._socket_poller.poll(1))
            if self._socket in sockets:
                more_messages = True
                while more_messages:
                    try:
                        msg = self._socket.recv_multipart(zmq.DONTWAIT)

                        local_received_time = time.time()
                        _debug_print("NetworkPacket (len=" + str(len(msg)) + ") from Controller received at: " + str(local_received_time))

                        topic = msg[0]
                        node_id = msg[1]
                        network_message_json_bytes = msg[2]
                        zmq_timestamp = float(msg[3].decode('utf-8'))

                        # Update the time offset
                        self._hamr_time_offset = zmq_timestamp - local_received_time


                        network_message_json_str = network_message_json_bytes.decode('utf-8')
                        network_message_jason = json.loads(network_message_json_str)

                        _debug_print("Network Packet received: " + network_message_json_str)

                        if "NodePacket" in network_message_jason:
                            # Process the NodePacket
                            node_packet = NodePacket.from_json(network_message_jason["NodePacket"])
                            self.process_node_packet(node_id, node_packet)

                        if "AcousticPacket" in network_message_jason:
                            # Process the AcousticPacket
                            acoustic_packet = AcousticPacket.from_json(network_message_jason["AcousticPacket"])
                            self.process_acoustic_packet(node_id, acoustic_packet)

                        if "ModemPacket" in network_message_jason:
                            # Process the ModemPacket
                            modem_packet = ModemPacket.from_json(network_message_jason["ModemPacket"])
                            self.process_modem_packet(node_id, modem_packet)
                            pass

                    except zmq.ZMQError:
                        more_messages = False


            # Update display
            self.update_display()


def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(
        description='NM3 Network Simulator - Map Visualisation Client. '
                    'Example usage: python -m nm3sim.mapvis_client')

    # Add Command Line Arguments
    # Network Port
    cmdline_parser.add_argument('--network_port',
                                help='The network port to connect to.', type=int)

    # Network Address
    cmdline_parser.add_argument('--network_address',
                                help='The network address to connect to.')




    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    network_port = cmdline_args.network_port

    network_address = cmdline_args.network_address





    #
    # Map Visualisation Client
    #

    # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
    client = MapVisualisation(network_address=network_address,
                            network_port=network_port  )

    client.run()


if __name__ == '__main__':
    main()

