#!/usr/bin/env python
#
# Controller
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
import math
import random
import time

from typing import Tuple, Union

from .acoustic_packet import AcousticPacket
from .node_base import NodeBase
from .node_packet import NodePacket
from .propagation_model_base import PropagationModelBase
from .time_packet import TimePacket


#import tornado
import zmq
from zmq.eventloop.zmqstream import ZMQStream
#from zmq.eventloop.ioloop import IOLoop
#from tornado.ioloop import IOLoop


def _debug_print(*args, **kwargs):
    """File local debug printing"""
    print(*args, **kwargs)
    pass


class Controller:
    """Controller. """

    def __init__(self, network_address=None, network_port=None, publish_port=None):
        self._network_address = network_address
        self._network_port = network_port
        self._socket = None

        self._publish_port = publish_port
        self._publish_socket = None

        # NTP Offset to synchronise timestamps
        self._ntp_offset = 0.0

        # Polling version
        self._socket_poller = None

        # Async version
        #self._socket_stream = None
        #self._socket_loop = None

        self._nodes = {} # Map unique_id to node

        self._propagation_model = PropagationModelBase() # Default model

        self._scheduled_network_packets = []

        self._startup_time = time.time()

    def __call__(self):
        return self


    @property
    def propagation_model(self) -> PropagationModelBase:
        return self._propagation_model

    @propagation_model.setter
    def propagation_model(self, propagation_model: PropagationModelBase):
        self._propagation_model = propagation_model


    def get_hamr_time(self, local_time=None):
        """Get Homogenous Acoustic Medium Relative time from either local_time or time.time()."""
        if local_time:
            hamr_time = self._ntp_offset + local_time
            return hamr_time
        else:
            hamr_time = self._ntp_offset + time.time()
            return hamr_time

    def get_local_time(self, hamr_time=None):
        """Get local time from Homogenous Acoustic Medium Relative time or time.time()."""
        if hamr_time:
            local_time = hamr_time - self._ntp_offset
            return local_time
        else:
            return time.time()


    def add_node(self, unique_id):
        """Add a new NodeBase to the controller.
        Return the unique id for this node."""
        self._nodes[unique_id] = NodeBase(unique_id)

        return unique_id

    def has_node(self, unique_id):
        """Check if Node exists for this unique_id."""
        return unique_id in self._nodes

    def delete_node(self, unique_id):
        """Delete the Node from the controller using the id."""
        self._nodes.pop(unique_id, None)


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

    def calculate_propagation(self, source_node: NodeBase,
                              destination_node: NodeBase,
                              acoustic_packet: AcousticPacket):
        """Calculate the propagation delay of acoustic packet from source to destination node.
        Uses the model provided by the user or a fallback isovelocity if model is None.
        Returns propagation_delay and probability."""

        if self._propagation_model:
            return self._propagation_model.calculate_propagation(
                source_node=source_node,
                destination_node=destination_node,
                acoustic_packet=acoustic_packet)

        # Fallback

        x0 = source_node.position_xy[0]
        y0 = source_node.position_xy[1]
        z0  = source_node.depth

        x1 = destination_node.position_xy[0]
        y1 = destination_node.position_xy[1]
        z1 = destination_node.depth

        # Please note: This is a **very** simplistic model to just get us started.
        # Assuming no losses. And isovelocity. And no obstructions. And no multipath. And no noise.
        # The joy of simulation.

        straight_line_range = math.sqrt(((x1-x0)*(x1-x0)) +  ((y1-y0)*(y1-y0)) + ((z1-z0)*(z1-z0)))
        speed_of_sound = 1500.0
        propagation_delay = straight_line_range / speed_of_sound

        return propagation_delay, 1.0



    def schedule_network_packet(self, transmit_time, unique_id, network_packet_json_string):
        """Schedule a network packet transmission."""
        self._scheduled_network_packets.append( (transmit_time, unique_id, network_packet_json_string) )
        self._scheduled_network_packets.sort(key=lambda tup: tup[0]) # sort by time


    def next_scheduled_network_packet(self, current_time):
        """Get the next scheduled network packet to be transmitted at the current time."""
        if self._scheduled_network_packets and self._scheduled_network_packets[0][0] <= current_time:
            scheduled_network_packet = self._scheduled_network_packets.pop(0)
            unique_id = scheduled_network_packet[1]
            network_packet_json_string = scheduled_network_packet[2]
            return unique_id, network_packet_json_string, scheduled_network_packet

        return None, None, None


    def on_recv(self, msg):
        """Callback handler for on_recv."""
        #unique_id, network_message_json_bytes = self._socket.recv_multipart(zmq.DONTWAIT)  # blocking
        unique_id = msg[0]
        network_message_json_bytes = msg[1]
        zmq_timestamp = float(msg[2].decode('utf-8'))

        local_received_time = time.time()
        _debug_print("NetworkPacket (len=" + str(len(msg)) + ") from " + str(unique_id) + " received at: " + str(local_received_time))
        network_message_json_str = network_message_json_bytes.decode('utf-8')
        network_message_jason = json.loads(network_message_json_str)

        # _debug_print("Network Packet received from: " + str(unique_id) + " -- " + network_message_json_str)


        if "TimePacket" in network_message_jason:
            _debug_print("TimePacket")
            # Quick Turnaround
            time_packet = TimePacket.from_json(network_message_jason["TimePacket"])
            time_packet.server_arrival_time = local_received_time
            #time_packet.client_transmit_time = zmq_timestamp

            time_packet.server_transmit_time = time.time()
            new_network_message_jason = {"TimePacket": time_packet.json()}
            new_network_message_json_str = json.dumps(new_network_message_jason)

            self._socket.send_multipart([unique_id, new_network_message_json_str.encode('utf-8'), str(time.time()).encode('utf-8')])


        if not self.has_node(unique_id):
            self.add_node(unique_id)

        if "NodePacket" in network_message_jason:
            _debug_print("NodePacket")
            if self._publish_socket:
                # Forward to loggers and visualisation clients
                self._publish_socket.send_multipart([b"NodePacket", unique_id, network_message_json_bytes, str(zmq_timestamp).encode('utf-8')])

            node = self.get_node(unique_id)
            if node:
                node_packet = NodePacket.from_json(network_message_jason["NodePacket"])
                node.position_xy = node_packet.position_xy
                node.depth = node_packet.depth
                node.label = node_packet.label
                self.update_node(node)

        if "AcousticPacket" in network_message_jason:
            _debug_print("AcousticPacket")
            if self._publish_socket:
                # Forward to loggers and visualisation clients
                self._publish_socket.send_multipart([b"AcousticPacket", unique_id, network_message_json_bytes, str(zmq_timestamp).encode('utf-8')])

            # Send to all nodes, except this one
            # For now this is instant: no propagation, or probability, or filtering.
            acoustic_packet = AcousticPacket.from_json(network_message_jason["AcousticPacket"])
            # Process Channels and Scheduling of acoustic_packet
            # This would need updating if the receiving node changes its position - work this out later.

            # Get source position Information
            source_node = self.get_node(unique_id)

            hamr_received_time = acoustic_packet.hamr_timestamp

            for socket_id in self._nodes:
                # Don't send to itself
                if socket_id != unique_id:
                    # Get destination position Information
                    destination_node = self.get_node(socket_id)

                    # Calculate propagation
                    acoustic_packet.hamr_timestamp = hamr_received_time
                    propagation_delay, probability = self.calculate_propagation(
                        source_node, destination_node,
                    acoustic_packet)


                    # Check probability
                    if random.random() < probability:
                        # Calculate a transmit_time
                        hamr_transmit_time = hamr_received_time + propagation_delay
                        local_transmit_time = self.get_local_time(hamr_transmit_time)

                        acoustic_packet.hamr_timestamp = hamr_transmit_time
                        new_network_message_jason = {"AcousticPacket": acoustic_packet.json()}
                        new_network_message_json_str = json.dumps(new_network_message_jason)
                        self.schedule_network_packet(local_transmit_time, socket_id,
                                                     new_network_message_json_str)
                        # IOLoop set the callback
                        #IOLoop.instance().call_at(local_transmit_time, self.check_for_packets_to_send)
            #for s in self._scheduled_network_packets:
            #    print(s)



    def check_for_packets_to_send(self):
        """Check for packets to send."""
        socket_id, network_packet_json_str, scheduled_network_packet = self.next_scheduled_network_packet(time.time())
        while socket_id:
            #_debug_print("Sending scheduled network packet: " + str(socket_id) + " - " + network_packet_json_str)
            self._socket.send_multipart([socket_id, network_packet_json_str.encode('utf-8'), str(time.time()).encode('utf-8')])
            sent_time = time.time()
            _debug_print("NetworkPacket to " + str(socket_id) + " transmitTime=" + str(scheduled_network_packet[0]) + " sent at: " + str(sent_time))
            # Get next scheduled network Packet
            socket_id, network_packet_json_str, scheduled_network_packet = self.next_scheduled_network_packet(time.time())

    def start(self):
        """Start the simulation. Bind to the address and port ready for
        virtual nodes."""
        if not self._network_address or not self._network_port:
            raise TypeError("Network Address/Port not set. Address("
                                    + self._network_address
                                    + ") Port( "
                                    + str(self._network_port) + ")" )
        if self._socket:
            # Already created?
            pass
        else:
            # https://stackoverflow.com/questions/38978804/zeromq-master-slave-two-way-communication
            # https://stackoverflow.com/questions/34242316/pyzmq-recv-json-cant-decode-message-sent-by-send-json
            context = zmq.Context()
            self._socket = context.socket(zmq.ROUTER)
            socket_string = "tcp://" + self._network_address + ":"+ str(self._network_port)
            print("Binding to: " + socket_string)
            self._socket.bind(socket_string)

            #print("HWM:" + str(self._socket.hwm))

            # Polling version
            self._socket_poller = zmq.Poller()
            self._socket_poller.register(self._socket, zmq.POLLIN)

            # Async version
            #self._socket_loop = IOLoop()
            #self._socket_stream = ZMQStream(self._socket)
            #self._socket_stream.on_recv(self.on_recv)

            #self._socket_loop.start()
            #IOLoop.instance().start() # Stays here

        if self._publish_socket:
            # Already created
            pass
        else:
            # Publish socket for subscribers such as loggers or visualisation
            context = zmq.Context()
            self._publish_socket = context.socket(zmq.PUB)
            socket_string = "tcp://" + self._network_address + ":"+ str(self._publish_port)
            print("Binding Publisher to: " + socket_string)
            self._publish_socket.bind(socket_string)


        while True:
            # Poll the socket
            # Router socket so first frame of multi part message is an identifier for the client.
            # Incoming Network Messages
            # NetworkMessage {
            # 1. AcousticPacket: { FrameSynch: Up/Dn, Address: 0-255, Command: 0-3, PayloadLength: 0-64, PayloadBytes: bytes(0-64) }
            # 2. NodePacket: { PositionXY: {x: float, y: float}, Depth: float }
            # }
            # Poll the socket for incoming messages
            # _debug_print("Checking socket poller")
            sockets = dict(self._socket_poller.poll(1))
            if self._socket in sockets:
                more_messages = True
                while more_messages:
                    try:
                        #unique_id, network_message_json_bytes = self._socket.recv_multipart(zmq.DONTWAIT) #  blocking
                        msg = self._socket.recv_multipart(zmq.DONTWAIT)  # blocking
                        self.on_recv(msg)
                    except zmq.ZMQError:
                        more_messages = False

            # Get next scheduled network Packet
            self.check_for_packets_to_send()




def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(
        description='NM3 Network Simulator - Default Controller. '
                    'Example usage: python -m nm3sim.controller')

    # Add Command Line Arguments
    # Network Port
    cmdline_parser.add_argument('--network_port',
                                help='The network port to connect to.', type=int)

    # Network Address
    cmdline_parser.add_argument('--network_address',
                                help='The network address to connect to.')


    # Publish Port
    cmdline_parser.add_argument('--publish_port',
                                help='The publish port to connect to.', type=int)

    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    network_port = cmdline_args.network_port

    network_address = cmdline_args.network_address

    publish_port = cmdline_args.publish_port



    #
    # Controller Mode
    #

    print("Starting Controller")
    print("zmq version: " + zmq.zmq_version())
    print("pyzmq version: " + zmq.pyzmq_version())

    controller = Controller(
        network_address=network_address, network_port=network_port, publish_port=publish_port)

    propagation_model = PropagationModelBase()
    controller.propagation_model = propagation_model

    controller.start()



if __name__ == '__main__':
    main()
