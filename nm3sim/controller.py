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
from enum import Enum
import json
import math
import random
import time

from typing import Tuple, Union

from .acoustic_packet import AcousticPacket
from .node_base import NodeBase
from .node_packet import NodePacket
from .propagation_model_base import PropagationModelBase
from .propagation_model_simple import PropagationModelSimple
from .time_packet import TimePacket


#import tornado
import zmq
from zmq.eventloop.zmqstream import ZMQStream
#from zmq.eventloop.ioloop import IOLoop
#from tornado.ioloop import IOLoop


def _debug_print(*args, **kwargs):
    """File local debug printing"""
    #print(*args, **kwargs)
    pass


class ControllerMode(Enum):
    """Controller Operating Mode. Live or Playback of json logs"""
    Live = 0,
    Playback = 1

class Controller:
    """Controller. """

    def __init__(self, network_address=None, network_port=None, publish_port=None, log_filename=None, controller_mode:ControllerMode=ControllerMode.Live, playback_start_time=0.0, playback_speed=1.0):
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

        self._controller_mode = controller_mode

        self._is_running = False

        self._nodes = {}  # Map unique_id to node

        self._propagation_model = PropagationModelBase()  # Default model

        self._scheduled_network_packets = []

        self._startup_time = time.time()

        self._log_filename = log_filename
        self._log_file = None
        self._log_file_first_entry = True

        if self._log_filename:
            try:
                if self._controller_mode == ControllerMode.Live:
                    # open in write mode
                    # 0=no buffer, 1=Line buffering, N=bytes buffering, None=system default
                    bufsize = 1
                    self._log_file = open(self._log_filename, "w", buffering=bufsize)

                    # Write the header
                    self.log_header()

                elif self._controller_mode == ControllerMode.Playback:
                    # Open in read mode
                    self._log_file = open(self._log_filename, "r")

            except Exception as ex:
                print(ex)
                pass

        self._playback_start_time = playback_start_time
        self._playback_speed = playback_speed


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
        Uses the model provided by the user.
        Returns propagation_delay and updates the provided acoustic packet."""

        if self._propagation_model:
            return self._propagation_model.calculate_propagation(
                source_node=source_node,
                destination_node=destination_node,
                acoustic_packet=acoustic_packet)

        return 0.0

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

    def log_header(self):
        """Write the header of the log file."""
        if self._log_file:
            self._log_file.write('{ "LogEntries": [\n')
            self._log_file_first_entry = True

    def log_footer(self):
        """Write the footer of the log file."""
        if self._log_file:
            self._log_file.write("\n")
            self._log_file.write('] }')

    def log_packet(self, simulation_time, zmq_timestamp, unique_id, network_message_jason):
        """Log the packet to file for analysis or playback"""
        if self._log_file:
            log_entry_jason = {
                "SimulationTime": simulation_time,
                "ZmqTimestamp": zmq_timestamp,
                "UniqueId": list(unique_id),
                "NetworkMessage": network_message_jason
            }
            log_entry_jason_str = json.dumps(log_entry_jason)

            if self._log_file_first_entry:
                self._log_file_first_entry = False
            else:
                self._log_file.write(",\n")

            self._log_file.write(log_entry_jason_str)


        pass

    def on_recv(self, msg):
        """Callback handler for on_recv."""
        #unique_id, network_message_json_bytes = self._socket.recv_multipart(zmq.DONTWAIT)  # blocking
        unique_id = msg[0]  # bytes object containing 5 or 4 bytes
        network_message_json_bytes = msg[1]  # bytes array of the json string
        zmq_timestamp = float(msg[2].decode('utf-8'))  # bytes array of the float string

        local_received_time = time.time()
        simulation_time = local_received_time - self._startup_time
        _debug_print("NetworkPacket (len=" + str(len(msg)) + ") from " + str(unique_id) + " received at: " + str(local_received_time))
        network_message_json_str = network_message_json_bytes.decode('utf-8')
        network_message_jason = json.loads(network_message_json_str)

        # _debug_print("Network Packet received from: " + str(unique_id) + " -- " + network_message_json_str)

        # Log received NetworkMessages.
        #self.log_packet(simulation_time, zmq_timestamp, unique_id, network_message_jason)

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
            # Log received NetworkMessages.
            self.log_packet(simulation_time, zmq_timestamp, unique_id, network_message_jason)

            if self._publish_socket:
                # Forward to loggers and visualisation clients
                self._publish_socket.send_multipart([b"NodePacket", unique_id, network_message_json_bytes, str(zmq_timestamp).encode('utf-8'), str(simulation_time).encode('utf-8'), str(self._playback_speed).encode('utf-8')])

            node = self.get_node(unique_id)
            if node:
                node_packet = NodePacket.from_json(network_message_jason["NodePacket"])
                node.position_xy = node_packet.position_xy
                node.depth = node_packet.depth
                node.label = node_packet.label
                self.update_node(node)

        if "AcousticPacket" in network_message_jason:
            _debug_print("AcousticPacket")
            # Log received NetworkMessages.
            self.log_packet(simulation_time, zmq_timestamp, unique_id, network_message_jason)

            if self._publish_socket:
                # Forward to loggers and visualisation clients
                self._publish_socket.send_multipart([b"AcousticPacket", unique_id, network_message_json_bytes, str(zmq_timestamp).encode('utf-8'), str(simulation_time).encode('utf-8'), str(self._playback_speed).encode('utf-8')])

            # Send to all nodes, except the transmitting node
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

                    # Take a copy of the acoustic packet
                    acoustic_packet_copy = copy.deepcopy(acoustic_packet)

                    # Calculate propagation
                    propagation_delay = self.calculate_propagation(source_node, destination_node, acoustic_packet_copy)

                    # Calculate a transmit_time
                    hamr_transmit_time = hamr_received_time + propagation_delay
                    local_transmit_time = self.get_local_time(hamr_transmit_time)

                    acoustic_packet_copy.hamr_timestamp = hamr_transmit_time
                    new_network_message_jason = {"AcousticPacket": acoustic_packet_copy.json()}
                    new_network_message_json_str = json.dumps(new_network_message_jason)
                    self.schedule_network_packet(local_transmit_time, socket_id,
                                                 new_network_message_json_str)
                    # IOLoop set the callback
                    #IOLoop.instance().call_at(local_transmit_time, self.check_for_packets_to_send)
            #for s in self._scheduled_network_packets:
            #    print(s)

        if "ModemPacket" in network_message_jason:
            _debug_print("ModemPacket")
            # Log received NetworkMessages.
            self.log_packet(simulation_time, zmq_timestamp, unique_id, network_message_jason)

            if self._publish_socket:
                # Forward to loggers and visualisation clients
                self._publish_socket.send_multipart([b"ModemPacket", unique_id, network_message_json_bytes, str(zmq_timestamp).encode('utf-8'), str(simulation_time).encode('utf-8'), str(self._playback_speed).encode('utf-8')])
            pass


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



    def run_live_mode(self):
        """Run in Live mode"""
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



        try:
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

                # Yield the thread
                time.sleep(0)

                pass

        finally:
            # Shutting down

            # Close the log file
            self.log_footer()
            if self._log_file:
                self._log_file.close()
                self._log_file = None



    def run_playback_mode(self):
        """Run in Playback mode. This mode takes the log entries and at the right time sends them to the publish port.
        There is no live interaction from modems, it is for analysing behaviour by using the client visualisations
        such as mapvis."""

        if not self._log_file:
            raise ValueError("Log file not provided for Playback")

        # Load log file as json
        log_file_jason = json.load(self._log_file)

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

        # Wait for clients to connect before commencing playback
        print("Waiting for 10 seconds for clients to connect")
        time.sleep(10.0)
        print("Starting Playback")

        # Run the system now
        simulation_start_time = time.time() - self._playback_start_time
        for log_entry in log_file_jason["LogEntries"]:

            # Prepare the parameters
            simulation_time = log_entry["SimulationTime"]
            unique_id = bytes(log_entry["UniqueId"])
            zmq_timestamp = log_entry["ZmqTimestamp"]
            network_message_jason = log_entry["NetworkMessage"]
            network_message_json_str = json.dumps(network_message_jason)
            network_message_json_bytes = network_message_json_str.encode('utf-8')

            topic = b"Packet"
            if "NodePacket" in network_message_jason:
                topic = b"NodePacket"
            elif "AcousticPacket" in network_message_jason:
                topic = b"AcousticPacket"
            elif "ModemPacket" in network_message_jason:
                topic = b"ModemPacket"

            # Wait until half time
            transmit_actual_time = simulation_start_time + (simulation_time/self._playback_speed)

            while transmit_actual_time > time.time():
                time.sleep( (transmit_actual_time - time.time()) / 2.0 )  # yield
                pass


            # Now publish
            if self._publish_socket:
                # Forward to loggers and visualisation clients
                self._publish_socket.send_multipart([topic, unique_id, network_message_json_bytes, str(zmq_timestamp).encode('utf-8'), str(simulation_time).encode('utf-8'), str(self._playback_speed).encode('utf-8')])

            pass

        # Playback complete
        print("Playback Complete")
        if self._log_file:
            self._log_file.close()

    def start(self):
        """Start the simulation. Bind to the address and port ready for
        virtual nodes."""

        if self._controller_mode == ControllerMode.Live:
            print("Starting Controller: Live mode")
            self.run_live_mode()

        elif self._controller_mode == ControllerMode.Playback:
            print("Starting Controller: Playback mode")
            self.run_playback_mode()



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

    # Log Filename
    cmdline_parser.add_argument('--log_filename',
                                help='The log filename for logging to in live mode or playback from.')

    # Controller Mode
    cmdline_parser.add_argument('--controller_mode',
                                help='The controller mode: live/playback',
                                choices=['live', 'playback'],
                                default='live')

    cmdline_parser.add_argument('--playback_start_time',
                                help='The simulation time to start playback from', type=float, default=0.0)

    cmdline_parser.add_argument('--playback_speed',
                                help='The simulation speed to playback the logs', type=float, default=1.0)

    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    network_port = cmdline_args.network_port

    network_address = cmdline_args.network_address

    publish_port = cmdline_args.publish_port

    log_filename = cmdline_args.log_filename


    controller_mode_str = cmdline_args.controller_mode

    controller_mode = None
    if controller_mode_str == 'live':
        controller_mode = ControllerMode.Live
    elif controller_mode_str == 'playback':
        controller_mode = ControllerMode.Playback

    playback_start_time = cmdline_args.playback_start_time

    playback_speed = cmdline_args.playback_speed

    #
    # Controller
    #

    print("Starting Controller")
    print("zmq version: " + zmq.zmq_version())
    print("pyzmq version: " + zmq.pyzmq_version())

    controller = Controller(
        network_address=network_address, network_port=network_port, publish_port=publish_port,
        log_filename=log_filename, controller_mode=controller_mode, playback_start_time=playback_start_time,
        playback_speed=playback_speed)

    propagation_model = PropagationModelSimple()
    controller.propagation_model = propagation_model

    controller.start()



if __name__ == '__main__':
    main()
