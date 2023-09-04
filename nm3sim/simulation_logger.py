#!/usr/bin/env python
#
# Simulation Logger
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
import time
from typing import Union
import zmq

from .acoustic_packet import AcousticPacket
from .modem_packet import ModemPacket
from .node_packet import NodePacket


def _debug_print(*args, **kwargs):
    """File local debug printing"""
    #print(*args, **kwargs)
    pass

class SimulationLogger:
    """Simulation Logger Client."""

    def __init__(self, network_address=None, network_port=None, log_filename=None):
        self._network_address = network_address
        self._network_port = network_port
        self._socket = None
        self._socket_poller = None

        self._log_filename = log_filename
        self._log_file = None
        self._log_file_first_entry = True

        self._is_running = False


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

    def log_packet(self, transmitted_simulation_time, received_simulation_time, unique_id, network_message_jason):
        """Log the packet to file for analysis or playback."""
        if self._log_file:
            log_entry_jason = {
                "TransmittedSimulationTime": transmitted_simulation_time,
                "ReceivedSimulationTime": received_simulation_time,
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


        # Open the log file
        if self._log_filename:
            try:
                # 0=no buffer, 1=Line buffering, N=bytes buffering, None=system default
                bufsize = 1
                self._log_file = open(self._log_filename, "w", buffering=bufsize)

                self.log_header()

            except Exception as ex:
                print(ex)
                pass

        try:
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
                            transmitted_simulation_timestamp = float(msg[3].decode('utf-8'))
                            received_simulation_time = float(msg[4].decode('utf-8'))

                            network_message_json_str = network_message_json_bytes.decode('utf-8')
                            network_message_jason = json.loads(network_message_json_str)

                            _debug_print("Network Packet received: " + network_message_json_str)

                            # Log received NetworkMessages.
                            self.log_packet(transmitted_simulation_timestamp, received_simulation_time, node_id,
                                            network_message_jason)

                        except zmq.ZMQError:
                            more_messages = False

        finally:
            # Shutting down

            # Close files
            self.log_footer()
            if self._log_file:
                self._log_file.close()
                self._log_file = None




def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(
        description='NM3 Network Simulator - Simulation Logger Client. '
                    'Example usage: python -m nm3sim.simulation_logger')

    # Add Command Line Arguments
    # Network Port
    cmdline_parser.add_argument('--network_port',
                                help='The network port to connect to.', type=int)

    # Network Address
    cmdline_parser.add_argument('--network_address',
                                help='The network address to connect to.')

    # Log Filename
    cmdline_parser.add_argument('--log_filename',
                                help='The filename to write the logs to.')




    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    network_port = cmdline_args.network_port

    network_address = cmdline_args.network_address

    log_filename = cmdline_args.log_filename





    #
    # Simulation Logger Client
    #

    # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
    client = SimulationLogger(network_address=network_address,
                              network_port=network_port,
                              log_filename=log_filename)

    client.run()


if __name__ == '__main__':
    main()



