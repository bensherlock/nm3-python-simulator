#!/usr/bin/env python
#
# Logger Client
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
from queue import Queue
from threading import Thread
from .modem import Modem
from .utils import *
from nm3driver.nm3logger import Nm3Logger
from nm3driver.nm3driver import Nm3

def node_position_parser(s):
    """Expects arguments as (x,y,depth)"""
    try:
        vals = s.split(",")
        x = float(vals[0])
        y = float(vals[1])
        depth = float(vals[2])
        return (x, y), depth
    except:
        raise argparse.ArgumentTypeError("Node parameters must be x,y,depth")


class BufferedIOQueueWrapper:
    """Wraps a Queue as IO with Read and Write binary functions."""

    def __init__(self):
        self._the_queue = Queue()

    def readable(self):
        return True

    def read(self, n = -1):
        """Read up to n bytes"""
        the_bytes = []
        while not self._the_queue.empty() and (n == -1 or len(the_bytes) < n):
            the_bytes.append(self._the_queue.get())

        return bytes(the_bytes)

    def writable(self):
        return True

    def write(self, the_bytes: bytes):
        """Returns the number of bytes written."""
        for b in the_bytes:
            self._the_queue.put(b)

        return len(the_bytes)

    def flush(self):
        """Nothing to flush to."""
        pass


def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(
        description='NM3 Network Simulator - Logger Client. '
                    'Example usage: python -m nm3sim.logger_client')

    # Add Command Line Arguments
    # Network Port
    cmdline_parser.add_argument('--network_port',
                                help='The network port to connect to.', type=int)

    # Network Address
    cmdline_parser.add_argument('--network_address',
                                help='The network address to connect to.')


    # Local Address
    cmdline_parser.add_argument('--address', help='The local node address on start.', type=int)

    # Virtual Modem Node Position
    cmdline_parser.add_argument('--position', help="Node position as (x,y,depth).", dest="position",
                                type=node_position_parser)

    # filenameroot for Nm3Logger
    cmdline_parser.add_argument('--filename_root', help='The filename root to log to.')


    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    network_port = cmdline_args.network_port

    network_address = cmdline_args.network_address


    position_xy = (0.0,0.0)
    depth = 10.0
    if cmdline_args.position:
        position_xy, depth = cmdline_args.position


    address = 255
    if cmdline_args.address:
        address = cmdline_args.address

    filename_root = "Nm3Log"
    if cmdline_args.filename_root:
        filename_root = cmdline_args.filename_root


    #
    # NM3 Logger on a Virtual Modem
    #


    # Pipes
    outgoing_stream = BufferedIOQueueWrapper()
    incoming_stream = BufferedIOQueueWrapper()

    nm3_driver = Nm3(input_stream=incoming_stream, output_stream=outgoing_stream)

    # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
    modem = Modem(input_stream=outgoing_stream,
                    output_stream=incoming_stream,
                    network_address=network_address,
                    network_port=network_port,
                    local_address=address,
                    position_xy=position_xy,
                    depth=depth)
    a_thread = Thread(target=modem.run)
    a_thread.start()  # nm3_modem.run()

    nm3_logger = Nm3Logger()

    nm3_logger.start_logging(nm3_modem=nm3_driver, filename_root=filename_root)


if __name__ == '__main__':
    main()
