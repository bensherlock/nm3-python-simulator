#!/usr/bin/env python
#
# Terminal Client
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
import os
os.system("")  # enables ansi escape characters in terminal

from queue import Queue
import sys
import time


from .modem import Modem
from .utils import *



from threading import Thread
class TtyWrapper:

    def __init__(self, stdin):
        self._stdin = stdin
        self._line = None

        self._thread = Thread(target=self._poll_stdin)
        self._thread.start()

    def readable(self):
        return self._stdin.readable()

    def read(self):
        bytes_input = None
        if self._line:
            bytes_input = self._line.encode('utf-8')
            self._line = None
            # Fire off another poll on stdin
            self._thread = Thread(target=self._poll_stdin)
            self._thread.start()

        return bytes_input

    def _poll_stdin(self):
        if self._stdin:
            self._line = self._stdin.readline()
            sys.stdout.write("\033[94m" + self._line.rstrip() + "\033[0m" + "\n") # echo to console
            #sys.stdout.write(self._line.rstrip() + "\n") # echo to console

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

class BinaryToTextStream:

    def __init__(self, output_text_stream):
        self._stream = output_text_stream

    def __getattr__(self, attr_name):
        return getattr(self._stream, attr_name)

    def writable(self):
        return self._stream.writable()

    def write(self, data):

        self._stream.write(data.decode('utf-8'))
        self._stream.flush()

    def flush(self):
        self._stream.flush()


class TimestampTextStreamFilter:
    """Intercepts a text stream and adds a timestamp at the beginning of each line."""

    def __init__(self, stream):
        self._stream = stream
        self._timestamp_next_time = True

    def __getattr__(self, attr_name):
        return getattr(self._stream, attr_name)

    def write(self, data):
        # Print timestamp first
        for c in data:
            if self._timestamp_next_time:
                self._timestamp_next_time = False
                timestamp_str = "[%d-%02d-%02d %02d:%02d:%02d] " % time.localtime()[:6]
                self._stream.write(timestamp_str)

            if c == '\n':
                self._timestamp_next_time = True
                self._stream.write("\n")
            else:
                self._stream.write(c)

        #self._stream.write(data)
        self._stream.flush()


    def flush(self):
        self._stream.flush()



def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(
        description='NM3 Network Simulator - Terminal Client. '
                    'Example usage: python -m nm3sim.terminal_client')

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

    # Label
    cmdline_parser.add_argument('--label',
                                help='The label for this node.')



    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    network_port = cmdline_args.network_port

    network_address = cmdline_args.network_address


    position_xy = (0.0,0.0)
    depth = 10.0
    if cmdline_args.position:
        position_xy, depth = cmdline_args.position

    label = cmdline_args.label

    address = 255
    if cmdline_args.address:
        address = cmdline_args.address





    #
    # Virtual NM3 Modem Mode (With stdin/stdout as interface)
    #

    input_stream = sys.stdin.buffer # bytes from a piped input
    if sys.stdin.isatty():
        sys.stdout = TimestampTextStreamFilter(sys.stdout) # Adds a timestamp to beginning of every line
        #sys.stderr = TimestampStreamFilter(sys.stderr)  # Adds a timestamp to beginning of every line
        input_stream = TtyWrapper(sys.stdin) # wrapped to grab lines and convert to bytes

    output_stream = BinaryToTextStream(sys.stdout)

    print("Starting NM3 Virtual Terminal Modem.")
    print("position=" + str(position_xy) + " depth=" + str(depth))

    # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
    modem = Modem(input_stream=input_stream,
                  output_stream=output_stream,
                  network_address=network_address,
                  network_port=network_port,
                  local_address=address,
                  position_xy=position_xy,
                  depth=depth,
                  label=label)

    try:
        modem.run()
    finally:
        print("Stopping NM3 Virtual Terminal Modem.")
        modem.stop()


if __name__ == '__main__':
    main()
