#!/usr/bin/env python
#
# Serial Client
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
import serial

from .modem import Modem
from .utils import *



def main():
    """Main Program Entry."""
    cmdline_parser = argparse.ArgumentParser(
        description='NM3 Network Simulator - Serial Client. '
                    'Example usage: python -m nm3sim.serial_client')

    # Add Command Line Arguments
    # Network Port
    cmdline_parser.add_argument('--network_port',
                                help='The network port to connect to.', type=int)

    # Network Address
    cmdline_parser.add_argument('--network_address',
                                help='The network address to connect to.')


    # Serial Port
    cmdline_parser.add_argument('--serial_port', help='The serial port to connect to.')

    # Local Address
    cmdline_parser.add_argument('--address', help='The local node address on start.', type=int)

    # Virtual Modem Node Position
    cmdline_parser.add_argument('--position', help="Node position as (x,y,depth).", dest="position",
                                type=node_position_parser)



    # Parse the command line
    cmdline_args = cmdline_parser.parse_args()

    # Get Arguments
    network_port = cmdline_args.network_port

    network_address = cmdline_args.network_address

    position_xy = (0.0,0.0)
    depth = 10.0
    if cmdline_args.position:
        position_xy, depth = cmdline_args.position

    serial_port_name = cmdline_args.serial_port

    address = 255
    if cmdline_args.address:
        address = cmdline_args.address




    #
    # Serial Port NM3 Virtual Modem Modem
    #

    # Serial Port is opened with a 100ms timeout for reading.
    with serial.Serial(serial_port_name, 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1) as serial_port:

        # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
        modem = Modem(input_stream=serial_port,
                        output_stream=serial_port,
                        network_address=network_address,
                        network_port=network_port,
                        local_address=address,
                        position_xy=position_xy,
                        depth=depth)

        modem.run()


if __name__ == '__main__':
    main()
