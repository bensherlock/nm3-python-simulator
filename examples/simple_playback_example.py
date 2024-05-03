#!/usr/bin/env python
#
# Simple Playback Example
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


from nm3sim.modem import Modem
from nm3sim.utils import *

from nm3sim.controller import Controller, ControllerMode
from nm3sim.propagation_model_simple import PropagationModelSimple
from nm3sim.terminal_client import *
from nm3sim.mapvis_client import MapVisualisation
from nm3sim.simulation_logger import SimulationLogger

from nm3driver.nm3driver import Nm3, MessagePacket

from _datetime import datetime
import signal
from threading import Thread
from multiprocessing import Process


def start_controller(network_address, network_port, publish_port, log_filename):
    #
    # Controller
    #

    controller = Controller(
        network_address=network_address, network_port=network_port, publish_port=publish_port, log_filename=log_filename, controller_mode= ControllerMode.Playback)

    propagation_model = PropagationModelSimple(speed_of_sound=1500.0, attenuation_alpha=4.0, noise_spectral_density=50.0)
    controller.propagation_model = propagation_model

    try:
        controller.start()
    finally:
        controller.stop()


def start_mapvis(network_address, network_port):
    #
    # Map Visualisation
    #

    client = MapVisualisation(network_address=network_address,
                              network_port=network_port)
    try:
        client.run()
    finally:
        client.stop()



def main():
    """Main Program Entry."""

    # Using separate processes for each of the independent nodes and controller - Controller, Modem clients, etc

    network_address = "127.0.0.1"
    network_port = 8080
    publish_port = 8081

    # Create and run a Controller in Playback mode
    log_filename = "SimpleExampleLogs-20240415T104239.json"
    print("Starting the Controller")
    controller_process = Process(target=start_controller, args=(network_address, network_port, publish_port,
                                                                log_filename))
    controller_process.start()

    # Wait for it to startup before proceeding
    time.sleep(5.0)

    # Create and run the MapVisualisation
    print("Starting the MapVisualisation")
    mapvis_process = Process(target=start_mapvis, args=(network_address, publish_port))
    mapvis_process.start()


    # Run until the controller finishes
    controller_process.join()




if __name__ == '__main__':
    main()
