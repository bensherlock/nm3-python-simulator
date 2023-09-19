#!/usr/bin/env python
#
# Transponder Client
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

from nm3sim.controller import Controller
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
        network_address=network_address, network_port=network_port, publish_port=publish_port, log_filename=log_filename)

    propagation_model = PropagationModelSimple(speed_of_sound=1500.0, attenuation_alpha=4.0, noise_spectral_density=50.0)
    controller.propagation_model = propagation_model

    try:
        controller.start()
    finally:
        controller.stop()


def start_logger(network_address, network_port, log_filename):
    #
    # Simulation Logger
    #

    client = SimulationLogger(network_address=network_address,
                              network_port=network_port,
                              log_filename=log_filename)

    try:
        client.run()
    finally:
        client.stop()


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



def start_transponder(network_address, network_port, address, position_xy, depth, label):
    #
    # Transponder NM3 Virtual Modem Modem
    #

    # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
    modem = Modem(input_stream=None,
                  output_stream=None,
                  network_address=network_address,
                  network_port=network_port,
                  local_address=address,
                  position_xy=position_xy,
                  depth=depth,
                  label=label)
    try:
        modem.run()
    finally:
        modem.stop()





def start_gateway_node(network_address, network_port, address, position_xy, depth, label):
    #
    # Gateway Node NM3 Virtual Modem Modem
    #

    input_stream = BufferedIOQueueWrapper()
    output_stream = BufferedIOQueueWrapper()


    # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
    modem = Modem(input_stream=input_stream,
                  output_stream=output_stream,
                  network_address=network_address,
                  network_port=network_port,
                  local_address=address,
                  position_xy=position_xy,
                  depth=depth,
                  label=label)

    nm3_modem = Nm3(input_stream=output_stream, output_stream=input_stream)

    modem_thread = Thread(target=modem.run)
    #modem.run()
    modem_thread.start()

    # Address of network nodes start at 100
    # TDMA offset time for response (after beacon) is (address - 100) * tdma_window_size
    tdma_window_size = 4.0
    sensor_node_count = 4
    frame_interval = 30.0
    last_frame_time = time.time()

    #tdma_offset = (address - 100) * tdma_window_size

    # Now here run a while loop to process the modem serial comms
    try:
        while True:
            if time.time() >= last_frame_time + frame_interval:
                # Send the beacon
                last_frame_time = time.time()
                nm3_modem.send_broadcast_message(b'BEACON')


            nm3_modem.poll_receiver() # non-blocking returns immediately if no bytes ready to read.

            # Periodically process any bytes received
            nm3_modem.process_incoming_buffer()

            # Periodically check for received packets
            while nm3_modem.has_received_packet():
                message_packet = nm3_modem.get_received_packet()

                print('Gateway Received message packet: ' +
                      MessagePacket.PACKETTYPE_NAMES[message_packet.packet_type] +
                      ' src: ' + str(message_packet.source_address) +
                      ' dest: ' + str(message_packet.destination_address) +
                      ' payload: ' + str(message_packet.packet_payload) +
                      ' payload text: ' + bytes(message_packet.packet_payload).decode('utf-8'))

            # Yield the thread
            time.sleep(0)

            pass

    finally:
        modem.stop()



def start_sensor_node(network_address, network_port, address, position_xy, depth, label):
    #
    # Sensor Node NM3 Virtual Modem Modem
    #

    input_stream = BufferedIOQueueWrapper()
    output_stream = BufferedIOQueueWrapper()


    # input_stream, output_stream, network_address=None, network_port=None, local_address=255, position_xy=(0.0,0.0), depth=10.0):
    modem = Modem(input_stream=input_stream,
                  output_stream=output_stream,
                  network_address=network_address,
                  network_port=network_port,
                  local_address=address,
                  position_xy=position_xy,
                  depth=depth,
                  label=label)

    nm3_modem = Nm3(input_stream=output_stream, output_stream=input_stream)

    modem_thread = Thread(target=modem.run)
    #modem.run()
    modem_thread.start()

    # Address of network nodes start at 100
    # TDMA offset time for response (after beacon) is (address - 100) * tdma_window_size
    tdma_window_size = 4.0
    tdma_offset = (address - 100) * tdma_window_size

    # Now here run a while loop to process the modem serial comms
    try:
        while True:
            nm3_modem.poll_receiver() # non-blocking returns immediately if no bytes ready to read.

            # Periodically process any bytes received
            nm3_modem.process_incoming_buffer()

            # Periodically check for received packets
            while nm3_modem.has_received_packet():
                message_packet = nm3_modem.get_received_packet()

                print('Sensor ' + label + ' Received message packet: ' +
                      MessagePacket.PACKETTYPE_NAMES[message_packet.packet_type] +
                      ' src: ' + str(message_packet.source_address) +
                      ' dest: ' + str(message_packet.destination_address) +
                      ' payload: ' + str(message_packet.packet_payload) +
                      ' payload text: ' + bytes(message_packet.packet_payload).decode('utf-8'))

                # Network Protocol
                if message_packet.packet_type == MessagePacket.PACKETTYPE_BROADCAST:
                    if message_packet.packet_payload == [int(i) for i in bytes(b'BEACON')]:
                        beacon_time = time.time()
                        beacon_address = message_packet.source_address

                        # Get the sensor data

                        while time.time() < (beacon_time + tdma_offset):
                            pass

                        # Now send the sensor data
                        payload = bytes(b'SENSOR') + bytes(label.encode('utf-8'))
                        nm3_modem.send_unicast_message(beacon_address, payload)



            # Yield the thread
            time.sleep(0.001)

            pass
    finally:
        modem.stop()

def start_terminal_modem(network_address, network_port, address, position_xy, depth, label):
    #
    # Virtual NM3 Modem Mode (With stdin/stdout as interface)
    #

    input_stream = sys.stdin.buffer # bytes from a piped input
    if sys.stdin.isatty():
        sys.stdout = TimestampTextStreamFilter(sys.stdout) # Adds a timestamp to beginning of every line
        #sys.stderr = TimestampStreamFilter(sys.stderr)  # Adds a timestamp to beginning of every line
        input_stream = TtyWrapper(sys.stdin) # wrapped to grab lines and convert to bytes

    output_stream = BinaryToTextStream(sys.stdout)

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
        modem.stop()


def main():
    """Main Program Entry."""

    # Using separate processes for each of the independent nodes and controller - Controller, Modem clients, etc

    network_address = "127.0.0.1"
    network_port = 8080
    publish_port = 8081

    # Create and run a Controller
    file_datetime = datetime.utcnow()
    dt_str = file_datetime.strftime('%Y%m%dT%H%M%S')
    log_filename = "SimpleExampleLogs" + '-' + dt_str + '.json'
    print("Starting the Controller")
    controller_process = Process(target=start_controller, args=(network_address, network_port, publish_port,
                                                                log_filename))
    controller_process.start()

    # Wait for it to startup before proceeding
    time.sleep(5.0)

    # Open a new logfile with current time
    file_datetime = datetime.utcnow()
    dt_str = file_datetime.strftime('%Y%m%dT%H%M%S')
    log_filename = "SimpleExampleLogs-SimulationLogger" + '-' + dt_str + '.json'
    print("Starting the Simulation Logger")
    simulation_logger_process = Process(target=start_logger, args=(network_address, publish_port, log_filename))
    simulation_logger_process.start()

    # Wait for it to startup before proceeding
    #time.sleep(5.0)

    # Create and run the MapVisualisation
    print("Starting the MapVisualisation")
    mapvis_process = Process(target=start_mapvis, args=(network_address, publish_port))
    mapvis_process.start()

    # Wait for it to startup before proceeding
    time.sleep(5.0)

    # Create and run a Transponder
    #print("Starting Transponder N001")
    #transponder_process = Process(target=start_transponder, args=(network_address, network_port, 1, (0.0,1000.0), 10.0, "N001"))
    #transponder_process.start()

    # Sensor Nodes
    sensor_nodes = [
        ("N100", 100, (400.0,  400.0),  20.0),
        ("N101", 101, (800.0,  800.0),  20.0),
        ("N102", 102, (1200.0, 1200.0), 20.0),
        ("N103", 103, (1600.0, 1600.0), 20.0),
        ("N104", 104, (2000.0, 2000.0), 20.0),
        ("N105", 105, (2400.0, 2400.0), 20.0),
    ]

    sensor_node_processes = []

    # Create and run Sensor Nodes
    for sensor_node in sensor_nodes:
        label = sensor_node[0]
        address = sensor_node[1]
        position_xy = sensor_node[2]
        depth = sensor_node[3]

        print("Starting Sensor Node " + label)
        sensor_node_process = Process(target=start_sensor_node, args=(network_address, network_port, address, position_xy, depth, label))
        sensor_node_process.start()

        sensor_node_processes.append(sensor_node_process)


    # Create and run Gateway Node
    print("Starting Gateway Node N000")
    gateway_node_process = Process(target=start_gateway_node, args=(network_address, network_port, 0, (0.0,0.0), 50.0, "N000"))
    gateway_node_process.start()


    # Lastly Create and run a Terminal Client
    #print("Starting Terminal Client N007 - In this process/thread")
    #start_terminal_modem(network_address, network_port, 7, (0.0,0.0), 10.0, "N007")


    controller_process.join()
    gateway_node_process.join()




if __name__ == '__main__':
    main()
