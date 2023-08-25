#!/usr/bin/env python
#
# Modem
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

from .acoustic_packet import AcousticPacket
from .modem_packet import ModemPacket
from .node_packet import NodePacket
from .time_packet import TimePacket

import json
import os
import random
import time
import zmq


def _debug_print(*args, **kwargs):
    """File local debug printing"""
    #print(*args, **kwargs)
    pass

class Modem:
    """Modem Class.
    Connects over network to Controller.
    Currently supports: Query ($?), Set Local Address ($Axxx), Ping ($Pxxx),
    Broadcast Message ($Byydd...dd), Unicast Message ($Uxxxyydd..dd),
    Unicast with Ack Message ($Mxxxyydd..dd)."""

    SIMULATOR_STATE_IDLE, SIMULATOR_STATE_COMMAND, \
    SIMULATOR_STATE_SET_ADDRESS, SIMULATOR_STATE_PING, SIMULATOR_STATE_TEST, \
    SIMULATOR_STATE_MESSAGE_ADDRESS, SIMULATOR_STATE_MESSAGE_LENGTH, \
    SIMULATOR_STATE_MESSAGE_DATA = range(8)

    SIMULATOR_STATE_NAMES = {
        SIMULATOR_STATE_IDLE: 'Idle',
        SIMULATOR_STATE_COMMAND: 'Command',
        SIMULATOR_STATE_SET_ADDRESS: 'SetAddress',
        SIMULATOR_STATE_PING: 'Ping',
        SIMULATOR_STATE_TEST: 'Test',
        SIMULATOR_STATE_MESSAGE_ADDRESS: 'MessageAddress',
        SIMULATOR_STATE_MESSAGE_LENGTH: 'MessageLength',
        SIMULATOR_STATE_MESSAGE_DATA: 'MessageData',
    }

    SIMULATOR_STATES = (SIMULATOR_STATE_IDLE, SIMULATOR_STATE_COMMAND,
                        SIMULATOR_STATE_SET_ADDRESS, SIMULATOR_STATE_PING, SIMULATOR_STATE_TEST,
                        SIMULATOR_STATE_MESSAGE_ADDRESS, SIMULATOR_STATE_MESSAGE_LENGTH,
                        SIMULATOR_STATE_MESSAGE_DATA)

    ACOUSTIC_STATE_IDLE, ACOUSTIC_STATE_WAIT_ACK = range(2)
    ACOUSTIC_STATE_NAMES = {
        ACOUSTIC_STATE_IDLE: 'Idle',
        ACOUSTIC_STATE_WAIT_ACK: 'WaitAck',
    }

    ACOUSTIC_STATES = (ACOUSTIC_STATE_IDLE, ACOUSTIC_STATE_WAIT_ACK)

    MODEM_STATE_LISTENING, MODEM_STATE_RECEIVING, MODEM_STATE_TRANSMITTING,\
        MODEM_STATE_UARTING, MODEM_STATE_SLEEPING = range(5)

    MODEM_STATE_NAMES = {
        MODEM_STATE_LISTENING: 'Listening',
        MODEM_STATE_RECEIVING: 'Receiving',
        MODEM_STATE_TRANSMITTING: 'Transmitting',
        MODEM_STATE_UARTING: 'UARTing',
        MODEM_STATE_SLEEPING: 'Sleeping'
    }

    MODEM_STATES = (MODEM_STATE_LISTENING, MODEM_STATE_RECEIVING, MODEM_STATE_TRANSMITTING,
                    MODEM_STATE_UARTING, MODEM_STATE_SLEEPING)

    MODEM_EVENT_RECEIVE_START, MODEM_EVENT_RECEIVE_SUCCESS, MODEM_EVENT_RECEIVE_FAIL, MODEM_EVENT_TRANSMIT_START, \
        MODEM_EVENT_TRANSMIT_COMPLETE = range(5)

    MODEM_EVENT_NAMES = {
        MODEM_EVENT_RECEIVE_START: 'Receive Start',
        MODEM_EVENT_RECEIVE_SUCCESS: 'Receive Success',
        MODEM_EVENT_RECEIVE_FAIL: 'Receive Fail',
        MODEM_EVENT_TRANSMIT_START: 'Transmit Start',
        MODEM_EVENT_TRANSMIT_COMPLETE: 'Transmit Complete'
    }

    MODEM_EVENTS = (MODEM_EVENT_RECEIVE_START, MODEM_EVENT_RECEIVE_SUCCESS, MODEM_EVENT_RECEIVE_FAIL,
                    MODEM_EVENT_TRANSMIT_START, MODEM_EVENT_TRANSMIT_COMPLETE)

    RECEIVER_STATE_QUIET, RECEIVER_STATE_SINGLE_ARRIVAL, RECEIVER_STATE_OVERLAPPED_ARRIVAL,\
        RECEIVER_STATE_SATURATED = range(4)

    RECEIVER_STATES = (RECEIVER_STATE_QUIET, RECEIVER_STATE_SINGLE_ARRIVAL,
                       RECEIVER_STATE_OVERLAPPED_ARRIVAL, RECEIVER_STATE_SATURATED)

    RECEIVER_STATE_NAMES = {
        RECEIVER_STATE_QUIET: 'Quiet',
        RECEIVER_STATE_SINGLE_ARRIVAL: 'Single Arrival',
        RECEIVER_STATE_OVERLAPPED_ARRIVAL: 'Overlapped Arrival',
        RECEIVER_STATE_SATURATED: 'Saturated'
    }

    BYTE_PARSER_TIMEOUT = 0.100

    def __init__(self, input_stream, output_stream,
                 network_address=None, network_port=None, local_address: int =255, position_xy=(0.0,0.0), depth=10.0, label=None):
        """input_stream and output_stream implement the Bytes IO interface.
        Namely: readable()->bool, writeable()->bool, read(bytes) and write(bytes)."""
        self._input_stream = input_stream
        self._output_stream = output_stream
        self._simulator_state = Modem.SIMULATOR_STATE_IDLE
        self._acoustic_state = Modem.ACOUSTIC_STATE_IDLE
        self._acoustic_ack_wait_address = None
        self._acoustic_ack_wait_time = None
        self._modem_state = Modem.MODEM_STATE_LISTENING
        self._receiver_state = Modem.RECEIVER_STATE_QUIET

        self._acoustic_ack_fixed_offset_time = 0.040  # 40ms

        self._network_address = network_address
        self._network_port = network_port
        self._socket = None
        self._socket_poller = None

        # Offset to synchronise times
        self._hamr_time_offset = 0.0

        self._local_address = local_address

        # Parser variables
        self._last_byte_time = None
        self._current_byte_counter = 0
        self._current_integer = 0

        # Sending message
        self._message_type = None
        self._message_address = None
        self._message_length = None
        self._message_bytes = None

        # Positional information
        self._position_xy = position_xy
        self._depth = depth
        self._position_information_updated = True
        self._label = label

        self._startup_time = time.time()
        self._local_received_time = None
        self._local_sent_time = None
        self._last_packet_received_time = None
        self._last_packet_sent_time = None

        # Transmitter
        self._transmitting_acoustic_packet = None  # The current outgoing acoustic packet

        # Receiver
        self._arriving_acoustic_packets = []

        # Receiver Performance to determine packet success probability
        # Lookup tables here based on varying data payload lengths and varying multipath severity.
        # Check curves in paper - Fig. 12.
        # Tables generated for the paper: 20190827-nm3paperdata.git 03-nm3packetsim\results
        # Anything below -9dB is lost, anything above 9dB is 100% successful.
        # Everything else is within the lookup table.
        # Multipath configurations (1,1,0) is a harsh multipath scenario.
        # (1,0,0) is less so. (0,0,0) is just AWGN.
        # Multipath level - 0=(0,0,0)/AWGN, 1=(1,0,0), 2=(1,1,0)
        self._snr_vs_per_results = [
            (0, [
                (4, [(-9, 0.919), (-8, 0.736), (-7, 0.465), (-6, 0.236), (-5, 0.076),
                     (-4, 0.012), (-3, 0.000), (-2, 0.000), (-1, 0.000), (0, 0.000),
                     (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                     (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L4 (0,0,0)
                (8, [(-9, 0.946), (-8, 0.732), (-7, 0.484), (-6, 0.234), (-5, 0.074),
                     (-4, 0.013), (-3, 0.002), (-2, 0.000), (-1, 0.000), (0, 0.000),
                     (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                     (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L8 (0,0,0)
                (16, [(-9, 0.984), (-8, 0.777), (-7, 0.482), (-6, 0.225), (-5, 0.070),
                      (-4, 0.017), (-3, 0.001), (-2, 0.000), (-1, 0.000), (0, 0.000),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L16 (0,0,0)
                (32, [(-9, 1.000), (-8, 0.910), (-7, 0.469), (-6, 0.228), (-5, 0.074),
                      (-4, 0.011), (-3, 0.000), (-2, 0.000), (-1, 0.000), (0, 0.000),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L32 (0,0,0)
                (64, [(-9, 1.000), (-8, 0.997), (-7, 0.644), (-6, 0.221), (-5, 0.066),
                      (-4, 0.011), (-3, 0.001), (-2, 0.000), (-1, 0.000), (0, 0.000),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L64 (0,0,0)
            ]),  # Multpath Level 0 (0,0,0) AWGN
            (1, [
                (4, [(-9, 1.000), (-8, 1.000), (-7, 0.998), (-6, 0.960), (-5, 0.776),
                     (-4, 0.434), (-3, 0.200), (-2, 0.062), (-1, 0.013), (0, 0.001),
                     (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                     (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L4 (1,0,0)
                (8, [(-9, 1.000), (-8, 1.000), (-7, 0.999), (-6, 0.981), (-5, 0.816),
                     (-4, 0.456), (-3, 0.194), (-2, 0.062), (-1, 0.011), (0, 0.001),
                     (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                     (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L8 (1,0,0)
                (16, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 0.998), (-5, 0.944),
                      (-4, 0.605), (-3, 0.228), (-2, 0.061), (-1, 0.011), (0, 0.001),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L16 (1,0,0)
                (32, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 1.000), (-5, 0.998),
                      (-4, 0.897), (-3, 0.388), (-2, 0.080), (-1, 0.013), (0, 0.002),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L32 (1,0,0)
                (64, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 1.000), (-5, 1.000),
                      (-4, 0.998), (-3, 0.856), (-2, 0.270), (-1, 0.024), (0, 0.001),
                      (1, 0.000), (2, 0.000), (3, 0.000), (4, 0.000), (5, 0.000),
                      (6, 0.000), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L64 (1,0,0)
            ]),  # Multpath Level 1 (1,0,0)
            (2, [
                (16, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 1.000), (-5, 1.000),
                      (-4, 0.998), (-3, 0.944), (-2, 0.717), (-1, 0.412), (0, 0.161),
                      (1, 0.047), (2, 0.015), (3, 0.006), (4, 0.003), (5, 0.001),
                      (6, 0.001), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L16 (1,1,0)
                (32, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 1.000), (-5, 1.000),
                      (-4, 1.000), (-3, 0.997), (-2, 0.952), (-1, 0.804), (0, 0.553),
                      (1, 0.245), (2, 0.089), (3, 0.020), (4, 0.010), (5, 0.003),
                      (6, 0.001), (7, 0.000), (8, 0.000), (9, 0.000)]),  # L32 (1,1,0)
                (64, [(-9, 1.000), (-8, 1.000), (-7, 1.000), (-6, 1.000), (-5, 1.000),
                      (-4, 1.000), (-3, 1.000), (-2, 0.999), (-1, 0.984), (0, 0.932),
                      (1, 0.793), (2, 0.483), (3, 0.217), (4, 0.068), (5, 0.020),
                      (6, 0.007), (7, 0.002), (8, 0.002), (9, 0.000)])  # L64 (1,1,0)
            ])  # Multipath Level 2 (1,1,0)
        ]  # SNR vs PER Lookup Table

    def __call__(self):
        return self


    @property
    def position_xy(self):
        return self._position_xy

    @position_xy.setter
    def position_xy(self, position_xy):
        self._position_xy = position_xy
        self._position_information_updated = True

    @property
    def depth(self):
        return self._depth

    @depth.setter
    def depth(self, depth):
        self._depth = depth
        self._position_information_updated = True

    @property
    def label(self) -> str:
        return self._label

    @label.setter
    def label(self, label: str):
        self._label = label

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


    def get_snr_to_per_table(self, payload_length, multipath_level=2):
        """Get the lookup table of SNR and packet error rate"""
        # Find nearest data_bytes value equal or greater in the lookup tables for given multipath.
        look_up_table = []
        for lut in self._snr_vs_per_results[multipath_level][1]:
            if payload_length <= lut[0]:
                # This one.
                look_up_table = lut[1]
                break

        return look_up_table

    def calculate_probability_of_delivery(self, receive_snr, payload_length, multipath_level=2):
        """Calculate the probability of delivery."""

        probability_of_delivery = 1.0

        # Find nearest data_bytes value equal or greater in the lookup tables for given multipath.
        look_up_table = self.get_snr_to_per_table(payload_length, multipath_level)

        # Search for packet error rate based on received SNR
        #
        # round SNR to nearest integer
        rounded_snr = int(round(receive_snr))
        if rounded_snr < look_up_table[0][0]:  # out of range?
            probability_of_delivery = 0.0
        elif rounded_snr > look_up_table[-1][0]:
            probability_of_delivery = 1.0  # short range?
        else:
            # read PER from table and convert to Pd
            start_snr = look_up_table[0][0]
            per = look_up_table[rounded_snr - start_snr][1]
            probability_of_delivery = 1.0 - per

        return probability_of_delivery

    def run(self):
        """Run the simulator. Never returns."""
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
            self._socket = context.socket(zmq.DEALER)
            self._socket.connect("tcp://" + self._network_address + ":"
                              + str(self._network_port))
            self._socket_poller = zmq.Poller()
            self._socket_poller.register(self._socket, zmq.POLLIN)
            self.send_time_packet()

        while True:  # Main Loop
            # 1. Node Position
            if self._position_information_updated:
                # Send positional information update
                self._position_information_updated = False
                node_packet = NodePacket(position_xy=self._position_xy, depth=self._depth, label=self._label)
                self.send_node_packet(node_packet)

            # 2. Serial Port
            if self._input_stream and self._input_stream.readable():
                #_debug_print("Checking input_stream")
                some_bytes = self._input_stream.read()  # Read

                if some_bytes:
                    if self._modem_state == Modem.MODEM_STATE_LISTENING \
                            or self._modem_state == Modem.MODEM_STATE_UARTING:
                        # Can only process received bytes if the modem is in listening mode or is
                        # already communicating over uart with the user.
                        #_debug_print("some_bytes=" + str(some_bytes))
                        self.process_bytes(some_bytes)

            # Poll the socket for incoming "acoustic" messages
            #_debug_print("Checking socket poller")
            # 3. Incoming Socket Messages
            sockets = dict(self._socket_poller.poll(1))
            if self._socket in sockets:
                more_messages = True
                while more_messages:
                    try:
                        msg = self._socket.recv_multipart(zmq.DONTWAIT)

                        network_message_json_bytes = msg[0]
                        zmq_timestamp = float(msg[1].decode('utf-8'))

                        self._local_received_time = time.time()
                        _debug_print("NetworkPacket (len=" + str(len(msg)) + ") from Controller received at: " + str(self._local_received_time))
                        network_message_json_str = network_message_json_bytes.decode('utf-8')
                        network_message_jason = json.loads(network_message_json_str)

                        _debug_print("Network Packet received: " + network_message_json_str)

                        # Update Time Offsets
                        if "TimePacket" in network_message_jason:
                            # Process the TimePacket
                            time_packet = TimePacket.from_json(network_message_jason["TimePacket"])
                            time_packet.client_arrival_time = self._local_received_time
                            #time_packet.server_transmit_time = zmq_timestamp
                            self._hamr_time_offset = time_packet.calculate_offset()

                            print(time_packet.to_string())

                            print("TimePacket offset: " + str(self._hamr_time_offset))

                        # Acoustic Packet - Queue up for processing
                        if "AcousticPacket" in network_message_jason:
                            # Queue up the AcousticPacket
                            acoustic_packet = AcousticPacket.from_json(network_message_jason["AcousticPacket"])
                            self._arriving_acoustic_packets.append(acoustic_packet)
                            if self._receiver_state == Modem.RECEIVER_STATE_QUIET:
                                # Single arrival
                                self._receiver_state = Modem.RECEIVER_STATE_SINGLE_ARRIVAL
                                if self._modem_state == Modem.MODEM_STATE_LISTENING:
                                    # Then start receiving
                                    self.update_modem_state(Modem.MODEM_STATE_RECEIVING, Modem.MODEM_EVENT_RECEIVE_START)
                            elif self._receiver_state == Modem.RECEIVER_STATE_SINGLE_ARRIVAL:
                                # Collision
                                self._receiver_state = Modem.RECEIVER_STATE_OVERLAPPED_ARRIVAL
                            else:
                                #  already a collision or is transmitting
                                pass


                            #self.process_acoustic_packet(acoustic_packet)

                    except zmq.ZMQError:
                        more_messages = False

            # Process Outgoing Acoustic Packet
            if self._transmitting_acoustic_packet:
                # Are we still transmitting
                if self._transmitting_acoustic_packet.hamr_timestamp + \
                        self._transmitting_acoustic_packet.transmit_duration <= self.get_hamr_time():
                    # Transmission complete
                    #print("Current HAMR Time=" + str(self.get_hamr_time()))
                    #print("self._transmitting_acoustic_packet.hamr_timestamp=" + str(self._transmitting_acoustic_packet.hamr_timestamp))
                    #print("self._transmitting_acoustic_packet.transmit_duration=" + str(self._transmitting_acoustic_packet.transmit_duration))
                    self._transmitting_acoustic_packet = None
                    # Return to listening
                    self.update_modem_state(Modem.MODEM_STATE_LISTENING, Modem.MODEM_EVENT_TRANSMIT_COMPLETE)

                    if self._arriving_acoustic_packets:
                        # Then these overlapped with the transmission
                        self._receiver_state = Modem.RECEIVER_STATE_OVERLAPPED_ARRIVAL
                else:
                    # Continue transmitting
                    self._receiver_state = Modem.RECEIVER_STATE_SATURATED
                    pass

            # Process Arriving Acoustic Packets
            if self._arriving_acoustic_packets:

                if self._arriving_acoustic_packets[0].hamr_timestamp + \
                        self._arriving_acoustic_packets[0].transmit_duration <= self.get_hamr_time():
                    # Packet has now completed arriving
                    acoustic_packet = self._arriving_acoustic_packets.pop(0)

                    probability_of_delivery = self.calculate_probability_of_delivery(acoustic_packet.receive_snr,
                                                                                     acoustic_packet.payload_length)
                    _debug_print("receive_snr=" + str(acoustic_packet.receive_snr))
                    _debug_print("probability_of_delivery=" + str(probability_of_delivery))

                    if self._modem_state == Modem.MODEM_STATE_RECEIVING:
                        if self._receiver_state == Modem.RECEIVER_STATE_SINGLE_ARRIVAL \
                            and random.random() < probability_of_delivery:
                            # Single arrival and received successfully
                            self.update_modem_state(Modem.MODEM_STATE_LISTENING, Modem.MODEM_EVENT_RECEIVE_SUCCESS)
                            self.process_acoustic_packet(acoustic_packet)
                        else:
                            # Receiving but failed
                            self.update_modem_state(Modem.MODEM_STATE_LISTENING, Modem.MODEM_EVENT_RECEIVE_FAIL)
                            pass

                    else:
                        # Not in a receiving state - do we need to alert to a lost packet if we hadn't synched?
                        pass

            else:
                # No arriving acoustic packets
                pass

            if not self._arriving_acoustic_packets and not self._transmitting_acoustic_packet:
                # Nothing coming or going so quiet
                self._receiver_state = Modem.RECEIVER_STATE_QUIET


            # 4. Check for timeout if awaiting an Ack
            #_debug_print("Checking ack status")
            if self._acoustic_state == self.ACOUSTIC_STATE_WAIT_ACK:
                delay_time = time.time() - self._acoustic_ack_wait_time
                if delay_time > 4.0:
                    # Cancel the wait for ack and indicate timeout
                    self._acoustic_state = self.ACOUSTIC_STATE_IDLE

                    if self._output_stream and self._output_stream.writable():
                        response_str = "#T0" + "\r\n"
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)
                        self._output_stream.flush()


            # Yield the thread
            time.sleep(0.00000000001)


            pass


    def send_time_packet(self):
        """Create and send a TimePacket to determine offset time."""
        time_packet = TimePacket()
        time_packet.client_transmit_time = time.time()
        jason = {"TimePacket": time_packet.json()}
        json_string = json.dumps(jason)
        self._socket.send_multipart([json_string.encode('utf-8'), str(time.time()).encode('utf-8')])
        self._local_sent_time = time.time()
        _debug_print("NetworkPacket (TimePacket) to Controller sent at: " + str(self._local_sent_time))
        return

    def send_acoustic_packet(self, acoustic_packet: AcousticPacket):
        """Send an AcousticPacket.
        Returns time sent"""
        jason = { "AcousticPacket": acoustic_packet.json() }
        json_string = json.dumps(jason)
        self._socket.send_multipart([json_string.encode('utf-8'), str(time.time()).encode('utf-8')])

        self._local_sent_time = time.time()
        _debug_print("NetworkPacket (AcousticPacket) to Controller sent at: " + str(self._local_sent_time))
        if self._local_received_time:
            _debug_print("-Turnaround: " + str(self._local_sent_time-self._local_received_time))

        return self._local_sent_time

    def send_node_packet(self, node_packet: NodePacket):
        """Send a NodePacket.
        Returns time sent"""
        jason = { "NodePacket": node_packet.json() }
        json_string = json.dumps(jason)
        self._socket.send_multipart([json_string.encode('utf-8'), str(time.time()).encode('utf-8')])

        self._local_sent_time = time.time()
        _debug_print("NetworkPacket (NodePacket) to Controller sent at: " + str(self._local_sent_time))

        return self._local_sent_time

    def send_modem_packet(self, modem_packet: ModemPacket):
        """Send a ModemPacket.
        Returns time sent"""
        jason = { "ModemPacket": modem_packet.json() }
        json_string = json.dumps(jason)
        self._socket.send_multipart([json_string.encode('utf-8'), str(time.time()).encode('utf-8')])

        self._local_sent_time = time.time()
        _debug_print("NetworkPacket (ModemPacket) to Controller sent at: " + str(self._local_sent_time))

        return self._local_sent_time

    def update_modem_state(self, modem_state, modem_event=None):
        self._modem_state = modem_state

        if self._modem_state == Modem.MODEM_STATE_TRANSMITTING:
            self._receiver_state = Modem.RECEIVER_STATE_SATURATED

        # Send state information to controller
        modem_packet = ModemPacket(modem_state=modem_state, modem_event=modem_event)
        self.send_modem_packet(modem_packet)

        #print("modem_state=" + Modem.MODEM_STATE_NAMES[modem_state])
        #if modem_event is not None:
        #    print("modem_event=" + Modem.MODEM_EVENT_NAMES[modem_event])

    def process_acoustic_packet(self, acoustic_packet: AcousticPacket):
        """Process an AcousticPacket."""

        # State
        if self._acoustic_state == self.ACOUSTIC_STATE_WAIT_ACK:
            # Check for DownChirp on the acoustic_packet - ignore if not downchip.
            if acoustic_packet.frame_synch == AcousticPacket.FRAMESYNCH_DN:
                # Ack Received
                if acoustic_packet.address == self._acoustic_ack_wait_address:
                    # This is the Ack we are looking for.
                    self.update_modem_state(Modem.MODEM_STATE_UARTING, Modem.MODEM_EVENT_RECEIVE_SUCCESS)
                    if self._output_stream and self._output_stream.writable():
                        local_received_time = self.get_local_time(acoustic_packet.hamr_timestamp)
                        delay_time = local_received_time - self._acoustic_ack_wait_time
                        _debug_print("Ack delay_time: " + str(delay_time))
                        timeval = int(delay_time * 16000.0)
                        response_str = "#R" + "{:03d}".format(
                            acoustic_packet.address) + "T" + "{:05d}".format(timeval) + "\r\n"
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)
                        self._output_stream.flush()

                    self._acoustic_state = self.ACOUSTIC_STATE_IDLE
                self.update_modem_state(Modem.MODEM_STATE_LISTENING)
        else:
            # Check for UpChirp on the acoustic_packet - ignore if not upchirp.
            if acoustic_packet.frame_synch == AcousticPacket.FRAMESYNCH_UP:
                if acoustic_packet.payload_length == 0 and acoustic_packet.address == self._local_address:
                    # Control commands
                    # CMD_PING_REQ, CMD_PING_REP, CMD_TEST_REQ, CMD_VBATT_REQ = range(4)
                    if acoustic_packet.command == AcousticPacket.CMD_PING_REQ:
                        # Ping request so send a reply
                        acoustic_packet_to_send = AcousticPacket(
                            frame_synch=AcousticPacket.FRAMESYNCH_DN,
                            address=self._local_address,
                            command=AcousticPacket.CMD_PING_REP,
                            hamr_timestamp=acoustic_packet.hamr_timestamp)
                        self.send_acoustic_packet(acoustic_packet_to_send)
                        self._transmitting_acoustic_packet = acoustic_packet_to_send
                        self.update_modem_state(Modem.MODEM_STATE_TRANSMITTING, Modem.MODEM_EVENT_TRANSMIT_START)

                    elif acoustic_packet.command == AcousticPacket.CMD_TEST_REQ:
                        # Test message acoustic message as a broadcast
                        payload_str = "This is a test message from a Virtual NM3"
                        payload_bytes = list(payload_str.encode('utf-8'))

                        acoustic_packet_to_send = AcousticPacket(
                            frame_synch=AcousticPacket.FRAMESYNCH_UP,
                            address=self._local_address,
                            command=AcousticPacket.CMD_BROADCAST_MSG,
                            payload_length=len(payload_bytes),
                            payload_bytes=payload_bytes,
                            hamr_timestamp=self.get_hamr_time())
                        self.send_acoustic_packet(acoustic_packet_to_send)
                        self._transmitting_acoustic_packet = acoustic_packet_to_send
                        self.update_modem_state(Modem.MODEM_STATE_TRANSMITTING, Modem.MODEM_EVENT_TRANSMIT_START)

                    # Other commands not supported at the moment.

                else:
                    # Message Packets
                    # CMD_UNICAST_MSG, CMD_BROADCAST_MSG, CMD_UNICAST_ACK_MSG, CMD_ECHO_MSG = range(4)
                    if acoustic_packet.command == AcousticPacket.CMD_UNICAST_MSG and acoustic_packet.address == self._local_address:
                        # Construct the bytes to be sent to the output_stream
                        # "#U..."
                        if self._output_stream and self._output_stream.writable():
                            response_bytes = b"#U" \
                                             + "{:02d}".format(
                                acoustic_packet.payload_length).encode('utf-8') \
                                             + bytes(acoustic_packet.payload_bytes) + b"\r\n"
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()

                    elif acoustic_packet.command == AcousticPacket.CMD_BROADCAST_MSG:
                        # Construct the bytes to be sent to the output_stream
                        # "#B..."
                        if self._output_stream and self._output_stream.writable():
                            response_bytes = b"#B" \
                                             + "{:03d}".format(
                                acoustic_packet.address).encode('utf-8') \
                                             + "{:02d}".format(
                                acoustic_packet.payload_length).encode('utf-8') \
                                             + bytes(acoustic_packet.payload_bytes) + b"\r\n"
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()

                    elif acoustic_packet.command == AcousticPacket.CMD_UNICAST_ACK_MSG and acoustic_packet.address == self._local_address:
                        # Ack request so send a reply
                        acoustic_packet_to_send = AcousticPacket(
                            frame_synch=AcousticPacket.FRAMESYNCH_DN,
                            address=self._local_address,
                            command=AcousticPacket.CMD_PING_REP,
                            hamr_timestamp=acoustic_packet.hamr_timestamp)
                        self.send_acoustic_packet(acoustic_packet_to_send)
                        self._transmitting_acoustic_packet = acoustic_packet_to_send
                        self.update_modem_state(Modem.MODEM_STATE_TRANSMITTING, Modem.MODEM_EVENT_TRANSMIT_START)

                        # Construct the bytes to be sent to the output_stream
                        # "#U..."
                        if self._output_stream and self._output_stream.writable():
                            response_bytes = b"#U" \
                                             + "{:02d}".format(
                                acoustic_packet.payload_length).encode('utf-8') \
                                             + bytes(acoustic_packet.payload_bytes) + b"\r\n"
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()

                    elif acoustic_packet.command == AcousticPacket.CMD_ECHO_MSG and acoustic_packet.address == self._local_address:
                        # Echo the acoustic message as a broadcast
                        acoustic_packet_to_send = AcousticPacket(
                            frame_synch=AcousticPacket.FRAMESYNCH_UP,
                            address=self._local_address,
                            command=AcousticPacket.CMD_BROADCAST_MSG,
                            payload_length=acoustic_packet.payload_length,
                            payload_bytes=acoustic_packet.payload_bytes,
                            hamr_timestamp=self.get_hamr_time())
                        self.send_acoustic_packet(acoustic_packet_to_send)
                        self._transmitting_acoustic_packet = acoustic_packet_to_send
                        self.update_modem_state(Modem.MODEM_STATE_TRANSMITTING, Modem.MODEM_EVENT_TRANSMIT_START)



    def process_bytes(self, some_bytes: bytes):
        """Process bytes in the state machine and act accordingly."""
        if not self._last_byte_time or (time.time() > (self._last_byte_time + Modem.BYTE_PARSER_TIMEOUT)):
            self._simulator_state = self.SIMULATOR_STATE_IDLE
            self._modem_state = Modem.MODEM_STATE_LISTENING

        self._last_byte_time = time.time()
        self._modem_state = Modem.MODEM_STATE_UARTING

        for b in some_bytes:
            if self._simulator_state == self.SIMULATOR_STATE_IDLE:
                if bytes([b]).decode('utf-8') == '$':
                    self._simulator_state = self.SIMULATOR_STATE_COMMAND

                    # Cancel any ongoing Ack wait state
                    self._acoustic_state = self.ACOUSTIC_STATE_IDLE

            elif self._simulator_state == self.SIMULATOR_STATE_COMMAND:
                if bytes([b]).decode('utf-8') == '?':
                    # Query Status - Send back #AxxxVyyyy<CR><LF>
                    if self._output_stream and self._output_stream.writable():
                        # Response
                        response_str = "#A" + "{:03d}".format(self._local_address) + "V0000" + "\r\n"
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)
                        self._output_stream.flush()


                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE
                    self._modem_state = Modem.MODEM_STATE_LISTENING
                elif bytes([b]).decode('utf-8') == 'A':
                    # Set Address
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_SET_ADDRESS

                elif bytes([b]).decode('utf-8') == 'P':
                    # Ping Address
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_PING

                elif bytes([b]).decode('utf-8') == 'T':
                    # Test Address
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_TEST

                elif bytes([b]).decode('utf-8') == 'B':
                    # Broadcast Message
                    #_debug_print("MessageType: B. Broadcast")
                    self._message_type = 'B'
                    self._current_byte_counter = 2
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_LENGTH

                elif bytes([b]).decode('utf-8') == 'U':
                    # Unicast Message
                    #_debug_print("MessageType: U. Unicast")
                    self._message_type = 'U'
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_ADDRESS

                elif bytes([b]).decode('utf-8') == 'M':
                    # Unicast with Ack Message
                    #_debug_print("MessageType: M. Unicast with Ack")
                    self._message_type = 'M'
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_ADDRESS

                elif bytes([b]).decode('utf-8') == 'E':
                    # Echo Request Message
                    #_debug_print("MessageType: E. Echo")
                    self._message_type = 'E'
                    self._current_byte_counter = 3
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_ADDRESS
                else:
                    # Unhandled
                    self._simulator_state = self.SIMULATOR_STATE_IDLE

            elif self._simulator_state == self.SIMULATOR_STATE_SET_ADDRESS:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + int(bytes([b]).decode('utf-8'))

                if self._current_byte_counter == 0:
                    self._local_address = self._current_integer
                    if self._output_stream and self._output_stream.writable():
                        # Response
                        response_str = "#A" + "{:03d}".format(self._local_address) + "\r\n"
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)
                        self._output_stream.flush()

                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE
                    self._modem_state = Modem.MODEM_STATE_LISTENING

            elif self._simulator_state == self.SIMULATOR_STATE_PING:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + int(bytes([b]).decode('utf-8'))

                if self._current_byte_counter == 0:
                    address_to_ping = self._current_integer
                    if self._output_stream and self._output_stream.writable():
                        # Response
                        if address_to_ping == self._local_address:
                            # Error - cannot ping self
                            response_str = "E" + "\r\n"
                            response_bytes = response_str.encode('utf-8')
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()
                            self._modem_state = Modem.MODEM_STATE_LISTENING
                        else:
                            # Immediate response
                            response_str = "$P" + "{:03d}".format(address_to_ping) + "\r\n"
                            response_bytes = response_str.encode('utf-8')
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()

                            # Send to the Controller
                            self._acoustic_ack_wait_time = self.get_local_time()

                            acoustic_packet_to_send = AcousticPacket(
                                frame_synch=AcousticPacket.FRAMESYNCH_UP,
                                address=address_to_ping,
                                command=AcousticPacket.CMD_PING_REQ,
                                hamr_timestamp=self.get_hamr_time(self._acoustic_ack_wait_time))
                            self.send_acoustic_packet(acoustic_packet_to_send)
                            self._transmitting_acoustic_packet = acoustic_packet_to_send
                            self.update_modem_state(Modem.MODEM_STATE_TRANSMITTING, Modem.MODEM_EVENT_TRANSMIT_START)

                            self._acoustic_ack_wait_address = address_to_ping
                            self._acoustic_state = self.ACOUSTIC_STATE_WAIT_ACK


                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE

            elif self._simulator_state == self.SIMULATOR_STATE_TEST:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + int(bytes([b]).decode('utf-8'))

                if self._current_byte_counter == 0:
                    address_to_test = self._current_integer
                    if self._output_stream and self._output_stream.writable():
                        # Response
                        if address_to_test == self._local_address:
                            # Error - cannot test self
                            response_str = "E" + "\r\n"
                            response_bytes = response_str.encode('utf-8')
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()
                            self._modem_state = Modem.MODEM_STATE_LISTENING
                        else:
                            # Immediate response
                            response_str = "$T" + "{:03d}".format(address_to_test) + "\r\n"
                            response_bytes = response_str.encode('utf-8')
                            self._output_stream.write(response_bytes)
                            self._output_stream.flush()

                            # Send to the Controller
                            self._acoustic_ack_wait_time = self.get_local_time()

                            acoustic_packet_to_send = AcousticPacket(
                                frame_synch=AcousticPacket.FRAMESYNCH_UP,
                                address=address_to_test,
                                command=AcousticPacket.CMD_TEST_REQ,
                                hamr_timestamp=self.get_hamr_time(self._acoustic_ack_wait_time))
                            self.send_acoustic_packet(acoustic_packet_to_send)
                            self._transmitting_acoustic_packet = acoustic_packet_to_send
                            self.update_modem_state(Modem.MODEM_STATE_TRANSMITTING, Modem.MODEM_EVENT_TRANSMIT_START)

                            self._acoustic_state = self.ACOUSTIC_STATE_IDLE


                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE

            elif self._simulator_state == self.SIMULATOR_STATE_MESSAGE_ADDRESS:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + int(bytes([b]).decode('utf-8'))

                if self._current_byte_counter == 0:
                    self._message_address = self._current_integer
                    #_debug_print("MessageAddress: " + str(self._message_address))

                    # Now the message length
                    self._current_byte_counter = 2
                    self._current_integer = 0
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_LENGTH

            elif self._simulator_state == self.SIMULATOR_STATE_MESSAGE_LENGTH:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next ascii string integer digit
                self._current_integer = (self._current_integer * 10) + int(bytes([b]).decode('utf-8'))

                if self._current_byte_counter == 0:
                    self._message_length = self._current_integer
                    #_debug_print("MessageLength: " + str(self._message_length))

                    # Now the Message Data
                    self._current_byte_counter = self._message_length
                    self._current_integer = 0
                    self._message_bytes = []
                    self._simulator_state = self.SIMULATOR_STATE_MESSAGE_DATA

            elif self._simulator_state == self.SIMULATOR_STATE_MESSAGE_DATA:
                self._current_byte_counter = self._current_byte_counter - 1

                # Append the next data byte
                self._message_bytes.append(b)

                if self._current_byte_counter == 0:
                    #_debug_print("MessageData: " + str(self._message_bytes))

                    if self._output_stream and self._output_stream.writable():
                        # Immediate Response
                        response_str = ""
                        if self._message_type == 'B':
                            response_str = "$B" + "{:02d}".format(self._message_length)  + "\r\n"
                            self._message_address = self._local_address
                        else:
                            response_str = "$" + self._message_type + "{:03d}".format(self._message_address) + "{:02d}".format(self._message_length) +  "\r\n"

                        #_debug_print("Sending Response: " + response_str)
                        response_bytes = response_str.encode('utf-8')
                        self._output_stream.write(response_bytes)
                        self._output_stream.flush()

                        # Send to the Controller
                        acoustic_packet_command_to_send =  AcousticPacket.CMD_BROADCAST_MSG
                        if self._message_type == 'U':
                            acoustic_packet_command_to_send = AcousticPacket.CMD_UNICAST_MSG
                        elif self._message_type == 'M':
                            acoustic_packet_command_to_send = AcousticPacket.CMD_UNICAST_ACK_MSG
                        elif self._message_type == 'E':
                            acoustic_packet_command_to_send = AcousticPacket.CMD_ECHO_MSG

                        self._acoustic_ack_wait_time = self.get_local_time()
                        acoustic_packet_to_send = AcousticPacket(
                            frame_synch=AcousticPacket.FRAMESYNCH_UP,
                            address=self._message_address,
                            command=acoustic_packet_command_to_send,
                            payload_length=len(self._message_bytes),
                            payload_bytes=self._message_bytes,
                            hamr_timestamp=self.get_hamr_time(self._acoustic_ack_wait_time))
                        self.send_acoustic_packet(acoustic_packet_to_send)
                        self._transmitting_acoustic_packet = acoustic_packet_to_send
                        self.update_modem_state(Modem.MODEM_STATE_TRANSMITTING, Modem.MODEM_EVENT_TRANSMIT_START)

                        # If Ack
                        if self._message_type == 'M':
                            # Then delay or timeout response
                            self._acoustic_ack_wait_address = self._message_address
                            self._acoustic_state = self.ACOUSTIC_STATE_WAIT_ACK

                    # Return to Idle
                    self._simulator_state = self.SIMULATOR_STATE_IDLE

            else:
                # Unknown state
                self._simulator_state = self.SIMULATOR_STATE_IDLE


