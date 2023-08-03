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
from .node_packet import NodePacket
from .time_packet import TimePacket

import json
import os
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

    MODEM_EVENT_RECEIVE_SUCCESS, MODEM_EVENT_RECEIVE_FAIL, MODEM_EVENT_TRANSMIT_COMPLETE = range(3)

    MODEM_EVENT_NAMES = {
        MODEM_EVENT_RECEIVE_SUCCESS: 'Receive Success',
        MODEM_EVENT_RECEIVE_FAIL: 'Receive Fail',
        MODEM_EVENT_TRANSMIT_COMPLETE: 'Transmit Complete'
    }

    MODEM_EVENTS = (MODEM_EVENT_RECEIVE_SUCCESS, MODEM_EVENT_RECEIVE_FAIL, MODEM_EVENT_TRANSMIT_COMPLETE)

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

        while True:
            if self._position_information_updated:
                # Send positional information update
                self._position_information_updated = False
                node_packet = NodePacket(position_xy=self._position_xy, depth=self._depth, label=self._label)
                self.send_node_packet(node_packet)

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

                        if "TimePacket" in network_message_jason:
                            # Process the TimePacket
                            time_packet = TimePacket.from_json(network_message_jason["TimePacket"])
                            time_packet.client_arrival_time = self._local_received_time
                            #time_packet.server_transmit_time = zmq_timestamp
                            self._hamr_time_offset = time_packet.calculate_offset()

                            print(time_packet.to_string())

                            print("TimePacket offset: " + str(self._hamr_time_offset))

                        if "AcousticPacket" in network_message_jason:
                            # Process the AcousticPacket
                            acoustic_packet = AcousticPacket.from_json(network_message_jason["AcousticPacket"])
                            self.process_acoustic_packet(acoustic_packet)

                    except zmq.ZMQError:
                        more_messages = False

            # Check for timeout if awaiting an Ack
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

    def process_acoustic_packet(self, acoustic_packet: AcousticPacket):
        """Process an AcousticPacket."""
        # State
        if self._acoustic_state == self.ACOUSTIC_STATE_WAIT_ACK:
            # Check for DownChirp on the acoustic_packet - ignore if not downchip.
            if acoustic_packet.frame_synch == AcousticPacket.FRAMESYNCH_DN:
                # Ack Received
                if acoustic_packet.address == self._acoustic_ack_wait_address:
                    # This is the Ack we are looking for.
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



    def process_bytes(self, some_bytes: bytes):
        """Process bytes in the state machine and act accordingly."""
        if not self._last_byte_time or (time.time() > (self._last_byte_time + Modem.BYTE_PARSER_TIMEOUT)):
            self._simulator_state = self.SIMULATOR_STATE_IDLE

        self._last_byte_time = time.time()

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


