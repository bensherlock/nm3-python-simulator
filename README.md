# Introduction

Python Simulator Framework using the NM3 Modems. Provides a real-time simulated acoustic environment and modems for connecting to either by human terminal interface, or a serial port with external hardware, or via Python programmed network nodes. Designed for all stages of prototyping prior to sea trials. Especially for checking embedded hardware implementations connected to modems by providing realistic propagation delays. 

# Installation

Simply clone the repository and use pip to install a few dependencies.

`https://github.com/bensherlock/nm3-python-simulator`

# Dependencies

## zmq

Provides the network connectivity between controller and client nodes. https://zeromq.org/

`pip install zmq`

## pyserial

Provides the connection to hardware nodes connected via USB-RS232 cables. 

`pip install pyserial`

## matplotlib

Provides the visualisations. https://matplotlib.org/

`pip install matplotlib`

# Usage

## Controller

A controller instance is always required. Has two network ports, one for node communications (network_port) and one for publishing (publish_port).

`D:\nm3-python-simulator>python -m nm3sim.controller --network_address 127.0.0.1 --network_port 8080 --publish_port 8081`

## Transponder Client

Connects to the node port (network_port) of the controller.

`D:\nm3-python-simulator>python -m nm3sim.transponder_client --network_address 127.0.0.1 --network_port 8080 --position (100,-100,10) --label "A" --address 2`

## Terminal Client

Connects to the node port (network_port) of the controller.

`D:\nm3-python-simulator>python -m nm3sim.terminal_client --network_address 127.0.0.1 --network_port 8080 --position (-100,100,10) --label "B" --address 3`

## Serial Client

Connects to the node port (network_port) of the controller.

`D:\nm3-python-simulator>python -m nm3sim.serial_client --network_address 127.0.0.1 --network_port 8080 --position (-100,100,10) --label "A" --address 4 --serial_port "COM5"`

## Map Visualisation Client

Connects to the publish port of the controller.

`D:\nm3-python-simulator>python -m nm3sim.mapvis_client --network_address 127.0.0.1 --network_port 8081`

# Examples

## Simple TDMA Example

This will run all the necessary components including Controller, Sensor Nodes and Gateway Node, plus Logger and Visualisation. The network protocol displayed is a simple TDMA wth the Gateway transmitting a Beacon broadcast and the Sensor Node then sending a Unicast duirng their alloted timeslot.  

`D:\nm3-python-simulator>python -m examples.simple_example`

# Issues

Please log any issues to https://github.com/bensherlock/nm3-python-simulator/issues
