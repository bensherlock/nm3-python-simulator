# Source code

## Node
+ NodeBase class
+ NodePacket class

The BaseNode holds a 3d position in the virtual sea. Changes to the position result in NodePackets being sent to the simulation controller. These positions are then used by the controller to determine propagation delay and transmission losses. Visualisations use the node positions as centre points for displaying the outward propagating acoustic signals.  


## Modem
+ Modem class
+ ModemPacket class
+ AcousticPacket class

The Modem has an inputstream interface exposed to the client (console client, serial port, etc) and sends and receives acoustic messages via the ZMQ interface.
The Modem has a ModemStatus:
+ Listening - The Modem is waiting for an acoustic packet and will continue to respond to SerialCommands.
+ Receiving - The Modem is demodulating an incoming acoustic packet and will not respond to SerialCommands.
+ Transmitting - The Modem is currently transmitting an acoustic packet and will not respond to SerialCommands.
+ SerialCommunicating - The Modem is currently processing SerialCommands and will not synch onto incoming acoustic packets.
+ Sleeping - The Modem is asleep and not listening for acoustic packets it will respond to SerialCommands and wakeup.

ReceiveChannelStatus covers the simulated front end acoustic receiver of the Modem:
+ Silent - Only ambient noise is being received by the front end.
+ SinglePacketArrival - A single acoustic transmission is currently arriving at the front end. The Modem is likely to be in Receiving state now.
+ OverlappedPacketArrival - More than one acoustic transmission is currently arriving at the front end. For now this will result in both acoustic packets counting as failed.
+ Saturated - A transmission is currently ongoing so no incoming acoustic transmissions would be received by the front end.

The Modem currently replicates the BaseNode properties rather than inheriting from or encapuslating it - this may change later.


## Client Applications
+ serial_client application
+ terminal_client application
+ transponder_client application

Each of these client applications instantiates a Modem that connects to the controller via the ZMQ socket. The serial interface is exposed to the user via the serial port, or a terminal window, or not at all in the case of the transponder. 

For custom network simulations, using these as a starting example, the network layer algorithms would run within this client application.


To be continued...
