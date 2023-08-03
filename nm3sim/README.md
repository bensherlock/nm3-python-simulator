# Source code

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


To be continued...
