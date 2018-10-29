# Arduino Code for Rocket Avionics
Lastly tested on the genuine Arduino Micro board.

### Installation
Upload the code in a Arduino Micro board with the most recent version of Arduino IDE.  
Be careful with the wire is connected correctly as written in the software!  
Arduino board sometimes send corrupted signals if the baud is too fast. Lower the baud rate if the XBee module does not seem to receive data properly. Any alteration of the packet content should also be followed by the decoder. Packet size mismatch between the core and the decoder will result in retrieval of empty data.

### Credit
[Jaerin Lee](https://github.com/ironjr)  
Lead Electronics and Software Developer  
SNU Rocket Team Hanaro