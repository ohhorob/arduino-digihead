# arduino-digihead
Head unit for display of Digijet/Digifant ECU signals captured with arduino-digicap project

## Docs

https://github.com/ohhorob/arduino-digihead/wiki

https://www.sparkfun.com/products/11189


## Development Setup

Workflow is based around PlatformIO.

On a Mac, use `homebrew` to install PlatformIO.

I use IntelliJ CLion IDE. The integration with PlatformIO is decent, and seems to be getting better.



## In Progress List

### Packet reader

Packet decoding updated to ready single bytes in a byte buffer. When the buffer has enough bytes for the whole packet,
the packet class builds into a packet buffer.


## TODO List

### Drive summary

