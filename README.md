# arduino-digihead
Head unit for display of Digijet/Digifant ECU signals captured with arduino-digicap project

## Docs

Max3232 Transceiver Breakout --
https://www.sparkfun.com/products/11189

Mayhew Labs I2C extender -- http://mayhewlabs.com/products/i2c-power-extender



## Development Setup

Arduino based workflow is based on [PlatformIO 3.1][pio].

On a Mac, use `homebrew` to install PlatformIO. Homebrew install instructions at [brew.sh][brew].

    `brew install platformio`

Checkout dependency libraries in central place (`~/arduino/libs`):

    mkdir -p ~/arduino/libs
    
    cd ~/arduino/libs
    
    git clone https://github.com/ohhorob/arduino-MTS.git MTS
    git clone https://github.com/ohhorob/arduino-RKW_Led RKW_Led

Then use `pio` to install platformio on the command line:

    
    cd ~/arduino/
    git clone https://github.com/ohhorob/arduino-digihead digihead
    cd digihead
    pio init --board teensy31 --ide clion

I use [IntelliJ CLion IDE](clion). The integration with PlatformIO is decent, and seems to be getting better.


## In Progress List

### Packet reader

Packet decoding updated to ready single bytes in a byte buffer. When the buffer has enough bytes for the whole packet,
the packet class builds into a packet buffer.


## TODO List



### Drive summary



[brew]: http://brew.sh "brew.sh"
[pio]: http://platformio.org
[clion]: https://www.jetbrains.com/clion/