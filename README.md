DavisRFM69 Library
-------------------
By DeKay (dekaymail@gmail.com)
<br>
Creative Commons Attribribution Share-Alike License
http://creativecommons.org/licenses/by-sa/3.0/

This library is an extension to [LowPowerLab's RFM69 library](https://github.com/LowPowerLab/RFM69) that enables reception of weather data from a Davis Instruments Integrated Sensor Suite (ISS) weather station.

##Background
The Davis Instruments ISS is a solar powered and battery-backed set of outdoor weather sensors monitored by a PIC microcontroller.  It uses a TI CC1020 RF Transmitter chip to send the weather data it collects back to a Davis Vantage VP2 or Vantage Vue console located indoors.  The ISS transmissions have been reverse engineered, and this has allowed receivers based on the TI CC1110 chip (amongst others) to receive its transmissions.

The drawback of the CC1110 is that it has its own embedded microcontroller [that requires separate hardware to program](http://madscientistlabs.blogspot.ca/2012/01/troubles-with-im-me-and-goodfet.html).  The simpler CC1101 does not have this controller and can be commanded over a SPI bus, but it is difficult to find this board in a 915 MHz flavor as used by Davis ISS units in North America.  Beware EBay units saying they are 915 MHz: they likely actually operate at 433 MHz and are of no use.  [Ask me how I know...](http://madscientistlabs.blogspot.ca/2013/04/dead-end.html)

The one potential bright spot in the CC11xx story is the [RFBee](http://www.seeedstudio.com/depot/rfbee-v11-wireless-arduino-compatible-node-p-614.html).  It couples a TI CC1101 with an Arduino and is easy to use and program, but it is based on an Atmega 168 whose limited RAM and FLASH have hampered efforts to implement a full blown emulation of the Davis indoor console.

The new kid on the block is the RFM69 module from HopeRF.  This module is inexpensive and can be bought either standalone or integrated on a ["Moteino"](http://lowpowerlab.com/blog/category/moteino/) Atmega 328 Arduino clone from LowPowerLabs.  This library demonstrates that the RFM69 is flexible enough to receive transmissions from the TI transmitter chip in the ISS.

**Note**: the previous generation HopeRF RFM12B module popularized by the [JeeNode](http://jeelabs.net/projects/cafe/wiki/Dive_Into_JeeNodeshttp://jeelabs.net/projects/cafe/wiki/Dive_Into_JeeNodes)...
- has hardcoded preamble bytes that prevent its use in this application
- has a very small FIFO that makes you [jump through hoops](https://github.com/gkaindl/rfm12b-linux) if you don't have something real-time to respond to incoming data
- [is no longer recommended for new designs](http://jeelabs.org/2013/06/28/status-of-the-rfm12b/)

## Features
This library sniffs the wireless packets transmitted from a Davis ISS.  Other features are on the drawing board.  After all, why just receive?

This library is in its early stages but I wanted to get something functional out there.  It has been developed on a Moteino R3 [(see here for the new R4 version)](http://lowpowerlab.com/shop/Moteino-R4)
fitted with an RFM69W (Semtech SX1231/SX1231) transceiver module.

##Installation
Install LowPowerLab's RFM69 library as described [on its Github page](https://github.com/LowPowerLab/RFM69).

Copy the content of this library to a folder called "DavisRFM69" in your Arduino library folder.  This will be a subfolder in your "Sketchbook location" specified in File>Preferences in the Arduino IDE.  See [this tutorial](http://learn.adafruit.com/arduino-tips-tricks-and-techniques/arduino-libraries) on Arduino libraries.

Easier yet is to just "git clone" the RFM69 and DavisRFM69 code from within your Arduino "libraries" folder.

##Miscellaneous / Possible Issues

I am aware of these issues:
- the first packet received after powerup on Channel 0 always has a bad CRC
- the 42nd channel always has a bad CRC (WTH?)
- error handling: there really isn't any yet

Please let me know if you find other issues.

My DavisRFM69 library subclasses the RFM69 library and overrides a couple of functions.  I've made some trivial changes to the LowPowerLab's RFM69 library in the process and have issued a pull request.  In the meantime, you'll want to use [my forked RFM69 library](https://github.com/dekay/RFM69) instead of LowPowerLab's.

##Sample Usage
[ISSRx](https://github.com/dekay/DavisRFM69/blob/master/Examples/ISSRx/ISSRx.ino) is an example that uses the library to listen in on a Davis ISS.  A new line of data spits out every 2.5 seconds, assuming the ISS you are listening to is on Channel 1 (the packet rate slows by 62.5 ms for every Channel ID higher than 1).

    Listening at 915 Mhz...
    SPI Flash Init OK!
    0 - Data: 90 D 7D AE 7D 89 38 1C   RSSI: -110dBm   CRC: 4C14
    1 - Data: E0 F 85 47 3 0 42 D2   RSSI: -72dBm   CRC: 42D2
    2 - Data: 50 D 85 FF 71 0 C4 98   RSSI: -74dBm   CRC: C498
    3 - Data: 40 B 85 FF C1 0 D 94   RSSI: -75dBm   CRC: D94
    4 - Data: 80 C 85 4 49 0 55 E2   RSSI: -80dBm   CRC: 55E2
    5 - Data: E0 B 85 47 3 0 CB D4   RSSI: -75dBm   CRC: CBD4

The first field is the channel number, the data is the eight data bytes sent by the ISS on every transmission, RSSI is the Received Signal Strength Indication, and the CRC is calculated from the first six bytes in the data packet.  Every packet is sent with a CRC in the seventh and eith bytes, so those two values will agree with the calculated CRC if the packet is good.  All of this is documented [here](https://github.com/dekay/im-me/blob/master/pocketwx/src/protocol.txt).

Note the first packet has a bad CRC as noted above.

##Blog Writeup
[Right here](http://madscientistlabs.blogspot.ca/2014/01/more-than-one-way-to-skin-cat.html), along with the best GIF ever.

##Why
I started playing around with my VP2 Wireless console when I discovered its little expansion port tucked away in the back.  Its purpose is primarily for connection of an exorbitantly priced datalogger that is little more than a one dollar flash chip.  After figuring out how to connect [first a serial interface](http://madscientistlabs.blogspot.ca/2011/01/davis-weatherlink-software-not-required.html) and then [building my own datalogger](http://madscientistlabs.blogspot.ca/2011/10/build-your-own-davis-console-datalogger.html), I figured out the [wireless protocol between the ISS and the console](http://madscientistlabs.blogspot.ca/2012/03/first-you-get-sugar.html) and put together the first standalone receive using an [IM-ME Pretty Pink Pager](http://madscientistlabs.blogspot.ca/2012/04/achievement-unlocked-im-me-weather.html).

I learned a lot by doing this and I like to think that opening up the console has been an overall win for Davis.  I also consider this to be MY data, and I want access to it ([Davis' failed attempts to shut this down notwithstanding](http://meteo.annoyingdesigns.com/DavisSPI.pdf)).

And, just because.