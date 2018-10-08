DavisRFM69 Library
-------------------
By DeKay (dekaymail@gmail.com)
<br>
Creative Commons Attribribution Share-Alike License
http://creativecommons.org/licenses/by-sa/3.0/

This library is a modified version of the [LowPowerLab's RFM69 library](https://github.com/LowPowerLab/RFM69) that enables reception of weather data from a Davis Instruments Integrated Sensor Suite (ISS) weather station.

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

This library is in its early stages but I wanted to get something functional out there.  Churn should be expected for the next little while.

This library has been developed on a Moteino R3 [(see here for the new R4 version)](http://lowpowerlab.com/shop/Moteino-R4)
fitted with an RFM69W (Semtech SX1231/SX1231) transceiver module.

Support has also been added for ESP8266 modules.  The ISSRx_ESP example was developed on a NodeMCU ESP-12E based module connected directly to an RFM69W transceiver.  See the code for a description of the hookup.

##Installation
[See this blog post](http://madscientistlabs.blogspot.ca/2014/02/build-your-own-davis-weather-station_17.html) where I combines ISS Reception capabilities along with hookups to sensors for indoor monitoring of temperature, pressure, and humidity.  Note that this code no longer requires the installation of [LowPowerLab's RFM69 library](https://github.com/LowPowerLab/RFM69) discussed in that post, but you will still need to install his [SPIFlash library](https://github.com/LowPowerLab/SPIFlash).  Since that post was written, I have also added support for the DS3231 Real Time Clock chip.  To get this to work from a software perspective, you will need [this RTC library](https://github.com/mizraith/RTClib) in your Arduino "libraries" folder.

##Miscellaneous / Possible Issues

Reception quality has been greatly improved in this release.  ~~There looks to be a bug where the hop-ahead code has broken, but I expect that will be fixed soon~~ I am getting around 99% good packets now.  Please let me know if you find any issues.

##Sample Usage
[ISSRx](https://github.com/dekay/DavisRFM69/blob/master/examples/ISSRx/ISSRx.ino) is an example of sniffing the wireless packets out of the air and streaming them out the serial port.  A version for the ESP8266 is in progress. 

[VP2](https://github.com/dekay/DavisRFM69/blob/master/Examples/VP2/VP2.ino) is an emulation of the Davis Vantage Pro2 console that works with Sandaysoft's Cumulus weather software.  As noted above, you will need [this RTC library](https://github.com/mizraith/RTClib) in your Arduino "libraries" folder.  Unfortunately, this example will be broken right now as I've concentrated on the ESP8266 for now.

##Blog Writeups
[ISS Reception](http://madscientistlabs.blogspot.ca/2014/01/more-than-one-way-to-skin-cat.html), along with the best GIF ever.

[Davis Console Emulation](http://madscientistlabs.blogspot.ca/2014/02/build-your-own-davis-weather-station_17.html) combines ISS Reception capabilities along with hookups to sensors for indoor monitoring of temperature, pressure, and humidity.

##Why
I started playing around with my VP2 Wireless console when I discovered its little expansion port tucked away in the back.  Its purpose is primarily for connection of an exorbitantly priced datalogger that is little more than a one dollar flash chip.  After figuring out how to connect [first a serial interface](http://madscientistlabs.blogspot.ca/2011/01/davis-weatherlink-software-not-required.html) and then [building my own datalogger](http://madscientistlabs.blogspot.ca/2011/10/build-your-own-davis-console-datalogger.html), I figured out the [wireless protocol between the ISS and the console](http://madscientistlabs.blogspot.ca/2012/03/first-you-get-sugar.html) and put together the first standalone receive using an [IM-ME Pretty Pink Pager](http://madscientistlabs.blogspot.ca/2012/04/achievement-unlocked-im-me-weather.html).

I learned a lot by doing this and I like to think that opening up the console has been an overall win for Davis.  I also consider this to be MY data, and I want access to it ([Davis' failed attempts to shut this down notwithstanding](http://meteo.annoyingdesigns.com/DavisSPI.pdf)).

And, just because.
