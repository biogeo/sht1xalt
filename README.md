sht1xalt: An "alternative" Arduino library for the SHT1x sensor
===============================================================

Provides an interface to the SHT1x temperature / humidity series of sensors from
Sensirion <http://www.sensirion.com>. I wrote this somewhat by accident when
trying to debug errors that arose when I tried to both Jonathan Oxer's SHT1x
library <http://github.com/practicalarduino/SHT1x> and Markus Schatzl's
Sensirion library <http://playground.arduino.cc/Code/Sensirion#Sensirion>. If
this library doesn't work for you, check out those!

Installation
------------
Download the sht1xalt directory into the "sketchbook/libraries" directory, then
restart the Arduino IDE. You can upload an example sketch to your Arduino from
File->Examples->sht1xalt.

Usage
-----
This library provides a class for managing interactions with the SHT1x, as well
as several associated functions, constants, and datatypes. These are all
provided within the sht1xalt namespace. If you're not familiar with namespaces,
all you need to know is that all of the identifiers provided by the library need
to be preceded by `sht1xalt::`, OR you can include the following line after the
`#include <sht1xalt.h>` line in your sketch:

    using namespace sht1xalt;

The only downside to the "using namespace" command is that if you also want to
use another library in your sketch, and that library happens to use any of the
same names that sht1xalt uses (e.g., "Sensor"), then you'll get errors. But if
sht1xalt is the only library you're using, then there should be no trouble.

Typical usage of this library will consist of creating a global Sensor object to
handle communication, configuring the interface in the `setup()` function, and
calling measurement functions in the `loop()` function. So a typical sketch will
look something like this:

    #include <sht1xalt.h>
    #define dataPin 10
    #define clockPin 11
    #define clockPulse 1
    #define voltage sht1xalt::VOLTAGE_5V
    #define units sht1xalt::UNITS_CELCIUS
    sht1xalt::Sensor sensor(dataPin, clockPin, clockPulse, voltage, units);
    
    void setup() {
      // Do some things
      sensor.configureConnection();
      sensor.softReset();
      // Do more things
    }
    
    void loop() {
      float temp;
      float rh;
      // Do some things
      sensor.measure(temp,rh);
      // Do more things
    }

That will cover most use cases, but you can get fancy and do things like put the
SHT1x in low resolution, low power consumption mode, or turn on the chip's
heater to raise its temperature.

### Features ###
- Read raw temperature and relative humidity values (great for logging or
  interfacing with a PC which can do the "hard" work of conversion)
- Read temperature in Celcius or Fahrenheit
- Read relative humidity as a percentage, without temperature compensation
- Read temperature and relative humidity together, with temperature compensation
  of the relative humdity (important if temperature is much different from 25 C
  or 77 F)
- Read/write the SHT1x's status register, allowing:
  - Toggling low resolution, low power consumption mode
  - Controlling the onboard heater
  - Controlling "OTP reload" (I don't know what this is, but if you do, you can
    set it!)
  - Querying the sensor's low voltage ("battery low") detector
- Set clock pulse width: longer pulses may help communication if the wires
  between the Arduino and the SHT1x are very long

### Limitations ###
- CRC checksum isn't yet implemented
- The library is intended for easy use, not optimal speed or size. If you need
  to pack a lot of other code on your Arduino, many of the lookup tables could
  be replaced with hard-coded values for your setup (e.g., if you know the
  operating voltage, temperature units, and clock speed you need.) That said, a
  minimalist test sketch using this library gave me a sketch size of 6524 bytes
  (of a 30720 byte maximum) on my Duemilanova, so there's plenty of room for a
  lot more code still.

