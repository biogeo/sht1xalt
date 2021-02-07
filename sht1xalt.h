/**
 * An "alternative" library for interfacing with SHT1x
 * Copyright 2013 Geoffrey Adams: github.com/biogeo/sht1xalt
 * Loosely based on the following libraries:
 *   SHT1x, GPL license, by:
 *     Jonathan Oxer: www.practicalarduino.com
 *     Maurice Ribble: www.glacialwanderer.com/hobbyrobotics/?p=5
 *     Wayne ?: ragingreality.blogspot.com/2008/01/ardunio-and-sht15.html
 *   Sensirion, public domain, by:
 *     Markus Schatzl
 *     Carl Jackson
 *
 * Manages communication with SHT1x temperature and humidity sensors from
 * Sensirion (www.sensirion.com). The previously existing libraries (SHT1x and
 * Sensirion) weren't working properly for me, and in the process of debugging
 * I ended up creating this.
 *
 * Note that this library uses namespaces. If this is new to you, all that it
 * means for the user is that all of the types, constants, functions, and
 * classes defined in this library need to be preceded with "sht1xalt::" in your
 * code, as in:
 *     sht1xalt::Sensor sensor(dataPin,
 *                             clockPin,
 *                             sht1xalt::VOLTAGE_5V,
 *                             sht1xalt::UNITS_CELCIUS);
 * OR you can add the line "using namespace sht1xalt;" to the top of your
 * sketch, and then omit the cumbersome "sht1xalt::" for the rest of the sketch.
 * Doing this is a problem only if you happen to want to also use another
 * library that uses any of the same identifiers as sht1xalt (e.g., "Sensor").
 *
 * Features:
 *   - Read raw temperature and relative humidity values (great for logging or
 *     interfacing with a PC which can do the "hard" work of conversion)
 *   - Read temperature in Celcius or Fahrenheit
 *   - Read relative humidity as a percentage, without temperature compensation
 *   - Read temperature and relative humidity together, with temperature
 *     compensation on the RH value
 *   - Read/write the SHT1x's status register, allowing:
 *     - Toggling low resolution, low power consumption mode
 *     - Controlling the onboard heater
 *     - Controlling "OTP reload" (I don't know what this is, but if you do, you
 *       can set it!)
 *   - Set clock pulse width -- longer pulses may help communication if the
 *     wires between the Arduino and the SHT1x are very long.
 * Limitations:
 *   - CRC checksum isn't yet implemented
 *   - Library is intended for easy use, not optimal speed or size. If you need
 *     to pack a lot of other code on your Arduino, many of the lookup tables
 *     could be replaced with hard-coded values for your setup (e.g., if you
 *     know the operating voltage, temperature units, and clock speed you need.)
 *     That said, a minimalist test sketch using this library gave me a sketch
 *     size of 6524 bytes (of a 30720 byte maximum) on my Duemilanova, so
 *     there's plenty of room for a lot more code still.
 **/
#ifndef SHT1XALT_h
#define SHT1XALT_h

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

namespace sht1xalt {
  /*****\
  * TYPE DEFINTIONS
  \*****/
  
  // A type to indicate which temperature units are desired
  typedef enum temp_units_e {
    UNITS_CELCIUS,
    UNITS_FAHRENHEIT
  } temp_units_t;
  
  // A type to indicate the intended operating voltage of the SHT1x, because the
  // temperature sensor is voltage-dependent. These values are taken from a
  // table in the SHT1x datasheet, so presumably if your voltage level isn't
  // exactly one of these, you can just round to the nearest one and get a
  // pretty good approximation.
  typedef enum voltage_e {
    VOLTAGE_5V,
    VOLTAGE_4V,
    VOLTAGE_3V5, // "3v5" means 3.5 volts
    VOLTAGE_3V,
    VOLTAGE_2V5
  } voltage_t;
  
  // A type to indicate the sampling resolution of the SHT1x. Low-res mode is
  // more power saving.
  typedef enum sample_res_e {
    SAMPLE_HIGH_RES, // Temperature 14 bit, RH 12 bit
    SAMPLE_LOW_RES   // Temperature 12 bit, RH 8 bit
  } sample_res_t;
  
  // A type to indicate communication errors with the SHT1x.
  typedef enum error_e {
    ERROR_NONE, // No error
    ERROR_MEASUREMENT_TIMEOUT, // SHT1x took too long to measure
    ERROR_NO_ACK, // SHT1x failed to ACK a byte
    ERROR_CRC // Received data failed CRC-8 checksum (not yet implemented)
  } error_t;
  
  // The main SHT1x sensor interface
  class Sensor {
    public:
      // Class constructor sets up the Sensor object, but doesn't initialize
      // the connection to the Sensor. Call configureConnection() for that.
      Sensor( // The pin to transfer data to and from the sensor:
              int dataPin,
              // The pin to send clock pulses to the sensor:
              int clockPin,
              // The approximate width of clock pulses, in microseconds (note
              // that at the time of this writing, there's a bug in the Arduino
              // delayMicroseconds function so that if clockWidth==0, it will
              // produce very slow transfer times instead of "as fast as 
              // possible" as might be expected, so only use clockWidth>=1):
              word clockWidth,
              // The supply voltage level to the SHT1x, use one of the
              // enumerated values from voltage_t, above:
              voltage_t voltage,
              // The units to express temperature in when converting to a
              // floating point value, currently either UNITS_CELCIUS or
              // UNITS_FAHRENHEIT:
              temp_units_t units
            );
      // Retrieve the currently selected temperature units:
      temp_units_t getUnits();
      // Change the temperature units to be used for conversion to floating
      // point values:
      void setUnits( temp_units_t units );
      // Determine whether the sensor is in high resolution (default) mode, or
      // low resolution (and lower power consumption) mode:
      sample_res_t getResolution();
      // Set up the data and clock pins for communication with the SHT1x sensor.
      // You must call this once before any communication functions, typically
      // within your sketch's setup() function:
      void configureConnection();
      // If communication with the SHT1x is interrupted somehow, it may still
      // have data to transfer in its buffer, or expect to receive data from the
      // Arduino. This resets the connection so that communication can resume:
      void resetConnection();
      
      // Take a temperature measurement and give the raw integer value. The
      // result is stored in the "temperature" parameter. Returns an error code
      // indicating the success or failure of the measurement:
      //     ERROR_NONE:               No error, the measurement was retrieved
      //     ERROR_NO_ACK:             SHT1x failed to acknowledge the command
      //     ERROR_MEASUREMENT_TMEOUT: SHT1x failed to measure the temperature
      //                               within the allotted time
      error_t readTemperatureRaw( word &temperature );
      // Take a relative humidity measurement and give the raw integer value.
      // The retrieved value is set in the "humidity" argument. Returns an error
      // code just like readTemperatureRaw.
      error_t readHumidityRaw( word &humidity );
      // Functions like readTemperatureRaw, but the result is a floating point
      // temperature value, in Celcius or Fahrenheit, depending on what was set
      // in the constructor or with setUnits:
      error_t readTemperature( float &temperature );
      // Functions like readHumidityRaw, but the result is a floating point
      // relative humidity value, as a percentage. This value is not corrected
      // for the current temperature, which will affect the accuracy of the
      // measurement if the temperature is much different from 25 C (77 F):
      error_t readHumidityUncorrected( float &humidity );
      // Measures both temperature and relative humidity. The temperature value
      // is the same as from readTemperature, and the relative humidity value
      // is temperature-compensated for greater accuracy. The error codes
      // ERROR_NO_ACK or ERROR_MEASUREMENT_TIMEOUT will be returned if either
      // measurement generates the error:
      error_t measure( float &temperature,
                       float &humidity );
      // Retrieve the SHT1x's status register. The result is stored in the
      // "status" parameter. Values in the status register can by checked using
      // the isStatus* functions, below, e.g. isStatusBatteryLow(status).
      // Returns ERROR_NONE if successful or ERROR_NO_ACK if
      // the SHT1x fails to acknowledge the command:
      error_t readStatus( byte &status );
      // Set the SHT1x's status register. The following constants are valid
      // inputs:
      //     STATUS_BIT_RES:    To turn on low resolution mode
      //     STATUS_BIT_OTP:    To turn off "OTP reload" (whatever that is)
      //     STATUS_BIT_HEATER: To turn on the heater
      // To set more than one of these at a time, use bitwise "|", as in:
      //     sensor.setStatus(STATUS_BIT_RES | STATUS_BIT_HEATER)
      // which will turn on low resolution mode and the heater. Note that the
      // status register is fully overwritten, so sending just STATUS_BIT_RES
      // will also turn off the heater if it was previously on. Returns
      // ERROR_NONE if successful or ERROR_NO_ACK if either the command or the
      // new status were not acknowledged.
      error_t setStatus( byte status );
      // Sends the soft reset command to the SHT1x. Effectively this resets the
      // status register, but it might do some other things as well (I'm not
      // really sure). This function also resets the connection before sending
      // the soft reset command. Returns ERROR_NONE if successful or
      // ERROR_NO_ACK if the command was not acknowledged.
      error_t softReset();
      
    private:
      // Private data and functions cannot be accessed outside the class.
      
      // Properties:
      int myDataPin; // Which pin to use for transferring data
      int myClockPin; // Which pin to use for the clock pulses
      word myClockWidth; // Approximate clock pulse width, in us
      voltage_t myVoltage; // Sensor operating voltage level
      temp_units_t myUnits; // Currently requested units for temperature
      sample_res_t myResolution; // Current measurement resolution
      
      // Methods:
      
      // Send the "Transmission Start" sequence to the sensor to tell it that a
      // command is coming. TS consists of the following sequence of events on
      // the DATA and SCK (clock) lines:
      //   While SCK is high, DATA falls to low. Then SCK falls low, then rises
      //   high. Then DATA rises high.
      // Or, graphically (and accounting for SCK being normally LOW):
      //          _____       _____
      // SCK:  __|     |_____|     |__
      //       _____             _____
      // DATA:      |___________|
      // Always returns ERROR_NONE.
      error_t initiateTransmission();
      // Sends a byte to the sensor. This consists of 8 clock pulses on which
      // the byte is transmitted, followed by a 9th clock pulse on which the
      // SHT1x must send and ACK signal (by pulling the DATA line low). Returns
      // ERROR_NONE if the ACK signal is received, or ERROR_NO_ACK if it is not.
      error_t sendByte(byte message);
      // After a temperature or humidity measurement command, the SHT1x allows
      // the DATA line to float high until the measurement is completed, at
      // which time it signals that the measurement is ready by pulling DATA
      // low. This function waits for that signal, up to 1000 ms. Returns
      // ERROR_NONE if the DATA line is pulled low before timeout, otherwise
      // ERROR_MEASUREMENT_TIMEOUT.
      error_t waitForMeasurement();
      // Receives a byte from the sensor. This consists of 8 clock pulses on
      // which the byte is transmitted. Generally the Arduino must also transmit
      // an ACK signal (a 9th clock pulse on which DATA is low). However, this
      // ACK signal can be omitted from the second byte of temperature and
      // humidity measurements, in order to skip the CRC checksum byte. The
      // parameter "ack" controls whether the ACK signal is sent.
      // The received byte is stored in "message".
      // Always returns ERROR_NONE.
      error_t receiveByte(byte &message, bool ack);
  };
  
  /*****\
  * INTERFACE CONSTANTS
  \*****/
  
  // Command bytes for interfacing with the SHT1x:
  const byte CMD_READ_TEMPERATURE = 0b00000011;
  const byte CMD_READ_HUMIDITY    = 0b00000101;
  const byte CMD_READ_STATUS      = 0b00000111;
  const byte CMD_WRITE_STATUS     = 0b00000110;
  const byte CMD_SOFT_RESET       = 0b00011110;
  
  // Single-bit masks for the status register of the SHT1x:
  const byte STATUS_BIT_RES    = 0b00000001; // Data resolution, 0=high, 1=low
  const byte STATUS_BIT_OTP    = 0b00000010; // "No OTP reload" (no clue)
  const byte STATUS_BIT_HEATER = 0b00000100; // Is the heater is on?
  const byte STATUS_BIT_LOW_V  = 0b01000000; // Is the voltage low (low batt)?
  
  // Some inline functions for testing the status byte:
  inline bool isStatusBatteryLow( byte status ) {
    return (status & STATUS_BIT_LOW_V);
  }
  inline bool isStatusHeaterOn( byte status ) {
    return (status & STATUS_BIT_HEATER);
  }
  inline bool isStatusNoReloadOtp( byte status ) {
    return (status & STATUS_BIT_OTP);
  }
  inline bool isStatusLowRes( byte status ) {
    return (status & STATUS_BIT_RES);
  }
  
  // The time the SHT1x has to complete a measurement before we give up on it:
  const int MEASUREMENT_TIMEOUT = 1000; // Could make this settable later.
  
  /*****\
  * CONVERSION CONSTANTS
  * The following constants store tables of values which are used in converting
  * the raw temperature and relative humidity observations into interpretable
  * floating-point values. They are used with the conversion formulas, below,
  * and their values are taken from the SHT1x datasheet.
  \*****/
  
  // Temperature conversion offest.
  // Index as CONV_D1[temperature_units][voltage]
  const float CONV_D1[][5] = {
    {-40.1, -39.8, -39.7, -39.6, -39.4},
    {-40.2, -39.6, -39.5, -39.3, -38.9}
  };
  // Temperature conversion slope.
  // Index as CONV_D2[temperature_units][sample_resolution]
  const float CONV_D2[][2] = {
    {0.010, 0.040},
    {0.018, 0.072}
  };
  
  // Linear RH conversion offset.
  const float CONV_C1 = -2.0468;
  // Linear RH conversion first order coefficient.
  // Index as CONV_C2[resolution]
  const float CONV_C2[] = {0.0367, 0.5872};
  // Linear RH conversion second order coefficient.
  // Index as CONV_C3[resolution]
  const float CONV_C3[] = {-1.5955e-6, -4.0845e-4};
  
  // RH temperature compensation zero-point.
  // Index as CONV_T0[temperature_units]
  const float CONV_T0[] = {25.0, 77.0};
  // RH temperature compensation offset.
  // Index as CONV_T1[temperature_units]
  const float CONV_T1[] = {0.01, 0.005556};
  // RH temperature compensation slope.
  // Index as CONV_T2[temperature_units][resolution]
  const float CONV_T2[][2] = {
    {0.00008, 0.00128},
    {4.444e-5, 7.111e-4}
  };
  
  // RH temperature compensation zero-point.
  // Index as CONV_A1[temperature_units]
  const float CONV_A1[] = {-25.0, -42.77778};
  // RH temperature compensation unit conversion (to put in Celcius)
  // Index as CONV_A1[temperature_units]
  const float CONV_A2[] = {1.0, 0.55556};
  // So that now:
  //     rh = (a1 + a2 * temp) * (t1 + t2 * rh_raw)
  //          + c1 + c2 * rh_raw + c3 * rh_raw * rh_raw
  // where a1 and a2 depend on units.
  
  /*****\
  * CONVERSION FUNCTIONS
  * The following inline functions convert raw temperature and relative humidity
  * into interpretable values. The formulas are taken from the SHT1x datasheet,
  * and use values from the tables above.
  \*****/
  
  // True temperature is a linear function of the measured raw value:
  //   temp_float = d1 + d2 * temp_raw
  // with:
  //   d1 = CONV_D1[temperature_units][voltage]
  //   d2 = CONV_D2[temperature_units][sample_resolution]
  inline float temperatureRawToFloat( word temperature,
                                      temp_units_t units,
                                      voltage_t voltage,
                                      sample_res_t resolution ) {
    return ( CONV_D1[units][voltage]
             + CONV_D2[units][resolution] * temperature );
  }
  
  // The "linear" relative humidity measurement (i.e., an estimate that does not
  // depend on temperature) is a quadratic function of the measured raw value:
  //     rh_lin = c1 + c2 * rh_raw + c3 * rh_raw * rh_raw
  // with:
  //     c1 = CONV_C1
  //     c2 = CONV_C2[resolution]
  //     c3 = CONV_C3[resolution]
  inline float humidityRawToLinear( word humidity,
                                    sample_res_t resolution ) {
    return ( CONV_C1
             + CONV_C2[resolution] * humidity
             + CONV_C3[resolution] * humidity * humidity);
  }
  
  // The formula to compensate for temperature effects on humidity is:
  //     rh = (temp_c - 25.0) * (t1 + t2 * rh_raw) + rh_lin
  // or:
  //     rh = (temp_c - 25.0) * (t1 + t2 * rh_raw) + c1 + c2 * rh_raw
  //           + c3 * rh_raw * rh_raw
  // where t1 is a true constant,and t2 depends on the sampling resolution:
  
  // The SHT1x datasheet gives the formula to compensate for the effect of
  // temperature on the humidity measurements only for units of Celcius:
  //     rh = (temp_c - 25.0) * (t1 + t2 * rh_raw) + rh_lin
  // with tables for t1 and t2. However, it's more convenient to be able to
  // supply the temperature in any unit, which just requires converting the 25
  // degree zero-point and supplying appropriate values for t1 and t2. That is,
  //     rh = (temp - t0) * (t1 + t2 * rh_raw) + rh_lin
  // with:
  //     t0 = CONV_T0[temperature_units]
  //     t1 = CONV_T1[temperature_units]
  //     t2 = CONV_T2[temperature_units][resolution]
  inline float humidityRawToCompensated( word humidity,
                                         float temperature,
                                         temp_units_t units,
                                         sample_res_t resolution ) {
   return ( (temperature - CONV_T0[units])
            * (CONV_T1[units] + CONV_T2[units][resolution] * humidity)
            + humidityRawToLinear(humidity, resolution) );
  }
  
  // I haven't gotten around to writing a dewpoint calculator yet!
  //float dewPoint(float temperature, float humidity, temp_units_t units);
}

#endif
