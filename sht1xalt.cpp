/**
 * An "alternative" library for interfacing with SHT1x
 * Copyright 2013 Geoffrey Adams: github.com/biogeo/sht1xalt
 * See sht1xalt.h for more information.
 **/
#include "sht1xalt.h"

using namespace sht1xalt;

// Class constructor for the Sensor object initializes the internal data storage
// with supplied or default values. For usage information see sht1xalt.h
Sensor::Sensor(
               int dataPin, // Pin to transmit data
               int clockPin, // Pin to transmit clock pulses
               word clockWidth, // Approximate clock pulse width, microseconds
               voltage_t voltage, // Operating voltage level
               temp_units_t units // Desired temperature units for conversion
               ) {
  myDataPin = dataPin;
  myClockPin = clockPin;
  myClockWidth = clockWidth;
  myVoltage = voltage;
  myUnits = units;
  myResolution = SAMPLE_HIGH_RES; // The sensor is in high res mode by default
}

// Retrieve the units currently being used to report temperature measurements.
temp_units_t Sensor::getUnits() {
  return myUnits;
}

// Set the units to use for reporting temperature measurements.
void Sensor::setUnits( temp_units_t units ) {
  myUnits = units;
}

// Retrieve the resolution level of the SHT1x, as currently known by the Sensor
// object.
sample_res_t Sensor::getResolution() {
  return myResolution;
}

// Set up the DATA and CLOCK pins. For usage information see sht1xalt.h
void Sensor::configureConnection() {
  // DATA pin is normally high, CLOCK pin is normally low, so set the pullup
  // resistors accordingly.
  digitalWrite(myDataPin, HIGH);
  digitalWrite(myClockPin, LOW);
  // The Arduino will always "own" the CLOCK pin, but takes control of the DATA
  // pin only when it needs to transmit.
  pinMode(myClockPin, OUTPUT);
  pinMode(myDataPin, INPUT); // Let the data pin float until we need it.
}

// Send the connection reset sequence to the SHT1x. This consists of 9 clock
// pulses while the DATA pin floats high. For usage information see sht1xalt.h
void Sensor::resetConnection() {
  // DATA should already be floating on input mode, and CLOCK should already be
  // low on output mode, but if we're calling this function there's a decent
  // chance that something's gone wrong, so we'll try to force the issue.
  pinMode(myDataPin, INPUT);
  pinMode(myClockPin, OUTPUT);
  digitalWrite(myDataPin, HIGH);
  delayMicroseconds(myClockWidth);
  digitalWrite(myClockPin, LOW);
  // Send 9 clock pulses
  for (int i=0; i<10; i++)
  {
    delayMicroseconds(myClockWidth);
    digitalWrite(myClockPin, HIGH);
    delayMicroseconds(myClockWidth);
    digitalWrite(myClockPin, LOW);
  }
}

// Retrieve a raw temperature reading from the SHT1x. For usage information, see
// sht1xalt.h
// Parameters:
//     word temperature (output): the temperature reading
// Return:
//     error_t ERROR_NONE: if no error occurred
//             ERROR_NO_ACK: if the SHT1x failed to acknowledge the command
//             ERROR_MEASUREMENT_TIMEOUT: if the SHT1x failed to respond in time
error_t Sensor::readTemperatureRaw(word &temperature) {
  byte msb; // Most significant byte (first one received)
  byte lsb; // Least significant byte (second one received)
  //byte crc; // CRC-8 checksum byte (third one received) (Not currently used)
  error_t err; // Transmission error code
  // Send the "transmission start" sequence to tell the SHT1x a command will
  // follow
  initiateTransmission();
  // Send the read temperature command (0b00000011)
  if (err = sendByte(CMD_READ_TEMPERATURE))
    return err;
  // Wait for the SHT1x to pull DATA low
  if (err = waitForMeasurement())
    return err;
  // Receive the first byte, giving an ACK signal to the SHT1x
  if (err = receiveByte(msb, true))
    return err;
  // Receive the second byte, omitting the ACK signal to skip the CRC byte
  if (err = receiveByte(lsb, false))
    return err;
  //if (err = receiveByte(crc, true))
  //  return err;
  // The two bytes together comprise the temperature measurement
  temperature = word(msb, lsb);
  return ERROR_NONE;
}

// Retrieve a raw relative humidity reading from the SHT1x. For usage
// information, see sht1xalt.h
// Parameters:
//     word humidity (output): the humidity reading
// Return:
//     error_t ERROR_NONE: if no error occurred
//             ERROR_NO_ACK: if the SHT1x failed to acknowledge the command
//             ERROR_MEASUREMENT_TIMEOUT: if the SHT1x failed to respond in time
error_t Sensor::readHumidityRaw(word &humidity) {
  byte msb; // Most significant byte (first one received)
  byte lsb; // Least significant byte (second one received)
  //byte crc; // CRC-8 checksum byte (third one received) (Not currently used)
  error_t err; // Transmission error code
  // Send the "transmission start" sequence to tell the SHT1x a command will
  // follow
  initiateTransmission();
  // Send the read humidity command (0b00000101)
  if (err = sendByte(CMD_READ_HUMIDITY))
    return err;
  // Wait for the SHT1x to pull DATA low
  if (err = waitForMeasurement())
    return err;
  // Receive the first byte, giving an ACK signal to the SHT1x
  if (err = receiveByte(msb, true))
    return err;
  // Receive the second byte, omitting the ACK signal to skip the CRC byte
  if (err = receiveByte(lsb, false))
    return err;
  //if (err = receiveByte(crc, true))
  //  return err;
  // The two bytes together comprise the humidity measurement
  humidity = word(msb, lsb);
  return ERROR_NONE;
}

// Retrieve a temperature reading from the SHT1x, expressed in the currently
// specified units. For usage information, see sht1xalt.h
// Parameters:
//     float temperature (output): the temperature reading
// Return:
//     error_t ERROR_NONE: if no error occurred
//             ERROR_NO_ACK: if the SHT1x failed to acknowledge the command
//             ERROR_MEASUREMENT_TIMEOUT: if the SHT1x failed to respond in time
error_t Sensor::readTemperature(float &temperature) {
  word raw_temperature;
  error_t err;
  // Retrieve the raw temperature
  if (err = readTemperatureRaw(raw_temperature))
    return err;
  // Convert it to a floating-point value in the currently specified units.
  temperature = temperatureRawToFloat(raw_temperature, myUnits, myVoltage,
      myResolution);
  return ERROR_NONE;
}

// Retrieve a humidity reading from the SHT1x, uncorrected for temperature. This
// value will be pretty accurate near 25 C (77 F), but become biased at higher
// or lower temperatures. For usage information, see sht1xalt.h
// Parameters:
//     float humidity (output): the relative humidity reading, as a percentage
// Return:
//     error_t ERROR_NONE: if no error occurred
//             ERROR_NO_ACK: if the SHT1x failed to acknowledge the command
//             ERROR_MEASUREMENT_TIMEOUT: if the SHT1x failed to respond in time
error_t Sensor::readHumidityUncorrected(float &humidity) {
  word raw_humidity;
  error_t err;
  // Retrieve the raw humidity
  if (err = readHumidityRaw(raw_humidity))
    return err;
  // Convert it to a percentage (% RH)
  humidity = humidityRawToLinear(raw_humidity, myResolution);
  return ERROR_NONE;
}

// Retrieve temperature and humidity readings from the SHT1x, with temperature
// in the specified units and humidity compensated for the temperature. For
// usage information, see sht1xalt.h
// Parameters:
//     float temperature (output): the temperature reading
//     float humidity (output): the relative humidity reading
// Return:
//     error_t ERROR_NONE: if no error occurred
//             ERROR_NO_ACK: if the SHT1x failed to acknowledge the command
//             ERROR_MEASUREMENT_TIMEOUT: if the SHT1x failed to respond in time
error_t Sensor::measure(float &temperature, float &humidity) {
  word raw_humidity;
  error_t err;
  // Retrieve the temperature in the currently specified units
  if (err = readTemperature(temperature))
    return err;
  // Retrieve the raw humidity reading
  if (err = readHumidityRaw(raw_humidity))
    return err;
  // Convert the raw humidity reading to a percentage, compensating for the
  // current temperature
  humidity = humidityRawToCompensated(raw_humidity, temperature,
      myUnits, myResolution);
  return ERROR_NONE;
}

// Retrieve the status register from the SHT1x. For usage information, see
// sht1xalt.h
// Parameters:
//     byte status (output): the status register
// Return:
//     error_t ERROR_NONE: if no error occurred
//             ERROR_NO_ACK: if the SHT1x failed to acknowledge the command
error_t Sensor::readStatus(byte &status) {
  error_t err;
  byte crc; // The CRC-8 checksum
  // Send the "transmission start" sequence to tell the SHT1x a command will
  // follow
  initiateTransmission();
  // Send the read status register command (0b00000111)
  if (err = sendByte(CMD_READ_STATUS))
    return err;
  // Receive the status register
  if (err = receiveByte(status, true))
    return err;
  // Can't skip the CRC for this one (at least, it didn't work for me)
  if (err = receiveByte(crc, true))
    return err;
  // Update myResolution from the returned status, just to be safe
  if (isStatusLowRes(status))
    myResolution = SAMPLE_LOW_RES;
  else
    myResolution = SAMPLE_HIGH_RES;
  return ERROR_NONE;
}

// Set the status register on the SHT1x. For usage information, see sht1xalt.h
// Parameters:
//     byte status (input): the new status register
// Return:
//     error_t ERROR_NONE: if no error occurred
//             ERROR_NO_ACK: if the SHT1x failed to acknowledge the command or
//                           the status register byte
error_t Sensor::setStatus(byte status) {
  error_t err;
  // Only the last three bits of the status register are actually writeable. I
  // haven't tested what happens if you try to write to one of the non-writeable
  // bits (maybe nothing). This will mask out the non-writeable bits:
  const byte STATUS_WRITEABLE_MASK = 0b00000111;
  // Send the "transmission start" sequence to the SHT1x
  initiateTransmission();
  // Send the write status register command (0b00000110)
  if (err = sendByte(CMD_WRITE_STATUS))
    return err;
  // Send the new status register (masking out unwriteable bits):
  if (err = sendByte(status & STATUS_WRITEABLE_MASK))
    return err;
  // Update myResolution from the newly set status
  if (isStatusLowRes(status))
    myResolution = SAMPLE_LOW_RES;
  else
    myResolution = SAMPLE_HIGH_RES;
  return ERROR_NONE;
}

// Send the "soft rest" command to the SHT1x. For usage information, see
// sht1xalt.h
// Return:
//     error_t ERROR_NONE: if no error occurred
//             ERROR_NO_ACK: if the SHT1x failed to acknowledge the command
error_t Sensor::softReset() {
  error_t err;
  // First reset the connection in case that's gone off
  resetConnection();
  // Then send the "transmission start" sequence
  initiateTransmission();
  // Then send the soft reset command (0b00011110)
  if (err = sendByte(CMD_SOFT_RESET))
    return err;
  // After soft reset, the status register should be restored to its default
  // state, so the sampling resolution should be high again:
  myResolution = SAMPLE_HIGH_RES;
  return ERROR_NONE;
}

// Send the "Transmission Start" sequence to the sensor to tell it that a
// command is coming. TS consists of the following sequence of events on the
// DATA and SCK (clock) lines:
//   While SCK is high, DATA falls to low. Then SCK falls low, then rises high.
//   Then DATA rises high.
// Or, graphically (and accounting for SCK being normally LOW):
//          _____       _____
// SCK:  __|     |_____|     |__
//       _____             _____
// DATA:      |___________|
//
// Return:
//     error_t ERROR_NONE: always
error_t Sensor::initiateTransmission() {
  pinMode(myDataPin, OUTPUT);
  delayMicroseconds(myClockWidth);
  digitalWrite(myClockPin, HIGH);
  delayMicroseconds(myClockWidth);
  digitalWrite(myDataPin, LOW);
  delayMicroseconds(myClockWidth);
  digitalWrite(myClockPin, LOW);
  delayMicroseconds(myClockWidth);
  digitalWrite(myClockPin, HIGH);
  delayMicroseconds(myClockWidth);
  digitalWrite(myDataPin, HIGH);
  delayMicroseconds(myClockWidth);
  digitalWrite(myClockPin, LOW);
  pinMode(myDataPin, INPUT);
  return ERROR_NONE;
}

// Send a byte to the sensor. This consists of 8 clock pulses during which 8
// bits are transmitted on the DATA line, followed by a 9th clock pulse during
// which the SHT1x must send an ACK signal (by pulling the DATA line low).
// Parameters:
//     byte message (input): the byte to send
// Return:
//     error_t ERROR_NONE:   if the ACK signal is received
//             ERROR_NO_ACK: if it is not
error_t Sensor::sendByte(byte message) {
  error_t err = ERROR_NONE;
  // Take control of the DATA line
  pinMode(myDataPin, OUTPUT);
  
  // Next write the message in 8 bits on 8 clock pulses:
  //shiftOut(myDataPin, myClockPin, MSBFIRST, message);
  
  // Simply using shiftOut actually works great for me on my Duemilanova with
  // about 40 cm of wires leading to the SHT1x. But evidently longer pulses are
  // needed if the wires are long, since the RC time constant on the DATA and
  // CLOCK lines could become significant. The following loop does the same job
  // as shiftOut, but allowing us to adjust to the width of the pulses. The
  // timing here is somewhat conservative, as low clock pulses probably don't
  // need to be quite as long as high ones, assuming CLOCK and DATA have similar
  // RC time constants.
  
  // writeMask is a mask to select one bit at a time out of the message for
  // transmission. It starts at the most significant bit (0b10000000) and moves
  // right one bit on each iteration. After the 8th iteration, where it is at
  // the least significant bit (0b00000001), rightward bit shift sets it to 0,
  // and the loop terminates.
  for (byte writeMask = 0b10000000; writeMask; writeMask >>= 1) {
    // Write the value of the bit in message under writeMask to DATA
    digitalWrite(myDataPin, (message & writeMask ? HIGH : LOW));
    // Wait one full clock pulse to give DATA time to take on the new value
    delayMicroseconds(myClockWidth);
    // Set CLOCK high to tell the SHT1x to read DATA
    digitalWrite(myClockPin, HIGH);
    // Wait one full clock pulse to give CLOCK time to take on the new value
    delayMicroseconds(myClockWidth);
    // Set CLOCK back to low now that the SHT1x has had time to read DATA
    digitalWrite(myClockPin, LOW);
  }
  
  // Now set DATA to input mode so we can check for the ACK signal
  pinMode(myDataPin, INPUT);
  digitalWrite(myDataPin, HIGH); // Set internal pullup resistor high
  // Wait a full clock pulse to give CLOCK time to fall low after the last
  // write operation
  delayMicroseconds(myClockWidth);
  // Now set CLOCK high again for the ACK signal
  digitalWrite(myClockPin, HIGH);
  // Give it time to adjust again...
  delayMicroseconds(myClockWidth);
  // Check to make sure that the SHT1x has pulled the DATA line low
  if (digitalRead(myDataPin) != LOW)
    err = ERROR_NO_ACK; // If it didn't, that's a NO_ACK error!
  // Set CLOCK low again
  digitalWrite(myClockPin, LOW);
  return err;
}

// After a temperature or humidity measurement command, the SHT1x allows the
// DATA line to float high until the measurement is completed, at which time it
// signals that the measurement is ready by pulling DATA low. This function
// waits for that signal, up to MEASUREMENT_TIMEOUT (1000) ms.
// Return:
//     error_t ERROR_NONE:                if the DATA line is pulled low before
//                                        timeout
//             ERROR_MEASUREMENT_TIMEOUT: otherwise
error_t Sensor::waitForMeasurement() {
  unsigned long waitStart = millis(); // Get the current uptime
  // Keep waiting as long as the current time is less than the original time
  // plus MEASUREMENT_TIMEOUT
  while (millis() < waitStart + MEASUREMENT_TIMEOUT) {
    // Check the DATA line
    if (digitalRead(myDataPin) == LOW)
      return ERROR_NONE; // All is well, we're done now
    else
      delay(1); // Still no measurement, keep waiting...
  }
  // We reached MEASUREMENT_TIMEOUT and the DATA line never fell low. That's an
  // error!
  return ERROR_MEASUREMENT_TIMEOUT;
}

// Receive a byte from the sensor. This consists of 8 clock pulses during which
// 8 bits are received on the DATA line. Generally the Arduino must also
// transmit an ACK signal (a 9th clock pulse on which DATA is low). However,
// this ACK signal can be omitted from the second byte of temperature and
// humidity measurements, in order to skip the CRC checksum byte.
// Parameters:
//     byte message (output): The byte received
//     bool ack (input):      Whether to send the ACK signal
// Return:
//     error_t ERROR_NONE: Always.
error_t Sensor::receiveByte(byte &message, bool ack) {
  // Read 8 bits in 8 clock pulses:
  //message = shiftIn(myDataPin, myClockPin, MSBFIRST);
  
  // Like with sendByte, shiftIn works great for my on my Duemilanova with about
  // 40 cm of wire leading to the SHT1x, but the following loop does the same
  // job while also allowing the clock pulse width to be adjusted.
  
  // First initialize the received byte to 0:
  message = 0;
  // On each iteration, we read the state of the DATA line as 1 or 0. This is
  // the new least significant bit for the message, pushing previous bits
  // leftward.
  for (byte i=0; i < 8; i++) {
    message <<= 1; // Shift received message left one bit
    delayMicroseconds(myClockWidth); // Inter-clock interval
    digitalWrite(myClockPin, HIGH); // Begin clock pulse
    delayMicroseconds(myClockWidth); // Wait until the end of the clock to read
    message |= digitalRead(myDataPin); // Set message lsb to data pin value
    digitalWrite(myClockPin, LOW); // End clock pulse
  }
  
  // Send ACK on the 9th clock pulse, if ACK is requested; otherwise skip it.
  if (ack) {
    pinMode(myDataPin, OUTPUT);
    digitalWrite(myDataPin, LOW);
    delayMicroseconds(myClockWidth);
    digitalWrite(myClockPin, HIGH);
    delayMicroseconds(myClockWidth);
    digitalWrite(myClockPin, LOW);
    pinMode(myDataPin, INPUT);
    digitalWrite(myDataPin, HIGH);
  }
  return ERROR_NONE; // No real errors possible here!
}

