/**
 * A simple example demonstrating usage of the sht1xalt library.
 **/

#include <Arduino.h>
#include <sht1xalt.h>

// Set these to whichever pins you connected the SHT1x to:
#define dataPin 10
#define clockPin 11

// Set this number larger to slow down communication with the SHT1x, which may
// be necessary if the wires between the Arduino and the SHT1x are long:
#define clockPulseWidth 1

// The next lines are fine if you're using a 5V Arduino. If you're using a 3.3V
// Arduino (such at the Due), comment the next line and uncomment the one after:
#define supplyVoltage sht1xalt::VOLTAGE_5V
//#define supplyVoltage sht1xalt::VOLTAGE_3V5

// If you want to report temperature units in Fahrenheit instead of Celcius,
// comment the next line and uncomment the one after:
#define temperatureUnits sht1xalt::UNITS_CELCIUS
//#define temperatureUnits sht1xalt::UNITS_FAHRENHEIT

sht1xalt::Sensor sensor( dataPin, clockPin, clockPulseWidth,
                         supplyVoltage, temperatureUnits );

void setup() {
  Serial.begin(38400);
  Serial.println("Starting up (2s delay)");
  delay(2000);
  
  sensor.configureConnection();
  // Reset the SHT1x, in case the Arduino was reset during communication:
  sensor.softReset();
}

void loop() {
  float temp;
  float rh;
  sht1xalt::error_t err;
  
  delay(2000);
  
  err = sensor.measure(temp, rh);
  if (err) {
    switch (err) {
      case sht1xalt::ERROR_NO_ACK:
        Serial.println("SHT1x failed to acknowledge a command!");
        break;
      case sht1xalt::ERROR_MEASUREMENT_TIMEOUT:
        Serial.println("SHT1x failed to produce a measurement!");
        break;
    }
    sensor.softReset();
    return;
  }
  
  Serial.print("The temperature is ");
  Serial.print(temp);
  if (sensor.getUnits() == sht1xalt::UNITS_CELCIUS) {
    Serial.print(" Celcius");
  } else {
    Serial.print(" Fahrenheit");
  }
  Serial.print(", the relative humidity is ");
  Serial.print(rh);
  Serial.println("%");
}
