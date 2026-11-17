// ina219_read.ino
//
// Bring-up sketch for Ina219Sense: proves the chip answers, shows the shunt
// drop and the current it implies, and demonstrates the NAN-on-failure
// contract.
//
// Requires adafruit/Adafruit INA219 in lib_deps. Copy lib/current_sense into
// the target project and add that dependency.
//
// The ESP32 ADC path is a separate class and is not exercised here. In short:
//
//   AdcCurrentSense bts;
//   bts.begin(34, 16);                 // ADC1 pin, 16 samples averaged
//   float mv = bts.readMillivolts();   // feed mv/1000 into your calibration

#include <Arduino.h>

#include "current_sense.h"

static constexpr int kSdaPin = 21;
static constexpr int kSclPin = 22;
static constexpr float kShuntOhms = 0.1f;

static Ina219Sense g_sense;

void setup() {
  Serial.begin(115200);
  delay(200);

  g_sense.setShuntOhms(kShuntOhms);
  // Default address 0x40; pass it to the constructor if A0/A1 are strapped.
  // begin() also prints the initial bus/shunt/current/power dump on success.
  if (!g_sense.begin(kSdaPin, kSclPin, &Serial)) {
    Serial.println("No INA219 found. Check wiring, pull-ups and the address.");
    // Keep going: readings return NAN, so a caller can decide to carry on
    // without current feedback rather than halt the machine.
  }
}

void loop() {
  const float milliamps = g_sense.readMilliamps();

  if (isnan(milliamps)) {
    Serial.println("current: sensor unavailable");
  } else {
    Serial.printf("shunt %.3f mV -> %.1f mA (bus %.2f V)\n",
                  g_sense.readShuntMillivolts(), milliamps, g_sense.readBusVolts());
  }

  delay(500);
}
