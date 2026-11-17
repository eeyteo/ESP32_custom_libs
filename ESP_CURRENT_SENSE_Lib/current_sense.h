// current_sense.h
// Reusable motor-current sensing for ESP32.
//
// Two independent mechanisms, each usable on its own:
//
//   Ina219Sense   - TI INA219 over I2C. Returns milliamps directly, computed as
//                   shunt voltage / shunt resistance. That is deliberately not
//                   the chip's own getCurrent_mA(), which applies the INA219's
//                   internal calibration register; dividing the measured shunt
//                   drop by the known resistor keeps the reading traceable to
//                   the hardware.
//
//   AdcCurrentSense - a shunt-amplifier output (e.g. the R_IS pin of a BTS7960)
//                   sampled with the ESP32 ADC and converted to millivolts
//                   through the esp_adc_cal calibration curve. Callers turn
//                   millivolts into amps with whatever calibration table their
//                   driver's datasheet gives.
//
// Neither class knows about health flags, logging sinks or application state.
// Both report failure through NAN and expose begin() results as bool, so the
// caller decides what a missing sensor means.

#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <esp_adc_cal.h>

// ---------------------------------------------------------------------------
// INA219 over I2C
// ---------------------------------------------------------------------------
class Ina219Sense {
 public:
  // address is the 7-bit I2C address; 0x40 unless the A0/A1 straps move it.
  explicit Ina219Sense(uint8_t address = INA219_ADDRESS) : m_sensor(address) {}

  // Starts Wire on the given pins and probes the chip.
  // Argument order matters: sda first, then scl.
  bool begin(int sdaPin, int sclPin, Print *trace = nullptr);

  bool isReady() const { return m_ready; }

  // Shunt resistor in ohms. Defaults to 0.1, the value on the cable machine.
  void setShuntOhms(float ohms) { m_shuntOhms = ohms; }
  float shuntOhms() const { return m_shuntOhms; }

  // All readings return NAN when the chip did not answer begin().
  float readMilliamps();
  float readShuntMillivolts();
  float readBusVolts();

  // One-line bus / shunt / current / power dump, for bring-up.
  void printDiagnostics();

  void setTrace(Print *trace) { m_trace = trace; }

 private:
  Adafruit_INA219 m_sensor;
  float m_shuntOhms = 0.1f;
  bool m_ready = false;
  int m_sdaPin = -1;
  int m_sclPin = -1;
  Print *m_trace = nullptr;
};

// ---------------------------------------------------------------------------
// ESP32 ADC current sense
// ---------------------------------------------------------------------------
class AdcCurrentSense {
 public:
  // Calibrates the pin and fixes the sample count. Use an ADC1 pin (GPIO 32-39)
  // if WiFi is active: ADC2 is owned by the radio and readings will fail.
  // Full-scale input is 0-3.3 V (11 dB attenuation).
  void begin(int pin, uint8_t samples = 16);

  bool isReady() const { return m_ready; }
  int pin() const { return m_pin; }

  // Averaged, calibrated reading in millivolts. Returns NAN before begin().
  float readMillivolts();

  void setSamples(uint8_t samples) { m_samples = samples; }

 private:
  esp_adc_cal_characteristics_t m_calibration;
  int m_pin = -1;
  uint8_t m_samples = 16;
  bool m_ready = false;
};

#endif  // CURRENT_SENSE_H
