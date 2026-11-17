// current_sense.cpp
#include "current_sense.h"

#include <Wire.h>

// ---------------------------------------------------------------------------
// INA219 over I2C
// ---------------------------------------------------------------------------

bool Ina219Sense::begin(int sdaPin, int sclPin, Print *trace) {
  m_sdaPin = sdaPin;
  m_sclPin = sclPin;
  if (trace != nullptr) m_trace = trace;

  Wire.begin(m_sdaPin, m_sclPin);
  m_ready = m_sensor.begin();

  if (m_trace != nullptr) {
    if (m_ready) {
      m_trace->println("INA219 initialized successfully.");
    } else {
      m_trace->println("Failed to find INA219 chip!");
    }
  }

  if (m_ready) printDiagnostics();
  return m_ready;
}

float Ina219Sense::readShuntMillivolts() {
  if (!m_ready) return NAN;
  return m_sensor.getShuntVoltage_mV();
}

float Ina219Sense::readBusVolts() {
  if (!m_ready) return NAN;
  return m_sensor.getBusVoltage_V();
}

float Ina219Sense::readMilliamps() {
  if (!m_ready) return NAN;
  // Ohm's law on the shunt: I[mA] = V[mV] / R[ohm]. The 1000 scaling of volts
  // to millivolts and amps to milliamps cancels, so no conversion factor here.
  return m_sensor.getShuntVoltage_mV() / m_shuntOhms;
}

void Ina219Sense::printDiagnostics() {
  if (m_trace == nullptr || !m_ready) return;

  const float shuntMv = m_sensor.getShuntVoltage_mV();
  const float busV = m_sensor.getBusVoltage_V();
  const float loadV = busV + (shuntMv / 1000.0f);

  char buffer[128];
  snprintf(buffer, sizeof(buffer),
           "Bus: %.2f V | Shunt: %.2f mV | Load: %.2f V | Current: %.2f mA | Power: %.2f mW",
           busV, shuntMv, loadV, readMilliamps(), m_sensor.getPower_mW());
  m_trace->println(buffer);
}

// ---------------------------------------------------------------------------
// ESP32 ADC current sense
// ---------------------------------------------------------------------------

void AdcCurrentSense::begin(int pin, uint8_t samples) {
  m_pin = pin;
  m_samples = (samples == 0) ? 1 : samples;

  analogReadResolution(12);  // 12-bit ADC, 0..4095
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100,
                           &m_calibration);
  analogSetPinAttenuation(pin, ADC_11db);  // full scale 0-3.3 V
  m_ready = true;
}

float AdcCurrentSense::readMillivolts() {
  if (!m_ready) return NAN;

  long sum = 0;
  for (uint8_t i = 0; i < m_samples; ++i) {
    sum += analogRead(m_pin);
    delayMicroseconds(50);
  }
  const int raw = static_cast<int>(sum / m_samples);
  return static_cast<float>(esp_adc_cal_raw_to_voltage(raw, &m_calibration));
}
