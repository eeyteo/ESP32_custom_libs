// servo_valve.cpp
#include "servo_valve.h"

#include <stdarg.h>

namespace {

constexpr int kReadingsCapacity = SERVO_VALVE_NUM_READINGS;

}  // namespace

int ServoValve::rampSpeed(float a, float b, float c, int t) const {
  // Quadratic sweep: t counts down in deciseconds from the manoeuvre duration.
  return static_cast<int>(a * t * t + b * t + c);
}

void ServoValve::begin(int pin, const ServoPort &port, const ServoParams &params,
                       Print *trace) {
  m_pin = pin;
  m_port = port;
  m_trace = trace;

  if (m_servo.attach(pin) == -1 && m_trace != nullptr) {
    m_trace->print("Failed to attach servo to pin ");
    m_trace->println(pin);
  }
  m_servo.write(params.idleSpeed);
}

void ServoValve::requestOpen() {
  if (m_busy) return;
  m_state = ServoState::StartOpen;
}

void ServoValve::requestClose() {
  if (m_busy) return;
  m_state = ServoState::StartClose;
}

void ServoValve::startSelfTest() {
  m_openTest = true;
  m_busy = true;
  m_state = ServoState::StartUp;
}

void ServoValve::startBootClose() {
  m_openTest = false;
  m_busy = true;
  m_state = ServoState::StartUp;
}

void ServoValve::tick(const ServoParams &params) {
  // The manoeuvre timers were historically decremented by the caller once per
  // 100 ms tick; the driver now owns that so the contract is self-contained.
  if (m_timerDs) --m_timerDs;

  switch (m_state) {
    case ServoState::Waiting:
      // Not yet released by startSelfTest(): no output, commands stay rejected.
      break;

    case ServoState::StartUp:
      tracef("Phase 0 - start closing valve with ctr pin %d", m_pin);
      resetReadings();
      m_timerDs = params.closeTimeDs;
      m_state = ServoState::StartUpSeat;
      tracef("Phase 1 - making sure the valve %d is closed", m_pin);
      break;

    case ServoState::StartUpSeat: {
      m_servo.write(rampSpeed(params.aClose, params.bClose, params.cClose, m_timerDs));
      addCurrent(m_port.readCurrent ? m_port.readCurrent(m_port.ctx) : 0.0f);
      const float average = averageLastCurrent(params.samplesToAverage);
      if (!m_timerDs || average >= params.currClose) {
        // Record the seating verdict here too, so startBootClose() has a result
        // to report without ever running the opening leg. A later StartUpClose
        // overwrites it in the full self-test path, which is why this is
        // invisible to callers that run the whole sequence.
        if (average >= static_cast<float>(params.currClose)) {
          m_selfTestClose = ServoResult::Ok;
        } else if (average < static_cast<float>(params.currMin)) {
          m_selfTestClose = ServoResult::ErrLowCurrent;
        } else {
          m_selfTestClose = ServoResult::ErrNoMovement;
        }
        m_state = m_openTest ? ServoState::StartUpOpenStart : ServoState::Idle;
      }
      break;
    }

    case ServoState::StartUpOpenStart:
      tracef("Phase 2 - opening valve %d", m_pin);
      resetReadings();
      m_timerDs = params.openTimeDs;
      m_state = ServoState::StartUpOpen;
      break;

    case ServoState::StartUpOpen: {
      const int speed = rampSpeed(params.aOpen, params.bOpen, params.cOpen, m_timerDs);
      m_servo.write(speed);

      // The sample is evaluated before it is stored in this state, matching the
      // original driver: the verdict uses the samples preceding this step.
      float average = averageLastCurrent(params.samplesToAverage);
      addCurrent(m_port.readCurrent ? m_port.readCurrent(m_port.ctx) : 0.0f);

      bool advance = false;
      if (average > static_cast<float>(params.currMax)) {
        m_selfTestOpen = ServoResult::ErrHighCurrent;
        advance = true;
      } else if (!m_timerDs) {
        average = averageAllCurrent();
        if (average < static_cast<float>(params.currMin)) {
          m_selfTestOpen = ServoResult::ErrLowCurrent;
        } else {
          m_selfTestOpen = ServoResult::Ok;
          m_lastOpenOk = true;
        }
        advance = true;
      }

      if (advance) {
        tracef("Phase 3 - opening manouvre of valve %d completed. State: %d (%d mA < %0.2f mA <= %d mA)",
               m_pin, static_cast<int>(m_selfTestOpen), params.currMin, average, params.currMax);
        m_state = ServoState::StartUpCloseStart;
      }
      break;
    }

    case ServoState::StartUpCloseStart:
      tracef("Phase 4 - closing valve %d", m_pin);
      resetReadings();
      m_timerDs = params.closeTimeDs;
      m_state = ServoState::StartUpClose;
      break;

    case ServoState::StartUpClose: {
      m_servo.write(rampSpeed(params.aClose, params.bClose, params.cClose, m_timerDs));
      addCurrent(m_port.readCurrent ? m_port.readCurrent(m_port.ctx) : 0.0f);
      const float average = averageLastCurrent(params.samplesToAverage);

      if (average > static_cast<float>(params.currMax)) {
        m_selfTestClose = ServoResult::ErrHighCurrent;
        m_state = ServoState::Idle;
        tracef("Phase 5 - Closing manouvre of valve %d completed. State: %d. %0.2f mA > %d mA",
               m_pin, static_cast<int>(m_selfTestClose), average, params.currMax);
      } else if (!m_timerDs) {
        m_selfTestClose = (average < static_cast<float>(params.currMin))
                              ? ServoResult::ErrLowCurrent
                              : ServoResult::ErrNoMovement;
        m_state = ServoState::Idle;
        tracef("Phase 5 - Closing manouvre of valve %d completed. State: %d.",
               m_pin, static_cast<int>(m_selfTestClose));
      } else if (average >= static_cast<float>(params.currClose)) {
        m_selfTestClose = ServoResult::Ok;
        m_lastCloseOk = true;
        m_state = ServoState::Idle;
        tracef("Phase 5 - Closing manouvre of valve %d completed. State: %d. %0.2f mA >= %d mA",
               m_pin, static_cast<int>(m_selfTestClose), average, params.currClose);
      }
      break;
    }

    case ServoState::Idle:
      m_servo.write(params.idleSpeed);
      m_busy = false;
      break;

    case ServoState::StartOpen:
      resetReadings();
      m_timerDs = params.openTimeDs;
      m_state = ServoState::Opening;
      m_busy = true;
      m_openManoeuvre = true;
      break;

    case ServoState::Opening: {
      const int speed = rampSpeed(params.aOpen, params.bOpen, params.cOpen, m_timerDs);
      m_servo.write(speed);
      const float current = m_port.readCurrent ? m_port.readCurrent(m_port.ctx) : 0.0f;
      const float average = averageLastCurrent(params.samplesToAverage);
      storeReading(current, speed, m_timerDs);

      if (!m_timerDs) {
        m_state = ServoState::Log;
        m_lastOpenOk = true;
        tracef("Nominal. Opening valve %d complete due to the timer of %d ds",
               m_pin, params.openTimeDs);
      } else if (average >= static_cast<float>(params.currOpen)) {
        m_state = ServoState::Log;
        m_lastOpenOk = false;
        tracef("Critical. Opening valve %d complete due to the current. %0.2f mA >= %d mA",
               m_pin, average, params.currOpen);
      }
      break;
    }

    case ServoState::StartClose:
      resetReadings();
      m_timerDs = params.closeTimeDs;
      m_state = ServoState::Closing;
      m_busy = true;
      m_openManoeuvre = false;
      break;

    case ServoState::Closing: {
      const int speed = rampSpeed(params.aClose, params.bClose, params.cClose, m_timerDs);
      m_servo.write(speed);
      const float current = m_port.readCurrent ? m_port.readCurrent(m_port.ctx) : 0.0f;
      const float average = averageLastCurrent(params.samplesToAverage);
      storeReading(current, speed, m_timerDs);

      if (!m_timerDs) {
        m_state = ServoState::Log;
        m_lastCloseOk = false;
        tracef("Closing valve %d complete due to the timer of %d ds",
               m_pin, params.closeTimeDs);
      } else if (average >= static_cast<float>(params.currClose)) {
        m_state = ServoState::Log;
        m_lastCloseOk = true;
        tracef("Closing valve %d complete due to the current. %0.2f mA >= %d mA",
               m_pin, average, params.currClose);
        m_servo.write(params.idleSpeed);
      }
      break;
    }

    case ServoState::Log: {
      if (m_port.logManoeuvre != nullptr) {
        ServoRecord record;
        record.pin = m_pin;
        record.isOpening = m_openManoeuvre;
        record.ok = m_openManoeuvre ? m_lastOpenOk : m_lastCloseOk;
        record.count = m_count;
        record.head = m_head;
        record.capacity = kReadingsCapacity;
        record.currentBuffer = m_current;
        record.timeBuffer = m_time;
        record.speedBuffer = m_speed;
        m_port.logManoeuvre(record, m_port.ctx);
      }
      m_state = ServoState::Idle;
      break;
    }

    default:
      m_state = ServoState::Idle;
      break;
  }
}

void ServoValve::resetReadings() {
  for (int i = 0; i < kReadingsCapacity; ++i) {
    m_current[i] = 0.0f;
    m_time[i] = 0;
    m_speed[i] = 0;
  }
  m_head = -1;
  m_count = 0;
}

void ServoValve::addCurrent(float current) {
  m_head = (m_head + 1) % kReadingsCapacity;
  m_current[m_head] = current;
  if (m_count < kReadingsCapacity) ++m_count;
}

void ServoValve::storeReading(float current, int speed, int t) {
  m_head = (m_head + 1) % kReadingsCapacity;
  m_current[m_head] = current;
  m_time[m_head] = static_cast<uint16_t>(t);
  m_speed[m_head] = static_cast<uint16_t>(speed);
  if (m_count < kReadingsCapacity) ++m_count;
}

float ServoValve::averageLastCurrent(uint8_t n) const {
  if (m_count == 0) return 0.0f;
  int samples = static_cast<int>(n);
  if (samples > m_count) samples = m_count;
  if (samples <= 0) return 0.0f;

  float sum = 0.0f;
  for (int i = 0; i < samples; ++i) {
    const int index = (m_head - i + kReadingsCapacity) % kReadingsCapacity;
    sum += m_current[index];
  }
  return sum / static_cast<float>(samples);
}

float ServoValve::averageAllCurrent() const {
  return averageLastCurrent(static_cast<uint8_t>(m_count));
}

void ServoValve::tracef(const char *fmt, ...) const {
  if (m_trace == nullptr) return;
  char buffer[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  m_trace->println(buffer);
}
