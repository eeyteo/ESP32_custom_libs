// servo_valve.h
// Reusable single-servo valve driver for ESP32.
//
// The valve is driven by sweeping the servo speed along a quadratic ramp while
// the motor current is watched through a callback. A manoeuvre ends on one of
// three conditions:
//   1. the valve seats / reaches the end stop   -> current rises past currClose
//      (opening instead ends when current rises past currOpen)
//   2. the manoeuvre timer runs out             -> no movement, or overload
//   3. the current exceeds the overload limit   -> currMax
//
// The state machine never blocks: tick() performs at most one state transition
// and returns immediately, which makes it safe to call from a cooperative loop
// alongside an asynchronous web server. Call it from a single task at a fixed
// period of 100 ms -- the manoeuvre timers are expressed in deciseconds.
//
// This driver owns no configuration and no hardware beyond the servo pulse
// output:
//   - configuration arrives on every tick as a ServoParams value
//   - the motor current arrives through ServoPort::readCurrent
//   - completed manoeuvres leave through ServoPort::logManoeuvre
//   - diagnostic text leaves through the Print* passed to begin()
//
// Nothing here knows about EEPROM, WiFi, HTTP or the rest of an application.

#ifndef SERVO_VALVE_H
#define SERVO_VALVE_H

#include <Arduino.h>
#include <ESP32Servo.h>

// Depth of the per-manoeuvre sample buffers. 100 samples at one sample per
// tick covers 10 s of manoeuvre, which matches the firmware's historic value.
#ifndef SERVO_VALVE_NUM_READINGS
#define SERVO_VALVE_NUM_READINGS 100
#endif

// Outcome of a manoeuvre. The numeric values are part of the contract: an
// application that used the earlier integer state codes can keep reporting
// them unchanged through static_cast<int>().
enum class ServoResult : int {
  Unknown = 0,
  Ok = 1,
  ErrLowCurrent = 2,
  ErrHighCurrent = 3,
  ErrNoMovement = 4,
};

// Numeric values mirror the legacy driver so a printed phase number stays
// meaningful to anyone reading older logs.
enum class ServoState : uint8_t {
  Waiting = 0,  // before the self-test has been requested; no output, stays busy
  StartUp = 1,
  StartUpSeat = 2,       // close until the current confirms the valve is seated
  StartUpOpenStart = 3,
  StartUpOpen = 4,
  StartUpCloseStart = 5,
  StartUpClose = 6,      // full closing self-test
  Idle = 7,
  StartOpen = 8,
  Opening = 9,
  StartClose = 10,
  Closing = 11,
  Log = 12,
};

// Everything the driver needs to know about how this particular valve should
// behave. Supply it on every tick() so a caller that persists its settings can
// change them at runtime without touching the driver.
struct ServoParams {
  float aOpen = 0.0f, bOpen = 0.0f, cOpen = 0.0f;    // opening ramp coefficients
  float aClose = 0.0f, bClose = 0.0f, cClose = 0.0f;  // closing ramp coefficients
  int openTimeDs = 0;    // nominal opening duration [deciseconds]
  int closeTimeDs = 0;   // nominal closing duration [deciseconds]
  int currOpen = 0;      // [mA] opening complete when current reaches this
  int currClose = 0;     // [mA] valve seated when current reaches this
  int currMin = 0;       // [mA] below this the valve did not move
  int currMax = 0;       // [mA] above this the manoeuvre is aborted as overload
  int idleSpeed = 90;    // [deg] pulse width that holds the servo still
  uint8_t samplesToAverage = 2;  // current samples averaged per evaluation
};

// Snapshot handed to the log sink when a manoeuvre finishes.
struct ServoRecord {
  int pin = -1;
  bool isOpening = false;   // true = opening manoeuvre, false = closing
  bool ok = false;          // manoeuvre completed nominally
  int count = 0;            // valid samples in the buffers
  int head = -1;            // index of the newest sample
  int capacity = 0;         // buffer capacity, needed to unwrap the ring
  const float *currentBuffer = nullptr;
  const uint16_t *timeBuffer = nullptr;
  const uint16_t *speedBuffer = nullptr;
};

// Read the motor current in milliamperes. Return 0 when no reading is
// available; the manoeuvre then relies on its timer alone.
typedef float (*ServoCurrentFn)(void *ctx);

// Called once per completed manoeuvre. Leave unset to disable logging.
typedef void (*ServoLogFn)(const ServoRecord &record, void *ctx);

// Hooks into the surrounding application. All are optional.
struct ServoPort {
  ServoCurrentFn readCurrent = nullptr;
  ServoLogFn logManoeuvre = nullptr;
  void *ctx = nullptr;  // passed back to both callbacks
};

class ServoValve {
 public:
  // Attach the servo pulse output and park the valve. The initial idle pulse
  // uses params.idleSpeed; pass trace = nullptr to silence diagnostics.
  void begin(int pin, const ServoPort &port, const ServoParams &params,
             Print *trace = nullptr);

  // Advance the state machine by one 100 ms step. Safe to call from a single
  // task only; every entry point mutates the same state.
  void tick(const ServoParams &params);

  // Non-blocking requests. Ignored while a manoeuvre is already running.
  void requestOpen();
  void requestClose();

  // Run the closing / opening / closing self-test used to prove the valve and
  // its current sensing work. Requests are rejected until this completes.
  void startSelfTest();

  // Seat the valve closed and release the driver, skipping the opening leg.
  // The outcome lands in selfTestCloseResult(). Use this at power-up on an
  // installation where opening the valve is itself the hazard: it still proves
  // the current sensing detects the seat, but never moves the valve open.
  void startBootClose();

  // Publish a new trace sink after begin(), e.g. to mute logging at runtime.
  void setTrace(Print *trace) { m_trace = trace; }

  bool isBusy() const { return m_busy; }
  bool isIdle() const { return m_state == ServoState::Idle; }
  ServoState state() const { return m_state; }
  int pin() const { return m_pin; }

  // Self-test outcomes as the legacy integer status codes (1 = ok, 2..4 = error)
  // so they drop straight into sysHealth::state_valves without conversion.
  int selfTestOpenResult() const { return static_cast<int>(m_selfTestOpen); }
  int selfTestCloseResult() const { return static_cast<int>(m_selfTestClose); }

  // Outcome of the most recent normal manoeuvre.
  bool lastOpenOk() const { return m_lastOpenOk; }
  bool lastCloseOk() const { return m_lastCloseOk; }

 private:
  int rampSpeed(float a, float b, float c, int t) const;

  void resetReadings();
  void addCurrent(float current);
  void storeReading(float current, int speed, int t);
  float averageLastCurrent(uint8_t n) const;
  float averageAllCurrent() const;
  void tracef(const char *fmt, ...) const;

  Servo m_servo;
  ServoState m_state = ServoState::Waiting;
  ServoPort m_port;
  int m_pin = -1;
  int m_timerDs = 0;
  bool m_busy = true;          // busy until the self-test has run
  bool m_openManoeuvre = false;
  bool m_openTest = true;      // false: stop once seated instead of opening

  ServoResult m_selfTestOpen = ServoResult::Unknown;
  ServoResult m_selfTestClose = ServoResult::Unknown;
  bool m_lastOpenOk = false;
  bool m_lastCloseOk = false;

  float m_current[SERVO_VALVE_NUM_READINGS];
  uint16_t m_time[SERVO_VALVE_NUM_READINGS];
  uint16_t m_speed[SERVO_VALVE_NUM_READINGS];
  int m_head = -1;
  int m_count = 0;

  Print *m_trace = nullptr;
};

#endif  // SERVO_VALVE_H
