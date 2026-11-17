// single_servo.ino
//
// Minimal example: one valve, driven by ServoValve, behind an asynchronous web
// server. Copy lib/servo_valve into the new project and this is the whole of
// the integration.
//
// The important shape to preserve from the cable machine firmware:
//   * loop() ticks the valve once every 100 ms and never blocks
//   * web handlers never touch the valve. They post a request into
//     g_requests, and loop() -- the only task that touches hardware -- applies
//     it. ESPAsyncWebServer runs handlers on the AsyncTCP task, so writing to
//     the valve directly from a handler would race the control tick.
//   * diagnostics are read from an immutable snapshot, not from the live object

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#include "servo_valve.h"

// ---------------------------------------------------------------- hardware ---

static constexpr int kServoPin = 32;        // servo signal
static constexpr int kCurrentPin = 34;      // shunt amplifier output
static constexpr float kShuntOhms = 0.1f;
static constexpr float kAmpGain = 50.0f;    // INA219-style amplifier gain

// ------------------------------------------------------------ configuration ---

// Ramp coefficients for a quadratic sweep between the idle pulse and the end
// stop, expressed over the manoeuvre duration in deciseconds. Derive them with
// the same quadratic the firmware uses, which passes through
// (0, y1), (tm/2, y2) and (tm, y1):
//     c = y1
//     b = (y2 - y1) * 4 / tm
//     a = (y1 - y2) * 4 / tm^2
// The values below are the cable machine defaults: closing sweeps 100 -> 180
// deg and opening sweeps 80 -> 20 deg, both over 2.0 s.
static ServoParams g_params;

static void buildDefaultParams() {
  g_params.aClose = -0.8f;
  g_params.bClose = 16.0f;
  g_params.cClose = 100.0f;
  g_params.aOpen = 0.6f;
  g_params.bOpen = -12.0f;
  g_params.cOpen = 80.0f;
  g_params.openTimeDs = 20;   // 2.0 s to open
  g_params.closeTimeDs = 20;  // 2.0 s to close
  g_params.currOpen = 300;    // [mA] opening stops when the valve hits its stop
  g_params.currClose = 250;   // [mA] closing stops when the valve seats
  g_params.currMin = 40;      // [mA] below this the valve did not move
  g_params.currMax = 900;     // [mA] above this the manoeuvre is aborted
  g_params.idleSpeed = 90;
  g_params.samplesToAverage = 2;
}

// The driver asks for the motor current through this callback. Swap the body
// for an Adafruit_INA219 read if the new board carries one.
static float readMotorCurrent(void *ctx) {
  (void)ctx;
  const float millivolts = analogReadMilliVolts(kCurrentPin);
  const float amps = millivolts / 1000.0f / kAmpGain / kShuntOhms;
  return amps * 1000.0f;  // mA
}

// Called once per finished manoeuvre. Keep this quick: it runs on the control
// task. Push a record onto a queue and let another task do any network I/O --
// never block here with an HTTPS POST.
static void onManoeuvre(const ServoRecord &record, void *ctx) {
  (void)ctx;
  Serial.printf("%s %s, %d samples\n", record.isOpening ? "open" : "close",
                record.ok ? "ok" : "FAILED", record.count);
}

// ------------------------------------------------------------------- queue ---

// Requests handed from the AsyncTCP task to the control task. A real project
// should use a proper ring buffer like the cable machine's cmdQueue; a pair of
// flags is enough to show the pattern.
enum class Request : uint8_t { None, Open, Close, SelfTest };

static portMUX_TYPE g_requestMux = portMUX_INITIALIZER_UNLOCKED;
static Request g_request = Request::None;
static uint32_t g_rejected = 0;  // requests dropped because one was pending

static bool postRequest(Request request) {
  bool accepted = false;
  portENTER_CRITICAL(&g_requestMux);
  if (g_request == Request::None) {
    g_request = request;
    accepted = true;
  } else {
    ++g_rejected;
  }
  portEXIT_CRITICAL(&g_requestMux);
  return accepted;
}

static Request takeRequest() {
  portENTER_CRITICAL(&g_requestMux);
  const Request request = g_request;
  g_request = Request::None;
  portEXIT_CRITICAL(&g_requestMux);
  return request;
}

// ------------------------------------------------------------------ server ---

static ServoValve g_valve;
static AsyncWebServer g_server(80);

static void handleStatus(AsyncWebServerRequest *request) {
  JsonDocument doc;
  doc["phase"] = static_cast<int>(g_valve.state());
  doc["busy"] = g_valve.isBusy();
  doc["self_test_open"] = g_valve.selfTestOpenResult();
  doc["self_test_close"] = g_valve.selfTestCloseResult();
  doc["last_open_ok"] = g_valve.lastOpenOk();
  doc["last_close_ok"] = g_valve.lastCloseOk();
  doc["rejected"] = g_rejected;

  String body;
  serializeJson(doc, body);
  request->send(200, "application/json", body);
}

static void handleValveCommand(AsyncWebServerRequest *request, Request command,
                               const char *label) {
  if (g_valve.isBusy()) {
    request->send(409, "text/plain", "valve busy");
    return;
  }
  if (!postRequest(command)) {
    request->send(503, "text/plain", "request pending, retry");
    return;
  }
  request->send(202, "text/plain", label);
}

void setupServer() {
  g_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "GET /status /open /close");
  });
  g_server.on("/status", HTTP_GET, handleStatus);
  g_server.on("/open", HTTP_GET, [](AsyncWebServerRequest *request) {
    handleValveCommand(request, Request::Open, "opening");
  });
  g_server.on("/close", HTTP_GET, [](AsyncWebServerRequest *request) {
    handleValveCommand(request, Request::Close, "closing");
  });
  g_server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "not found");
  });
  g_server.begin();
}

// -------------------------------------------------------------------- main ---

void setup() {
  Serial.begin(115200);
  delay(200);

  buildDefaultParams();

  ServoPort port;
  port.readCurrent = readMotorCurrent;
  port.logManoeuvre = onManoeuvre;  // nullptr to disable logging entirely
  port.ctx = nullptr;

  g_valve.begin(kServoPin, port, g_params, &Serial);
  g_valve.startSelfTest();  // closes, opens, closes; commands are refused until done

  WiFi.mode(WIFI_STA);
  WiFi.begin("your-ssid", "your-password");
  setupServer();
}

void loop() {
  static uint32_t lastTick = 0;

  switch (takeRequest()) {
    case Request::Open: g_valve.requestOpen(); break;
    case Request::Close: g_valve.requestClose(); break;
    case Request::SelfTest: g_valve.startSelfTest(); break;
    case Request::None: break;
  }

  // The manoeuvre timers are counted in deciseconds, so the tick period is part
  // of the driver's contract. 100 ms also matches the cable machine firmware.
  const uint32_t now = millis();
  if (now - lastTick >= 100) {
    lastTick = now;
    g_valve.tick(g_params);
  }

  // Everything else the application needs may run between ticks. It must never
  // block longer than the tick period if the ramps need to stay smooth.
}
