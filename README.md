# ESP32 Custom Libraries

A collection of libraries for the ESP32 platform, designed to simplify common tasks and hardware integration.

## Features

- MQTT client support
- Motion detection publishing
- mmWave LD2411S sensor support
- integration with Home Assistant

## Usage

Include the libraries in your PlatformIO project. Add to plarformIO.ini file this project

```xml
lib_extra_dirs = ..\PlatformIO\Libs
```

## ESP32_MQTT_Lib

```xml
void mqttCallback(char* topic, byte* payload, unsigned int length);
```
This function will listen to messages coming from esp home

## ESP32_MMWAVE_Lib
Since the 2025.2.0 breking changes to ESP Home, unfortunatelly it's no longer possible to easily integrate sensors such as the mmWave LD2411S into ESP Home. A standard platformIO project is required.

### Initialization

```cpp
void initMMWaveSensor(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
```
Initializes the LD2411S mmWave sensor and checks connectivity.

### Listening for Presence

```cpp
void listenMMwave(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
```
Reads data from the sensor and updates presence status.

### Checking Connection

```cpp
bool checkMMWaveConnection(HardwareSerial &ld2411Serial);
```
Checks if the sensor is connected and responsive.

### Reading Parameters

```cpp
bool getParam(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
```
Reads configuration parameters from the sensor.

### Setting Parameters

```cpp
bool setMaxMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value);
bool setMinMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value);
bool setMaxMicroMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value);
bool setMinMicroMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value);
bool setNoOneWaitingTime(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value);
```
Use these functions to configure the sensor’s detection ranges and waiting time.

---

**Example Usage:**

```cpp
HardwareSerial Serial2(2);
mmWaveSensor sensor;

void setup() {
  Serial.begin(115200);
  initMMWaveSensor(Serial2, sensor);
}

void loop() {
  listenMMwave(Serial2, sensor);
  // Use sensor.presenceDetected for your logic
}
```



## License

MIT License