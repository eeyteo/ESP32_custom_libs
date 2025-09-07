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
lib_extra_dirs = C:\Users\khons\Documents\PlatformIO\Libs
```

## ESP32_MQQT_Lib
The main function in this library is
```xml
void publish_motion(bool motion, PubSubClient &mqttClient,const char* state_topic)
```
This function publish a boolean value to your Home Assistant MQTT adds on's state topic

## ESP32_MMWAVE_Lib
Since all the 2025's breking changes to ESP Home, unfortunatelly it's no longer possible to easily integrate sensors such as the mmWave LD2411S into ESP Home. A standard platformIO project is required.
The main function here is 
```xml
void listenMMwave(HardwareSerial &ld2411Serial, mmWaveSensor &sensor) {
```
This function analized the standard frame sent by the sensor and provides the boolean presence detected state.

## License

MIT License