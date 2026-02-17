# ESP32 Custom Libraries

A collection of libraries for the ESP32 platform, designed to simplify common tasks and hardware integration.

## Features


- mmWave LD2411S sensor support
- integration with Home Assistant

## Usage

Include the libraries in your PlatformIO project. Add to plarformIO.ini file this project

```xml
lib_extra_dirs = ..\PlatformIO\Libs
```

## ESP32_MMWAVE_Lib
Since the 2025.2.0 breking changes to ESP Home, unfortunatelly it's no longer possible to easily integrate sensors such as the mmWave LD2411S into ESP Home. A standard platformIO project is required.


## License

MIT License