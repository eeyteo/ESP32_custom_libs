# LD2411S library for ESP
This simple library purpose is to integrate the mm wave sensor LD2411s into a platformIO based
ESP32 powered IoT project.
The comunication protocol (UART) has been published by the manufacturer and therefore integration is 
possible without reverse engineerein.

## Parameters
The sensor has 5 parameters
- maxMotionRange [cm] 
- minMotionRange [cm]
- maxMicroMotionRange [cm]
- minMicroMotionRange[cm]
- noOneWaitingTime[ds]

## Methods
The object mmWaveSensor has the following methods:

- `void showFrame();`

Description

Prints the current received frame in hexadecimal format for debugging.

Used during protocol reverse engineering or validation.

- `void reset();`

Description

Resets:

+ Buffer index

+ Frame length

+ Clears internal buffer

Used after frame processing or error recovery.

- `void showParam();`

Description

Prints all current configuration parameters including:

Value

+ Allowed range

+ Converted time (seconds)

Used for debugging or verification.

## Functions

- `void listenMMwave(mmWaveSensor &sensor);`
- `void processFrame(mmWaveSensor &sensor);`
- `void initMMWaveSensor(mmWaveSensor &sensor);`
- `bool checkMMWaveConnection(HardwareSerial &ld2411Serial);`
- `bool waitForAck(uint16_t expectedid, mmWaveSensor &sensor, unsigned long timeout = 2000);`
- `bool sendEnableConfig(mmWaveSensor &sensor);`
- `bool sendEndConfig(mmWaveSensor &sensor);`
- `bool sendCommand(uint16_t cmdId, const uint8_t* data, uint8_t dataLen, mmWaveSensor &sensor);`
- `bool getParam(mmWaveSensor &sensor);`
- `bool setParameter(mmWaveSensor &sensor, intValue &paramToSet, uint16_t newValue);`
- `bool setMaxMotionRange(mmWaveSensor &sensor, uint16_t value);`
- `bool setMinMotionRange(mmWaveSensor &sensor, uint16_t value);`
- `bool setMaxMicroMotionRange(mmWaveSensor &sensor, uint16_t value);`
- `bool setMinMicroMotionRange(mmWaveSensor &sensor, uint16_t value);`
- `bool setNoOneWaitingTime(mmWaveSensor &sensor, uint16_t value);`

## How to use
Add the folder ESP_MMWAVE_Lib to your PlatoformIO\Libs folder. You can usually find it in
`C:\Users\khons\Documents\PlatformIO\Libs`
In your main project's platformio.ini config file,
`lib_extra_dirs = C:\Users\khons\Documents\PlatformIO\Libs`
