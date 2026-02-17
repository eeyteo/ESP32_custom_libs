#ifndef MANAGE_MM_WAVE
#define MANAGE_MM_WAVE

#include <HardwareSerial.h>

#define BUF_SIZE 128

struct intValue{
   uint16_t value;
   uint16_t min;
   uint16_t max;
   // default constructor
   intValue(): value(0), min(0), max(0){}
   // parameterized constructor
   intValue(uint16_t v, uint16_t mi, uint16_t ma): value(v), min(mi), max(ma){}
};

struct mmWaveSensor {

    // Serial port for this sensor
    HardwareSerial* serialPort;
    uint8_t rxPin;
    uint8_t txPin;
    int t;
    bool connected;
    uint16_t distance; // in cm
    uint8_t mmWaveBuffer[BUF_SIZE];
    uint8_t mmWaveIdx;
    uint16_t buff_size;
    uint8_t buf_len;
    uint8_t lastByte;
   
    bool presenceDetected;
    bool notifiedPresence;
    bool notifiedAbsence;

    // Parameters
    intValue maxMotionRange;        //[cm] 
    intValue minMotionRange;        //[cm]
    intValue maxMicroMotionRange;   //[cm]
    intValue minMicroMotionRange;   //[cm]
    intValue noOneWaitingTime;      //[ds]

    // debounce variables
    unsigned long lastPresenceTime;
    unsigned long lastAbsenceTime;
    unsigned long presenceDebounceDelay;  
    unsigned long absenceDebounceDelay;  


    // default constructor - using parameterized intValue constructors
    mmWaveSensor() : 
        serialPort(nullptr), rxPin(16), txPin(17), t(0),
        connected(false), distance(0), mmWaveIdx(0), 
        buff_size(BUF_SIZE), buf_len(0), lastByte(0), 
        presenceDetected(false), notifiedPresence(false), 
        notifiedAbsence(false),
        maxMotionRange(0, 30, 717),
        minMotionRange(0, 30, 717),
        maxMicroMotionRange(0, 30, 425),
        minMicroMotionRange(0, 30, 425),
        noOneWaitingTime(0, 0, 65535),
        lastPresenceTime(0), 
        lastAbsenceTime(0),
        presenceDebounceDelay(1000),
        absenceDebounceDelay(3000) {}

    // Parameterized constructor
    mmWaveSensor(HardwareSerial* serial, uint8_t rx, uint8_t tx) : 
        serialPort(serial), rxPin(rx), txPin(tx), t(0),
        connected(false), distance(0), mmWaveIdx(0), 
        buff_size(BUF_SIZE), buf_len(0), lastByte(0), 
        presenceDetected(false), notifiedPresence(false), 
        notifiedAbsence(false),
        maxMotionRange(0, 30, 717),
        minMotionRange(0, 30, 717),
        maxMicroMotionRange(0, 30, 425),
        minMicroMotionRange(0, 30, 425),
        noOneWaitingTime(0, 0, 65535),
        lastPresenceTime(0), 
        lastAbsenceTime(0),
        presenceDebounceDelay(1000),
        absenceDebounceDelay(3000) {}
                    
    void showFrame() {
        // This method prints the contents of the mmWaveBuffer in a readable format for debugging.
        Serial.print("mmWave Frame of lenth ");
        Serial.print(buf_len);
        Serial.print(". Data: ");
        for (int i = 0; i < buf_len; i++) {
            Serial.print(mmWaveBuffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    void reset() {
        // This method resets the buffer index and length, and clears the buffer for the next frame.
        mmWaveIdx = 0;
        buf_len = 0;
        memset(mmWaveBuffer, 0, BUF_SIZE);
    }

    void showParam() {
        // This method prints the current parameters of the sensor in a readable format.
        Serial.println("=== LD2411-S Current Parameters ===");
        Serial.printf("Maximum Motion Range: %d cm (Range: %d-%d)\n", 
                      maxMotionRange.value, maxMotionRange.min, maxMotionRange.max);
        Serial.printf("Minimum Motion Range: %d cm (Range: %d-%d)\n", 
                      minMotionRange.value, minMotionRange.min, minMotionRange.max);
        Serial.printf("Maximum Micro-Motion Range: %d cm (Range: %d-%d)\n", 
                      maxMicroMotionRange.value, maxMicroMotionRange.min, maxMicroMotionRange.max);
        Serial.printf("Minimum Micro-Motion Range: %d cm (Range: %d-%d)\n", 
                      minMicroMotionRange.value, minMicroMotionRange.min, minMicroMotionRange.max);
        Serial.printf("No One Waiting Time: %d units (%.1f seconds) (Range: %d-%d)\n", 
                      noOneWaitingTime.value, noOneWaitingTime.value * 0.1, 
                      noOneWaitingTime.min, noOneWaitingTime.max);
        Serial.println("===================================");
    }
};

void listenMMwave(mmWaveSensor &sensor);
void processFrame(mmWaveSensor &sensor);
void initMMWaveSensor(mmWaveSensor &sensor);
bool checkMMWaveConnection(HardwareSerial &ld2411Serial) ;
bool waitForAck(uint16_t expectedid, mmWaveSensor &sensor, unsigned long timeout = 2000);
bool sendEnableConfig(mmWaveSensor &sensor);
bool sendEndConfig(mmWaveSensor &sensor);
bool sendCommand(uint16_t cmdId, const uint8_t* data, uint8_t dataLen, mmWaveSensor &sensor);
bool getParam(mmWaveSensor &sensor);
bool setParameter(mmWaveSensor &sensor, intValue &paramToSet, uint16_t newValue);
bool setMaxMotionRange(mmWaveSensor &sensor, uint16_t value);
bool setMinMotionRange(mmWaveSensor &sensor, uint16_t value);
bool setMaxMicroMotionRange(mmWaveSensor &sensor, uint16_t value);
bool setMinMicroMotionRange(mmWaveSensor &sensor, uint16_t value);
bool setNoOneWaitingTime(mmWaveSensor &sensor, uint16_t value);
#endif