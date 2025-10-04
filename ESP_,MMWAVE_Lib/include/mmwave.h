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
    intValue maxMotionRange;
    intValue minMotionRange;
    intValue maxMicroMotionRange;
    intValue minMicroMotionRange;
    intValue noOneWaitingTime; //[ds]
    // debounce variables
    unsigned long lastPresenceTime;
    unsigned long lastAbsenceTime;
    const unsigned long presenceDebounceDelay = 1000;  // 1 second debounce for presence
    const unsigned long absenceDebounceDelay = 3000;   // 3 second debounce for absence


    // constructor - using parameterized intValue constructors
    mmWaveSensor() : connected(false), distance(0), mmWaveIdx(0), 
                    buff_size(BUF_SIZE), buf_len(0), lastByte(0), 
                    presenceDetected(false), notifiedPresence(false), 
                    notifiedAbsence(false),
                    maxMotionRange(0, 30, 717),      // Use parameterized constructor
                    minMotionRange(0, 30, 717),
                    maxMicroMotionRange(0, 30, 425),
                    minMicroMotionRange(0, 30, 425),
                    noOneWaitingTime(0, 0, 65535),
                    lastPresenceTime(0), 
                    lastAbsenceTime(0) {}
                    
    void showFrame() {
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
        mmWaveIdx = 0;
        buf_len = 0;
        memset(mmWaveBuffer, 0, BUF_SIZE);
    }

    void showParam() {
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

void listenMMwave(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
void processFrame(mmWaveSensor &sensor);
void initMMWaveSensor(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
bool checkMMWaveConnection(HardwareSerial &ld2411Serial) ;
bool sendEnableConfig(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
bool sendEndConfig(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
bool waitForAck(HardwareSerial &ld2411Serial, uint16_t expectedID, mmWaveSensor &sensor, unsigned long timeout = 2000);
bool sendReboot(HardwareSerial &ld2411Serial);
bool sendCommand(HardwareSerial &ld2411Serial, uint16_t cmdId, const uint8_t* data, uint8_t dataLen, mmWaveSensor &sensor);
bool getParam(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
bool setParameter(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, 
                  intValue &paramToSet, uint16_t newValue);
bool setMaxMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value);
bool setMinMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value);
bool setMaxMicroMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value);
bool setMinMicroMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value);
bool setNoOneWaitingTime(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value);
#endif