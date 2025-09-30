#ifndef MANAGE_MM_WAVE
#define MANAGE_MM_WAVE

#include <HardwareSerial.h>

#define BUF_SIZE 128

struct mmWaveSensor {
    bool connected;
    uint16_t distance; // in cm
    uint8_t mmWaveBuffer[BUF_SIZE];
    uint8_t mmWaveIdx;
    uint8_t buff_size;
    uint8_t buf_len;
    uint8_t lastByte;
   
    bool presenceDetected;
    bool notifiedPresence;
    bool notifiedAbsence;

    // Parameters
    uint16_t maxMotionRange;
    uint16_t minMotionRange;
    uint16_t maxMicroMotionRange;
    uint16_t minMicroMotionRange;
    uint16_t noOneWaitingTime; //[ds]
    // debounce variables
    unsigned long lastPresenceTime;
    unsigned long lastAbsenceTime;
    const unsigned long presenceDebounceDelay = 1000;  // 1 second debounce for presence
    const unsigned long absenceDebounceDelay = 3000;   // 3 second debounce for absence


    // default constructor
   mmWaveSensor() : connected(false), distance(0), mmWaveIdx(0), 
                    buff_size(BUF_SIZE), buf_len(0), lastByte(0), 
                    presenceDetected(false), notifiedPresence(false), 
                    maxMotionRange(0), minMotionRange(0), maxMicroMotionRange(0), minMicroMotionRange(0), noOneWaitingTime(0),
                    notifiedAbsence(false), lastPresenceTime(0), 
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
};

void listenMMwave(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
void processFrame(mmWaveSensor &sensor);
void initMMWaveSensor(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
bool checkMMWaveConnection(HardwareSerial &ld2411Serial) ;
bool sendEnableConfig(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
bool sendEndConfig(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
bool waitForAck(HardwareSerial &ld2411Serial, uint16_t expectedID, mmWaveSensor &sensor, unsigned long timeout = 1000);
bool sendReboot(HardwareSerial &ld2411Serial);
bool sendCommand(HardwareSerial &ld2411Serial, uint16_t cmdId, const uint8_t* data, uint8_t dataLen, mmWaveSensor &sensor);
bool setMinDistance(HardwareSerial &ld2411Serial, uint8_t decimeters);
bool setMaxDistance(HardwareSerial &ld2411Serial, uint8_t decimeters);
bool setSensitivity(HardwareSerial &ld2411Serial, uint8_t level);
bool setHoldTime(HardwareSerial &ld2411Serial, uint8_t seconds);
bool setWorkMode(HardwareSerial &ld2411Serial, uint8_t mode);
bool enableUART(HardwareSerial &ld2411Serial, bool enable);
bool resetToDefault(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
bool getParam(HardwareSerial &ld2411Serial, mmWaveSensor &sensor);
#endif