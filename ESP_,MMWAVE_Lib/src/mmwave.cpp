#include "mmwave.h"



void listenMMwave(HardwareSerial &ld2411Serial, mmWaveSensor &sensor) {

  while (ld2411Serial.available()) {
    uint8_t c = ld2411Serial.read();
    
    // Check for start of frame: AA AA
    if (c == 0xAA && sensor.lastByte == 0xAA) {
      // two consecutive AA bytes - start of frame
      sensor.reset();
      sensor.mmWaveBuffer[sensor.mmWaveIdx++] = sensor.lastByte; 
      sensor.mmWaveBuffer[sensor.mmWaveIdx++] = c;     
      //Serial.println("- Frame start (AA AA)");
    } 
    else if (sensor.mmWaveIdx > 0) {
      // in a frame, continue storing
      if (sensor.mmWaveIdx < sensor.buff_size) {
        sensor.mmWaveBuffer[sensor.mmWaveIdx++] = c;
        //Serial.println("- Stored");
      } else {
        //Serial.println("- Buffer full, resetting");
        sensor.reset();
      }
    }
    
    // end of frame: 55 55
    if (sensor.mmWaveIdx >= 2 && 
        sensor.mmWaveBuffer[sensor.mmWaveIdx-1] == 0x55 && 
        sensor.mmWaveBuffer[sensor.mmWaveIdx-2] == 0x55) {
      sensor.buf_len = sensor.mmWaveIdx;
      //Serial.println("- End of frame (55 55)");
      processFrame(sensor);
      sensor.reset();
    }
    
    sensor.lastByte = c;
  }
}

void processFrame(mmWaveSensor &sensor) {
 
  // Check for AAAA header
  if(sensor.buf_len >= 4 && sensor.mmWaveBuffer[0] == 0xAA && sensor.mmWaveBuffer[1] == 0xAA) {
    //Serial.println("Found AAAA header");
    
    // Check minimum length for complete frame
    if(sensor.buf_len < 6) {
      Serial.println("Frame too short");
      return;
    }
    
    // Check data type (byte 2)
    uint8_t dataType = sensor.mmWaveBuffer[2];
    //Serial.print("Data type: 0x");
    //Serial.println(dataType, HEX);
    
    if((dataType == 0x02 || dataType == 0x01)  && !sensor.notifiedPresence) { // Micromotion or motion
      Serial.println("Data type: Micromotion target (0x02)");
      
      sensor.notifiedPresence = true; // Reset notification flag
      // Extract distance data (bytes 5-6, little endian)
      uint16_t distance = (sensor.mmWaveBuffer[3] << 8) | sensor.mmWaveBuffer[4];
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      
      // Check for end of frame (bytes 7-8: 55 55)
      if(sensor.buf_len >= 7 && sensor.mmWaveBuffer[5] == 0x55 && sensor.mmWaveBuffer[6] == 0x55) {
        //Serial.println("End of frame: 55 55 found");
      }
      sensor.presenceDetected = true;
      sensor.notifiedAbsence = false; // Allow absence notification
    }else if(dataType == 0x00 && !sensor.notifiedAbsence){ // No target
      sensor.presenceDetected = false; 
      Serial.print("No target detected: 0x");
      Serial.println(dataType, HEX);
      sensor.notifiedPresence = false; // Allow new presence notification
      sensor.notifiedAbsence = true; // Set absence notification flag
    }

    
    
  } else {
    Serial.println("No AAAA header found");
  }
}

void initMMWaveSensor(HardwareSerial &ld2411Serial, mmWaveSensor &sensor) {
    bool mmWaveConnected = false;
  // Initialize serial
  ld2411Serial.begin(256000, SERIAL_8N1, 16, 17);
  
  // Verify by checking if we receive data
  mmWaveConnected = checkMMWaveConnection(ld2411Serial);
  if(mmWaveConnected) {
      sensor.connected = true;
  } else {
      sensor.connected = false;
  }
  Serial.println(mmWaveConnected ? "LD2411S connected!" : "LD2411S not detected");
}

bool checkMMWaveConnection(HardwareSerial &ld2411Serial) {
    Serial.println("Checking LD2411S connection...");
    
    // Clear any existing data
    while(ld2411Serial.available()) {
        char c = ld2411Serial.read();
        Serial.print("Flushed: ");
        Serial.println(c, HEX);
    }
    
    Serial.println("Listening for LD2411S data (move in front of sensor)...");
    
    // Wait for data from sensor
    unsigned long startTime = millis();
    while(millis() - startTime < 5000) { // 5 second timeout
        if(ld2411Serial.available()) {
            uint8_t data = ld2411Serial.read();
            Serial.print("Received byte: 0x");
            Serial.println(data, HEX);
            return true;
        }
        delay(10);
    }
    
    Serial.println("No data received from LD2411S");
    Serial.println("Troubleshooting:");
    Serial.println("1. Check wiring: ESP32 GPIO16 → Sensor TXD");
    Serial.println("2. Check wiring: ESP32 GPIO17 → Sensor RXD"); 
    Serial.println("3. Check power: Sensor needs 5V");
    Serial.println("4. Check baud rate: 256000");
    Serial.println("5. Try moving in front of sensor");
    
    return false;
}