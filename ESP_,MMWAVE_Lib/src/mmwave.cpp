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
    unsigned long currentTime = millis();
    
    if((dataType == 0x02 || dataType == 0x01)  && !sensor.notifiedPresence) { // Micromotion or motion
      Serial.println("Data type: Micromotion target (0x02)");
      
      
      // Extract distance data (bytes 5-6, little endian)
      uint16_t distance = (sensor.mmWaveBuffer[3] << 8) | sensor.mmWaveBuffer[4];
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      
      // Check for end of frame (bytes 7-8: 55 55)
      if(sensor.buf_len >= 7 && sensor.mmWaveBuffer[5] == 0x55 && sensor.mmWaveBuffer[6] == 0x55) {
        //Serial.println("End of frame: 55 55 found");
      }
      sensor.lastPresenceTime = currentTime;
      sensor.notifiedPresence = true; // Reset notification flag
      sensor.presenceDetected = true;
      sensor.notifiedAbsence = false; // Allow absence notification
    }else if(dataType == 0x00 && !sensor.notifiedAbsence){ // No target

      // Debounce absence
      if (currentTime - sensor.lastPresenceTime > sensor.absenceDebounceDelay) {
        sensor.presenceDetected = false;
        sensor.notifiedPresence = false;
        sensor.notifiedAbsence = true;
        Serial.print("No target detected: 0x");
        Serial.print(dataType, HEX);
        Serial.println(", Confirmed absence (after debounce)");
      } else {
        //Serial.println("Ignoring brief absence (debouncing)");
      }
    }

    
    
  } else {
    Serial.println("No AAAA header found");
  }
}

void initMMWaveSensor(HardwareSerial &ld2411Serial, mmWaveSensor &sensor) {
  // Initialize serial
  ld2411Serial.begin(256000, SERIAL_8N1, 16, 17);
  
  delay(200); // allow sensor to boot

  // Try to read parameters as a connectivity test
  if (getParam(ld2411Serial, sensor)) {
    sensor.connected = true;
    Serial.println("LD2411S connected!"); 
  } else {
    sensor.connected = false;
    Serial.println("LD2411S not responding to getParam");
  }
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

bool waitForAck(HardwareSerial &ld2411Serial, uint16_t expectedID, mmWaveSensor &sensor, unsigned long timeout) {
  unsigned long start = millis();
  uint8_t tempBuffer[128];
  size_t idx = 0;

  Serial.printf("Waiting for ACK for command 0x%04X, timeout: %lums\n", expectedID, timeout);

  while (millis() - start < timeout) {
    if (ld2411Serial.available()) {
      uint8_t b = ld2411Serial.read();
      
      Serial.printf("%02X ", b);

      // Store in temporary buffer
      if (idx < sizeof(tempBuffer)) {
        tempBuffer[idx++] = b;
      } else {
        Serial.println("\nTemp buffer full, resetting");
        idx = 0;
        continue;
      }

      // Check for complete command frame
      if (idx >= 10) { // Minimum size for a command frame
        if (tempBuffer[0] == 0xFD && tempBuffer[1] == 0xFC && 
            tempBuffer[2] == 0xFB && tempBuffer[3] == 0xFA) {
          
          uint16_t dataLen = tempBuffer[4] | (tempBuffer[5] << 8);
          uint16_t totalLen = 4 + 2 + dataLen + 4;
          
          if (idx >= totalLen) {
            // Verify footer
            if (tempBuffer[totalLen-4] == 0x04 && tempBuffer[totalLen-3] == 0x03 &&
                tempBuffer[totalLen-2] == 0x02 && tempBuffer[totalLen-1] == 0x01) {
              
              // Copy valid frame to sensor buffer
              if (totalLen <= sensor.buff_size) {
                memcpy(sensor.mmWaveBuffer, tempBuffer, totalLen);
                sensor.buf_len = totalLen;
                
                uint16_t ackCmdId = tempBuffer[6] | (tempBuffer[7] << 8);
                Serial.printf("\nReceived ACK command ID: 0x%04X\n", ackCmdId);
                
                uint16_t expectedAckId = expectedID | 0x0100;
                if (ackCmdId == expectedAckId) {
                  uint16_t status = tempBuffer[8] | (tempBuffer[9] << 8);
                  if (status == 0x0000) {
                    Serial.println("ACK SUCCESS");
                    return true;
                  }
                }
              }
            }
            // Frame processed, reset for next one
            idx = 0;
          }
        }
        // Check for distance frame and skip it
        else if (tempBuffer[0] == 0xAA && tempBuffer[1] == 0xAA) {
          // Look for distance frame end
          if (idx >= 6 && tempBuffer[idx-2] == 0x55 && tempBuffer[idx-1] == 0x55) {
            Serial.println("\nSkipped distance frame");
            idx = 0; // Reset temp buffer, ready for next frame
          }
          // If buffer getting full without finding end, reset
          else if (idx >= sizeof(tempBuffer) - 10) {
            Serial.println("\nBuffer full, no complete frame found");
            idx = 0;
          }
        }
      }
    }
    delay(1);
  }
  
  Serial.println("\nACK timeout");
  return false;
}

bool sendEnableConfig(HardwareSerial &ld2411Serial, mmWaveSensor &sensor) {

  // More aggressive buffer clearing
  unsigned long clearStart = millis();
  Serial.println("Clearing serial buffer...");
  while (ld2411Serial.available() || (millis() - clearStart < 100)) {
    if (ld2411Serial.available()) {
      uint8_t discarded = ld2411Serial.read();
      Serial.printf("Discarded: %02X ", discarded);
    }
    delay(1);
  }
  Serial.println("\nBuffer cleared");
  
  uint8_t cmd[] = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
  Serial.print("Sending enable config: ");
  for (int i = 0; i < sizeof(cmd); i++) {
    Serial.printf("%02X ", cmd[i]);
  }
  Serial.println();
  
  ld2411Serial.write(cmd, sizeof(cmd));
  ld2411Serial.flush();
  
  return waitForAck(ld2411Serial, 0xFF, sensor);
}


bool sendEndConfig(HardwareSerial &ld2411Serial, mmWaveSensor &sensor) {
  // Clear any pending data first
  while (ld2411Serial.available()) {
    ld2411Serial.read();
  }
  
  uint8_t cmd[] = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xFE,0x00,0x04,0x03,0x02,0x01};
  
  Serial.print("Sending end config: ");
  for (int i = 0; i < sizeof(cmd); i++) {
    Serial.printf("%02X ", cmd[i]);
  }
  Serial.println();
  
  ld2411Serial.write(cmd, sizeof(cmd));
  ld2411Serial.flush();
  
  return waitForAck(ld2411Serial, 0xFE, sensor);
}


bool sendReboot(HardwareSerial &ld2411Serial, mmWaveSensor &sensor) {
  uint8_t cmd[] = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0x04,0x00,0x04,0x03,0x02,0x01};
  ld2411Serial.write(cmd, sizeof(cmd));
  return waitForAck(ld2411Serial, 0x0004, sensor);
}

bool sendCommand(HardwareSerial &ld2411Serial, uint16_t cmdId, const uint8_t* data, uint8_t dataLen, mmWaveSensor &sensor) {
  // Clear any pending data first
  while (ld2411Serial.available()) {
    ld2411Serial.read();
  }
  // Header
  uint8_t packet[64];
  int idx = 0;
  packet[idx++] = 0xFD;
  packet[idx++] = 0xFC;
  packet[idx++] = 0xFB;
  packet[idx++] = 0xFA;

  // Lunghezza = CMD(2) + DATA
  uint16_t len = 2 + dataLen;
  packet[idx++] = len & 0xFF;
  packet[idx++] = (len >> 8) & 0xFF;

  // Command ID
  packet[idx++] = cmdId & 0xFF;
  packet[idx++] = (cmdId >> 8) & 0xFF;

  // Dati
  for (uint8_t i = 0; i < dataLen; i++) {
    packet[idx++] = data[i];
  }

  // Footer
  packet[idx++] = 0x04;
  packet[idx++] = 0x03;
  packet[idx++] = 0x02;
  packet[idx++] = 0x01;

  // Invio
  ld2411Serial.write(packet, idx);
  ld2411Serial.flush();

  return waitForAck(ld2411Serial, cmdId, sensor);
}




bool getParam(HardwareSerial &ld2411Serial, mmWaveSensor &sensor) {
  // Variables to store all parameters
  int maxMotionRange;        // Maximum range of motion
  int minMotionRange;        // Recent range of motion  
  int maxMicroMotionRange;   // Maximum micro-movement range
  int minMicroMotionRange;   // Recent micro-movement range
  int noOneWaitingTime;      // Unattended waiting time (in 100ms units)
  
  if(!sendEnableConfig(ld2411Serial, sensor)){
    Serial.println("Failed to enable configuration mode");
    return false;
  }
  
  delay(100); // Small delay between commands

  if(!sendCommand(ld2411Serial, 0x0073, nullptr, 0, sensor)){
    Serial.println("Failed to send read parameters command");
    sendEndConfig(ld2411Serial, sensor); // Try to clean up
    return false;
  } 
  Serial.println("Parameter data received in ACK:");
  //sensor.showFrame();

  // Parse parameters from the ACK response data
  // Bytes 10-19 contain the parameters (as we saw in your debug output)
  if (sensor.buf_len >= 20) {
    maxMotionRange = sensor.mmWaveBuffer[10] | (sensor.mmWaveBuffer[11] << 8);
    minMotionRange = sensor.mmWaveBuffer[12] | (sensor.mmWaveBuffer[13] << 8);
    maxMicroMotionRange = sensor.mmWaveBuffer[14] | (sensor.mmWaveBuffer[15] << 8);
    minMicroMotionRange = sensor.mmWaveBuffer[16] | (sensor.mmWaveBuffer[17] << 8);
    noOneWaitingTime = sensor.mmWaveBuffer[18] | (sensor.mmWaveBuffer[19] << 8);

    // Print all parameters
    Serial.println("=== LD2411-S Current Parameters ===");
    Serial.printf("Maximum Motion Range: %d cm\n", maxMotionRange);
    Serial.printf("Minimum Motion Range: %d cm\n", minMotionRange);
    Serial.printf("Maximum Micro-Motion Range: %d cm\n", maxMicroMotionRange);
    Serial.printf("Minimum Micro-Motion Range: %d cm\n", minMicroMotionRange);
    Serial.printf("No One Waiting Time: %d units (%.1f seconds)\n", 
                  noOneWaitingTime, noOneWaitingTime * 0.1);
    Serial.println("===================================");

    // Update the parameters
    sensor.maxMotionRange.value = maxMotionRange;
    sensor.minMotionRange.value = minMotionRange;
    sensor.maxMicroMotionRange.value = maxMotionRange;
    sensor.minMicroMotionRange.value = minMotionRange;
    sensor.noOneWaitingTime.value = noOneWaitingTime;

  } else {
    Serial.println("ACK response too short to contain parameters");
    sendEndConfig(ld2411Serial, sensor);
    return false;
  }

  // Step 3: Disable configuration mode
  if (!sendEndConfig(ld2411Serial, sensor)) {
    Serial.println("Warning: Failed to disable configuration mode");
  }

  return true;
}

bool setParameter(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, 
                  intValue &paramToSet, uint16_t newValue) {
    
    // Validate the new value
    if (newValue < paramToSet.min || newValue > paramToSet.max) {
        Serial.printf("Error: Value %d out of range [%d, %d]\n", 
                     newValue, paramToSet.min, paramToSet.max);
        return false;
    }

    Serial.printf("Setting parameter to %d (range: %d-%d)\n", 
                 newValue, paramToSet.min, paramToSet.max);

    // Enable configuration mode first
    if (!sendEnableConfig(ld2411Serial, sensor)) {
        Serial.println("Failed to enable configuration mode");
        return false;
    }

    delay(100);

    // Update the parameter in our struct
    paramToSet.value = newValue;

    // Prepare the parameter data according to Table 7 in protocol
    uint8_t paramData[30] = {0}; // 30 bytes parameter data

    // Fill the parameter data structure
    // Bytes 0-1: Reserved (00 00)
    paramData[0] = 0x00;
    paramData[1] = 0x00;
    
    // Bytes 2-3: Maximum range of motion (little-endian)
    paramData[2] = sensor.maxMotionRange.value & 0xFF;
    paramData[3] = (sensor.maxMotionRange.value >> 8) & 0xFF;
    
    // Bytes 4-5: Reserved (00 00)
    paramData[4] = 0x00;
    paramData[5] = 0x00;
    
    // Bytes 6-7: Fixed (01 00)
    paramData[6] = 0x01;
    paramData[7] = 0x00;
    
    // Bytes 8-9: Recent range of motion (little-endian)
    paramData[8] = sensor.minMotionRange.value & 0xFF;
    paramData[9] = (sensor.minMotionRange.value >> 8) & 0xFF;
    
    // Bytes 10-11: Reserved (00 00)
    paramData[10] = 0x00;
    paramData[11] = 0x00;
    
    // Bytes 12-13: Fixed (02 00)
    paramData[12] = 0x02;
    paramData[13] = 0x00;
    
    // Bytes 14-15: Maximum micro-movement range (little-endian)
    paramData[14] = sensor.maxMicroMotionRange.value & 0xFF;
    paramData[15] = (sensor.maxMicroMotionRange.value >> 8) & 0xFF;
    
    // Bytes 16-17: Reserved (00 00)
    paramData[16] = 0x00;
    paramData[17] = 0x00;
    
    // Bytes 18-19: Fixed (03 00)
    paramData[18] = 0x03;
    paramData[19] = 0x00;
    
    // Bytes 20-21: Recent micro-movement range (little-endian)
    paramData[20] = sensor.minMicroMotionRange.value & 0xFF;
    paramData[21] = (sensor.minMicroMotionRange.value >> 8) & 0xFF;
    
    // Bytes 22-23: Reserved (00 00)
    paramData[22] = 0x00;
    paramData[23] = 0x00;
    
    // Bytes 24-25: Fixed (04 00)
    paramData[24] = 0x04;
    paramData[25] = 0x00;
    
    // Bytes 26-27: No one duration (little-endian)
    paramData[26] = sensor.noOneWaitingTime.value & 0xFF;
    paramData[27] = (sensor.noOneWaitingTime.value >> 8) & 0xFF;
    
    // Bytes 28-29: Reserved (00 00)
    paramData[28] = 0x00;
    paramData[29] = 0x00;

    // Send the set parameter command
    if (!sendCommand(ld2411Serial, 0x0067, paramData, 30, sensor)) {
        Serial.println("Failed to set parameters");
        sendEndConfig(ld2411Serial, sensor);
        return false;
    }

    Serial.println("Parameters set successfully");

    // Disable configuration mode
    if (!sendEndConfig(ld2411Serial, sensor)) {
        Serial.println("Warning: Failed to disable configuration mode");
    }

    return true;
}

bool setMaxMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value) {
    return setParameter(ld2411Serial, sensor, sensor.maxMotionRange, value);
}

bool setMinMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value) {
    return setParameter(ld2411Serial, sensor, sensor.minMotionRange, value);
}

bool setMaxMicroMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value) {
    return setParameter(ld2411Serial, sensor, sensor.maxMicroMotionRange, value);
}

bool setMinMicroMotionRange(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value) {
    return setParameter(ld2411Serial, sensor, sensor.minMicroMotionRange, value);
}

bool setNoOneWaitingTime(HardwareSerial &ld2411Serial, mmWaveSensor &sensor, uint16_t value) {
    return setParameter(ld2411Serial, sensor, sensor.noOneWaitingTime, value);
}