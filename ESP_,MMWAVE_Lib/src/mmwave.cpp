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

bool waitForAck(HardwareSerial &ld2411Serial, uint16_t expectedID, mmWaveSensor &sensor, unsigned long timeout) {
  unsigned long start = millis();
  size_t idx = 0;

  Serial.printf("Waiting for ACK for command 0x%04X, timeout: %lums\n", expectedID, timeout);
  sensor.reset(); // Clear the buffer

  while (millis() - start < timeout) {
    if (ld2411Serial.available()) {
      uint8_t b = ld2411Serial.read();
      
      // DEBUG: Show what we're receiving
      // Serial.printf("%02X ", b);

      // Store byte if space available in sensor buffer
      if (idx < sensor.buff_size) {
        sensor.mmWaveBuffer[idx++] = b;
        sensor.buf_len = idx;
      } else {
        Serial.printf("\nSensor buffer full at %d bytes! Buffer content: ", idx);
        for (int i = 0; i < sensor.buf_len; i++) {
          Serial.printf("%02X ", sensor.mmWaveBuffer[i]);
        }
        Serial.println();
        return false;
      }
      
      // Check if we have at least the header + length
      if (idx >= 6) {
        // Verify command response header
        if (sensor.mmWaveBuffer[0] == 0xFD && sensor.mmWaveBuffer[1] == 0xFC && 
            sensor.mmWaveBuffer[2] == 0xFB && sensor.mmWaveBuffer[3] == 0xFA) {
          
          // Extract data length (little-endian)
          uint16_t dataLen = sensor.mmWaveBuffer[4] | (sensor.mmWaveBuffer[5] << 8);
          
          // Calculate total frame length: header(4) + length(2) + data + footer(4)
          uint16_t totalLen = 4 + 2 + dataLen + 4;
          
          // Check if we have complete frame
          if (idx >= totalLen) {
            // Verify footer
            if (sensor.mmWaveBuffer[totalLen-4] == 0x04 && sensor.mmWaveBuffer[totalLen-3] == 0x03 &&
                sensor.mmWaveBuffer[totalLen-2] == 0x02 && sensor.mmWaveBuffer[totalLen-1] == 0x01) {
              
              // Extract command ID from ACK (little-endian)
              uint16_t ackCmdId = sensor.mmWaveBuffer[6] | (sensor.mmWaveBuffer[7] << 8);
              
              Serial.printf("Received ACK command ID: 0x%04X\n", ackCmdId);
              
              // In ACK responses, the command ID has bit 8 set
              uint16_t expectedAckId = expectedID | 0x0100;
              
              if (ackCmdId == expectedAckId) {
                // Check ACK status (bytes 8-9)
                uint16_t status = sensor.mmWaveBuffer[8] | (sensor.mmWaveBuffer[9] << 8);
                if (status == 0x0000) {
                  Serial.println("ACK SUCCESS - Data captured in sensor buffer");
                  return true;
                } else {
                  Serial.printf("ACK failed with status: 0x%04X\n", status);
                  return false;
                }
              }
            }
          }
        }
      }
    }
    delay(1);
  }
  
  Serial.println("ACK timeout");
  return false;
}

bool sendEnableConfig(HardwareSerial &ld2411Serial, mmWaveSensor &sensor) {

  // Clear all previous distance data from serial buffer
  while (ld2411Serial.available()) {
    ld2411Serial.read();
  }

  uint8_t cmd[] = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
  ld2411Serial.write(cmd, sizeof(cmd));
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



bool resetToDefault(HardwareSerial &ld2411Serial, mmWaveSensor &sensor) {
  return sendCommand(ld2411Serial, 0x0068, nullptr, 0, sensor);
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
    sensor.maxMotionRange = maxMotionRange;
    sensor.minMotionRange = minMotionRange;
    sensor.maxMicroMotionRange = maxMotionRange;
    sensor.minMicroMotionRange = minMotionRange;
    sensor.noOneWaitingTime = noOneWaitingTime;

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

