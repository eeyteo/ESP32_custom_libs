#include <EEPROM.h>  // For saving credentials
#include "EEPROMhelper.h"

// Helper functions for EEPROM read/write


void writeEEPROM(int addr, String data) {
  int len = data.length();
  if (len > EEPROM_STRING_MAX_LEN) len = EEPROM_STRING_MAX_LEN;  // do not overflow into the next field
  for (int i = 0; i < len; i++) {
    EEPROM.write(addr + i, data[i]);
  }
  EEPROM.write(addr + len, '\0');  // Add null terminator
  EEPROM.commit();
  Serial.println("saving the value " + data + " of size " + String(data.length()) + " at the adress " + String(addr));
  Serial.println("Reading back, the value: " + String(readEEPROM(addr)));
}

String readEEPROM(int addr) {
  String value = "";
  for (int i = 0; i <= EEPROM_STRING_MAX_LEN; i++) {  // stop at the field boundary (31 chars + null)
    char ch = EEPROM.read(addr + i);
    if (ch == '\0') break;
    value += ch;
  }
  return value;
}



int readIntEEPROM(int addr) {
  int value = 0;
  byte* intPtr = (byte*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    intPtr[i] = EEPROM.read(addr + i);
  }
  return value;
}

void writeIntEEPROM(int addr, int value) {
  // Cast the integer to a byte pointer and write 2 or 4 bytes (depending on size of int)
  byte* intPtr = (byte*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    EEPROM.write(addr + i, intPtr[i]);
  }
  EEPROM.commit();
  //Serial.println("saving the value " + String(value) + " of size " + String(sizeof(value)) + " at the adress " + String(addr));
  //Serial.println("Reading back, the value: " + String(readIntEEPROM(addr)));
}

void writeBoolEEPROM(int addr, bool value) {
  EEPROM.write(addr, value ? 1 : 0);  // Convert boolean to 1 or 0
  EEPROM.commit();
}

bool readBoolEEPROM(int addr) {
  return EEPROM.read(addr) == 1;  // Return true if the value is 1
}