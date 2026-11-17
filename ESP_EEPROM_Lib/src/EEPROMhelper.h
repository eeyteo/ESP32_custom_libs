#ifndef GUARD_EEPROM
#define GUARD_EEPROM

#include <Arduino.h>  // Make sure String class is defined

#define EEPROM_STRING_MAX_LEN 31  // string fields are 32 bytes apart (31 chars + null terminator)

String readEEPROM(int addr);
void writeEEPROM(int addr, String data);
int readIntEEPROM(int addr);
void writeIntEEPROM(int addr, int value);
bool readBoolEEPROM(int addr);
void writeBoolEEPROM(int addr, bool value);

#endif