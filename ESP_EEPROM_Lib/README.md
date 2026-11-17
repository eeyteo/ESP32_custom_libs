# ESP_EEPROM_Lib

Small, dependency-free set of typed helpers around the Arduino EEPROM emulation
(`<EEPROM.h>`). Built for ESP32, which has no real EEPROM: the core emulates a
byte array in a flash partition that must be committed after every write.

## API

```cpp
#include "EEPROMhelper.h"

EEPROM.begin(size);                    // the sketch owns init and commit-in-flash sizing

String readEEPROM(int addr);
void   writeEEPROM(int addr, String data);
int    readIntEEPROM(int addr);
void   writeIntEEPROM(int addr, int value);
bool   readBoolEEPROM(int addr);
void   writeBoolEEPROM(int addr, bool value);
```

- `EEPROM_STRING_MAX_LEN` (default `31`) is the usable length of a string field.
  A field occupies 32 bytes: up to 31 characters plus the null terminator, so a
  long value can never bleed into the next field.
- Strings are stored null-terminated, so the byte right after the last character
  is consumed by the terminator.
- Integers are stored little-endian in `sizeof(int)` bytes (4 on ESP32).
- Booleans occupy a single byte, `1` for true and `0` for false.
- Every write calls `EEPROM.commit()`, so each call performs a flash write. Group
  your changes instead of writing in a tight loop.

## Address map convention

The library is address-based; the layout is the caller's decision. This project
uses, for 512 bytes:

| Range     | Purpose                          |
| --------- | -------------------------------- |
| 0-95      | 24 integers                      |
| 193-243   | 50 booleans                      |
| 244-500   | 8 strings of 32 characters       |

## Reusing this library in another project

This folder is a standard PlatformIO library: drop it in a project's `lib/`
directory, or in a shared folder listed under `lib_extra_dirs` in
`platformio.ini`:

```ini
lib_extra_dirs = C:\Users\khons\Documents\PlatformIO\Libs
```

Then copy `ESP_EEPROM_Lib` next to the other shared libraries (for example
`PlatformIO\Libs\ESP_EEPROM_Lib`) and every project using that
`lib_extra_dirs` can do `#include "EEPROMhelper.h"` with no `lib_deps` entry.

`EEPROM.begin(size)` must be called by the sketch before any read or write, and
`size` has to cover the highest address you use.

## Example

```cpp
#include <EEPROM.h>
#include "EEPROMhelper.h"

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);

  int boots = readIntEEPROM(0);
  writeIntEEPROM(0, boots + 1);
}
```
