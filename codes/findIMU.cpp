#include <Arduino.h>
#include <Wire.h>

#define MUX_ADDR 0x70

void selectChannel(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delay(50);
}

void scanChannel(uint8_t ch) {
  selectChannel(ch);

  Serial.print("Scanning MUX channel ");
  Serial.println(ch);

  bool found = false;

  for (uint8_t addr = 1; addr < 127; addr++) {
    if (addr == MUX_ADDR) continue;

    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      Serial.println(addr, HEX);
      found = true;
    }
  }

  if (!found) {
    Serial.println("  No device found on this channel");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  Serial.println("Scanning MUX channels...");
}

void loop() {
  for (int ch = 0; ch < 8; ch++) {
    scanChannel(ch);
    Serial.println("------------------");
    delay(500);
  }

  delay(3000);
}