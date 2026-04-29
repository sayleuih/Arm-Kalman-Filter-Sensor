#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define MUX_ADDR 0x70
#define BNO_ADDR 0x28

struct IMU {
  const char* name;
  uint8_t muxChannel;
};

IMU imus[] = {
  {"BASE", 1},
  {"ARM", 7},
  {"WRIST", 0}
};

const int NUM_IMUS = sizeof(imus) / sizeof(imus[0]);

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDR);

void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  Serial.println("Initializing BNO055 IMUs...");

  for (int i = 0; i < NUM_IMUS; i++) {
    selectMuxChannel(imus[i].muxChannel);

    Serial.print("Checking ");
    Serial.print(imus[i].name);
    Serial.print(" on MUX channel ");
    Serial.println(imus[i].muxChannel);

    if (!bno.begin()) {
      Serial.print("ERROR: Could not find ");
      Serial.println(imus[i].name);
    } else {
      Serial.print("FOUND: ");
      Serial.println(imus[i].name);
      bno.setExtCrystalUse(true);
    }

    delay(500);
  }

  Serial.println("Starting yaw/roll/pitch readings...");
}

void loop() {
  for (int i = 0; i < NUM_IMUS; i++) {
    selectMuxChannel(imus[i].muxChannel);

    sensors_event_t event;
    bno.getEvent(&event);

    Serial.print(imus[i].name);
    Serial.print(" | Yaw: ");
    Serial.print(event.orientation.x, 2);
    Serial.print(" | Roll: ");
    Serial.print(event.orientation.y, 2);
    Serial.print(" | Pitch: ");
    Serial.println(event.orientation.z, 2);
  }

  Serial.println("-------------------------");
  delay(200);
}