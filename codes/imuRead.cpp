#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define MUX_ADDR 0x70
#define BNO_ADDR 0x28

class imuRead {
private:
  const char* name;
  uint8_t channel;
  Adafruit_BNO055* bno;

  float homeRoll = 0.0;
  float homePitch = 0.0;
  float homeYaw = 0.0;

  bool homeSet = false;

  void selectMuxChannel() {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delay(10);
  }

  float wrapAngle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
  }

public:
  imuRead(const char* imuName, uint8_t muxChannel, Adafruit_BNO055* sharedBNO) {
    name = imuName;
    channel = muxChannel;
    bno = sharedBNO;
  }

  bool begin() {
    selectMuxChannel();

    if (!bno->begin()) {
      Serial.print("ERROR: Could not find ");
      Serial.println(name);
      return false;
    }

    bno->setExtCrystalUse(true);

    Serial.print("READY: ");
    Serial.print(name);
    Serial.print(" on MUX channel ");
    Serial.println(channel);

    return true;
  }

  bool readRaw(float &roll, float &pitch, float &yaw) {
    selectMuxChannel();

    sensors_event_t event;
    bno->getEvent(&event);

    // Adafruit BNO055 gives:
    // orientation.x = yaw/heading
    // orientation.y = roll
    // orientation.z = pitch
    yaw = event.orientation.x;
    roll = event.orientation.y;
    pitch = event.orientation.z;

    return true;
  }

  void setHome() {
    float roll, pitch, yaw;
    readRaw(roll, pitch, yaw);

    homeRoll = roll;
    homePitch = pitch;
    homeYaw = yaw;

    homeSet = true;

    Serial.print(name);
    Serial.println(" HOME set.");
  }

  void printRelativeCSV() {
    float roll, pitch, yaw;
    readRaw(roll, pitch, yaw);

    float relRoll = 0.0;
    float relPitch = 0.0;
    float relYaw = 0.0;

    if (homeSet) {
      relRoll = wrapAngle(roll - homeRoll);
      relPitch = wrapAngle(pitch - homePitch);
      relYaw = wrapAngle(yaw - homeYaw);
    } else {
      relRoll = roll;
      relPitch = pitch;
      relYaw = yaw;
    }

    Serial.print(name);
    Serial.print(",");
    Serial.print(relRoll, 2);
    Serial.print(",");
    Serial.print(relPitch, 2);
    Serial.print(",");
    Serial.println(relYaw, 2);
  }
};

// One shared BNO055 object because only one MUX channel is active at a time
Adafruit_BNO055 sharedBNO = Adafruit_BNO055(55, BNO_ADDR);

// Your current mapping
imuRead armIMU("ARM", 7, &sharedBNO);
imuRead wristIMU("WRIST", 4, &sharedBNO);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);      // ESP32 SDA, SCL
  Wire.setClock(100000);   // stable I2C speed

  Serial.println("Starting ARM/WRIST HOME-relative IMU reader...");
  Serial.println("Format: IMU,relRoll,relPitch,relYaw");

  armIMU.begin();
  delay(500);

  wristIMU.begin();
  delay(500);

  Serial.println("Place robot in HOME pose.");
  Serial.println("Type h in Serial Monitor to set HOME.");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'h' || command == 'H') {
      Serial.println("Setting HOME reference...");
      armIMU.setHome();
      wristIMU.setHome();
      Serial.println("HOME complete.");
    }
  }

  armIMU.printRelativeCSV();
  wristIMU.printRelativeCSV();

  Serial.println("---");
  delay(200);
}