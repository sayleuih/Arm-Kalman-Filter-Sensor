#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include "estimator.h"

#define MUX_ADDR 0x70
#define BNO_ADDR 0x28

Adafruit_BNO055 bno(55, BNO_ADDR);

Estimator estimator;

struct IMUMap {
    const char* name;
    uint8_t ch;
};

IMUMap imus[3] = {
    {"BASE", 1},
    {"ARM", 7},
    {"WRIST", 0}
};

void selectMux(uint8_t ch) {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << ch);
    Wire.endTransmission();
}

void readIMU(uint8_t ch, IMUData &out) {
    selectMux(ch);

    imu::Quaternion q = bno.getQuat();
    imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    out.q[0] = q.w();
    out.q[1] = q.x();
    out.q[2] = q.y();
    out.q[3] = q.z();

    out.gyro[0] = g.x();
    out.gyro[1] = g.y();
    out.gyro[2] = g.z();

    out.accel[0] = a.x();
    out.accel[1] = a.y();
    out.accel[2] = a.z();

    out.t = micros();
}

IMUData base, arm, wrist;

uint64_t last = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(400000);

    delay(1000);

    for(int i=0;i<3;i++){
        selectMux(imus[i].ch);
        if(!bno.begin()){
            Serial.println("IMU FAIL");
        }
        bno.setExtCrystalUse(true);
        delay(100);
    }

    last = micros();
}

void loop() {

    uint64_t now = micros();
    float dt = (now - last) * 1e-6f;
    last = now;

    readIMU(1, base);
    readIMU(7, arm);
    readIMU(0, wrist);

    estimator.update(dt, base, arm, wrist);

    static int count = 0;
    if (++count % 20 == 0) {
        JointState js = estimator.get();

        Serial.print("Q: ");
        for(int i=0;i<6;i++){
            Serial.print(js.q[i], 3);
            Serial.print(" ");
        }
        Serial.println();
    }
}