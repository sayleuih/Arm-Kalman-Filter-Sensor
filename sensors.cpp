#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define I2C_DEVICE "/dev/i2c-1"
#define TCA_ADDR 0x70
#define BNO055_ADDR 0x28

int i2c_fd;

// Select channel on TCA9548A
void selectChannel(uint8_t channel) {
    uint8_t data = 1 << channel;

    if (ioctl(i2c_fd, I2C_SLAVE, TCA_ADDR) < 0) {
        perror("TCA ioctl failed");
        return;
    }

    write(i2c_fd, &data, 1);
}

// Write to BNO055 register
void writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};

    if (ioctl(i2c_fd, I2C_SLAVE, BNO055_ADDR) < 0) {
        perror("BNO ioctl failed");
        return;
    }

    write(i2c_fd, buffer, 2);
}

// Read multiple bytes from BNO055
void readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
    if (ioctl(i2c_fd, I2C_SLAVE, BNO055_ADDR) < 0) {
        perror("BNO ioctl failed");
        return;
    }

    write(i2c_fd, &reg, 1);
    read(i2c_fd, buffer, length);
}

// Initialize BNO055 (basic)
void initBNO055() {
    // Set to config mode
    writeRegister(0x3D, 0x00);
    usleep(10000);

    // Set to NDOF mode
    writeRegister(0x3D, 0x0C);
    usleep(10000);
}

// Read Euler angles
void readEuler() {
    uint8_t buffer[6];
    readRegisters(0x1A, buffer, 6);

    int16_t heading = (buffer[1] << 8) | buffer[0];
    int16_t roll    = (buffer[3] << 8) | buffer[2];
    int16_t pitch   = (buffer[5] << 8) | buffer[4];

    std::cout << "Heading: " << heading / 16.0
              << " Roll: " << roll / 16.0
              << " Pitch: " << pitch / 16.0 << std::endl;
}

int main() {
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open I2C");
        return 1;
    }

    const int NUM_SENSORS = 4;

    // Initialize all sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        selectChannel(i);
        usleep(10000);
        initBNO055();
        std::cout << "Initialized sensor on channel " << i << std::endl;
    }

    while (true) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            selectChannel(i);
            usleep(5000);

            std::cout << "Sensor " << i << ": ";
            readEuler();
        }

        usleep(500000); // 0.5 sec
    }

    close(i2c_fd);
    return 0;
}