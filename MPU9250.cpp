#include "MPU9250.h"

MPU9250::MPU9250(){}

void MPU9250::init(int8_t mpu_address, int8_t magnetometer_adress) {

    MPU9250_ADDRESS = mpu_address;
    MAG_ADDRESS = magnetometer_adress;

    Wire.begin();

    // Set accelerometers low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
    // Set gyroscope low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);

    // Configure gyroscope range
    I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);
    // Configure accelerometers range
    I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

    // Request continuous magnetometer measurements in 16 bits
    I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);
}

// Read a byte (Data) in device (Address) at register (Register)
void MPU9250::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();

    // Read Nbytes
    Wire.requestFrom(Address, Nbytes);
    uint8_t index = 0;

    while (Wire.available())
        Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void MPU9250::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}

void MPU9250::read() {
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, buffer);

    // Read magnetometer data
    uint8_t ST1;
    do {
        I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
    } while (!(ST1 & 0x01));

    I2Cread(MAG_ADDRESS, 0x03, 7, buffer_magnetometer);
}

int16_t MPU9250::getAccelerationX() {
    return -( buffer[0] << 8 | buffer[1] );
}

int16_t MPU9250::getAccelerationY() {
    return -( buffer[2] << 8 | buffer[3] );
}

int16_t MPU9250::getAccelerationZ() {
    return ( buffer[4] << 8 | buffer[5] );
}

int16_t MPU9250::getGyroscopeX() {
    return -( buffer[8] << 8 | buffer[9] );
}

int16_t MPU9250::getGyroscopeY() {
    return -( buffer[10] << 8 | buffer[11] );
}

int16_t MPU9250::getGyroscopeZ() {
    return ( buffer[12] << 8 | buffer[13] );
}

int16_t MPU9250::getMagnetometerX() {
    return -( buffer_magnetometer[3] << 8 | buffer_magnetometer[2] );
}

int16_t MPU9250::getMagnetometerY() {
    return -( buffer_magnetometer[1] << 8 | buffer_magnetometer[0] );
}

int16_t MPU9250::getMagnetometerZ() {
    return -( buffer_magnetometer[5] << 8 | buffer_magnetometer[4] );
}
