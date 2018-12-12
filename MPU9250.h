#include <Wire.h>

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

class MPU9250 {

public:
    MPU9250();

    void init(int8_t, int8_t);
    void read();

    int16_t getAccelerationX();
    int16_t getAccelerationY();
    int16_t getAccelerationZ();
    int16_t getGyroscopeX();
    int16_t getGyroscopeY();
    int16_t getGyroscopeZ();
    int16_t getMagnetometerX();
    int16_t getMagnetometerY();
    int16_t getMagnetometerZ();

private:
    void I2Cread(uint8_t, uint8_t, uint8_t, uint8_t*);
    void I2CwriteByte(uint8_t, uint8_t, uint8_t);

    int8_t MPU9250_ADDRESS;
    int8_t MAG_ADDRESS;

    uint8_t buffer[14];
    uint8_t buffer_magnetometer[7];
};
