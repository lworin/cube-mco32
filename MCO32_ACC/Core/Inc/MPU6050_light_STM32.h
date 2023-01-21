/**
  * MPU6050_light
  * Adapatação da biblioteca para MCUs STM32,
  * originariamente desenvolvidade para Arduino
  * Por: Romain JL. Fétick e Tockn (v1.5.2)
  * Adaptado por: Leandro Poloni Dantas
  * Data: 12/07/2020
  * Versão: 1.0
  */

#ifndef MPU6050_LIGHT_STM32_H
#define MPU6050_LIGHT_STM32_H

#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42

#define GYRO_LSB_2_DEGSEC  65.5     // [bit/(°/s)]
#define ACC_LSB_2_G        16384.0  // [bit/gravity]
#define RAD_2_DEG          57.29578 // [°/rad]
#define GYRO_OFFSET_NB_MES 3000     //
#define TEMP_LSB_2_DEGREE  340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET    12412.0  //

#define DEFAULT_GYRO_COEFF 0.98

void MPU6050_init(I2C_HandleTypeDef *hi2c);
void MPU6050_init2(I2C_HandleTypeDef *hi2c, float aC, float gC);
void MPU6050_begin();
void MPU6050_setGyroOffsets(float x, float y, float z);

void writeMPU6050(uint8_t reg, uint8_t data);
uint8_t readMPU6050(uint8_t reg);
uint8_t readMPU6050_n(uint8_t reg, uint8_t *data, uint8_t size);

float MPU6050_getTemp();

float MPU6050_getAccX();
float MPU6050_getAccY();
float MPU6050_getAccZ();

float MPU6050_getGyroX();
float MPU6050_getGyroY();
float MPU6050_getGyroZ();

void MPU6050_calcGyroOffsets();

float MPU6050_getGyroXoffset();
float MPU6050_getGyroYoffset();
float MPU6050_getGyroZoffset();

void MPU6050_update();

float MPU6050_getAccAngleX();
float MPU6050_getAccAngleY();

float MPU6050_getAngleX();
float MPU6050_getAngleY();
float MPU6050_getAngleZ();

void error_led(void);

#endif
