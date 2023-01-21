/**
  * MPU6050_light
  * Adapatação da biblioteca para MCUs STM32,
  * originariamente desenvolvidade para Arduino
  * Por: Romain JL. Fétick e Tockn (v1.5.2)
  * Adaptado por: Leandro Poloni Dantas
  * Data: 12/07/2020
  * Versão: 1.0
  */

#include "stm32f4xx_hal.h"
#include "MPU6050_light_STM32.h"
#include <math.h>

float gyroXoffset, gyroYoffset, gyroZoffset;
float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleAccX, angleAccY;
float angleX, angleY, angleZ;
long preInterval;
float accCoef, gyroCoef;

I2C_HandleTypeDef *HI2C;

void MPU6050_init(I2C_HandleTypeDef *hi2c)
{
	//Atribuo um ponteiro ao handler da porta I2C
	HI2C = hi2c;
	accCoef = 1.0-DEFAULT_GYRO_COEFF;
	gyroCoef = DEFAULT_GYRO_COEFF;

	MPU6050_begin();
}

void MPU6050_init2(I2C_HandleTypeDef *hi2c, float aC, float gC)
{
	//Atribuo um ponteiro ao handler da porta I2C
	HI2C = hi2c;
	accCoef = aC;
	gyroCoef = gC;

	MPU6050_begin();
}

void MPU6050_begin()
{
	writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
	writeMPU6050(MPU6050_CONFIG, 0x00);
	writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);	// Configure the gyroscopes’ full scale range of 500 degree/second
	writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);	// Configure the accelerometer full scale of range +/- 2 g
	writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);	// Configure the power mode and clock source of PLL with X axis gyroscope reference
	MPU6050_update();
	angleX = MPU6050_getAccAngleX();
	angleY = MPU6050_getAccAngleY();
	preInterval = HAL_GetTick();
}

void writeMPU6050(uint8_t reg, uint8_t data)
{
    uint8_t pacote[2];
  	pacote[0] = reg;
  	pacote[1] = data;
  	 //HAL_I2C_Master_Transmit(hi2c, DevAddress,      pData,  Size, Timeout)
  	if(HAL_I2C_Master_Transmit(HI2C, MPU6050_ADDR<<1, pacote, 2,    5) == HAL_ERROR)
  	{
  		error_led();
  	}
  	//HAL_Delay(1);	//Estava no código original, mas se mostrou desnecessário
}

uint8_t readMPU6050(uint8_t reg)
{
	uint8_t data;
	HAL_I2C_Master_Transmit(HI2C, (MPU6050_ADDR<<1)|1, &reg, 1, 5);
	if(HAL_I2C_Master_Receive(HI2C, (MPU6050_ADDR<<1)|1, &data, 1, 5) == HAL_ERROR)
		error_led();
	return data;
}

uint8_t readMPU6050_n(uint8_t reg, uint8_t *data, uint8_t size)
{
	HAL_I2C_Master_Transmit(HI2C, (MPU6050_ADDR<<1)|1, &reg, 1, 5);
	if(HAL_I2C_Master_Receive(HI2C, (MPU6050_ADDR<<1)|1, data, size, 5) == HAL_ERROR)
		error_led();
	return 0;
}

void MPU6050_setGyroOffsets(float x, float y, float z)
{
	gyroXoffset = x;
	gyroYoffset = y;
	gyroZoffset = z;
}

void MPU6050_calcGyroOffsets()
{
	float xyz[3] = {0,0,0};
	uint8_t offset[6];
	int16_t b;

	for(int i = 0; i < GYRO_OFFSET_NB_MES; i++)
	{
		readMPU6050_n(0x43, offset, 6);

		for(int j=0;j<3;j++)
		{
			b  = offset[j*2] << 8;
			b |= offset[j*2+1];
			xyz[j] += ((float)b) / GYRO_LSB_2_DEGSEC;
		}
	}
	gyroXoffset = xyz[0] / GYRO_OFFSET_NB_MES;
	gyroYoffset = xyz[1] / GYRO_OFFSET_NB_MES;
	gyroZoffset = xyz[2] / GYRO_OFFSET_NB_MES;
}

void MPU6050_update()
{
	uint8_t raw[14];
	int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

	//Lê todos os parâmetros byte a byte
	readMPU6050_n(0x3B, raw, 14);	// 0x3B é o endereço do ACCEL_XOUT_H, os demais parâmetros vem na sequência

	//Combina o LSB com o MSB, todos os valores são de 16 bits
	for(int i=0;i<7;i++)
	{
		rawData[i]  = raw[i*2] << 8;
		rawData[i] |= raw[i*2+1];
	}

	//Conversão para g, grau Celsius e grau respectivamente de acordo com os parâmetros de conversão
	accX = ((float)rawData[0]) / ACC_LSB_2_G;
	accY = ((float)rawData[1]) / ACC_LSB_2_G;
	accZ = ((float)rawData[2]) / ACC_LSB_2_G;
	temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
	gyroX = ((float)rawData[4]) / GYRO_LSB_2_DEGSEC - gyroXoffset;
	gyroY = ((float)rawData[5]) / GYRO_LSB_2_DEGSEC - gyroYoffset;
	gyroZ = ((float)rawData[6]) / GYRO_LSB_2_DEGSEC - gyroZoffset;

	float sgZ = (accZ>=0)-(accZ<0);
	angleAccX = atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG;
	angleAccY = - atan2(accX, sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG;

	unsigned long Tnew = HAL_GetTick();
	float dt = (Tnew - preInterval) * 1e-3;
	preInterval = Tnew;

	angleX = (gyroCoef * (angleX + gyroX*dt)) + (accCoef * angleAccX);
	angleY = (gyroCoef * (angleY + gyroY*dt)) + (accCoef * angleAccY);
	angleZ += gyroZ*dt;
}

void error_led(void)
{
	while (1)
	{
		//Espera por um reset
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(100);
	}
}

float MPU6050_getTemp(){ return temp; };

float MPU6050_getAccX(){ return accX; };
float MPU6050_getAccY(){ return accY; };
float MPU6050_getAccZ(){ return accZ; };

float MPU6050_getGyroX(){ return gyroX; };
float MPU6050_getGyroY(){ return gyroY; };
float MPU6050_getGyroZ(){ return gyroZ; };

float MPU6050_getGyroXoffset(){ return gyroXoffset; };
float MPU6050_getGyroYoffset(){ return gyroYoffset; };
float MPU6050_getGyroZoffset(){ return gyroZoffset; };

float MPU6050_getAccAngleX(){ return angleAccX; };
float MPU6050_getAccAngleY(){ return angleAccY; };

float MPU6050_getAngleX(){ return angleX; };
float MPU6050_getAngleY(){ return angleY; };
float MPU6050_getAngleZ(){ return angleZ; };
