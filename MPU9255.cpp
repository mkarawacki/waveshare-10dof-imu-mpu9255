// 
// 
// 

#include "MPU9255.h"
#include <Wire.h>

enum Ascale {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum Gscale {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

enum Mscale {
	MFS_14BITS = 0,// 0.6 mG na LSB
	MFS_16BITS // 0.15 mG na LSB
	//1 Gauss = 10^-4 Tesli
};


uint8_t Gscale, Ascale, Mscale;
uint8_t Mmode;
float aRes, gRes, mRes;
int intPin;
int myLed = 13;
float SelfTest[6];
///

MPU9255::MPU9255(int interruptPin, int doklG, int doklA, int doklM)
{
	/*Gscale = GFS_250DPS;
	Ascale = AFS_2G;
	Mscale = MFS_16BITS;*/
	switch (doklG)
	{
	case 250:Gscale = GFS_250DPS;  break;
	case 500:Gscale = GFS_500DPS;  break;
	case 1000:Gscale = GFS_1000DPS;  break;
	case 2000:Gscale = GFS_2000DPS;  break;
	default:Gscale = GFS_250DPS; break;
	}
	switch (doklA)
	{
	case 2:Ascale = AFS_2G;  break;
	case 4:Ascale = AFS_4G;  break;
	case 8:Ascale = AFS_8G;  break;
	case 16:Ascale = AFS_16G;  break;
	default:Ascale = AFS_2G;  break;
	}
	switch (doklM) 
	{
	case 6: Mscale = MFS_14BITS; break;
	case 15: Mscale = MFS_16BITS; break;
	default: Mscale = MFS_14BITS; break;
	}
	Mmode = 0x02;
	intPin = interruptPin;
}

void MPU9255::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{

	Wire.beginTransmission(address);
	Wire.write(subAddress);
	Wire.write(data);
	Wire.endTransmission();
}

uint8_t MPU9255::readByte(uint8_t address, uint8_t subAddress)
{

	uint8_t data;
	Wire.beginTransmission(address);
	Wire.write(subAddress);
	Wire.endTransmission(false);
	Wire.requestFrom(address, (uint8_t)1);
	data = Wire.read();
	return data;
}

void MPU9255::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(address);
	Wire.write(subAddress);
	Wire.endTransmission(false);
	uint8_t i = 0;
	Wire.requestFrom(address, count);
	while (Wire.available()) {
		dest[i++] = Wire.read();

	}
}


//dokladnosc w jednostkach przyspieszenia grawitacyjnego g
void  MPU9255::readAccelData(int16_t * destination)
{
	uint8_t rawData[6];
	
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
	destination[0] = (((int16_t)rawData[0] << 8) | rawData[1]);
	destination[1] = (((int16_t)rawData[2] << 8) | rawData[3]);
	destination[2] = (((int16_t)rawData[4] << 8) | rawData[5]);
}

//dokladnosc w stopniach na sekunde
void  MPU9255::readGyroData(int16_t * destination)
{
	uint8_t rawData[6];
	
	readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
	destination[0] = (((int16_t)rawData[0] << 8) | rawData[1]);
	destination[1] = (((int16_t)rawData[2] << 8) | rawData[3]);
	destination[2] = (((int16_t)rawData[4] << 8) | rawData[5]);
}

void  MPU9255::readMagData(int16_t * destination)
{
	uint8_t rawData[7];
	if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) {
		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
		uint8_t c = rawData[6];
		if (!(c & 0x08)) {
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		}
	}
}
//dokladnosc w jednostkach przyspieszenia grawitacyjnego g
void  MPU9255::readAccelData(float * destination)
{
	uint8_t rawData[6];
	
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
	destination[0] = (((int16_t)rawData[0] << 8) | rawData[1]);
	destination[1] = (((int16_t)rawData[2] << 8) | rawData[3]);
	destination[2] = (((int16_t)rawData[4] << 8) | rawData[5]);
}

//dokladnosc w stopniach na sekunde
void  MPU9255::readGyroData(float * destination)
{
	uint8_t rawData[6];
	
	readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
	destination[0] = (((int16_t)rawData[0] << 8) | rawData[1]);
	destination[1] = (((int16_t)rawData[2] << 8) | rawData[3]);
	destination[2] = (((int16_t)rawData[4] << 8) | rawData[5]);
}

void  MPU9255::readMagData(float * destination)
{
	uint8_t rawData[7];
	if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) {
		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
		uint8_t c = rawData[6];
		if (!(c & 0x08)) {
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		}
	}
}

int16_t  MPU9255::readTempData()
{
	uint8_t rawData[2];
	readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);
	return ((int16_t)rawData[0] << 8) | rawData[1];
}

void  MPU9255::initAK8963(float * destination)
{
	uint8_t rawData[3];
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
	delay(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F);
	delay(10);
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);
	destination[0] = (float)(rawData[0] - 128) / 256. + 1.;
	destination[1] = (float)(rawData[1] - 128) / 256. + 1.;
	destination[2] = (float)(rawData[2] - 128) / 256. + 1.;
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
	delay(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode);
	delay(10);
}
/// inicjalizacja MPU9250
void  MPU9255::initMPU9250()
{
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
	delay(100);
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	delay(200);
	writeByte(MPU9250_ADDRESS, CONFIG, 0x03);
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);
	uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x02);
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18);
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3);
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18);
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3);
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F);
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03);
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
	delay(1000);
}
