// MPU9255.h

#ifndef _MPU9255_h
#define _MPU9255_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
// rejestry czujnikow
#define AK8963_ADDRESS 0x0C
#define AK8963_ST1 0x02
#define AK8963_XOUT_L 0x03
#define AK8963_XOUT_H 0x04
#define AK8963_YOUT_L 0x05
#define AK8963_YOUT_H 0x06
#define AK8963_ZOUT_L 0x07
#define AK8963_ZOUT_H 0x08
#define AK8963_CNTL 0x0A
#define AK8963_ASAX 0x10
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define MOT_DUR 0x20
#define ZMOT_THR 0x21
#define ZRMOT_DUR 0x22
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define PWR_MGMT_1 0x6B
#define WHO_AM_I_MPU9250 0x75
#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E


#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69
#else
#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C
#endif
class MPU9255 {
public:
	MPU9255::MPU9255(int interruptPin, int doklG, int doklA, int doklM);

	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
	uint8_t readByte(uint8_t address, uint8_t subAddress);
	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
	void readAccelData(int16_t * destination);
	void readGyroData(int16_t * destination);
	void readMagData(int16_t * destination);
	void readAccelData(float * destination);
	void readGyroData(float * destination);
	void readMagData(float * destination);
	int16_t readTempData();
	void initAK8963(float * destination);
	void initMPU9250();
};
#endif

