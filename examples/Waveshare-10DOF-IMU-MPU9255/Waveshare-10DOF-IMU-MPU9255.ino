
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include "MPU9255.h"

int16_t accelCount[3];
int16_t gyroCount[3];
int16_t magCount[3];
int16_t tempCount;
float f_accelCount[3];
float f_gyroCount[3];
float f_magCount[3];
float AccelScale, GyroScale, MagScale;
float temperature;
int doklG = 250; // stopni / sekunde - dokladnosc dla czujnika zyroskopowego
int doklA = 2; // +- 2g - dokladnosc dla akcelerometru
int doklM = 6; // 0.6 mGaussa - dokladnosc dla magnetometru
float approxGndPress;
/*
stworzenie obiektow czujnikow wchodzacych w sklad Waveshare 10 DOF IMU
*/
Adafruit_BMP280 bmp; // inicjalizacja czujnika cisnienia/altymetru + termometru
MPU9255 mpu(12, doklG, doklA,doklM); // inicjalizacja akcelerometru-zyroskopu-magnetometru

void setup()
{

	Wire.begin();//inicjalizacja protokolu I2C
	TWBR = 24;
	Serial.begin(115200);
	mpu.initMPU9250();//inicjalizacja akcelerometru-zyroskopu
	float magCalibration[3];
	mpu.initAK8963(magCalibration);//inicjalizacja magnetometru
	if (!bmp.begin()) {
		Serial.println("Nie mozna nawiazac polaczenia z czujnikiem cisnienia BMP280!");
		while (1);
	}
	
  for(int i=0;i<5;i++)
  {
    approxGndPress+=bmp.readPressure();
    delay(1000);
    }
    approxGndPress/=5;
	switch (doklG)
	{
	case 250: GyroScale = 131.0; break;
	case 500: GyroScale = 65.5; break;
	case 1000: GyroScale = 32.8; break;
	case 2000: GyroScale = 16.4; break;
	default: GyroScale = 131.0; break;
	}
	switch (doklA)
	{
	case 2: AccelScale = 16384.0; break;
	case 4: AccelScale = 8192.0; break;
	case 8: AccelScale = 4096.0; break;
	case 16: AccelScale = 2048.0; break;
	default: AccelScale = 16384.0; break;
	}
	switch (doklM)
	{
	case 6: MagScale=0.6; break;
	case 15: MagScale=0.15; break;
	default: MagScale =1; break;
	}
}
void loop()
{
	while (1)
	{
		
		mpu.readAccelData(f_accelCount);
    
    for(int i=0;i<3;i++)f_accelCount[i]/=AccelScale; // podziel kazda ze skladowych przyspieszenia przez czynnik skali dla akcelerometru
		/*
		skladowe przyspieszenia
     * Serial.print("\tax "); Serial.print(f_accelCount[0]);
		Serial.print("\tay "); Serial.print(f_accelCount[1]);
		Serial.print("\taz "); Serial.print(f_accelCount[2]);
		*/
    Serial.print("\t |a| = "); Serial.print(sqrt(sq(f_accelCount[0]) + sq(f_accelCount[1]) + sq(f_accelCount[2])));
		mpu.readGyroData(f_gyroCount);
    for(int i=0;i<3;i++)f_gyroCount[i]/=GyroScale;// podziel kazda ze skladowych przyspieszenia katowego przez czynnik skali dla zyroskopu
      /*
	  skladowe przyspieszenia katowego

		Serial.print("\tal_x "); Serial.print(f_gyroCount[0]);
		Serial.print("\tal_y "); Serial.print(f_gyroCount[1]);
		Serial.print("\tal_z "); Serial.print(f_gyroCount[2]);
		*/
        Serial.print("\t |w| = "); Serial.print(sqrt(sq(f_gyroCount[0]) + sq(f_gyroCount[1]) + sq(f_gyroCount[2])));
		mpu.readMagData(f_magCount);
		for(int i=0;i<3;i++)f_magCount[i]/=MagScale;// podziel kazda ze skladowych pola magnetycznego przez czynnik skali dla magnetometru

      float kierunek=atan2(f_magCount[1],f_magCount[0]); // kierunek magnetyczny z funkcji arcustangens2
      if(kierunek<0) kierunek+=2*PI;
      float kierStopnie = kierunek * 180/PI; // konwersja kierunku z radianow na stopnie
      Serial.print("\t kierunek = ");
	  Serial.print(kierStopnie);
		/* skladowe pola magnetycznego
		
		Serial.print("\tBx "); Serial.print(f_magCount[0]);
		Serial.print("\tBy "); Serial.print(f_magCount[1]);
		Serial.print("\tBz "); Serial.print(f_magCount[2]);
		
		*/
		tempCount = mpu.readTempData();
		Serial.print("\tT ");
		Serial.print(tempCount / 100.0);
		Serial.print(" *C");
		Serial.print("\tP = ");
		Serial.print(bmp.readPressure()/100.0);
		Serial.print(" hPa");
		Serial.print("\tApprox alt = ");
		Serial.print(bmp.readAltitude(1019.0)); // wysokość liczona wzgledem cisnienia przy powierzchni
		Serial.println(" m");
		delay(1000);
	}
}
