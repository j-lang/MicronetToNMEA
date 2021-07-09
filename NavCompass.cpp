/*
 * CompassDecoder.cpp
 *
 *  Created on: 13 juin 2021
 *      Author: Ronan
 */

#include "BoardConfig.h"
#include "Globals.h"
#include "NavCompass.h"

#include <Wire.h>

static float Mag_LSB_XY = 1100.0F; // Varies with gain
static float Mag_LSB_Z = 980.0F;   // Varies with gain
static float Acc_LSB = 0.003;      // Varies with gain

NavCompass::NavCompass() :
		heading(0), magX(0), magY(0), magZ(0), accX(0), accY(0), accZ(0)
{
}

NavCompass::~NavCompass()
{
}

bool NavCompass::Init()
{
	LSM303DLH_I2C.begin();

	uint8_t ira = I2CRead(LSM303DLH_MAG_ADDR, IRA_REG_M);
	uint8_t irb = I2CRead(LSM303DLH_MAG_ADDR, IRB_REG_M);
	uint8_t irc = I2CRead(LSM303DLH_MAG_ADDR, IRC_REG_M);

	if ((ira != 'H') || (irb != '4') || (irc != '3'))
	{
		return false;
	}

	// Magnetic register
//	I2CWrite(LSM303DLH_MAG_ADDR, 0x0C, CRA_REG_M); // 0x0C=0b00001100 ODR 7.5Hz
//	I2CWrite(LSM303DLH_MAG_ADDR, 0x10, CRA_REG_M); // 0x10=0b00010000 ODR 15Hz
//	I2CWrite(LSM303DLH_MAG_ADDR, 0x14, CRA_REG_M); // 0x14=0b00010100 ODR 30Hz
	I2CWrite(LSM303DLH_MAG_ADDR, 0x18, CRA_REG_M); // 0x18=0b00011000 ODR 75Hz
#if defined LSM303DLH
	// DLH Acceleration register
	I2CWrite(LSM303DLH_ACC_ADDR, 0x27, CTRL_REG1_A); // 0x27=0b00100111 Normal Mode, ODR 50hz, all axes on
	I2CWrite(LSM303DLH_ACC_ADDR, 0x00, CTRL_REG4_A); // 0x00=0b00000000 full scale +/- 2Gauss, highRes on
	Acc_LSB = 0.0039;
	// DLH Magnetic register
	I2CWrite(LSM303DLH_MAG_ADDR, 0x60, CRB_REG_M);   // 0x60=0b01100000 Gauss range: +/-2.5Gauss
	Mag_LSB_XY = 635;
	Mag_LSB_Z = 570;
#elif defined LSM303DLHC
	// DLHC Acceleration register
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x47, CTRL_REG1_A); // 0x47=0b01000111 Normal Mode, ODR 50hz, all axes on
	I2CWrite(LSM303DLH_ACC_ADDR, 0x57, CTRL_REG1_A); // 0x57=0b01010111 Normal Mode, ODR 100hz, all axes on
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x67, CTRL_REG1_A); // 0x67=0b01100111 Normal Mode, ODR 200hz, all axes on
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x77, CTRL_REG1_A); // 0x77=0b01110111 Normal Mode, ODR 400hz, all axes on
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x08, CTRL_REG4_A); // 0x08=0b00001000 full scale +/-2Gauss, highRes on
//	Acc_LSB = 0.00098;
	I2CWrite(LSM303DLH_ACC_ADDR, 0x18, CTRL_REG4_A); // 0x18=0b00011000 full scale +/-4Gauss, highRes on
	Acc_LSB = 0.00195;
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x38, CTRL_REG4_A); // 0x38=0b00111000 full scale +/-8Gauss, highRes on
//	Acc_LSB = 0.0039;
	// DLHC Magnetic register
	I2CWrite(LSM303DLH_MAG_ADDR, 0x20, CRB_REG_M);   // 0x20=0b00100000 Gauss range: +/-1.3Gauss gain: 1100LSB/Gauss
	Mag_LSB_XY = 1100;
	Mag_LSB_Z = 980;
//	I2CWrite(LSM303DLH_MAG_ADDR, 0x60, CRB_REG_M);   // 0x60=0b01100000 Gauss range: +/-2.5Gauss gain: 670LSB/Gauss
//	Mag_LSB_XY = 670;
//	Mag_LSB_Z = 600;
#endif
	I2CWrite(LSM303DLH_MAG_ADDR, 0x00, MR_REG_M);    // Continuous mode

#ifdef Working_with_raw_data
	Mag_LSB_XY = 1.0F;
	Mag_LSB_Z = 1.0F;
	Acc_LSB = 1.0F;
#endif

	return true;
}

void NavCompass::GetMagneticField(float *magX, float *magY, float *magZ)
{
	uint8_t magBuffer[6];
	int16_t mx, my, mz;

	I2CBurstRead(LSM303DLH_MAG_ADDR, OUT_X_H_M, magBuffer, 6);

#if defined LSM303DLH
	mx = ((int16_t) (magBuffer[0] << 8)) | magBuffer[1];
	my = ((int16_t) (magBuffer[2] << 8)) | magBuffer[3];
	mz = ((int16_t) (magBuffer[4] << 8)) | magBuffer[5];
#elif defined LSM303DLHC
	mx = ((int16_t) (magBuffer[0] << 8)) | magBuffer[1];
	mz = ((int16_t) (magBuffer[2] << 8)) | magBuffer[3]; // stupid change in order for DLHC
	my = ((int16_t) (magBuffer[4] << 8)) | magBuffer[5];
#endif

#if defined LSM303DLH
	*magX = ((float) mx / Mag_LSB_XY * GAUSS_TO_MICROTESLA;);
	*magY = ((float) my / Mag_LSB_XY * GAUSS_TO_MICROTESLA;);
	*magZ = ((float) mz / Mag_LSB_Z * GAUSS_TO_MICROTESLA;);
#elif defined LSM303DLHC
	*magX = (float) mx / Mag_LSB_XY * GAUSS_TO_MICROTESLA;
	*magY = (float) my / Mag_LSB_XY * GAUSS_TO_MICROTESLA;
	*magZ = (float) mz / Mag_LSB_Z * GAUSS_TO_MICROTESLA;
	m.x = mx / Mag_LSB_XY * GAUSS_TO_MICROTESLA;
	m.y = my / Mag_LSB_XY * GAUSS_TO_MICROTESLA;
	m.z = mz / Mag_LSB_Z * GAUSS_TO_MICROTESLA;
#endif
}

void NavCompass::GetAcceleration(float *accX, float *accY, float *accZ)
{
	int16_t ax, ay, az;

	ax = I2CRead(LSM303DLH_ACC_ADDR, OUT_X_H_A);
	ax = (ax << 8) | I2CRead(LSM303DLH_ACC_ADDR, OUT_X_L_A);
	ay = I2CRead(LSM303DLH_ACC_ADDR, OUT_Y_H_A);
	ay = (ay << 8) | I2CRead(LSM303DLH_ACC_ADDR, OUT_Y_L_A);
	az = I2CRead(LSM303DLH_ACC_ADDR, OUT_Z_H_A);
	az = (az << 8) | I2CRead(LSM303DLH_ACC_ADDR, OUT_Z_L_A);

#if defined LSM303DLH
	*accX = (float) (ax >> 6) * Acc_LSB * GRAVITY_STANDARD; // DLH acc resolution is 10 bit
	*accY = (float) (ay >> 6) * Acc_LSB * GRAVITY_STANDARD;
	*accZ = (float) (az >> 6) * Acc_LSB * GRAVITY_STANDARD;
#elif defined LSM303DLHC
	*accX = (float) (ax >> 4) * Acc_LSB * GRAVITY_STANDARD; // DLHC registers contain a left-aligned 12-bit number, so values should be shifted right by 4 bits (divided by 16)
	*accY = (float) (ay >> 4) * Acc_LSB * GRAVITY_STANDARD;
	*accZ = (float) (az >> 4) * Acc_LSB * GRAVITY_STANDARD;
	a.x = (ax >> 4) * Acc_LSB * GRAVITY_STANDARD;
	a.y = (ay >> 4) * Acc_LSB * GRAVITY_STANDARD;
	a.z = (az >> 4) * Acc_LSB * GRAVITY_STANDARD;
#endif
}

float NavCompass::GetHeading()
{
  float mx, my, mz;
	float ax, ay, az;
#if defined LSM303DLH
	float pBow, pStarboard;
	float ey, ez;
	float normE;
#elif defined LSM303DLHC
//	const float alpha = 0.1;
//	uint16_t fXa=0, fYa=0, fZa=0;
	vector<int16_t> from = {1, 0, 0}; // x axis is reference direction
#endif

	// Get Acceleration and Magnetic data from LSM303
	GetAcceleration(&ax, &ay, &az);
//Serial.printf("ax %f ay %f az %f\n", ax, ay, az);
	GetMagneticField(&mx, &my, &mz);
//Serial.printf("mx %f my %f mz %f\n", mx, my, mz);
	// Substract calibration offsets from magnetic readings
#if defined LSM303DLH
	mx -= gConfiguration.xMagOffset;
	my -= gConfiguration.yMagOffset;
	mz -= gConfiguration.zMagOffset;
//Serial.printf("mx %f my %f mz %f\n", mx, my, mz);
	// Build starboard axis from boat's bow & gravity vector
	ey = az;
	ez = -ay;
	normE = sqrtf(ey * ey + ez * ez);

	// Project magnetic field on bow & starboard axis
	// TODO : optimize
	pBow = mx;
	pStarboard = (my * ey + mz * ez) / normE;

	float heading = atan2(-pStarboard, pBow) * 180 / M_PI;
#elif defined LSM303DLHC
	// subtract offset (average of min and max) from magnetometer readings
	m.x -= (int16_t) gConfiguration.xMagOffset;
	m.y -= (int16_t) gConfiguration.yMagOffset;
	m.z -= (int16_t) gConfiguration.zMagOffset;
//Serial.printf("mx %d my %d mz %d\n", m.x, m.y, m.z);

	//TODO : filter data, has to be synchronized with transmission, ev. higher ODR
	// Apply low-pass filter to accelerometer data
//	for (byte i = 0; i < 10; i++) {
//		GetAcceleration(&ax, &ay, &az); // Read accelerometer data
//		fXa = a.x * alpha + (fXa * (1.0 - alpha));
//		fYa = a.y * alpha + (fYa * (1.0 - alpha));
//		fZa = a.z * alpha + (fZa * (1.0 - alpha));
//		delay(2);
//	}
//	a.x = fXa;
//	a.y = fYa;
//	a.z = fZa;

	// compute E and N
	vector<float> E;
	vector<float> N;
	// D X M = E, cross acceleration vector Down with M (magnetic north + inclination) to produce "East"
	vector_cross(&m, &a, &E);
	vector_normalize(&E);
	// E X D = N, cross "East" with "Down" to produce "North" (parallel to the ground)
	vector_cross(&a, &E, &N);
	vector_normalize(&N);

	// compute heading
	float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / PI;
#endif
	if (heading < 0)
		heading += 360;
//Serial.printf("heading %f\n", heading);
	return heading;
}

unsigned char NavCompass::I2CRead(uint8_t i2cAddress, uint8_t address)
{
	char temp;

	LSM303DLH_I2C.beginTransmission(i2cAddress);
	LSM303DLH_I2C.write(address);
	LSM303DLH_I2C.endTransmission();
	LSM303DLH_I2C.requestFrom(i2cAddress, (uint8_t) 1);
	temp = LSM303DLH_I2C.read();
	LSM303DLH_I2C.endTransmission();

	return temp;
}

void NavCompass::I2CBurstRead(uint8_t i2cAddress, uint8_t address, uint8_t *buffer, uint8_t length)
{
	LSM303DLH_I2C.beginTransmission(i2cAddress);
	LSM303DLH_I2C.write(address);
	LSM303DLH_I2C.endTransmission();
	LSM303DLH_I2C.requestFrom(i2cAddress, (uint8_t) length);
	LSM303DLH_I2C.readBytes(buffer, LSM303DLH_I2C.available());
	LSM303DLH_I2C.endTransmission();
}

void NavCompass::I2CWrite(uint8_t i2cAddress, uint8_t data, uint8_t address)
{
	LSM303DLH_I2C.beginTransmission(i2cAddress);
	LSM303DLH_I2C.write(address);
	LSM303DLH_I2C.write(data);
	LSM303DLH_I2C.endTransmission();
}

void NavCompass::vector_normalize(vector<float> *a)
{
	float mag = sqrt(vector_dot(a, a));
	a->x /= mag;
	a->y /= mag;
	a->z /= mag;
}
