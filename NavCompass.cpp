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

static float LSB_per_Gauss_XY = 1100.0F; // Varies with range
static float LSB_per_Gauss_Z = 980.0F;   // Varies with range
static float mGal_per_LSB = 1.0F;        // Varies with range

const float alpha = 0.15;
float fXa = 0;
float fYa = 0;
float fZa = 0;
float fXm = 0;
float fYm = 0;
float fZm = 0;

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
	I2CWrite(LSM303DLH_ACC_ADDR, 0x00, CTRL_REG4_A); // 0x00=0b00000000 Range: +/-2 Gal, Sens.: 1mGal/LSB
	mGal_per_LSB = 1.0;
	// DLH Magnetic register
	I2CWrite(LSM303DLH_MAG_ADDR, 0x60, CRB_REG_M);   // 0x60=0b01100000 Range: +/-2.5 Gauss gain: 635LSB/Gauss
	LSB_per_Gauss_XY = 635;
	LSB_per_Gauss_Z = 570;
#elif defined LSM303DLHC
	// DLHC Acceleration register
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x47, CTRL_REG1_A); // 0x47=0b01000111 Normal Mode, ODR 50hz, all axes on
	I2CWrite(LSM303DLH_ACC_ADDR, 0x57, CTRL_REG1_A); // 0x57=0b01010111 Normal Mode, ODR 100hz, all axes on
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x67, CTRL_REG1_A); // 0x67=0b01100111 Normal Mode, ODR 200hz, all axes on
	I2CWrite(LSM303DLH_ACC_ADDR, 0x08, CTRL_REG4_A); // 0x08=0b00001000 Range: +/-2 Gal, Sens.: 1mGal/LSB, highRes on
	mGal_per_LSB = 1.0;
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x18, CTRL_REG4_A); // 0x18=0b00011000 Range: +/-4 Gal, Sens.: 2mGal/LSB, highRes on
//	mGal_per_LSB = 2.0;
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x28, CTRL_REG4_A); // 0x28=0b00101000 Range: +/-8 Gal, Sens.: 4mGal/LSB, highRes on
//	mGal_per_LSB = 4.0;
	// DLHC Magnetic register
	I2CWrite(LSM303DLH_MAG_ADDR, 0x20, CRB_REG_M);   // 0x20=0b00100000 Range: +/-1.3 Gauss gain: 1100LSB/Gauss
	LSB_per_Gauss_XY = 1100;
	LSB_per_Gauss_Z = 980;
//	I2CWrite(LSM303DLH_MAG_ADDR, 0x60, CRB_REG_M);   // 0x60=0b01100000 Range: +/-2.5 Gauss gain: 670LSB/Gauss
//	LSB_per_Gauss_XY = 670;
//	LSB_per_Gauss_Z = 600;
#endif
	I2CWrite(LSM303DLH_MAG_ADDR, 0x00, MR_REG_M);    // Continuous mode

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

	*magX = (float) mx;
	*magY = (float) my;
	*magZ = (float) mz;
#if defined LSM303DLHC
	m.x = mx;
	m.y = my;
	m.z = mz;
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
	*accX = (float) (ax >> 6); // DLH acc resolution is 10 bit
	*accY = (float) (ay >> 6);
	*accZ = (float) (az >> 6);
#elif defined LSM303DLHC
	*accX = (float) (ax >> 4); // DLHC registers contain a left-aligned 12-bit number, so values should be shifted right by 4 bits (divided by 16)
	*accY = (float) (ay >> 4);
	*accZ = (float) (az >> 4);
	a.x = (ax >> 4);
	a.y = (ay >> 4);
	a.z = (az >> 4);
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
#if defined SIMPLE_CALIBRATION
	vector<int16_t> from = {1, 0, 0}; // x axis is reference direction
#endif
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
#if defined SIMPLE_CALIBRATION

	// subtract offset (average of min and max) from magnetometer readings
	m.x -= (int16_t) gConfiguration.xMagOffset;
	m.y -= (int16_t) gConfiguration.yMagOffset;
	m.z -= (int16_t) gConfiguration.zMagOffset;
//Serial.printf("mx %d my %d mz %d\n", m.x, m.y, m.z);

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
#else
	float pitch, roll, Xa_off, Ya_off, Za_off, Xa_cal, Ya_cal, Za_cal, Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal, fXm_comp, fYm_comp;

	// Accelerometer calibration made in mGal
	Xa_off = a.x*mGal_per_LSB + 17.863863; //X-axis combined bias (Non calibrated data - bias)
	Ya_off = a.y*mGal_per_LSB - 31.487140; //Y-axis combined bias (Default: substracting bias)
	Za_off = a.z*mGal_per_LSB + 101.113647; //Z-axis combined bias
	Xa_cal =  0.957747*Xa_off - 0.020159*Ya_off - 0.014851*Za_off; //X-axis correction for combined scale factors (Default: positive factors)
	Ya_cal = -0.020159*Xa_off + 1.000078*Ya_off - 0.005467*Za_off; //Y-axis correction for combined scale factors
	Za_cal = -0.014851*Xa_off - 0.005467*Ya_off + 0.943386*Za_off; //Z-axis correction for combined scale factors

	// Magnetometer calibration made in nT
	Xm_off = m.x*(GAUSS_TO_NANOTESLA/LSB_per_Gauss_XY) + 3349.912409; //X-axis combined bias (Non calibrated data - bias)
	Ym_off = m.y*(GAUSS_TO_NANOTESLA/LSB_per_Gauss_XY) + 7566.200966; //Y-axis combined bias (Default: substracting bias)
	Zm_off = m.z*(GAUSS_TO_NANOTESLA/LSB_per_Gauss_Z) - 916.252655; //Z-axis combined bias
	Xm_cal =  1.023288*Xm_off + 0.006969*Ym_off + 0.012757*Zm_off; //X-axis correction for combined scale factors (Default: positive factors)
	Ym_cal =  0.006969*Xm_off + 0.950484*Ym_off + 0.026035*Zm_off; //Y-axis correction for combined scale factors
	Zm_cal =  0.012757*Xm_off + 0.026035*Ym_off + 0.988869*Zm_off; //Z-axis correction for combined scale factors

	// Low-Pass filter accelerometer
	fXa = Xa_cal * alpha + (fXa * (1.0 - alpha));
	fYa = Ya_cal * alpha + (fYa * (1.0 - alpha));
	fZa = Za_cal * alpha + (fZa * (1.0 - alpha));

	// Low-Pass filter magnetometer
	fXm = Xm_cal * alpha + (fXm * (1.0 - alpha));
	fYm = Ym_cal * alpha + (fYm * (1.0 - alpha));
	fZm = Zm_cal * alpha + (fZm * (1.0 - alpha));

	// Pitch and roll
	roll  = atan2(fYa, sqrt(fXa*fXa + fZa*fZa));
	pitch = atan2(fXa, sqrt(fYa*fYa + fZa*fZa));
//Serial.print("Pitch (X): "); Serial.print(pitch*180.0/M_PI); Serial.print("  ");
//Serial.print("Roll (Y): "); Serial.print(roll*180.0/M_PI); Serial.print("  ");

	// Tilt compensated magnetic sensor measurements
	fXm_comp = fXm*cos(pitch)+fZm*sin(pitch);
	fYm_comp = fXm*sin(roll)*sin(pitch)+fYm*cos(roll)-fZm*sin(roll)*cos(pitch);

	// Arctangent of y/x
	heading = (atan2(fYm_comp,fXm_comp)*180.0)/M_PI;
#endif

#endif
	if (heading < 0)
		heading += 360;
//Serial.print("Heading: "); Serial.println(heading);
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
