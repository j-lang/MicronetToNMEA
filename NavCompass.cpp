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

static float LSB_per_Gauss_XY = 1100.0f; // Varies with range
static float LSB_per_Gauss_Z = 980.0f;   // Varies with range
static float mGal_per_LSB = 1.0f;        // Varies with range

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

	// DLHC Acceleration register
	I2CWrite(LSM303DLH_ACC_ADDR, 0x47, CTRL_REG1_A); // 0x47=0b01000111 Normal Mode, ODR 50Hz, all axes on
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x57, CTRL_REG1_A); // 0x57=0b01010111 Normal Mode, ODR 100Hz, all axes on
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x67, CTRL_REG1_A); // 0x67=0b01100111 Normal Mode, ODR 200Hz, all axes on

	I2CWrite(LSM303DLH_ACC_ADDR, 0x08, CTRL_REG4_A); // 0x08=0b00001000 Range: +/-2 Gal, Sens.: 1mGal/LSB, highRes on
	mGal_per_LSB = 1.0;
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x18, CTRL_REG4_A); // 0x18=0b00011000 Range: +/-4 Gal, Sens.: 2mGal/LSB, highRes on
//	mGal_per_LSB = 2.0;
//	I2CWrite(LSM303DLH_ACC_ADDR, 0x28, CTRL_REG4_A); // 0x28=0b00101000 Range: +/-8 Gal, Sens.: 4mGal/LSB, highRes on
//	mGal_per_LSB = 4.0;

	// Magnetic register
//	I2CWrite(LSM303DLH_MAG_ADDR, 0x0C, CRA_REG_M); // 0x0C=0b00001100 ODR 7.5Hz
//	I2CWrite(LSM303DLH_MAG_ADDR, 0x8C, CRA_REG_M); // 0x18=0b10001100 ODR 7.5Hz, temperature sensor on
//	I2CWrite(LSM303DLH_MAG_ADDR, 0x10, CRA_REG_M); // 0x10=0b00010000 ODR 15Hz
	I2CWrite(LSM303DLH_MAG_ADDR, 0x90, CRA_REG_M); // 0x10=0b10010000 ODR 15Hz, temperature sensor on
//	I2CWrite(LSM303DLH_MAG_ADDR, 0x14, CRA_REG_M); // 0x14=0b00010100 ODR 30Hz
//	I2CWrite(LSM303DLH_MAG_ADDR, 0x98, CRA_REG_M); // 0x18=0b10011000 ODR 75Hz, temperature sensor on

	I2CWrite(LSM303DLH_MAG_ADDR, 0x20, CRB_REG_M); // 0x20=0b00100000 Range: +/-1.3 Gauss gain: 1100LSB/Gauss
	LSB_per_Gauss_XY = 1100;
	LSB_per_Gauss_Z = 980;
//	I2CWrite(LSM303DLH_MAG_ADDR, 0x60, CRB_REG_M); // 0x60=0b01100000 Range: +/-2.5 Gauss gain: 670LSB/Gauss
//	LSB_per_Gauss_XY = 670;
//	LSB_per_Gauss_Z = 600;

	I2CWrite(LSM303DLH_MAG_ADDR, 0x00, MR_REG_M); // Continuous mode

	return true;
}

void NavCompass::GetMagneticField(float *magX, float *magY, float *magZ)
{
	uint8_t magBuffer[6];
	int16_t mx, my, mz;

	I2CBurstRead(LSM303DLH_MAG_ADDR, OUT_X_H_M, magBuffer, 6);

	mx = (int16_t) ((magBuffer[0] << 8) | magBuffer[1]);
	mz = (int16_t) ((magBuffer[2] << 8) | magBuffer[3]); // stupid change in order for DLHC
	my = (int16_t) ((magBuffer[4] << 8) | magBuffer[5]);

	*magX = (float) mx;
	*magY = (float) my;
	*magZ = (float) mz;
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

	*accX = (float) (ax >> 4); // DLHC registers contain a left-aligned 12-bit number, so values should be shifted right by 4 bits (divided by 16)
	*accY = (float) (ay >> 4);
	*accZ = (float) (az >> 4);
}

float NavCompass::GetHeading()
{
	float mx, my, mz;
	float ax, ay, az;
	// Get Acceleration and Magnetic data from LSM303
	GetAcceleration(&ax, &ay, &az);
//Serial.printf("ax %f ay %f az %f\n", ax, ay, az);
	GetMagneticField(&mx, &my, &mz);
//Serial.printf("mx %f my %f mz %f\n", mx, my, mz);

#if defined VECTOR_METHOD
// Adopted from:
// https://github.com/pololu/lsm303-arduino

	vector<float> from = {1.0f, 0.0f, 0.0f}; // x axis is reference direction
	// subtract offset (average of min and max) from magnetometer readings
	mx -= gConfiguration.xMagOffset;
	my -= gConfiguration.yMagOffset;
	mz -= gConfiguration.zMagOffset;
//Serial.printf("mx %f my %f mz %f\n", mx, my, mz);

#define WITH_LP_FILTER
#ifdef WITH_LP_FILTER
	// Low-Pass filter accelerometer
	fXa = ax * alpha + fXa * (1.0f - alpha);
	fYa = ay * alpha + fYa * (1.0f - alpha);
	fZa = az * alpha + fZa * (1.0f - alpha);
	a.x = fXa;
	a.y = fYa;
	a.z = fZa;
#else
	a.x = ax;
	a.y = ay;
	a.z = az;
#endif
#ifdef WITH_LP_FILTER
	// Low-Pass filter magnetometer
	fXm = mx * alpha + fXm * (1.0f - alpha);
	fYm = my * alpha + fYm * (1.0f - alpha);
	fZm = mz * alpha + fZm * (1.0f - alpha);
	m.x = fXm/LSB_per_Gauss_XY*GAUSS_TO_MICROTESLA;
	m.y = fYm/LSB_per_Gauss_XY*GAUSS_TO_MICROTESLA;
	m.z = fZm/LSB_per_Gauss_Z*GAUSS_TO_MICROTESLA;
//Serial.printf("%d mx %f my %f mz %f\n", millis(), m.x, m.y, m.z);
#else
	m.x = mx/LSB_per_Gauss_XY*GAUSS_TO_MICROTESLA;
	m.y = my/LSB_per_Gauss_XY*GAUSS_TO_MICROTESLA;
	m.z = mz/LSB_per_Gauss_Z*GAUSS_TO_MICROTESLA;
#endif

	vector<float> temp_a = a;
	// normalize
	vector_normalize(&temp_a);
	//vector_normalize(&m);

	// compute E and N
	vector<float> E;
	vector<float> N;
	// D X M = E, cross acceleration vector Down with M (magnetic north + inclination) to produce "East"
	vector_cross(&m, &temp_a, &E);
	vector_normalize(&E);
	// E X D = N, cross "East" with "Down" to produce "North" (parallel to the ground)
	vector_cross(&temp_a, &E, &N);
	vector_normalize(&N);

	// compute heading
	heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180.0f / PI;
#elif defined ANGLE_METHOD
// Adopted from:
// "Implementing a Tilt-Compensated eCompass using Accelerometer and Magnetometer Sensors"
// Freescale AN4248

	int16_t imx, imy, imz;
	int16_t iax, iay, iaz;
	/* tilt-compensated e-Compass code */
	// boost the readings more to the max. range to reduce quantization noise in the mathematical routines
	// we have only 12bit data - so multiply with 8
	// subtract offset (average of min and max) from magnetometer readings
	iVx = (int16_t) 8.0f * gConfiguration.xMagOffset;
	iVy = (int16_t) 8.0f * gConfiguration.yMagOffset;
	iVz = (int16_t) 8.0f * gConfiguration.zMagOffset;
	imx = 8 * (int16_t) mx;
	imy = 8 * (int16_t) my;
	imz = 8 * (int16_t) mz;
	iax = 8 * (int16_t) ax;
	iay = 8 * (int16_t) ay;
	iaz = 8 * (int16_t) az;

	iecompass(imx, imy, imz, iax, iay, iaz);

	iLPThe = iFilter(1, iThe, iLPThe);
	iLPPhi = iFilter(0, iPhi, iLPPhi);
	iLPPsi = iFilter(0, iPsi, iLPPsi);

	heading = (float) -iPsi/100; // seems we haven't NED
//Serial.printf("Pitch (X): %d Roll (Y): %d Heading (Z): %d\n", iLPThe/100, iLPPhi/100, -iLPPsi/100);

#elif defined MAGNETO_METHOD
// Adopted from:
// https://forum.arduino.cc/t/lsm303dlhc-calibration-pitch-roll-and-tilt-compensated-heading/256406

	float pitch, roll, Xa_off, Ya_off, Za_off, Xa_cal, Ya_cal, Za_cal, Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal, fXm_comp, fYm_comp;

	// Accelerometer calibration made in mGal
	Xa_off = ax*mGal_per_LSB + 17.863863f; //X-axis combined bias (Non calibrated data - bias)
	Ya_off = ay*mGal_per_LSB - 31.487140f; //Y-axis combined bias (Default: substracting bias)
	Za_off = az*mGal_per_LSB + 101.113647f; //Z-axis combined bias
	Xa_cal =  0.957747f*Xa_off - 0.020159f*Ya_off - 0.014851f*Za_off; //X-axis correction for combined scale factors (Default: positive factors)
	Ya_cal = -0.020159f*Xa_off + 1.000078f*Ya_off - 0.005467f*Za_off; //Y-axis correction for combined scale factors
	Za_cal = -0.014851f*Xa_off - 0.005467f*Ya_off + 0.943386f*Za_off; //Z-axis correction for combined scale factors

	// Magnetometer calibration made in nT
	Xm_off = mx*(GAUSS_TO_NANOTESLA/LSB_per_Gauss_XY) + 3349.912409f; //X-axis combined bias (Non calibrated data - bias)
	Ym_off = my*(GAUSS_TO_NANOTESLA/LSB_per_Gauss_XY) + 7566.200966f; //Y-axis combined bias (Default: substracting bias)
	Zm_off = mz*(GAUSS_TO_NANOTESLA/LSB_per_Gauss_Z) - 916.252655f; //Z-axis combined bias
	Xm_cal =  1.023288f*Xm_off + 0.006969f*Ym_off + 0.012757f*Zm_off; //X-axis correction for combined scale factors (Default: positive factors)
	Ym_cal =  0.006969f*Xm_off + 0.950484f*Ym_off + 0.026035f*Zm_off; //Y-axis correction for combined scale factors
	Zm_cal =  0.012757f*Xm_off + 0.026035f*Ym_off + 0.988869f*Zm_off; //Z-axis correction for combined scale factors

	// Low-Pass filter accelerometer
	fXa = Xa_cal * alpha + (fXa * (1.0f - alpha));
	fYa = Ya_cal * alpha + (fYa * (1.0f - alpha));
	fZa = Za_cal * alpha + (fZa * (1.0f - alpha));

	// Low-Pass filter magnetometer
	fXm = Xm_cal * alpha + (fXm * (1.0f - alpha));
	fYm = Ym_cal * alpha + (fYm * (1.0f - alpha));
	fZm = Zm_cal * alpha + (fZm * (1.0f - alpha));

	// Pitch and roll
	roll  = atan2(fYa, fZa);
	pitch = atan2(-fXa, sqrt(fYa*fYa + fZa*fZa));
//Serial.printf("Pitch (X): %f Roll (Y): %f\n", pitch*180.0/M_PI, roll*180.0/M_PI);

	// Tilt compensated magnetic sensor measurements
	fXm_comp = fXm*cos(pitch)+fYm*sin(roll)*sin(pitch)+fZm*cos(roll)*sin(pitch);
	fYm_comp = fYm*cos(roll)-fZm*sin(roll);

	// Arctangent of y/x
	heading = (atan2(fYm_comp,fXm_comp)*180.0f)/M_PI;
#endif

	if (heading < 0)
		heading += 360;
//Serial.print("Heading: "); Serial.println(heading);
	return heading;
}

#if defined LSM303DLHC
float NavCompass::GetTemperature()
{
	uint8_t tl = I2CRead(LSM303DLH_MAG_ADDR, TEMP_OUT_L_M);
	uint8_t th = I2CRead(LSM303DLH_MAG_ADDR, TEMP_OUT_H_M);
	int16_t temperature_raw = (int16_t)(th << 8 | tl);
	if (temperature_raw > 32767) // 2's complement
		temperature_raw -= 65536;
	float temperature = 20.0f + (float) (temperature_raw >> 4) / 8.0f;
	return temperature;
}
#endif

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

#ifdef VECTOR_METHOD
void NavCompass::vector_normalize(vector<float> *a)
{
	float mag = sqrt(vector_dot(a, a));
	a->x /= mag;
	a->y /= mag;
	a->z /= mag;
}
#endif

#ifdef ANGLE_METHOD
void NavCompass::iecompass(int16_t iBpx, int16_t iBpy, int16_t iBpz,
int16_t iGpx, int16_t iGpy, int16_t iGpz)
{
	/* stack variables */
	/* iBpx, iBpy, iBpz: the three components of the magnetometer sensor */
	/* iGpx, iGpy, iGpz: the three components of the accelerometer sensor */

	/* local variables */
	int16_t iSin, iCos; /* sine and cosine */

	/* subtract the hard iron offset */
	iBpx -= iVx; /* see Eq 16 */
	iBpy -= iVy; /* see Eq 16 */
	iBpz -= iVz; /* see Eq 16 */
	/* calculate current roll angle Phi */
	iPhi = iHundredAtan2Deg(iGpy, iGpz);/* Eq 13 */
	/* calculate sin and cosine of roll angle Phi */
	iSin = iTrig(iGpy, iGpz); /* Eq 13: sin = opposite / hypotenuse */
	iCos = iTrig(iGpz, iGpy); /* Eq 13: cos = adjacent / hypotenuse */
	/* de-rotate by roll angle Phi */
	iBfy = (int16_t)((iBpy * iCos - iBpz * iSin) >> 15);/* Eq 19 y component */
	iBpz = (int16_t)((iBpy * iSin + iBpz * iCos) >> 15);/* Bpy*sin(Phi)+Bpz*cos(Phi)*/
	iGpz = (int16_t)((iGpy * iSin + iGpz * iCos) >> 15);/* Eq 15 denominator */

	/* calculate current pitch angle Theta */
	iThe = iHundredAtan2Deg((int16_t)-iGpx, iGpz);/* Eq 15 */
	/* restrict pitch angle to range -90 to 90 degrees */
	if (iThe > 9000) iThe = (int16_t) (18000 - iThe);
	if (iThe < -9000) iThe = (int16_t) (-18000 - iThe);
	/* calculate sin and cosine of pitch angle Theta */
	iSin = (int16_t)-iTrig(iGpx, iGpz); /* Eq 15: sin = opposite / hypotenuse */
	iCos = iTrig(iGpz, iGpx);	/* Eq 15: cos = adjacent / hypotenuse */
	/* correct cosine if pitch not in range -90 to 90 degrees */
	if (iCos < 0) iCos = (int16_t)-iCos;
	/* de-rotate by pitch angle Theta */
	iBfx = (int16_t)((iBpx * iCos + iBpz * iSin) >> 15); /* Eq 19: x component */
	iBfz = (int16_t)((-iBpx * iSin + iBpz * iCos) >> 15);/* Eq 19: z component */

	/* calculate current yaw = e-compass angle Psi */
	iPsi = iHundredAtan2Deg((int16_t)-iBfy, iBfx); /* Eq 22 */
}

const uint16_t MINDELTATRIG = 1; /* final step size for iTrig */

/* function to calculate ir = ix / sqrt(ix*ix+iy*iy) using binary division */
int16_t NavCompass::iTrig(int16_t ix, int16_t iy)
{
	uint32_t itmp;  /* scratch */
	uint32_t ixsq;  /* ix * ix */
	int16_t isignx; /* storage for sign of x. algorithm assumes x >= 0 then corrects later */
	uint32_t ihypsq;/* (ix * ix) + (iy * iy) */
	int16_t ir;     /* result = ix / sqrt(ix*ix+iy*iy) range -1, 1 returned as signed int16_t */
	int16_t idelta; /* delta on candidate result dividing each stage by factor of 2 */

	/* stack variables */
	/* ix, iy: signed 16 bit integers representing sensor reading in range -32768 to 32767 */
	/* function returns signed int16_t as signed fraction (ie +32767=0.99997, -32768=-1.0000) */
	/* algorithm solves for ir*ir*(ix*ix+iy*iy)=ix*ix */

	/* correct for pathological case: ix==iy==0 */
	if ((ix == 0) && (iy == 0)) ix = iy = 1;

	/* check for -32768 which is not handled correctly */
	if (ix == -32768) ix = -32767;
	if (iy == -32768) iy = -32767;

	/* store the sign for later use. algorithm assumes x is positive for convenience */
	isignx = 1;
	if (ix < 0)
	{
		ix = (int16_t)-ix;
		isignx = -1;
	}

	/* for convenience in the boosting set iy to be positive as well as ix */
	iy = (int16_t)abs(iy);

	/* to reduce quantization effects, boost ix and iy but keep below maximum signed 16 bit */
	while ((ix < 16384) && (iy < 16384))
	{
		ix = (int16_t)(ix + ix);
		iy = (int16_t)(iy + iy);
	}

	/* calculate ix*ix and the hypotenuse squared */
	ixsq = (uint32_t)(ix * ix);/* ixsq=ix*ix: 0 to 32767^2 = 1073676289 */
	ihypsq = (uint32_t)(ixsq + iy * iy);/* ihypsq=(ix*ix+iy*iy) 0 to 2*32767*32767=2147352578 */

	/* set result r to zero and binary search step to 16384 = 0.5 */
	ir = 0;
	idelta = 16384;/* set as 2^14 = 0.5 */

	/* loop over binary sub-division algorithm */
	do
	{
		/* generate new candidate solution for ir and test if we are too high or too low */
		/* itmp=(ir+delta)^2, range 0 to 32767*32767 = 2^30 = 1073676289 */
		itmp = (uint32_t)((ir + idelta) * (ir + idelta));
		/* itmp=(ir+delta)^2*(ix*ix+iy*iy), range 0 to 2^31 = 2147221516 */
		itmp = (itmp >> 15) * (ihypsq >> 15);
		if (itmp <= ixsq) ir += idelta;
		idelta = (int16_t)(idelta >> 1);/* divide by 2 using right shift one bit */
	} while (idelta >= MINDELTATRIG);/* last loop is performed for idelta=MINDELTATRIG */

	/* correct the sign before returning */
	return (int16_t)(ir * isignx);
}

/* calculates 100*atan2(iy/ix)=100*atan2(iy,ix) in deg for ix, iy in range -32768 to 32767 */
int16_t NavCompass::iHundredAtan2Deg(int16_t iy, int16_t ix)
{
	int16_t iResult;/* angle in degrees times 100 */

	/* check for -32768 which is not handled correctly */
	if (ix == -32768) ix = -32767;
	if (iy == -32768) iy = -32767;

	/* check for quadrants */
	if ((ix >= 0) && (iy >= 0))/* range 0 to 90 degrees */
		iResult = iHundredAtanDeg(iy, ix);
	else if ((ix <= 0) && (iy >= 0))/* range 90 to 180 degrees */
		iResult = (int16_t)(18000 - (int16_t)iHundredAtanDeg(iy, (int16_t)-ix));
	else if ((ix <= 0) && (iy <= 0))/* range -180 to -90 degrees */
		iResult = (int16_t)((int16_t)-18000 + iHundredAtanDeg((int16_t)-iy, (int16_t)-ix));
	else/* ix >=0 and iy <= 0 giving range -90 to 0 degrees */
		iResult = (int16_t)(-iHundredAtanDeg((int16_t)-iy, ix));
	return (iResult);
}

/* fifth order of polynomial approximation giving 0.05 deg max error */
const int16_t K1 = 5701;
const int16_t K2 = -1645;
const int16_t K3 = 446;

/* calculates 100*atan(iy/ix) range 0 to 9000 for all ix, iy positive in range 0 to 32767 */
int16_t NavCompass::iHundredAtanDeg(int16_t iy, int16_t ix)
{
	int32_t iAngle;/* angle in degrees times 100 */
	int16_t iRatio;/* ratio of iy / ix or vice versa */
	int32_t iTmp;/* temporary variable */

	/* check for pathological cases */
	if ((ix == 0) && (iy == 0)) return (0);
	if ((ix == 0) && (iy != 0)) return (9000);

	/* check for non-pathological cases */
	if (iy <= ix)
		iRatio = iDivide(iy, ix); /* return a fraction in range 0. to 32767 = 0. to 1. */
	else
		iRatio = iDivide(ix, iy); /* return a fraction in range 0. to 32767 = 0. to 1. */

	/* first, third and fifth order polynomial approximation */
	iAngle = (int32_t) K1 * (int32_t) iRatio;
	iTmp = ((int32_t) iRatio >> 5) * ((int32_t) iRatio >> 5) * ((int32_t) iRatio >> 5);
	iAngle += (iTmp >> 15) * (int32_t) K2;
	iTmp = (iTmp >> 20) * ((int32_t) iRatio >> 5) * ((int32_t) iRatio >> 5);
	iAngle += (iTmp >> 15) * (int32_t) K3;
	iAngle = iAngle >> 15;

	/* check if above 45 degrees */
	if (iy > ix) iAngle = (int16_t)(9000 - iAngle);

	/* for tidiness, limit result to range 0 to 9000 equals 0.0 to 90.0 degrees */
	if (iAngle < 0) iAngle = 0;
	if (iAngle > 9000) iAngle = 9000;

	return ((int16_t) iAngle);
}

const uint16_t MINDELTADIV = 1; /* final step size for iDivide */

/* function to calculate ir = iy / ix with iy <= ix, and ix, iy both > 0 */
int16_t NavCompass::iDivide(int16_t iy, int16_t ix)
{
	int16_t itmp;/* scratch */
	int16_t ir;/* result = iy / ix range 0., 1. returned in range 0 to 32767 */
	int16_t idelta;/* delta on candidate result dividing each stage by factor of 2 */

	/* set result r to zero and binary search step to 16384 = 0.5 */
	ir = 0;
	idelta = 16384;/* set as 2^14 = 0.5 */

	/* to reduce quantization effects, boost ix and iy to the maximum signed 16 bit value */
	while ((ix < 16384) && (iy < 16384))
	{
		ix = (int16_t)(ix + ix);
		iy = (int16_t)(iy + iy);
	}

	/* loop over binary sub-division algorithm solving for ir*ix = iy */
	do
	{
		/* generate new candidate solution for ir and test if we are too high or too low */
		itmp = (int16_t)(ir + idelta);/* itmp=ir+delta, the candidate solution */
		itmp = (int16_t)((itmp * ix) >> 15);
		if (itmp <= iy) ir += idelta;
		idelta = (int16_t)(idelta >> 1);/* divide by 2 using right shift one bit */
	} while (idelta >= MINDELTADIV);/* last loop is performed for idelta=MINDELTADIV */

	return (ir);
}


int16_t NavCompass::iFilter(uint8_t aswitch, int16_t iAng, int16_t iLPAng)
{
	/* Exponential filtering for yaw iAng (roll iPhi - same code) : */
	// iAng - unfiltered input angle */
	// iLPAng - low pass filtered angle*100 deg: range -18000 to 18000 */
	int32_t tmpAngle; /* temporary angle*100 deg: range -36000 to 36000 */
	static uint16_t ANGLE_LPF = 3276; /* low pass filter: set to 32768 / N for N samples averaging */

	/* implement a modulo arithmetic exponential low pass filter on the yaw angle */
	/* compute the change in angle modulo 360 degrees */
	tmpAngle = (int32_t)iAng - (int32_t)iLPAng;
	if (tmpAngle > 18000) tmpAngle -= 36000;
	if (tmpAngle < -18000) tmpAngle += 36000;
	/* calculate the new low pass filtered angle */
	tmpAngle = (int32_t)iLPAng + ((ANGLE_LPF * tmpAngle) >> 15);
	/* check that the angle remains in -180 to 180 deg bounds */
	if (aswitch == 0) {
		if (tmpAngle > 18000) tmpAngle -= 36000;
		if (tmpAngle < -18000) tmpAngle += 36000;
	} else {
	/* For the pitch angle θ , which is restricted to the range -90° to 90°,
	the final bounds check should be changed to: */
		if (tmpAngle > 9000) tmpAngle = (int16_t) (18000 - tmpAngle);
		if (tmpAngle < -9000) tmpAngle = (int16_t) (-18000 - tmpAngle);
	}
	/* store the correctly bounded low pass filtered angle */
	iLPAng = (int16_t)tmpAngle;
	return (iLPAng);
}

#endif
