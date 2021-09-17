/*
 * CompassDecoder.h
 *
 *  Created on: 13 juin 2021
 *      Author: Ronan
 */

#ifndef NAVCOMPASS_H_
#define NAVCOMPASS_H_

#include <stdint.h>
#define LSM303DLH_MAG_ADDR  0x1E
#define LSM303DLH_ACC_ADDR  0x19

#define GAUSS_TO_MICROTESLA 100.0f
#define GAUSS_TO_NANOTESLA 100000.0f

#define CTRL_REG1_A       0x20
#define CTRL_REG2_A       0x21
#define CTRL_REG3_A       0x22
#define CTRL_REG4_A       0x23
#define CTRL_REG5_A       0x24
#define CTRL_REG6_A       0x25
#define REFERENCE_A       0x26
#define STATUS_REG_A      0x27
#define OUT_X_L_A         0x28
#define OUT_X_H_A         0x29
#define OUT_Y_L_A         0x2a
#define OUT_Y_H_A         0x2b
#define OUT_Z_L_A         0x2c
#define OUT_Z_H_A         0x2d
#define FIFO_CTRL_REG_A   0x2e
#define FIFO_SRC_REG_A    0x2f
#define INT1_CFG_A        0x30
#define INT1_SOURCE_A     0x31
#define INT1_THS_A        0x32
#define INT1_DURATION_A   0x33
#define INT2_CFG_A        0x34
#define INT2_SOURCE_A     0x35
#define INT2_THS_A        0x36
#define INT2_DURATION_A   0x37
#define CRA_REG_M         0x00
#define CRB_REG_M         0x01
#define MR_REG_M          0x02
#define OUT_X_H_M         0x03
#define OUT_X_L_M         0x04
#define OUT_Y_H_M         0x05
#define OUT_Y_L_M         0x06
#define OUT_Z_H_M         0x07
#define OUT_Z_L_M         0x08
#define SR_REG_M          0x09
#define IRA_REG_M         0x0a
#define IRB_REG_M         0x0b
#define IRC_REG_M         0x0c
#define TEMP_OUT_H_M      0x31
#define TEMP_OUT_L_M      0x32 

class NavCompass
{
private:
	float heading;
	float magX, magY, magZ;
	float accX, accY, accZ;
#if defined VECTOR_METHOD || defined MAGNETO_METHOD
	float alpha = 0.15f;
	float fXa = 0.0f, fYa = 0.0f, fZa = 0.0f, fXm = 0.0f, fYm = 0.0f, fZm = 0.0f;
#endif
#ifdef ANGLE_METHOD
	/* roll pitch and yaw angles computed by iecompass */
	int16_t iPhi, iThe, iPsi;
	/* filtered roll pitch and yaw angles computed by iecompass */
	int16_t iLPPhi, iLPThe, iLPPsi;
	/* magnetic field readings corrected for hard iron effects and PCB orientation */
	int16_t iBfx, iBfy, iBfz;
	/* hard iron estimate */
	int16_t iVx, iVy, iVz;
#endif

public:
	NavCompass();
	virtual ~NavCompass();

	bool Init();
	void GetMagneticField(float *magX, float *magY, float *magZ);
	void GetAcceleration(float *accX, float* accY, float *accZ);
	float GetHeading();
#if defined LSM303DLHC
	float GetTemperature();
#endif
	
	template <typename T> struct vector
	{
	  T x, y, z;
	};
	vector<float> a; // accelerometer readings
	vector<float> m; // magnetometer readings

#ifdef VECTOR_METHOD
	// vector functions
	template <typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
	template <typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
	static void vector_normalize(vector<float> *a);
#endif
#ifdef ANGLE_METHOD
	void iecompass(int16_t iBpx, int16_t iBpy, int16_t iBpz, int16_t iGpx, int16_t iGpy, int16_t iGpz);
	int16_t iTrig(int16_t ix, int16_t iy);
	int16_t iHundredAtan2Deg(int16_t iy, int16_t ix);
	int16_t iHundredAtanDeg(int16_t iy, int16_t ix);
	int16_t iDivide(int16_t iy, int16_t ix);
	int16_t iFilter(uint8_t aswitch, int16_t iAng, int16_t iLPAng);
#endif

private:
	unsigned char I2CRead(uint8_t i2cAddress, uint8_t address);
	void I2CBurstRead(uint8_t i2cAddress, uint8_t address, uint8_t *buffer, uint8_t length);
	void I2CWrite(uint8_t i2cAddress, uint8_t data, uint8_t address);
};

#ifdef VECTOR_METHOD
template <typename Ta, typename Tb, typename To> void NavCompass::vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
	out->x = (a->y * b->z) - (a->z * b->y);
	out->y = (a->z * b->x) - (a->x * b->z);
	out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float NavCompass::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
	return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}
#endif

#endif /* NAVCOMPASS_H_ */
