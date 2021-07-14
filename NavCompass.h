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
#if defined LSM303DLH
#define LSM303DLH_ACC_ADDR  0x18
#elif defined LSM303DLHC
#define LSM303DLH_ACC_ADDR  0x19
#endif

//#define SIMPLE_CALIBRATION
#define GAUSS_TO_NANOTESLA 100000.0F

#define CTRL_REG1_A       0x20
#define CTRL_REG2_A       0x21
#define CTRL_REG3_A       0x22
#define CTRL_REG4_A       0x23
#define CTRL_REG5_A       0x24
#if defined LSM303DLH
#define HP_FILTER_RESET_A 0x25
#elif defined LSM303DLHC
#define CTRL_REG6_A       0x25
#endif
#define REFERENCE_A       0x26
#define STATUS_REG_A      0x27
#define OUT_X_L_A         0x28
#define OUT_X_H_A         0x29
#define OUT_Y_L_A         0x2a
#define OUT_Y_H_A         0x2b
#define OUT_Z_L_A         0x2c
#define OUT_Z_H_A         0x2d
#if defined LSM303DLHC
#define FIFO_CTRL_REG_A   0x2e
#define FIFO_SRC_REG_A    0x2f
#endif
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
#if defined LSM303DLHC
#define TEMP_OUT_H_M      0x01 // this may conflicting with CRB_REG_M
#define TEMP_OUT_L_M      0x40 
#endif

class NavCompass
{
private:
	float heading;
	float magX, magY, magZ;
	float accX, accY, accZ;

public:
	NavCompass();
	virtual ~NavCompass();

	bool Init();
	void GetMagneticField(float *magX, float* magY, float *magZ);
	void GetAcceleration(float *accX, float* accY, float *accZ);
	float GetHeading();
	
	template <typename T> struct vector
	{
	T x, y, z;
	};
	vector<int16_t> a; // accelerometer readings
	vector<int16_t> m; // magnetometer readings

	// vector functions
	template <typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
	template <typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
	static void vector_normalize(vector<float> *a);

private:
	unsigned char I2CRead(uint8_t i2cAddress, uint8_t address);
	void I2CBurstRead(uint8_t i2cAddress, uint8_t address, uint8_t *buffer, uint8_t length);
	void I2CWrite(uint8_t i2cAddress, uint8_t data, uint8_t address);
};

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

#endif /* NAVCOMPASS_H_ */
