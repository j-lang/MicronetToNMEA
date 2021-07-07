/*
 * CompassDecoder.h
 *
 *  Created on: 13 juin 2021
 *      Author: Ronan
 */

#ifndef NAVCOMPASS_H_
#define NAVCOMPASS_H_

#include <stdint.h>

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
