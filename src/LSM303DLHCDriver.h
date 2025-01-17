/*
 * LSM303DLHDriver.h
 *
 *  Created on: 11 sept. 2021
 *      Author: Ronan
 */

#ifndef LSM303DLHCDRIVER_H_
#define LSM303DLHCDRIVER_H_

#include "NavCompassDriver.h"

using string = std::string;

class LSM303DLHCDriver: public NavCompassDriver
{
public:
	LSM303DLHCDriver();
	virtual ~LSM303DLHCDriver();

	virtual bool Init() override;
	virtual string GetDeviceName() override;
	virtual void GetMagneticField(float *magX, float *magY, float *magZ) override;
	virtual void GetAcceleration(float *accX, float *accY, float *accZ) override;

private:
	uint8_t accAddr, magAddr;
	float magX, magY, magZ;
	float accX, accY, accZ;
	float LSB_per_Gauss_XY;
	float LSB_per_Gauss_Z;
	float mGal_per_LSB;

	bool I2CRead(uint8_t i2cAddress, uint8_t address, uint8_t *data);
	bool I2CBurstRead(uint8_t i2cAddress, uint8_t address, uint8_t *buffer, uint8_t length);
	bool I2CWrite(uint8_t i2cAddress, uint8_t data, uint8_t address);
};

#endif /* LSM303DLHDRIVER_H_ */
