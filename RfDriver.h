/*
 * RfDriver.h
 *
 *  Created on: 18 sept. 2021
 *      Author: Ronan
 */

#ifndef RFDRIVER_H_
#define RFDRIVER_H_

#include "Micronet.h"
#include "MicronetMessageFifo.h"
#include "ELECHOUSE_CC1101_SRC_DRV.h"

// ISR's should work from SRAM in ESP32
#ifdef ESP32
#define SRAM_USE IRAM_ATTR
#elif TEENSYDUINO
#define SRAM_USE
#endif

typedef enum {
	RF_STATE_RX_IDLE = 0,
	RF_STATE_RX_RECEIVING,
	RF_STATE_TX_TRANSMITTING,
	RF_STATE_TX_LAST_TRANSMIT
} RfDriverState_t;

class RfDriver
{
public:
	RfDriver();
	virtual ~RfDriver();

	bool Init(int gdo0_pin, MicronetMessageFifo *messageFifo, float frequencyOffset_mHz);
	void SetFrequencyOffset(float offsetMHz);
	void SetFrequency(float freqMHz);
	void SetDeviation(float freqKHz);
	void SetBandwidth(float bwKHz);
	void GDO0Callback();
	void RestartReception();
	void TransmitMessage(MicronetMessage_t *message, uint32_t transmitTimeUs);

private:
	int gdo0Pin;
	ELECHOUSE_CC1101 cc1101Driver;
	MicronetMessageFifo *messageFifo;
	RfDriverState_t rfState;
	MicronetMessage_t messageToTransmit;
	int messageBytesSent;
	float frequencyOffset_mHz;

	void GDO0RxCallback();
	void GDO0TxCallback();
	void GDO0LastTxCallback();
	void TransmitCallback();
#ifdef TEENSYDUINO
	static void TimerHandler();
#elif ESP32
  static void SRAM_USE TimerHandler(void *);
#endif
	static RfDriver *rfDriver;
};

#endif /* RFDRIVER_H_ */
