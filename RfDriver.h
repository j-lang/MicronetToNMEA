/*
 * RfDriver.h
 *
 *  Created on: 18 sept. 2021
 *      Author: Ronan
 */

#ifndef RFDRIVER_H_
#define RFDRIVER_H_

#include "BoardConfig.h"
#include "Micronet.h"
#include "MicronetMessageFifo.h"
#include "ELECHOUSE_CC1101_SRC_DRV.h"

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
	void IRAM_ATTR GDO0Callback();
	void IRAM_ATTR RestartReception();
	void IRAM_ATTR TransmitMessage(MicronetMessage_t *message, uint32_t transmitTimeUs);

private:
	int gdo0Pin;
	ELECHOUSE_CC1101 cc1101Driver;
	MicronetMessageFifo *messageFifo;
	RfDriverState_t rfState;
	MicronetMessage_t messageToTransmit;
	int messageBytesSent;
	float frequencyOffset_mHz;

	void IRAM_ATTR GDO0RxCallback();
	void IRAM_ATTR GDO0TxCallback();
	void IRAM_ATTR GDO0LastTxCallback();
	void IRAM_ATTR TransmitCallback();
	static void IRAM_ATTR TimerHandler(void *);
	static RfDriver *rfDriver;
};

#endif /* RFDRIVER_H_ */
