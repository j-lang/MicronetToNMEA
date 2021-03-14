/*
 * PacketDecoder.cpp
 *
 *  Created on: 12 mars 2021
 *      Author: Ronan
 */

#include "MicronetDecoder.h"

#include <arduino.h>
#include <string.h>

MicronetDecoder::MicronetDecoder()
{
	memset(&micronetData, 0, sizeof(micronetData));
}

MicronetDecoder::~MicronetDecoder()
{
}

uint32_t MicronetDecoder::GetNetworkId(MicronetMessage_t *message)
{
	unsigned int networkId;

	networkId = message->data[MICRONET_NUID_OFFSET];
	networkId = (networkId << 8) | message->data[MICRONET_NUID_OFFSET + 1];
	networkId = (networkId << 8) | message->data[MICRONET_NUID_OFFSET + 2];
	networkId = (networkId << 8) | message->data[MICRONET_NUID_OFFSET + 3];

	return networkId;
}

uint8_t MicronetDecoder::GetDeviceType(MicronetMessage_t *message)
{
	return message->data[MICRONET_DT_OFFSET];
}

uint32_t MicronetDecoder::GetDeviceId(MicronetMessage_t *message)
{
	unsigned int deviceId;

	deviceId = message->data[MICRONET_DUID_OFFSET];
	deviceId = (deviceId << 8) | message->data[MICRONET_DUID_OFFSET + 1];
	deviceId = (deviceId << 8) | message->data[MICRONET_DUID_OFFSET + 2];
	deviceId = (deviceId << 8) | message->data[MICRONET_DUID_OFFSET + 3];

	return deviceId;
}

uint8_t MicronetDecoder::GetMessageId(MicronetMessage_t *message)
{
	return message->data[MICRONET_MI_OFFSET];
}

uint8_t MicronetDecoder::GetSource(MicronetMessage_t *message)
{
	return message->data[MICRONET_SO_OFFSET];
}

uint8_t MicronetDecoder::GetDestination(MicronetMessage_t *message)
{
	return message->data[MICRONET_DE_OFFSET];
}

uint8_t MicronetDecoder::GetHeaderCrc(MicronetMessage_t *message)
{
	return message->data[MICRONET_CRC_OFFSET];
}

bool MicronetDecoder::VerifyHeaderCrc(MicronetMessage_t *message)
{
	uint8_t crc = 0;
	for (int i = 0; i < MICRONET_CRC_OFFSET; i++)
	{
		crc += message->data[i];
	}

	return (crc == message->data[MICRONET_CRC_OFFSET]);
}

void MicronetDecoder::DecodeMessage(MicronetMessage_t *message)
{
	if (message->data[MICRONET_MI_OFFSET] == MICRONET_MESSAGE_ID_SEND_DATA)
	{
		int fieldOffset = MICRONET_PAYLOAD_OFFSET;
		while (fieldOffset < message->len)
		{
			fieldOffset = DecodeDataField(message, fieldOffset);
			if (fieldOffset < 0)
			{
				break;
			}
		}
	}
}

int MicronetDecoder::DecodeDataField(MicronetMessage_t *message, int offset)
{
	int8_t value8;
	int16_t value16;
	int32_t value_32_1, value32_2;

	if (message->data[offset] == MICRONET_FIELD_TYPE_3)
	{
		uint8_t crc = message->data[offset] + message->data[offset + 1] + message->data[offset + 2] + message->data[offset + 3];
		if (crc == message->data[offset + 4])
		{
			value8 = message->data[offset + 3];
			UpdateMicronetData(message->data[offset + 1], value8);
		}
	}
	else if (message->data[offset] == MICRONET_FIELD_TYPE_4)
	{
		uint8_t crc = message->data[offset] + message->data[offset + 1] + message->data[offset + 2] + message->data[offset + 3]
				+ message->data[offset + 4];
		if (crc == message->data[offset + 5])
		{
			value16 = message->data[offset + 3];
			value16 = (value16 << 8) | message->data[offset + 4];
			UpdateMicronetData(message->data[offset + 1], value16);
		}
	}
	else if (message->data[offset] == MICRONET_FIELD_TYPE_5)
	{
		uint8_t crc = message->data[offset] + message->data[offset + 1] + message->data[offset + 2] + message->data[offset + 3]
				+ message->data[offset + 4] + message->data[offset + 5];
		if (crc == message->data[offset + 6])
		{
			value16 = message->data[offset + 3];
			value16 = (value16 << 8) | message->data[offset + 4];
			UpdateMicronetData(message->data[offset + 1], value16);
		}
	}
	else if (message->data[offset] == MICRONET_FIELD_TYPE_A)
	{
		uint8_t crc = message->data[offset] + message->data[offset + 1] + message->data[offset + 2] + message->data[offset + 3]
				+ message->data[offset + 4] + message->data[offset + 5] + message->data[offset + 6] + message->data[offset + 7]
				+ message->data[offset + 8] + message->data[offset + 9] + message->data[offset + 10];
		if (crc == message->data[offset + 11])
		{
			value_32_1 = message->data[offset + 3];
			value_32_1 = (value_32_1 << 8) | message->data[offset + 4];
			value_32_1 = (value_32_1 << 8) | message->data[offset + 5];
			value_32_1 = (value_32_1 << 8) | message->data[offset + 6];
			value32_2 = message->data[offset + 7];
			value32_2 = (value32_2 << 8) | message->data[offset + 8];
			value32_2 = (value32_2 << 8) | message->data[offset + 9];
			value32_2 = (value32_2 << 8) | message->data[offset + 10];
			UpdateMicronetData(message->data[offset + 1], value_32_1, value32_2);
		}
	}

	return offset + message->data[offset] + 2;
}

void MicronetDecoder::UpdateMicronetData(uint8_t fieldId, int8_t value)
{
	switch (fieldId)
	{
	case MICRONET_FIELD_ID_STP:
		micronetData.stp.value = ((float) value) / 2.0f;
		micronetData.stp.valid = true;
		micronetData.stp.updated = true;
		micronetData.stp.timeStamp = millis();
		break;
	}
}

void MicronetDecoder::UpdateMicronetData(uint8_t fieldId, int16_t value)
{
	switch (fieldId)
	{
	case MICRONET_FIELD_ID_STW:
		micronetData.stw.value = ((float) value) / 100.0f;
		micronetData.stw.valid = true;
		micronetData.stw.updated = true;
		micronetData.stw.timeStamp = millis();
		break;
	case MICRONET_FIELD_ID_DPT:
		if (value < 5000)
		{
			micronetData.dpt.value = ((float) value) * 0.3048f / 10.0f;
			micronetData.dpt.valid = true;
			micronetData.dpt.updated = true;
			micronetData.dpt.timeStamp = millis();
		}
		else
		{
			micronetData.dpt.valid = false;
			micronetData.dpt.updated = true;
		}
		break;
	case MICRONET_FIELD_ID_AWS:
		micronetData.aws.value = ((float) value) / 10.0f;
		micronetData.aws.valid = true;
		micronetData.aws.updated = true;
		micronetData.aws.timeStamp = millis();
		break;
	case MICRONET_FIELD_ID_AWA:
		micronetData.awa.value = (float) value;
		micronetData.awa.valid = true;
		micronetData.awa.updated = true;
		micronetData.awa.timeStamp = millis();
		break;
	case MICRONET_FIELD_ID_VCC:
		micronetData.vcc.value = ((float) value) / 10.0f;
		micronetData.vcc.valid = true;
		micronetData.vcc.updated = true;
		micronetData.vcc.timeStamp = millis();
		break;
	}
}

void MicronetDecoder::UpdateMicronetData(uint8_t fieldId, int32_t value1, int32_t value2)
{
	switch (fieldId)
	{
	case MICRONET_FIELD_ID_LOG:
		micronetData.trip.value = ((float) value1) / 100.0f;
		micronetData.trip.valid = true;
		micronetData.trip.updated = true;
		micronetData.trip.timeStamp = millis();
		micronetData.log.value = ((float) value2) / 10.0f;
		micronetData.log.valid = true;
		micronetData.log.updated = true;
		micronetData.log.timeStamp = millis();
		break;
	}
}

MicronetData_t* MicronetDecoder::GetCurrentData()
{
	return &micronetData;
}