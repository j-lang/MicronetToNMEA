/***************************************************************************
 *                                                                         *
 * Project:  MicronetToNMEA                                                *
 * Purpose:  Decode data from Micronet devices send it on an NMEA network  *
 * Author:   Ronan Demoment                                                *
 *                                                                         *
 ***************************************************************************
 *   Copyright (C) 2021 by Ronan Demoment                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************
 */

/***************************************************************************/
/*                              Includes                                   */
/***************************************************************************/

#include "MicronetSlaveDevice.h"
#include "Globals.h"

/***************************************************************************/
/*                              Constants                                  */
/***************************************************************************/

/***************************************************************************/
/*                             Local types                                 */
/***************************************************************************/

/***************************************************************************/
/*                           Local prototypes                              */
/***************************************************************************/

/***************************************************************************/
/*                               Globals                                   */
/***************************************************************************/

/***************************************************************************/
/*                              Functions                                  */
/***************************************************************************/

MicronetSlaveDevice::MicronetSlaveDevice() :
		deviceId(0), dataFields(0), networkId(0)
{
}

MicronetSlaveDevice::~MicronetSlaveDevice()
{
}

void MicronetSlaveDevice::SetDeviceId(uint32_t deviceId)
{
	this->deviceId = deviceId;
}

void MicronetSlaveDevice::SetNetworkId(uint32_t networkId)
{
	this->networkId = networkId;
}

void MicronetSlaveDevice::SetDataFields(uint32_t dataFields)
{
	this->dataFields = dataFields;
}

void MicronetSlaveDevice::ProcessMessage(MicronetMessage_t *message)
{
	TxSlotDesc_t txSlot;
	uint32_t payloadLength;
	MicronetMessage_t txMessage;

	if ((micronetCodec.GetNetworkId(message) == networkId) && (micronetCodec.VerifyHeaderCrc(message)))
	{
		if (micronetCodec.GetMessageId(message) == MICRONET_MESSAGE_ID_REQUEST_DATA)
		{
			txSlot = micronetCodec.GetSyncTransmissionSlot(message, deviceId);
			if (txSlot.start_us != 0)
			{
				payloadLength = micronetCodec.EncodeDataMessage(&txMessage, networkId, deviceId, &gNavData,
						dataFields);
				if (txSlot.payloadBytes < payloadLength)
				{
					txSlot = micronetCodec.GetAsyncTransmissionSlot(message);
					micronetCodec.EncodeSlotUpdateMessage(&txMessage, networkId, deviceId, payloadLength);
				}
			}
			else
			{
				txSlot = micronetCodec.GetAsyncTransmissionSlot(message);
				micronetCodec.EncodeSlotRequestMessage(&txMessage, networkId, deviceId, micronetCodec.GetDataMessageLength(dataFields));
			}

			gRfReceiver.Transmit(&txMessage, txSlot.start_us);
		}
	}
}