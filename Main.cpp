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

#include "CC1101Driver.h"
#include "PacketStore.h"
#include "MenuManager.h"

#include "Arduino.h"
#include <SPI.h>
#include <wiring.h>
#include <iostream>

/***************************************************************************/
/*                              Constants                                  */
/***************************************************************************/

#define GDO0_PIN 24
#define GDO2_PIN 25
#define LED_PIN LED_BUILTIN

/***************************************************************************/
/*                             Local types                                 */
/***************************************************************************/

/***************************************************************************/
/*                               Globals                                   */
/***************************************************************************/

CC1101Driver cc1101;     // CC1101 Driver object
MenuManager menu;        // Menu manager object
PacketStore packetStore; // Micronet packet store, used for communication between CC1101 interrupt and main loop code

/***************************************************************************/
/*                           Local prototypes                              */
/***************************************************************************/

void SyncWordDetectedISR();

/***************************************************************************/
/*                              Functions                                  */
/***************************************************************************/

void setup()
{
	Serial.begin(115200);

	// Print banner
	Serial.println("");
	Serial.println("----------------------------");
	Serial.println("--- MicronetToNMEA v0.1a ---");
	Serial.println("----------------------------");
	Serial.println("");

	// Set SPI pin configuration
	SPI.setMOSI(11);
	SPI.setMISO(12);
	SPI.setSCK(14);
	SPI.begin();

	// Check connection to CC1101
	Serial.print("Connecting to CC1101 Transciever ... ");
	if (cc1101.getCC1101())
	{
		Serial.println("OK");
	}
	else
	{
		Serial.println("Failed");
		Serial.println("Aborting execution : Verify connection to CC1101 board");
		Serial.println("Halted");

		while (1)
		{
			digitalWrite(LED_PIN, HIGH);
			delay(500);
			digitalWrite(LED_PIN, LOW);
			delay(500);
		}
	}

	// Configure CC1101 for listening Micronet devices
	cc1101.Init(); // must be set to initialize the cc1101!
	cc1101.setGDO(GDO0_PIN, GDO2_PIN); // set lib internal gdo pins (gdo0,gdo2). Gdo2 not use for this example.
	cc1101.setCCMode(1); // set config for internal transmission mode.
	cc1101.setModulation(0); // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
	cc1101.setMHZ(869.845); // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
	cc1101.setDeviation(32); // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
	cc1101.setChannel(0); // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
	cc1101.setChsp(199.95); // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
	cc1101.setRxBW(250); // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
	cc1101.setDRate(76.8); // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
	cc1101.setPA(0); // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
	cc1101.setSyncMode(2); // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
	cc1101.setSyncWord(0xb3, 0x20); // Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low)
	cc1101.setAdrChk(0); // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
	cc1101.setAddr(0); // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
	cc1101.setWhiteData(0); // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
	cc1101.setPktFormat(0); // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
	cc1101.setLengthConfig(0); // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved
	cc1101.setPacketLength(30); // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
	cc1101.setCrc(0); // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
	cc1101.setCRC_AF(0); // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
	cc1101.setDcFilterOff(0); // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
	cc1101.setManchester(0); // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
	cc1101.setFEC(0); // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.
	cc1101.setPQT(0); // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
	cc1101.setAppendStatus(0); // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.

	// Attach callback to GDO0 pin
	// According to CC1101 configuration this callback will be executed when CC1101 will have detected Micronet's sync word
	attachInterrupt(digitalPinToInterrupt(GDO0_PIN), SyncWordDetectedISR, RISING);

	// Start listening
	cc1101.SetRx();
}

void loop()
{
	// Process console input
	while (Serial.available() > 0)
	{
		menu.PushChar(Serial.read());
	}

	// If we have packets in the store, process them
	MicronetPacket_t *packet;
	while ((packet = packetStore.PeekPacket()) != nullptr)
	{
		for (int j = 0; j < packet->len; j++)
		{
			if (packet->data[j] < 16)
			{
				Serial.print("0");
			}
			Serial.print(packet->data[j], HEX);
			Serial.print(" ");
		}
		Serial.print(" (");
		Serial.print((int)packet->len);
		Serial.print(",");
		Serial.print((int)packet->rssi);
		Serial.print(",");
		Serial.print((int)packet->lqi);
		Serial.print(")");

		Serial.println();
		packetStore.Deletepacket();
	}
}

void SyncWordDetectedISR()
{
	MicronetPacket_t packet;

	// We know that CC1101 has detected Micronet's sync word. We however have to wait for packet reception to be finished.
	// CC11001 will signal end of packet by setting GDO0 back to zero. So we wait for GDO0 to switch to zero.
	while (digitalRead(GDO0_PIN))
	{
	}

	// Now we collect all data from CC1101 and store it into the "packet" structure
	packet.rssi = cc1101.getRssi();               // Rssi Level in dBm
	packet.lqi = cc1101.getLqi();                 // Link Quality Indicator
	packet.len = cc1101.ReceiveData(packet.data); // Get FIFO Data and packet length
	bool overflow = ((packet.len & 0x80) != 0);   // FIFO overflow flag
	packet.len &= ~0xc0;                          // Remove flags from packet length

	if (overflow || (packet.len == 0))
	{
		// ReceiveData automatically flushes FIFO after reading, so no need to do that here
		// However, we consider data as invalid and ignore the packet
		return;
	}

	// Add packet to the store
	packetStore.AddPacket(packet);
}
