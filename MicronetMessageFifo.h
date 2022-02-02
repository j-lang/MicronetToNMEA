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

#ifndef MICRONETMESSAGEFIFO_H_
#define MICRONETMESSAGEFIFO_H_

/***************************************************************************/
/*                              Includes                                   */
/***************************************************************************/

#include <Arduino.h>
#include <stdint.h>
#include "Micronet.h"
#include "BoardConfig.h"

/***************************************************************************/
/*                              Constants                                  */
/***************************************************************************/

#define MESSAGE_STORE_SIZE 16

/***************************************************************************/
/*                                Types                                    */
/***************************************************************************/

class MicronetMessageFifo
{
public:
	MicronetMessageFifo();
	virtual ~MicronetMessageFifo();

	bool SRAM_USE Push(MicronetMessage_t &message);
	bool SRAM_USE Pop(MicronetMessage_t *message);
	MicronetMessage_t SRAM_USE *Peek();
	void SRAM_USE DeleteMessage();
	void SRAM_USE ResetFifo();

private:
	int writeIndex;
	int readIndex;
	int nbMessages;
	MicronetMessage_t store[MESSAGE_STORE_SIZE];
};

/***************************************************************************/
/*                              Prototypes                                 */
/***************************************************************************/

#endif /* MICRONETMESSAGEFIFO_H_ */
