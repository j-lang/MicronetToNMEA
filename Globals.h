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

#ifndef GNSSDECODER_H_
#define GNSSDECODER_H_

/***************************************************************************/
/*                              Includes                                   */
/***************************************************************************/

#include "MenuManager.h"
#include "MicronetMessageFifo.h"
#include "MicronetCodec.h"
#include "Configuration.h"
#include "NmeaEncoder.h"
#include "NmeaDecoder.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "NavigationData.h"

/***************************************************************************/
/*                              Constants                                  */
/***************************************************************************/

/***************************************************************************/
/*                                Types                                    */
/***************************************************************************/

/***************************************************************************/
/*                               Globals                                   */
/***************************************************************************/

extern ELECHOUSE_CC1101 gRfReceiver;
extern MenuManager gMenuManager;
extern MicronetMessageFifo gRxMessageFifo;
extern MicronetCodec gMicronetCodec;
extern Configuration gConfiguration;
extern NmeaEncoder gNmeaEncoder;
extern NmeaDecoder gGnssDecoder;
extern NmeaDecoder gNavDecoder;
extern NavigationData gNavData;

/***************************************************************************/
/*                              Prototypes                                 */
/***************************************************************************/

#endif /* GNSSDECODER_H_ */
