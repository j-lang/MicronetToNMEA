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

#ifndef BOARDCONFIG_H_
#define BOARDCONFIG_H_

/***************************************************************************/
/*                              Includes                                   */
/***************************************************************************/

/***************************************************************************/
/*                              Constants                                  */
/***************************************************************************/

// Selects on which I2C bus is connected compass as per Wiring library definition
#define NAVCOMPASS_I2C Wire // SDA0: 18 SCL0: 19
#ifdef ESP32
#define I2C_SDA       21 // default pins
#define I2C_SCL       22
#endif

// CC1101/SPI pins
#ifdef TEENSYDUINO
#define CS0_PIN       10
#define MOSI_PIN      11
#define MISO_PIN      12
#define SCK_PIN       14
#define GDO0_PIN      24
#elif ESP32
#define CS0_PIN        5 //default pins
#define MOSI_PIN      23
#define MISO_PIN      19
#define SCK_PIN       18
#define GDO0_PIN       2
#endif

// ERROR LED pin
#ifdef TEENSYDUINO
#define LED_PIN       LED_BUILTIN
#elif ESP32
#define LED_PIN        2
#endif

// NMEA/GNSS UART pins
#ifdef TEENSYDUINO
#define GNSS_SERIAL   Serial1
#define GNSS_BAUDRATE 38400
#define GNSS_CALLBACK serialEvent1
#define GNSS_RX_PIN   0
#define GNSS_TX_PIN   1
#elif ESP32
#define GNSS_SERIAL   Serial2
#define GNSS_BAUDRATE 38400
#define GNSS_CALLBACK serialEvent2
#define GNSS_RX_PIN   16
#define GNSS_TX_PIN   17
#endif

// USB UART params
#define USB_CONSOLE  Serial
#define USB_BAUDRATE 115200

// Wired UART params
#ifdef TEENSYDUINO
#define WIRED_SERIAL   Serial5
#define WIRED_BAUDRATE 115200
#define WIRED_RX_PIN   34
#define WIRED_TX_PIN   33
#elif ESP32
#define WIRED_SERIAL   Serial1
#define WIRED_BAUDRATE 115200
#define WIRED_RX_PIN   12
#define WIRED_TX_PIN   13
#endif

// The console to use for menu and NMEA output
#define CONSOLE USB_CONSOLE
#define NMEA_OUT USB_CONSOLE
#define NMEA_IN USB_CONSOLE

/***************************************************************************/
/*                                Types                                    */
/***************************************************************************/

/***************************************************************************/
/*                              Prototypes                                 */
/***************************************************************************/

#endif /* BOARDCONFIG_H_ */
