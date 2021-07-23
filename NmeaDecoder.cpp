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

#include "BoardConfig.h"
#include "Globals.h"
#include "NmeaDecoder.h"

#include <Arduino.h>
#include <string.h>

/***************************************************************************/
/*                              Constants                                  */
/***************************************************************************/

// Central European Time (Paris, Berlin)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     // Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       // Central European Standard Time
Timezone CE(CEST, CET);

// United Kingdom (London, Belfast)
TimeChangeRule BST = {"BST", Last, Sun, Mar, 1, 60};        // British Summer Time
TimeChangeRule GMT = {"GMT", Last, Sun, Oct, 2, 0};         // Standard Time
Timezone UK(BST, GMT);

// US Eastern Time Zone (New York, Detroit)
TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};  // Eastern Daylight Time = UTC - 4 hours
TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};   // Eastern Standard Time = UTC - 5 hours
Timezone usET(usEDT, usEST);

// US Central Time Zone (Chicago, Houston)
TimeChangeRule usCDT = {"CDT", Second, Sun, Mar, 2, -300};
TimeChangeRule usCST = {"CST", First, Sun, Nov, 2, -360};
Timezone usCT(usCDT, usCST);

// US Mountain Time Zone (Denver, Salt Lake City)
TimeChangeRule usMDT = {"MDT", Second, Sun, Mar, 2, -360};
TimeChangeRule usMST = {"MST", First, Sun, Nov, 2, -420};
Timezone usMT(usMDT, usMST);

// Arizona is US Mountain Time Zone but does not use DST
Timezone usAZ(usMST);

// US Pacific Time Zone (Las Vegas, Los Angeles)
TimeChangeRule usPDT = {"PDT", Second, Sun, Mar, 2, -420};
TimeChangeRule usPST = {"PST", First, Sun, Nov, 2, -480};
Timezone usPT(usPDT, usPST);

// Australia Eastern Time Zone (Sydney, Melbourne)
TimeChangeRule aEDT = {"AEDT", First, Sun, Oct, 2, 660};    // UTC + 11 hours
TimeChangeRule aEST = {"AEST", First, Sun, Apr, 3, 600};    // UTC + 10 hours
Timezone ausET(aEDT, aEST);

// Moscow Standard Time (MSK, does not observe DST)
TimeChangeRule msk = {"MSK", Last, Sun, Mar, 1, 180};
Timezone tzMSK(msk);

static const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};

/***************************************************************************/
/*                                Macros                                   */
/***************************************************************************/

// leap year calculator expects year argument as years offset from 1970
#define LEAP_YEAR(Y) ( ((1970+(Y))>0) && !((1970+(Y))%4) && ( ((1970+(Y))%100) || !((1970+(Y))%400) ) ) 

/***************************************************************************/
/*                             Local types                                 */
/***************************************************************************/

static tmElements_t tm;

/***************************************************************************/
/*                           Local prototypes                              */
/***************************************************************************/

/***************************************************************************/
/*                               Globals                                   */
/***************************************************************************/

/***************************************************************************/
/*                              Functions                                  */
/***************************************************************************/

NmeaDecoder::NmeaDecoder()
{
	writeIndex = 0;
	sentenceWriteIndex = 0;
	serialBuffer[0] = 0;
}

NmeaDecoder::~NmeaDecoder()
{
}

void NmeaDecoder::PushChar(char c, NavigationData *navData)
{
	if ((serialBuffer[0] != '$') || (c == '$'))
	{
		serialBuffer[0] = c;
		writeIndex = 1;
		return;
	}

	if (c == 13)
	{
		if ((writeIndex >= 10) && (sentenceWriteIndex < NMEA_SENTENCE_HISTORY_SIZE))
		{
			memcpy(sentenceBuffer[sentenceWriteIndex], serialBuffer, writeIndex);
			sentenceBuffer[sentenceWriteIndex][writeIndex] = 0;
			DecodeSentence(sentenceWriteIndex, navData);
			sentenceWriteIndex++;

		}
		else
		{
			serialBuffer[0] = 0;
			writeIndex = 0;
			return;
		}
	}

	serialBuffer[writeIndex++] = c;

	if (writeIndex >= NMEA_SENTENCE_MAX_LENGTH)
	{
		serialBuffer[0] = 0;
		writeIndex = 0;
		return;
	}
}

int NmeaDecoder::GetNbSentences()
{
	return sentenceWriteIndex;
}

const char* NmeaDecoder::GetSentence(int i)
{
	return sentenceBuffer[i];
}

void NmeaDecoder::resetSentences()
{
	sentenceWriteIndex = 0;
}

void NmeaDecoder::DecodeSentence(int sentenceIndex, NavigationData *navData)
{
	if (sentenceBuffer[sentenceIndex][0] != '$')
		return;

#if DISABLE_NMEA_CHECKSUM == 0
	char *pCs = strrchr(sentenceBuffer[sentenceIndex], '*') + 1;
	if (pCs == nullptr)
		return;
	int16_t Cs = (NibbleValue(pCs[0]) << 4) | NibbleValue(pCs[1]);
	if (Cs < 0)
		return;

	uint8_t crc = 0;
	for (char *pC = sentenceBuffer[sentenceIndex] + 1; pC < (pCs - 1); pC++)
	{
		crc = crc ^ (*pC);
	}

	if (crc != Cs)
		return;
#endif

	char sId[3];
	sId[0] = sentenceBuffer[sentenceIndex][3];
	sId[1] = sentenceBuffer[sentenceIndex][4];
	sId[2] = sentenceBuffer[sentenceIndex][5];
	char *pField = sentenceBuffer[sentenceIndex] + 7;
	if (strncmp(sId, "RMB", 3) == 0)
	{
		DecodeRMBSentence(pField, navData);
	}
	else if (strncmp(sId, "RMC", 3) == 0)
	{
		DecodeRMCSentence(pField, navData);
	}
	else if (strncmp(sId, "GGA", 3) == 0)
	{
		DecodeGGASentence(pField, navData);
	}
	else if (strncmp(sId, "VTG", 3) == 0)
	{
		DecodeVTGSentence(pField, navData);
	}

	return;
}

void NmeaDecoder::DecodeRMBSentence(char *sentence, NavigationData *navData)
{
	float value;

	if (sentence[0] != 'A')
	{
		return;
	}
	if ((sentence = strchr(sentence, ',')) == nullptr)
		return;
	sentence++;
	if (sscanf(sentence, "%f", &value) == 1)
	{
		navData->xte_nm.value = value;
		navData->xte_nm.valid = true;
		navData->xte_nm.timeStamp = millis();
	}
	if ((sentence = strchr(sentence, ',')) == nullptr)
		return;
	sentence++;
	if (sentence[0] == 'R')
		navData->xte_nm.value = -navData->xte_nm.value;
	for (int i = 0; i < 7; i++)
	{
		if ((sentence = strchr(sentence, ',')) == nullptr)
			return;
		sentence++;
	}
	if (sscanf(sentence, "%f", &value) == 1)
	{
		navData->dtw_nm.value = value;
		navData->dtw_nm.valid = true;
		navData->dtw_nm.timeStamp = millis();
	}
	if ((sentence = strchr(sentence, ',')) == nullptr)
		return;
	sentence++;
	if (sscanf(sentence, "%f", &value) == 1)
	{
		navData->btw_deg.value = value;
		navData->btw_deg.valid = true;
		navData->btw_deg.timeStamp = millis();
	}
	if ((sentence = strchr(sentence, ',')) == nullptr)
		return;
	sentence++;
	if (sscanf(sentence, "%f", &value) == 1)
	{
		navData->vmc_kt.value = value;
		navData->vmc_kt.valid = true;
		navData->vmc_kt.timeStamp = millis();
	}
}

void NmeaDecoder::DecodeRMCSentence(char *sentence, NavigationData *navData)
{
	int hr=0, mint=0, sec=0, dy=1, mnth=1, yr=1970;
	time_t Local_sec=0;

	//Get UTC from NMEA format
	if (sentence[0] != ',')
	{
		hr = (sentence[0] - '0') * 10 + (sentence[1] - '0');
		mint = (sentence[2] - '0') * 10 + (sentence[3] - '0');
		sec = (sentence[4] - '0') * 10 + (sentence[5] - '0');
	}
	for (int i = 0; i < 8; i++)
	{
		if ((sentence = strchr(sentence, ',')) == nullptr)
			return;
		sentence++;
	}
	if (sentence[0] != ',')
	{
		dy = (sentence[0] - '0') * 10 + (sentence[1] - '0');
		mnth = (sentence[2] - '0') * 10 + (sentence[3] - '0');
		yr = (sentence[4] - '0') * 10 + (sentence[5] - '0') + 2000;
	}

	//Calculate UTC in time_t format
	time_t UTC_sec = NmeaDecoder::makeUTC_sec(hr, mint, sec, dy, mnth, yr);

	// given a Timezone object and UTC convert to local time with time zone
	switch(gConfiguration.timezone)
	{
	case 1:
		Local_sec = NmeaDecoder::ConvertTime(CE, UTC_sec);
		break;
	case 2:
		Local_sec = NmeaDecoder::ConvertTime(UK, UTC_sec);
		break;
	case 3:
		Local_sec = NmeaDecoder::ConvertTime(usET, UTC_sec);
		break;
	case 4:
		Local_sec = NmeaDecoder::ConvertTime(usCT, UTC_sec);
		break;
	case 5:
		Local_sec = NmeaDecoder::ConvertTime(usMT, UTC_sec);
		break;
	case 6:
		Local_sec = NmeaDecoder::ConvertTime(usAZ, UTC_sec);
		break;
	case 7:
		Local_sec = NmeaDecoder::ConvertTime(usPT, UTC_sec);
		break;
	case 8:
		Local_sec = NmeaDecoder::ConvertTime(ausET, UTC_sec);
		break;
	case 9:
		Local_sec = NmeaDecoder::ConvertTime(tzMSK, UTC_sec);
		break;
	}

	navData->time.hour = hour(Local_sec); 
	navData->time.minute = minute(Local_sec);
	navData->date.day = day(Local_sec);
	navData->date.month = month(Local_sec); 
	navData->date.year = year(Local_sec) - 2000;
	navData->time.valid = true;
	navData->time.timeStamp = millis();
	navData->date.valid = true;
	navData->date.timeStamp = millis();

}

void NmeaDecoder::DecodeGGASentence(char *sentence, NavigationData *navData)
{
	float degs, mins;

	if ((sentence = strchr(sentence, ',')) == nullptr)
		return;
	sentence++;

	if (sentence[0] != ',')
	{
		degs = (sentence[0] - '0') * 10 + (sentence[1] - '0');
		sscanf(sentence + 2, "%f,", &mins);
		navData->latitude_deg.value = degs + mins / 60.0f;
		if ((sentence = strchr(sentence, ',')) == nullptr)
			return;
		sentence++;
		if (sentence[0] == 'S')
			navData->latitude_deg.value = -navData->latitude_deg.value;
		navData->latitude_deg.valid = true;
		navData->latitude_deg.timeStamp = millis();
	}
	if ((sentence = strchr(sentence, ',')) == nullptr)
		return;
	sentence++;
	if (sentence[0] != ',')
	{
		degs = (sentence[0] - '0') * 100 + (sentence[1] - '0') * 10 + (sentence[2] - '0');
		sscanf(sentence + 3, "%f,", &mins);
		navData->longitude_deg.value = degs + mins / 60.0f;
		if ((sentence = strchr(sentence, ',')) == nullptr)
			return;
		sentence++;
		if (sentence[0] == 'W')
			navData->longitude_deg.value = -navData->longitude_deg.value;
		navData->longitude_deg.valid = true;
		navData->longitude_deg.timeStamp = millis();
	}
}

void NmeaDecoder::DecodeVTGSentence(char *sentence, NavigationData *navData)
{
	float value;

	if (sscanf(sentence, "%f", &value) == 1)
	{
		navData->cog_deg.value = value;
		navData->cog_deg.valid = true;
		navData->cog_deg.timeStamp = millis();
	}
	for (int i = 0; i < 4; i++)
	{
		if ((sentence = strchr(sentence, ',')) == nullptr)
			return;
		sentence++;
	}
	if (sscanf(sentence, "%f", &value) == 1)
	{
		navData->sog_kt.value = value;
		navData->sog_kt.valid = true;
		navData->sog_kt.timeStamp = millis();
	}
}

int16_t NmeaDecoder::NibbleValue(char c)
{
	if ((c >= '0') && (c <= '9'))
	{
		return (c - '0');
	}
	else if ((c >= 'A') && (c <= 'F'))
	{
		return (c - 'A') + 0x0a;
	}
	else if ((c >= 'a') && (c <= 'f'))
	{
		return (c - 'a') + 0x0a;
	}

	return -1;
}

time_t NmeaDecoder::makeUTC_sec(int hr, int mint, int sec, int dy, int mnth, int yr)
{
  int i;
  uint32_t seconds;

 // year can be given as full four digit year or two digts (2010 or 10 for 2010);  
 //it is converted to years since 1970
  if( yr > 99)
      yr = yr - 1970;
  else
      yr += 30;  
  tm.Year = yr;
  tm.Month = mnth;
  tm.Day = dy;
  tm.Hour = hr;
  tm.Minute = mint;
  tm.Second = sec;

// assemble time elements into time_t 
// note year argument is offset from 1970 (see macros in time.h to convert to other formats)
// previous version used full four digit year (or digits since 2000),i.e. 2009 was 2009 or 9
  
  // seconds from 1970 till 1 jan 00:00:00 of the given year
  seconds= tm.Year*(SECS_PER_DAY * 365);
  for (i = 0; i < tm.Year; i++) {
    if (LEAP_YEAR(i)) {
      seconds += SECS_PER_DAY;   // add extra days for leap years
    }
  }
  
  // add days for this year, months start from 1
  for (i = 1; i < tm.Month; i++) {
    if ( (i == 2) && LEAP_YEAR(tm.Year)) { 
      seconds += SECS_PER_DAY * 29;
    } else {
      seconds += SECS_PER_DAY * monthDays[i-1];  //monthDay array starts from 0
    }
  }
  seconds += (tm.Day-1) * SECS_PER_DAY;
  seconds += tm.Hour * SECS_PER_HOUR;
  seconds += tm.Minute * SECS_PER_MIN;
  seconds += tm.Second;
  return (time_t)seconds; 
}

// given a Timezone object and UTC, convert to local time with time zone
time_t NmeaDecoder::ConvertTime(Timezone tz, time_t utc)
{
    TimeChangeRule *tcr; // pointer to the time change rule, use to get the TZ abbrev

    time_t t = tz.toLocal(utc, &tcr);
    return t; 
}
