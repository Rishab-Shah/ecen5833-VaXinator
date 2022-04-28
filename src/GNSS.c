/*
 * GNSS.c
 *
 *  Created on: 03.10.2020
 *      Author: SimpleMethod
 *
 *Copyright 2020 SimpleMethod
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of
 *this software and associated documentation files (the "Software"), to deal in
 *the Software without restriction, including without limitation the rights to
 *use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *of the Software, and to permit persons to whom the Software is furnished to do
 *so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *THE SOFTWARE.
 ******************************************************************************
 */

#include "GNSS.h"

union u_Short uShort;
union i_Short iShort;
union u_Long uLong;
union i_Long iLong;

GNSS_StateHandle GNSS_Handle;

/*!
 * Structure initialization.
 * @param GNSS Pointer to main GNSS structure.
 * @param huart Pointer to uart handle.
 */
void GNSS_Init(GNSS_StateHandle *GNSS)
{
  GNSS->year = 0;
  GNSS->month = 0;
  GNSS->day = 0;
  GNSS->hour = 0;
  GNSS->min = 0;
  GNSS->sec = 0;
  GNSS->fixType = 0;
  GNSS->lon = 0;
  GNSS->lat = 0;
  GNSS->height = 0;
  GNSS->hMSL = 0;
  GNSS->hAcc = 0;
  GNSS->vAcc = 0;
  GNSS->gSpeed = 0;
  GNSS->headMot = 0;
}

/*
 * brief: Process a received conf file string, and send response.
 * input: The null-terminated string of conf file name as typed by the user
 * reference: from PES assignment 6 - Howdy Pierce, modified as per requirement
 */
void getNavdata(uint8_t * input)
{
  char *p = input;
  char *end;

  // find end of string
  for (end = input; *end != '\0'; end++)
  ;

  // Lexical analysis: Tokenize input in place
  bool in_token = false;
  char *argv[30];
  int argc = 0;
  memset(argv, 0, sizeof(argv));
  for (p = input; p < end; p++)
  {
    if (in_token && (*p == ','))
    {
      *p = '\0';
      in_token = false;
    }
    else if (!in_token && (*p != ','))
    {
      argv[argc++] = p;
      in_token = true;

      if (argc == sizeof(argv)/sizeof(char*) - 1)
      // too many arguments! drop remainder
        break;
    }
  }
  argv[argc] = NULL;
  if (argc == 0)   // no command
    return;

  //0-GNRMC, 1-UTC time HHMMSS.mm, 2-A, 3-Latitude, 4-N/S, 5-Longitude, 6-E/W, 7-Speed, 8-Date DDMMYY, 9-Checksum
  GNSS_Handle.year = (10 * (argv[8][4] - '0')) + (argv[8][5] - '0') + 2000;
  GNSS_Handle.month = (10 * (argv[8][2] - '0')) + (argv[8][3] - '0');
  GNSS_Handle.day = (10 * (argv[8][0] - '0')) + (argv[8][1] - '0');
  GNSS_Handle.hour = (10 * (argv[1][0] - '0')) + (argv[1][1] - '0');
  GNSS_Handle.min = (10 * (argv[1][2] - '0')) + (argv[1][3] - '0');
  GNSS_Handle.sec = (10 * (argv[1][4] - '0')) + (argv[1][5] - '0');

  char * temp = strchr(argv[3], '.');
  *temp = '\0';

  int lat1 = atoi(argv[3]);

  temp = strchr(argv[5], '.');
  *temp = '\0';

  int lon1 = atoi(argv[5]);

  GNSS_Handle.fLon = (float)lon1*0.01;
  GNSS_Handle.fLat = (float)lat1*0.01;

  if(argv[4][0] == 'S')
    GNSS_Handle.fLat *= -1;
  if(argv[6][0] == 'W')
    GNSS_Handle.fLon *= -1;

  sprintf(GNSS_Handle.uartNavData, "Date: %d/%d/%d Time: %d:%d:%d Lat: %f Lon: %f", GNSS_Handle.day, GNSS_Handle.month, GNSS_Handle.year,
          GNSS_Handle.hour, GNSS_Handle.min, GNSS_Handle.sec, GNSS_Handle.fLat, GNSS_Handle.fLon);
}

/*!
 * Searching for a header in data buffer and matching class and message ID to buffer data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseBuffer(GNSS_StateHandle *GNSS) {

  for (int var = 0; var <= 100; ++var) {
    if (GNSS->uartWorkingBuffer[var] == 0xB5
        && GNSS->uartWorkingBuffer[var + 1] == 0x62) {
      if (GNSS->uartWorkingBuffer[var + 2] == 0x27
          && GNSS->uartWorkingBuffer[var + 3] == 0x03) { //Look at: 32.19.1.1 u-blox 8 Receiver description
        GNSS_ParseUniqID(GNSS);
      } else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
          && GNSS->uartWorkingBuffer[var + 3] == 0x21) { //Look at: 32.17.14.1 u-blox 8 Receiver description
        GNSS_ParseNavigatorData(GNSS);
      } else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
          && GNSS->uartWorkingBuffer[var + 3] == 0x07) { //ook at: 32.17.30.1 u-blox 8 Receiver description
        GNSS_ParsePVTData(GNSS);
      } else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
          && GNSS->uartWorkingBuffer[var + 3] == 0x02) { // Look at: 32.17.15.1 u-blox 8 Receiver description
        GNSS_ParsePOSLLHData(GNSS);
      }
    }
  }
}

/*!
 * Make request for unique chip ID data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetUniqID(GNSS_StateHandle *GNSS) {

  LEUART_Transmit(getDeviceID, sizeof(getDeviceID) / sizeof(uint8_t), 17);
  LEUART_Receive(GNSS_Handle.uartWorkingBuffer, 17);
}

/*!
 * Make request for UTC time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetNavDataDump(GNSS_StateHandle *GNSS) {

  while(1)
    {
      memset(GNSS_Handle.uartWorkingBuffer, 0, 1024);
      LEUART_Transmit(getNavDataDump, sizeof(getNavDataDump) / sizeof(uint8_t), 800);
      LEUART_Receive(GNSS_Handle.uartWorkingBuffer, 800);

      char *temp = strstr(GNSS_Handle.uartWorkingBuffer, "$GNRMC");
      if(strlen(temp) > 80)
        {
          getNavdata(temp);
          break;
        }
    }
}

/*!
 * Make request for UTC time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetNavigatorData(GNSS_StateHandle *GNSS) {
  LEUART_Transmit(getNavigatorData, sizeof(getNavigatorData) / sizeof(uint8_t), 100);
  LEUART_Receive(GNSS_Handle.uartWorkingBuffer, 100);
}

/*!
 * Make request for geodetic position solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetPOSLLHData(GNSS_StateHandle *GNSS) {
  LEUART_Transmit(getPOSLLHData, sizeof(getPOSLLHData) / sizeof(uint8_t), 36);
  LEUART_Receive(GNSS_Handle.uartWorkingBuffer, 36);
}

/*!
 * Make request for navigation position velocity time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetPVTData(GNSS_StateHandle *GNSS) {
  LEUART_Transmit(getPVTData, sizeof(getPVTData) / sizeof(uint8_t), 100);
  LEUART_Receive(GNSS_Handle.uartWorkingBuffer, 100);
}

/*!
 * Parse data to unique chip ID standard.
 * Look at: 32.19.1.1 u-blox 8 Receiver description
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseUniqID(GNSS_StateHandle *GNSS) {
  for (int var = 0; var < 5; ++var) {
    GNSS->uniqueID[var] = GNSS_Handle.uartWorkingBuffer[10 + var];
  }
}

/*!
 * Changing the GNSS mode.
 * Look at: 32.10.19 u-blox 8 Receiver description
 */
void GNSS_SetMode(GNSS_StateHandle *GNSS, short gnssMode) {
  if (gnssMode == 0) {
      LEUART_Transmit(setPortableMode,sizeof(setPortableMode) / sizeof(uint8_t), 0);
  } else if (gnssMode == 1) {
      LEUART_Transmit(setStationaryMode,sizeof(setStationaryMode) / sizeof(uint8_t), 0);
  } else if (gnssMode == 2) {
      LEUART_Transmit(setPedestrianMode,sizeof(setPedestrianMode) / sizeof(uint8_t), 0);
  } else if (gnssMode == 3) {
      LEUART_Transmit(setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t), 0);
  } else if (gnssMode == 4) {
      LEUART_Transmit(setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t), 0);
  } else if (gnssMode == 5) {
      LEUART_Transmit(setAirbone1GMode,sizeof(setAirbone1GMode) / sizeof(uint8_t), 0);
  } else if (gnssMode == 6) {
      LEUART_Transmit(setAirbone2GMode,sizeof(setAirbone2GMode) / sizeof(uint8_t), 0);
  } else if (gnssMode == 7) {
      LEUART_Transmit(setAirbone4GMode,sizeof(setAirbone4GMode) / sizeof(uint8_t), 0);
  } else if (gnssMode == 8) {
      LEUART_Transmit(setWirstMode,sizeof(setWirstMode) / sizeof(uint8_t), 0);
  } else if (gnssMode == 9) {
      LEUART_Transmit(setBikeMode,sizeof(setBikeMode) / sizeof(uint8_t), 0);
  }
}
/*!
 * Parse data to navigation position velocity time solution standard.
 * Look at: 32.17.15.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParsePVTData(GNSS_StateHandle *GNSS) {
  uShort.bytes[0] = GNSS_Handle.uartWorkingBuffer[10];
  GNSS->yearBytes[0]=GNSS_Handle.uartWorkingBuffer[10];
  uShort.bytes[1] = GNSS_Handle.uartWorkingBuffer[11];
  GNSS->yearBytes[1]=GNSS_Handle.uartWorkingBuffer[11];
  GNSS->year = uShort.uShort;
  GNSS->month = GNSS_Handle.uartWorkingBuffer[12];
  GNSS->day = GNSS_Handle.uartWorkingBuffer[13];
  GNSS->hour = GNSS_Handle.uartWorkingBuffer[14];
  GNSS->min = GNSS_Handle.uartWorkingBuffer[15];
  GNSS->sec = GNSS_Handle.uartWorkingBuffer[16];
  GNSS->fixType = GNSS_Handle.uartWorkingBuffer[26];

  for (int var = 0; var < 4; ++var) {
    iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 30];
    GNSS->lonBytes[var]= GNSS_Handle.uartWorkingBuffer[var + 30];
  }
  GNSS->lon = iLong.iLong;
  GNSS->fLon=(float)iLong.iLong/10000000.0;
  for (int var = 0; var < 4; ++var) {
    iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 34];
    GNSS->latBytes[var]=GNSS_Handle.uartWorkingBuffer[var + 34];
  }
  GNSS->lat = iLong.iLong;
  GNSS->fLat=(float)iLong.iLong/10000000.0;
  for (int var = 0; var < 4; ++var) {
    iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 38];
  }
  GNSS->height = iLong.iLong;

  for (int var = 0; var < 4; ++var) {
    iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 42];
    GNSS->hMSLBytes[var] = GNSS_Handle.uartWorkingBuffer[var + 42];
  }
  GNSS->hMSL = iLong.iLong;

  for (int var = 0; var < 4; ++var) {
    uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 46];
  }
  GNSS->hAcc = uLong.uLong;

  for (int var = 0; var < 4; ++var) {
    uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 50];
  }
  GNSS->vAcc = uLong.uLong;

  for (int var = 0; var < 4; ++var) {
    iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 66];
    GNSS->gSpeedBytes[var] = GNSS_Handle.uartWorkingBuffer[var + 66];
  }
  GNSS->gSpeed = iLong.iLong;

  for (int var = 0; var < 4; ++var) {
    iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 70];
  }
  GNSS->headMot = iLong.iLong * 1e-5; // todo I'm not sure this good options.
}

/*!
 * Parse data to UTC time solution standard.
 * Look at: 32.17.30.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseNavigatorData(GNSS_StateHandle *GNSS) {
  uShort.bytes[0] = GNSS_Handle.uartWorkingBuffer[18];
  uShort.bytes[1] = GNSS_Handle.uartWorkingBuffer[19];
  GNSS->year = uShort.uShort;
  GNSS->month = GNSS_Handle.uartWorkingBuffer[20];
  GNSS->day = GNSS_Handle.uartWorkingBuffer[21];
  GNSS->hour = GNSS_Handle.uartWorkingBuffer[22];
  GNSS->min = GNSS_Handle.uartWorkingBuffer[23];
  GNSS->sec = GNSS_Handle.uartWorkingBuffer[24];
}

/*!
 * Parse data to geodetic position solution standard.
 * Look at: 32.17.14.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParsePOSLLHData(GNSS_StateHandle *GNSS) {
  for (int var = 0; var < 4; ++var) {
    iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 10];
  }
  GNSS->lon = iLong.iLong;
  GNSS->fLon=(float)iLong.iLong/10000000.0;

  for (int var = 0; var < 4; ++var) {
    iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 14];
  }
  GNSS->lat = iLong.iLong;
  GNSS->fLat=(float)iLong.iLong/10000000.0;

  for (int var = 0; var < 4; ++var) {
    iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 18];
  }
  GNSS->height = iLong.iLong;

  for (int var = 0; var < 4; ++var) {
    iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 22];
  }
  GNSS->hMSL = iLong.iLong;

  for (int var = 0; var < 4; ++var) {
    uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 26];
  }
  GNSS->hAcc = uLong.uLong;

  for (int var = 0; var < 4; ++var) {
    uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 30];
  }
  GNSS->vAcc = uLong.uLong;
}

/*!
 *  Sends the basic configuration: Activation of the UBX standard, change of NMEA version to 4.10 and turn on of the Galileo system.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_LoadConfig(GNSS_StateHandle *GNSS) {
  LEUART_Transmit(configUBX, sizeof(configUBX) / sizeof(uint8_t), 0);
  timerWaitUs_polled(250000);
  LEUART_Transmit(setNMEA410, sizeof(setNMEA410) / sizeof(uint8_t), 0);
  timerWaitUs_polled(250000);
  LEUART_Transmit(setGNSS, sizeof(setGNSS) / sizeof(uint8_t), 0);
  timerWaitUs_polled(250000);
}



/*!
 *  Creates a checksum based on UBX standard.
 * @param class Class value from UBX doc.
 * @param messageID MessageID value from UBX doc.
 * @param dataLength Data length value from UBX doc.
 * @param payload Just payload.
 * @return  Returns checksum.
 */
uint8_t GNSS_Checksum(uint8_t class, uint8_t messageID, uint8_t dataLength,uint8_t *payload) {
//todo: Look at 32.4 UBX Checksum
  return 0;
}
