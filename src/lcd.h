/***********************************************************************
 * @file      lcd.h
 * @version   1.0
 * @brief     LCD header file. A complete re-write of the LCD support code
 *            based on Gecko SDK 3.1 and Simplicity Studio 5.1.
 *
 * @author    Dave Sluiter, David.Sluiter@colorado.edu
 * @date      March 15, 2021
 *
 * @editor    Vishnu Dodballapur
 * @date      Sep 27, 2021
 * @change    Initialized LCD, filled in refresh function
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823: IoT Embedded Firmware
 * @instructor  David Sluiter
 *
 * @assignment Starter code
 * @due        NA
 *
 * @resources  This code is based on the Silicon Labs example MEMLCD_baremetal
 *             as part of SSv5 and Gecko SDK 3.1.
 *
 * @copyright  All rights reserved. Distribution allowed only for the
 * use of assignment grading. Use of code excerpts allowed at the
 * discretion of author. Contact for permission.
 */


#ifndef SRC_LCD_H_
#define SRC_LCD_H_

#include "string.h"
#include "sl_bt_api.h"

#include "ble_device_type.h"
#include "gpio.h"

#include "glib.h" // the low-level graphics driver/library
#include "dmd.h"  // the dot matrix display driver



/**
 * Display row definitions, used for writing specific content based on
 * assignment requirements. See assignment text for details.
 */
enum display_row {
	DISPLAY_ROW_NAME,          // 0
	DISPLAY_ROW_BTADDR,        // 1
	DISPLAY_ROW_BTADDR2,       // 2
	DISPLAY_ROW_CLIENTADDR,    // 3
	DISPLAY_ROW_CONNECTION,    // 4
	DISPLAY_ROW_PASSKEY,       // 5
	DISPLAY_ROW_ACTION,        // 6
	DISPLAY_ROW_X,             // 7
	DISPLAY_ROW_Y,             // 8
	DISPLAY_ROW_Z,             // 9
	DISPLAY_ROW_HEARTBEAT,     // 10
	DISPLAY_ROW_ACTIVITY_STATE,// 11
	DISPLAY_ROW_ASSIGNMENT,    // 12
	DISPLAY_NUMBER_OF_ROWS     // 13
};

// The number of characters per row
#define DISPLAY_ROW_LEN      20

#define LCD_TIMER_HANDLE     2
#define LCD_TIMER_INTERVAL   32768


// function prototypes

void displayInit();
void displayUpdate();
void displayPrintf(enum display_row row, const char *format, ...);




#endif /* SRC_LCD_H_ */
