/*
 * calendar.h
 *
 *  Created on: 2016. okt. 13.
 *      Author: arkalvac
 */

#ifndef CALENDAR_H_
#define CALENDAR_H_

#include "em_rtcc.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

void updateCalendar();
void setDateAndTime(uint16_t year, uint8_t month, uint8_t day, uint8_t weekday, uint8_t hour, uint8_t min, uint8_t sec, uint16_t ms);
void getDateAndTime(uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* weekday, uint8_t* hour, uint8_t* min, uint8_t* sec, uint16_t* ms);
void setTimeZone(int8_t time_zone);
int8_t getTimeZone();
void setDst(uint8_t dst);
uint8_t getDst();


#endif /* CALENDAR_H_ */
