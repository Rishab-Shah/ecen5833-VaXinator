/*
 * calendar.c
 *
 *  Created on: 2016. okt. 12.
 *      Author: arkalvac
 */


#include "calendar.h"

#define INCLUDE_LOG_DEBUG 0
#include "src/log.h"
static uint16_t cal_year;
static uint8_t cal_month;
static uint8_t cal_day;
static uint8_t cal_weekday;
static uint8_t cal_hour;
static uint8_t cal_min;
static uint8_t cal_sec;
static uint16_t cal_ms;
static uint16_t cal_tick;
static int8_t  cal_time_zone;
static uint8_t cal_dst;

static uint32_t last_RTCC_value;

void updateCalendar()
{
	uint32_t ticks,ticks_since_last;
	uint8_t diff_day;
	uint8_t diff_hour;
	uint8_t diff_min;
	uint8_t diff_sec;
	uint16_t diff_tick;

	ticks = RTCC_CounterGet();
	ticks_since_last = ticks - last_RTCC_value;
	last_RTCC_value = ticks;

	diff_tick     = (ticks_since_last) % 32768;
	diff_sec      = (ticks_since_last / 32768) % 60;
	diff_min      = (ticks_since_last / 32768 / 60) % 60;
	diff_hour     = (ticks_since_last / 32768 / 60 / 60) % 24;
	diff_day      = (ticks_since_last / 32768 / 60 / 60 / 24);

	cal_tick += diff_tick;
	if (cal_tick >= 32768)
	{
		cal_tick -= 32768;
		cal_sec++;
	}
	cal_ms = cal_tick * 1000 / 32768;
	cal_sec += diff_sec;
	if (cal_sec >= 60)
	{
		cal_sec -= 60;
		cal_min++;
	}
	cal_min += diff_min;
	if (cal_min >= 60)
	{
		cal_min -= 60;
		cal_hour++;
	}
	cal_hour += diff_hour;
	if (cal_hour >= 24)
	{
		cal_hour -= 24;
		cal_day++;
		cal_weekday++;
	}
	cal_weekday += diff_day;
	if (cal_weekday > 7)
	{
		cal_weekday -= 7;
	}
	cal_day += diff_day;
	switch (cal_month)
	{
		case 1:
		case 3:
		case 5:
		case 7:
		case 8:
		case 10:
			if (cal_day > 31)
			{
				cal_day -= 31;
				cal_month++;
			}
		break;

		case 12:
			if (cal_day > 31)
			{
				cal_day -= 31;
				cal_month = 1;
				cal_year++;
			}
		break;

		case 4:
		case 6:
		case 9:
		case 11:
			if (cal_day > 30)
			{
				cal_day -= 30;
				cal_month++;
			}
		break;

		case 2:
			if (cal_year % 4 == 0 && (cal_year % 100 != 0 || cal_year % 400 == 0))
			{
				if (cal_day > 29)
				{
					cal_day -= 29;
					cal_month++;
				}
			}
			else
			{
				if (cal_day > 28)
				{
					cal_day -= 28;
					cal_month++;
				}
			}
		break;
	}
}

void setDateAndTime(uint16_t year, uint8_t month, uint8_t day, uint8_t weekday, uint8_t hour, uint8_t min, uint8_t sec, uint16_t ms)
{
	cal_year    = year;
	cal_month   = month;
	cal_day     = day;
	cal_weekday = weekday;
	cal_hour    = hour;
	cal_min     = min;
	cal_sec     = sec;
	cal_ms      = ms;

	last_RTCC_value = RTCC_CounterGet();
}

void getDateAndTime(uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* weekday, uint8_t* hour, uint8_t* min, uint8_t* sec, uint16_t* ms)
{
	updateCalendar();
  char buffer[100]; memset(buffer,0,sizeof(buffer));

  static bool status = false;
  if(status == false)
  {
    int year_t = 22;
    int month_t = 5;
    int day_t = 1;

    int hour_t =0, min_t = 0, sec_t =0;
    strcpy(buffer,__TIME__);
    LOG_INFO("TIME IS ==%s==\r",buffer);
    sscanf(buffer,"%d:%d:%d",&hour_t,&min_t,&sec_t);
    LOG_INFO("%4d-%02d-%02d %02d:%02d:%02d\r",year_t,month_t,day_t,hour_t,min_t,sec_t);

    *year = year_t;
    *month = month_t;
    *day = day_t;
    *hour = hour_t;
    *min = min_t;
    *sec = sec_t;
    *ms = cal_ms;
    *weekday = cal_weekday;

    cal_year = *year ;
    cal_month = *month;
    cal_day = *day;
    cal_weekday = *weekday;
    cal_hour = *hour;
    cal_min = *min;
    cal_sec = *sec;
    cal_ms = *ms;
    //LOG_INFO("%4d-%02d-%02d %02d:%02d:%02d\r",*year,*month,*day,*hour,*min,*sec);
    status = true;
    return;
  }

	*year = cal_year;
	*month = cal_month;
	*day = cal_day;
	*weekday = cal_weekday;
	*hour = cal_hour;
	*min = cal_min;
	*sec = cal_sec;
	*ms = cal_ms;
}

void setTimeZone(int8_t time_zone)
{
	cal_time_zone = time_zone;
}

int8_t getTimeZone()
{
	return cal_time_zone;
}

void setDst(uint8_t dst)
{
	cal_dst = dst;
}

uint8_t getDst()
{
	return cal_dst;
}
