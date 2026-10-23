#include <limits.h>
#include "sys.h"

static const unsigned int mon_len[] =
  {31,28,31,30,31,30,31,31,30,31,30,31 };
static const unsigned int mon_day[] =
  {0,31,59,90,120,151,181,212,243,273,304,334};

#define JAN11900    (time_t)1
#define DAYS_IN_YR  (time_t)365
#define SECS_IN_MIN (time_t)60
#define MINS_IN_HR  (time_t)60
#define HRS_IN_DAY  (time_t)24
#define SECS_IN_HR  (SECS_IN_MIN * MINS_IN_HR)
#define SECS_IN_DAY (SECS_IN_HR  * HRS_IN_DAY)
#define SECS_IN_YR  (DAYS_IN_YR  * SECS_IN_DAY)

#define LEAPYEAR(y)   ( ((y + 1900) % 4 == 0) && (((y + 1900) % 100 != 0) || ((y + 1900) % 400 == 0)) )

/*****************************************************************************/
/* 将日期时间类型转换为秒计数                                                */
/*****************************************************************************/
time_t mktime (struct tm *tptr)
{
    time_t   result;
    time_t   daycount;
    int      mdays;
    int      adjust;

    /*-----------------------------------------------------------------*/
    /* HANDLE SECONDS.  IF TOO MANY OR TOO FEW, MODIFY MINUTES.        */
    /*-----------------------------------------------------------------*/
    adjust = (tptr->tm_sec < 0) ? -((-tptr->tm_sec + 59) / 60)
             : (tptr->tm_sec / 60);

    tptr->tm_min += adjust;
    tptr->tm_sec -= adjust * 60;

    /*-----------------------------------------------------------------*/
    /* HANDLE MINUTES.  IF TOO MANY OR TOO FEW, MODIFY HOURS           */
    /*-----------------------------------------------------------------*/
    adjust = (tptr->tm_min < 0) ? -((-tptr->tm_min + 59) / 60)
             : (tptr->tm_min / 60);

    tptr->tm_hour += adjust;
    tptr->tm_min  -= adjust * 60;

    /*-----------------------------------------------------------------*/
    /* HANDLE HOURS.    IF TOO MANY OR TOO FEW, MODIFY DAYS            */
    /*-----------------------------------------------------------------*/
    adjust = (tptr->tm_hour < 0) ? -((-tptr->tm_hour + 23) / 24)
              : (tptr->tm_hour / 24);

    tptr->tm_mday += adjust;
    tptr->tm_hour -= adjust * 24;

    for (;;)
    {
      /*-----------------------------------------------------------------*/
      /* HANDLE MONTHS.   IF TOO MANY OR TOO FEW, MODIFY YEARS           */
      /*-----------------------------------------------------------------*/
      adjust = (tptr->tm_mon < 0) ? -((-tptr->tm_mon + 11) / 12)
          : (tptr->tm_mon / 12);

      tptr->tm_year += adjust;
      tptr->tm_mon  -= adjust * 12;

      /*-----------------------------------------------------------------*/
      /* HANDLE DAYS.     IF TOO MANY OR TOO FEW, MODIFY MONTHS          */
      /*-----------------------------------------------------------------*/
      if (tptr->tm_mday <= 0)
      {
        tptr->tm_mon--;
        tptr->tm_mday += mon_len[tptr->tm_mon < 0 ? 11 : tptr->tm_mon] +
            ((tptr->tm_mon == 1) && ((tptr->tm_year % 4) == 0));
        continue;
      }

      /*-----------------------------------------------------------------*/
      /* CALCULATE NUMBER OF DAYS IN THIS MONTH.                         */
      /*-----------------------------------------------------------------*/
      mdays = mon_len[tptr->tm_mon] + ( (tptr->tm_mon == 1) && LEAPYEAR(tptr->tm_year) );

      if (tptr->tm_mday > mdays)
      {
        tptr->tm_mday -= mdays;
        tptr->tm_mon++;
      }
      else break;
    }

    /*--------------------------------------------------------------------*/
    /* WE CAN NOW BE SURE THAT ALL FIELDS IN THE TIME STRUCTURE ARE RIGHT */
    /*--------------------------------------------------------------------*/
    tptr->tm_yday = mon_day[tptr->tm_mon]  + tptr->tm_mday - 1 +
                   ( (tptr->tm_mon > 1) && LEAPYEAR(tptr->tm_year) );

    daycount = tptr->tm_year * DAYS_IN_YR  +
             ((tptr->tm_year - 1) / 4)   + tptr->tm_yday;

    result = daycount      * SECS_IN_DAY +
             tptr->tm_hour * SECS_IN_HR  +
             tptr->tm_min  * SECS_IN_MIN + tptr->tm_sec;

    tptr->tm_wday = (JAN11900 + daycount) % 7;

    return result;
}

#define DAYS_IN(y) ((time_t)365 + LEAPYEAR(y))

/*****************************************************************************/
/* 将秒计数转换为日期时间类型                                                */
/*****************************************************************************/
struct tm *localtime(const time_t *timer)
{
    static struct tm local;
    time_t ltime  = timer ? *timer : 0;

    local.tm_sec  = 0;
    local.tm_min  = 0;
    local.tm_hour = 0;
    local.tm_mday = 1;
    local.tm_mon  = 0;
    local.tm_year = 0;

    if (timer == 0 || ltime == (time_t)-1) return &local;

#if INT_MAX <= 32767
    /*------------------------------------------------------------------*/
    /* MAKE SURE THE NUMBER OF SECONDS SINCE Jan 1, 1900 CAN BE         */
    /* REPRESENTED IN SIGNED INTS.                                      */
    /*------------------------------------------------------------------*/
    local.tm_sec   =  ltime % SECS_IN_MIN;
    local.tm_min   = (ltime / SECS_IN_MIN) % MINS_IN_HR;
    local.tm_hour  = (ltime / SECS_IN_HR)  % HRS_IN_DAY;

    /*------------------------------------------------------------------*/
    /* CONVERT ltime TO NUMBER OF DAYS                                  */
    /*------------------------------------------------------------------*/
    ltime /= SECS_IN_DAY;

    /*------------------------------------------------------------------*/
    /* TO DETERMINE THE YEAR, INSTEAD OF DIVIDING BY 365, DO A SUBTRACT */
    /* LOOP THAT ACCOUNTS FOR LEAP YEARS.                               */
    /*------------------------------------------------------------------*/
    {
      int year = 0;
      while (ltime >= DAYS_IN(year))
      {
        ltime -= DAYS_IN(year);
        ++year;
      }

      local.tm_year  = year;
      local.tm_mday += (int)ltime;
    }

#else
    /*------------------------------------------------------------------*/
    /* MAKE SURE THE NUMBER OF SECONDS SINCE Jan 1, 1900 CAN BE         */
    /* REPRESENTED IN SIGNED INTS.                                      */
    /*------------------------------------------------------------------*/
    if ((int)ltime < 0)
    {
      local.tm_sec  = ltime % 60;
      local.tm_min  = ltime / 60;
    }
    else local.tm_sec = ltime;
#endif

    /*------------------------------------------------------------------*/
    /* MAKE VALUES IN local INTO A VALID TIME.                          */
    /*------------------------------------------------------------------*/
    (void) mktime(&local);
    return &local;
}
