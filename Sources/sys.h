#include <hidef.h>            /* common defines and macros */
#include <mc9s12xdp512.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12xdp512"

/*系统错误标志位*/
#define SYSERR_FRAM_READ_ERROR    0x1UL   /*铁电读错误*/
#define SYSERR_FRAM_WRITE_ERROR   0x2UL   /*铁电写错误*/
#define SYSERR_FLASH_READ_ERROR   0x4UL   /*FLASH读错误*/
#define SYSERR_FLASH_WRITE_ERROR  0x8UL   /*FLASH写错误*/
#define SYSERR_RTC_ERROR          0x10UL  /*时钟错误*/
#define SYSERR_PARA_LOST_ERROR    0x20UL  /*参数丢失*/
#define SYSERR_TIME_LOST_ERROR    0x40UL  /*时间丢失*/
#define SYSERR_CAN4_ERROR         0x80UL  /*与采集板通讯错误*/

#define SYSERR_CURADC_ERROR       0x10000UL /*电流ADC错误*/
#define SYSERR_ISOADC_ERROR       0x20000UL

/*系统运行标志*/
#define SYSFLAG_RUN               0x1UL   /*500ms周期标志*/
#define SYSFLAG_AGE               0x2UL   /*1s老化一次采集板状态*/
#define SYSFLAG_SYSRT             0x4UL   /*1s计算一次整车数据*/
#define SYSFLAG_ZIBO              0x8UL   /*100ms发送一次仪表CAN帧*/
#define SYSFLAG_EVCHK             0x10UL

/*系统标准类型定义*/
typedef unsigned char   BOOL;
typedef unsigned char   UINT8;
typedef unsigned short  UINT16;
typedef unsigned long   UINT32;
typedef signed char     SINT8;
typedef signed short    SINT16;
typedef signed long     SINT32;

struct tm
{
    int     tm_year;
    int     tm_mon;
    int     tm_yday;
    int     tm_mday;
    char    tm_wday;
    char    tm_hour;
    char    tm_min;
    char    tm_sec;
};

/*系统时钟滴答计数器*/
extern volatile UINT32  gSysTicks;
extern volatile UINT32  gSysTicks_2ms;
extern volatile UINT32  gSysSec;
extern volatile UINT32  gSysErrFlags;

#pragma DATA_SEG SHARED_DATA
extern volatile BOOL    gSleepMode;
extern   BOOL  BCU_Update ;
#pragma DATA_SEG DEFAULT

/*版本信息*/
#pragma CONST_SEG VER_VARS
//extern const volatile UINT16  HWVer;
extern const volatile UINT16  OEMVer;
#pragma CONST_SEG DEFAULT



/*硬件版本信息*/
#pragma CONST_SEG HW_VARS
 extern const volatile UINT16  HWVer ;
#pragma CONST_SEG DEFAULT









extern volatile UINT16 SWVer;//软件版本合成字节，位于ptlc


/*****************************************************************************/
/*系统维护标志*/
/*****************************************************************************/
extern volatile BOOL  service;




/*****************************************************************************/
/* 检查并清除系统标志位                                                      */
/*****************************************************************************/
BOOL Sys_getFlags (UINT32 flags);

/*****************************************************************************/
/* 检查系统标志位                                                            */
/*****************************************************************************/
BOOL Sys_hasFlags (UINT32 flags);

/*****************************************************************************/
/* 设置系统标志位                                                            */
/*****************************************************************************/
void Sys_setFlags (UINT32 flags);

/*****************************************************************************/
/* HEX转BCD                                                                  */
/*****************************************************************************/
UINT8 HEX2BCD (UINT8 hex);

/*****************************************************************************/
/* BCD转HEX                                                                  */
/*****************************************************************************/
UINT8 BCD2HEX (UINT8 bcd);

/*****************************************************************************/
/* 16位校验和算法(TCP/IP)                                                    */
/*****************************************************************************/
UINT16 Chksum_BUG (const void *x, UINT16 bytes);
UINT16 Chksum (const void *x, UINT16 bytes);

/*****************************************************************************/
/* CRC8：生成多项式 = X8+X5+X4+1                                             */
/*****************************************************************************/
UINT8 CRC8 (const UINT8 *buf, UINT8 n);

/*****************************************************************************/
/* 系统毫秒延时                                                              */
/*****************************************************************************/
void mdelay (UINT16 ms);

/*****************************************************************************/
/* 系统微秒延时                                                              */
/*****************************************************************************/
void udelay (UINT16 us);

/*****************************************************************************/
/* 大、小端数转换                                                            */
/*****************************************************************************/
UINT16 swap16 (UINT16 x);
UINT32 swap32 (UINT32 x);

/*****************************************************************************/
/* 将日期时间类型转换为秒计数                                                */
/*****************************************************************************/
time_t mktime (struct tm *tptr);

/*****************************************************************************/
/* 将秒计数转换为日期时间类型                                                */
/*****************************************************************************/
struct tm *localtime(const time_t *timer);

#define MS2TICKS(ms)    ((ms) / 10)
#define TICKS2MS(t)     ((t) * 10)

/*避免使用far指针函数库*/
#define MEMCPY(D,S,sz) { UINT16 i; for (i = 0; i < (sz); i++) (D)[i] = (S)[i]; }
#define MEMSET(D,C,sz) { UINT16 i; for (i = 0; i < (sz); i++) (D)[i] = (C); }
