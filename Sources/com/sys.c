#include "sys.h"
#include "ptl.h"

/*系统滴答计数器*/
volatile UINT32           gSysTicks = 0;
volatile UINT32           gSysTicks_2ms = 0;
volatile UINT8            gSysTicks_count = 0;


/*系统秒时钟*/
volatile UINT32           gSysSec;

/*系统错误标志*/
volatile UINT32           gSysErrFlags = 0x0;

/*系统维护标志*/
volatile BOOL service;

volatile UINT16 SWVer = 0;//软件版本合成字节，位于ptlc


#pragma DATA_SEG SHARED_DATA
/*系统标志*/
static volatile UINT32    SysFlags = 0x0;
volatile BOOL             gSleepMode = FALSE;
#pragma DATA_SEG DEFAULT






#pragma CODE_SEG NON_BANKED

/*****************************************************************************/
/* 系统滴答中断 (100Hz) *5  2ms                                                     */
/*****************************************************************************/
#pragma TRAP_PROC
void Rti_Handle(void)
{
      UINT32  ticks =0;

       gSysTicks_2ms ++;
       gSysTicks_count ++;
       
       CRGFLG_RTIF = 1;                        /*清中断*/

  if(gSysTicks_count >=5)
     {
     
       gSysTicks_count =0;
     
    
     ticks = ++gSysTicks;

    XGSWT = 0x0101;                         /*启动AD转换，读电流*/

    if (!((UINT8)ticks & 1))
    {
        #if PTL_EGV
        extern void EGV_Timer (void);

        EGV_Timer ();                       /*EGV协议定时器：每20ms调用一次*/
        #endif

        #if PTL_ZIBO || PTL_LINYI
        if (!(ticks % 10))
            Sys_setFlags (SYSFLAG_ZIBO);
        #endif

        if (!(ticks % 50))
        {
            Sys_setFlags (SYSFLAG_RUN | SYSFLAG_EVCHK); /*500ms闪烁一次运行灯*/

            if (!(ticks % 100))
            {
                /*需要老化采集板,计算整车数据*/
                Sys_setFlags (SYSFLAG_AGE | SYSFLAG_SYSRT);

                gSysSec++;
            }
        }
    }
    }
}
#pragma CODE_SEG DEFAULT

/*****************************************************************************/
/* HEX转BCD                                                                  */
/*****************************************************************************/
UINT8 HEX2BCD (UINT8 hex)
{
    return ((hex/10) << 4) | (hex%10);
}

/*****************************************************************************/
/* BCD转HEX                                                                  */
/*****************************************************************************/
UINT8 BCD2HEX (UINT8 bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0xf);
}

/*****************************************************************************/
/* 16位校验和算法(TCP/IP)                                                    */
/*****************************************************************************/
UINT16 Chksum_BUG (const void *x, UINT16 bytes)
{
    UINT16  i, chksum = 0;

    for (i = 0; i < bytes; i++)
      chksum += *(UINT8 *)x;
    return chksum;
}

UINT16 Chksum (const void *x, UINT16 bytes)
{
    UINT16  i, chksum = 0;

    for (i = 0; i < bytes/2; i++)
      chksum += *((UINT16 *)x + i);
    return chksum;
}

/*****************************************************************************/
/* CRC8：生成多项式 = X8+X5+X4+1                                             */
/*****************************************************************************/
UINT8 CRC8 (const UINT8 *buf, UINT8 n)
{
    UINT8  i, j, crc = 0x0;

    for (i = 0; i < n; i++)
    {
      UINT8 x = buf[i];

      for (j = 0; j < 8; j++)
      {
        if (((crc ^ x) & 0x01) == 0)
          crc >>= 1;
        else
        {
          crc ^= 0x18;   //CRC=X8+X5+X4+1
          crc >>= 1;
          crc |= 0x80;
        }
        x >>= 1;
     }
   }
   return crc;
}

/*****************************************************************************/
/* 大、小端数转换                                                            */
/*****************************************************************************/
UINT16 swap16 (UINT16 x)
{
    return (x >> 8) | (x << 8);
}

UINT32 swap32 (UINT32 x)
{
    return (x >> 24) | (x << 24) | ((x >> 8) & 0x00FF00) | ((x << 8) & 0xFF0000);
}

/*****************************************************************************/
/* 检查并清除系统标志位                                                      */
/*****************************************************************************/
BOOL Sys_getFlags (UINT32 flags)
{
    if ((SysFlags & flags) != flags)
        return FALSE;

    DisableInterrupts;

    if ((SysFlags & flags) != flags)
    {
        EnableInterrupts;
        return FALSE;
    }

    SysFlags &= ~flags;

    EnableInterrupts;
    return TRUE;
}

/*****************************************************************************/
/* 检查系统标志位                                                            */
/*****************************************************************************/
uchar Sys_hasFlags (UINT32 flags)
{
    if ((SysFlags & flags) != flags)
        return FALSE;

    return TRUE;
}

/*****************************************************************************/
/* 设置系统标志位                                                            */
/*****************************************************************************/
void  Sys_setFlags (UINT32 flags)
{
    DisableInterrupts;

    SysFlags |= flags;

    EnableInterrupts;
}
