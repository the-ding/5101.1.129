#include <string.h>
#include "sys.h"
#include "drv_iic.h"
#include "drv_rtc.h"

#define RTC_ADDR    0xd0

/*****************************************************************************/
/* 写RTC寄存器                                                               */
/*****************************************************************************/
static BOOL rtcWr (UINT8 reg, const void *buf, UINT8 n)
{
    UINT8 i;

    /*主模式、START BIT、发送方式*/
    IIC0_IBCR = IIC0_IBCR_IBEN_MASK | IIC0_IBCR_MS_SL_MASK | IIC0_IBCR_TX_RX_MASK;

    if (!IIC_Write (RTC_ADDR | 0x0))                    /*IIC从设备地址、写*/
    {
we:     IIC0_IBCR_MS_SL = 0;                            /*STOP*/
        gSysErrFlags |= SYSERR_RTC_ERROR;
        return FALSE;
    }

    if (!IIC_Write (reg))
      goto we;

    for (i = 0; i < n; i++)
    {
      if (!IIC_Write (*((UINT8 *)buf + i)))
        goto we;
    }

    IIC0_IBCR_MS_SL = 0;                              /*STOP*/

    udelay (20);
    return TRUE;
}

/*****************************************************************************/
/* 读RTC寄存器                                                               */
/*****************************************************************************/
static BOOL rtcRd (UINT8 reg, void *buf, UINT8 n)
{
    UINT8 i;

    /*主模式、START BIT、发送方式*/
    IIC0_IBCR = IIC0_IBCR_IBEN_MASK | IIC0_IBCR_MS_SL_MASK | IIC0_IBCR_TX_RX_MASK;

    if (!IIC_Write (RTC_ADDR | 0x0))                    /*IIC从设备地址、写*/
    {
re:     IIC0_IBCR_MS_SL = 0;                            /*STOP*/
        gSysErrFlags |= SYSERR_RTC_ERROR;
        return FALSE;
    }

    if (!IIC_Write (reg))
      goto re;

    IIC0_IBCR_RSTA = 1;                                 /*START*/

    if (!IIC_Write (RTC_ADDR | 0x1))                    /*IIC从设备地址、读*/
      goto re;

    IIC0_IBCR_TX_RX = 0;                              /*接收方式*/

    i = IIC0_IBDR;                                    /*启动接收 (第一个字节内容无效)*/

    for (i = 0; i < n; i++)         //lint !e838
    {
        if (!IIC_Wait (IIC0_IBSR_IBIF_MASK, IIC0_IBSR_IBIF_MASK))
          goto re;

        *((UINT8 *)buf + i) = IIC0_IBDR;
    }

    IIC0_IBCR_TXAK = 1;                               /*NACK*/

    (void) IIC_Wait (IIC0_IBSR_IBIF_MASK, IIC0_IBSR_IBIF_MASK);

    IIC0_IBCR_MS_SL = 0;                              /*STOP*/

    udelay (20);
    return TRUE;
}

/*****************************************************************************/
/* 读取RTC日期时间                                                           */
/*****************************************************************************/
BOOL  RTC_read (struct tm *t)
{
    BOOL  rc;
    UINT8 regs[10];

    regs[0] = 0x1;
    (void) rtcWr (0x0, &regs[0], 1);

    rc = rtcRd (0x2, &regs[2], 8);

    regs[0] = 0x0;
    (void) rtcWr (0x0, &regs[0], 1);

    if (!rc)
      return FALSE;

    t->tm_year = 2000 + BCD2HEX (regs[8]) - 1900;
    t->tm_mon  = BCD2HEX (regs[7]) - 1;
    t->tm_mday = BCD2HEX (regs[6]);
    t->tm_wday = (char) BCD2HEX (regs[5]);
    t->tm_hour = (char) BCD2HEX (regs[4]);
    t->tm_min  = (char) BCD2HEX (regs[3]);
    t->tm_sec  = (char) BCD2HEX (regs[2]);
    return TRUE;
}

/*****************************************************************************/
/* 设置RTC日期时间                                                           */
/*****************************************************************************/
BOOL  RTC_write (const struct tm *t)
{
    UINT8 regs[10];

    regs[8] = HEX2BCD ((UINT8) (t->tm_year + 1900 - 2000));
    regs[7] = HEX2BCD ((UINT8) (t->tm_mon + 1));
    regs[6] = HEX2BCD ((UINT8) (t->tm_mday));
    regs[5] = HEX2BCD ((UINT8) (t->tm_wday));
    regs[4] = HEX2BCD ((UINT8) (t->tm_hour));
    regs[3] = HEX2BCD ((UINT8) (t->tm_min));
    regs[2] = HEX2BCD ((UINT8) (t->tm_sec));

    regs[0] = 0x2;
    (void) rtcWr (0x0, &regs[0], 1);

    (void) rtcWr (0x2, &regs[2], 8);

    regs[0] = 0x0;
    (void) rtcWr (0x0, &regs[0], 1);
    return TRUE;
}

/*****************************************************************************/
/* IIC接口RTC初始化                                                          */
/*****************************************************************************/
void  RTC_Init (void)
{
    UINT8 regs[18];
    BOOL  rc;
   

re_rtc_init:   

     /*
      regs[0] = 0x02;
      (void) rtcWr (0xb, &regs[0], 1);    //时钟寄存器写入3.9电压复位参数
    */
    
    /*00h - Flags/Control*/
    rc = rtcRd (0x1, &regs[1], 1);

    if (rc && (regs[1] & 0x80))      /* /OSCEN */
    {
        regs[1] &= 0x7f;

        (void) rtcWr (0x1, &regs[1], 1);
    }

    regs[0] = 0x0;
    (void) rtcWr (0x0, &regs[0], 1);

    /*0Bh - Companion Control*/
    (void) rtcRd (0xb, &regs[0xB], 1);

    regs[0xB] = 0x00;                 /* SNL,WP1,WP0 = 0 */

    (void) rtcWr (0xb, &regs[0xB], 1);
    
    //新加入用于诊断铁电错误的参数，在错误模式下不启动模块，直到恢复为止//
    if((gSysErrFlags & SYSERR_RTC_ERROR) ) 
    {
      gSysErrFlags &= ~SYSERR_RTC_ERROR;
     mdelay(2);
      goto re_rtc_init;
    }
}