#include "sys.h"
#include "drv_adc.h"
#include "drv_spi.h"

/*****************************************************************************/
/* 启动ADC采样，返回AD原始值                                                 */
/*****************************************************************************/
UINT16 ADC_read (void)
{
    UINT8 hi, lo;

    PTP_PTP6 = 0;

    udelay (5);

    (void) SPI2_sndrcv (0x12);

    hi = SPI2_sndrcv (0xff);
    lo = SPI2_sndrcv (0xff);

    PTP_PTP6 = 1;

    return ((UINT16)hi << 8) + lo;
}

UINT8   Regs[16];

/*****************************************************************************/
/* ADC控制器初始化                                                           */
/*****************************************************************************/
void ADC_Init (void)
{
    UINT8   i;
    BOOL    ok = FALSE;

    for (i = 0; i < 70; i++)
    {
        PTP_PTP6 = 0;
        udelay (20);                //避免SS下拉时间太短，造成下拉不成功

        (void) SPI2_sndrcv (0x43);
        (void) SPI2_sndrcv (0);     // 一个寄存器

        (void) SPI2_sndrcv (0x6);   //SYS0: 用于sps设置

        PTP_PTP6 = 1;

        mdelay (3);

        //读AD寄存器
        PTP_PTP6 = 0;
        udelay (20);                //避免SS下拉时间太短，造成下拉不成功

        (void) SPI2_sndrcv (0x20);
        (void) SPI2_sndrcv (10);    // 11个寄存器
        Regs[0] = SPI2_sndrcv (0xff);
        Regs[1] = SPI2_sndrcv (0xff);
        Regs[2] = SPI2_sndrcv (0xff);
        Regs[3] = SPI2_sndrcv (0xff);
        Regs[4] = SPI2_sndrcv (0xff);
        Regs[5] = SPI2_sndrcv (0xff);
        Regs[6] = SPI2_sndrcv (0xff);
        Regs[7] = SPI2_sndrcv (0xff);
        Regs[8] = SPI2_sndrcv (0xff);
        Regs[9] = SPI2_sndrcv (0xff);
        Regs[10] = SPI2_sndrcv (0xff);

        PTP_PTP6 = 1;
        //END

        mdelay (3);

        if ((Regs[0] & 0x1) == 0x1 && Regs[3] == 0x6 && (Regs[10] & 0x3) == 0x0)
        {
            ok = TRUE;
            break;
        }
    }
    
    if (!ok)
        gSysErrFlags |= SYSERR_CURADC_ERROR;
}
