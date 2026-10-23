#include "sys.h"
#include "db.h"
#include "drv_iic.h"
#include "drv_ads11x.h"

#define ACK             0
#define NACK            1

   /*****************************************************************************/
/* 读绝缘检测ADC寄存器                                                       */
/*****************************************************************************/
UINT16 TmpAdc_getreg (UINT8 reg)
{
    volatile UINT8 msb, lsb;
    UINT8 i;

    for (i = 0; i < 10; i++)
    {
        IIC9_Start ();

        if (!IIC9_Write (0x90 | 0x0))    /*IIC从设备地址、写*/
        {
we2:        udelay (100);
            continue;
        }

        if (!IIC9_Write (reg))
          goto we2;

        IIC9_Start ();                 /*RESTART*/

        if (!IIC9_Write (0x90 | 0x1))    /*IIC从设备地址、读*/
          goto we2;

        msb = IIC9_Read (ACK);
        lsb = IIC9_Read (NACK);

        IIC9_End ();
        
        return ((UINT16)msb << 8) | lsb;
    }
    return 0x0000;
}

/*****************************************************************************/
/* 写绝缘检测ADC寄存器                                                       */
/*****************************************************************************/
void TmpAdc_setreg (UINT8 reg, UINT16 v)
{
    UINT8 i;

    for (i = 0; i < 10; i++)
    {
        IIC9_Start ();

        if (!IIC9_Write (0x90 | 0x0))    /*IIC从设备地址、写*/
        {
we:         udelay (100);
            continue;
        }

        if (!IIC9_Write (reg))
          goto we;

        if (!IIC9_Write (v >> 8))
          goto we;                         ;

        if (!IIC9_Write (v & 0xff))
          goto we;

        IIC9_End ();

        mdelay (5);
        break;
    }
}

/*****************************************************************************/
/* 绝缘检测ADC初始化                                                         */
/*****************************************************************************/
void TmpAdc_Init (void)
{
   volatile UINT16 cfg = 0;

    TmpAdc_setreg (0x01, 0x0580); /*Single-shot*/
    mdelay (2);
    
    //TmpAdc_setreg (0x02, 0x0000);
    //mdelay (2);
    //TmpAdc_setreg (0x03, 0x8000);
    //mdelay (2);

#if 1
    /*校验配置参数*/

    cfg = TmpAdc_getreg (0x01);
    if (cfg != 0x8580)
    {
        //sts.isoad_id = 0xff;
        //gSysErrFlags |= SYSERR_TMPADC_ERROR;
    }
    else
    {
        //sts.isoad_id = 0x55;
    }
#endif
}



