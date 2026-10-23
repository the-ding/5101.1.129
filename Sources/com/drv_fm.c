#include "sys.h"
#include "drv_iic.h"
#include "drv_fm.h"

#define FM_ADDR   0xa0


/*****************************************************************************/
/* 铁电RAM读操作                                                             */
/*****************************************************************************/
BOOL  Fm_read (UINT16 addr, void *buf, UINT16 bytes)
{
    UINT16  i;
    
    if (bytes == 0)
      return 0;

    /*主模式、START BIT、发送方式*/
    IIC0_IBCR = IIC0_IBCR_IBEN_MASK | IIC0_IBCR_MS_SL_MASK | IIC0_IBCR_TX_RX_MASK;

    if (!IIC_Write (FM_ADDR | 0x0))                   /*IIC从设备地址、写*/
    {
re:     IIC0_IBCR_MS_SL = 0;                          /*STOP*/
        gSysErrFlags |= SYSERR_FRAM_READ_ERROR;
        return 0 ;
    }

    if (!IIC_Write (addr >> 8))                       /*偏移地址高字节*/
      goto re;

    if (!IIC_Write (addr & 0xff))                     /*偏移地址低字节*/
      goto re;


    IIC0_IBCR_RSTA = 1;                               /*START BIT*/

    if (!IIC_Write (FM_ADDR | 0x1))                   /*IIC从设备地址、读*/
      goto re;

    IIC0_IBCR_TX_RX = 0;                              /*接收方式*/

    i = IIC0_IBDR;                                    /*启动接收 (第一个字节内容无效)*/

    for (i = 0; i < bytes; i++)     //lint !e838
    {
        if (!IIC_Wait (IIC0_IBSR_IBIF_MASK, IIC0_IBSR_IBIF_MASK))
          goto re;

        *((UINT8 *)buf + i) = IIC0_IBDR;
    }

    IIC0_IBCR_TXAK = 1;                               /*NACK*/

    (void) IIC_Wait (IIC0_IBSR_IBIF_MASK, IIC0_IBSR_IBIF_MASK);

    IIC0_IBCR_MS_SL = 0;                              /*STOP*/
    udelay (10);
    return 1;
}

/*****************************************************************************/
/* 铁电RAM写操作                                                             */
/*****************************************************************************/
BOOL Fm_write (UINT16 addr, const void *buf, UINT16 bytes)
{
    UINT16  i;
    
    if (bytes == 0)
      return;

    /*主模式、START BIT、发送方式*/
    IIC0_IBCR = IIC0_IBCR_IBEN_MASK | IIC0_IBCR_MS_SL_MASK | IIC0_IBCR_TX_RX_MASK;

    if (!IIC_Write (FM_ADDR | 0x0))                   /*IIC从设备地址、写*/
    {
we:     IIC0_IBCR_MS_SL = 0;                          /*STOP*/
        gSysErrFlags |= SYSERR_FRAM_WRITE_ERROR;
        return 0;
    }

    if (!IIC_Write (addr >> 8))                       /*偏移地址高字节*/
      goto we;

    if (!IIC_Write (addr & 0xff))                     /*偏移地址低字节*/
      goto we;

    for (i = 0; i < bytes; i++)
    {
        if (!IIC_Write (*((UINT8 *)buf + i)))
          goto we;
    }

    IIC0_IBCR_MS_SL = 0;                              /*STOP*/
    udelay (10);
    return 1; 
}
