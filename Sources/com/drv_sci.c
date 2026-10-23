#include "sys.h"
#include "drv_sci.h"
#include "ptl.h"
#include "modbus.h"

/*外部变量*/
#if PTL_HMI
extern ModbusFrame  HmiBuf;
#endif

/*串口发送缓冲区 */
#pragma DATA_SEG SHARED

UINT8            txBuf[BUFSZ];
volatile UINT16  txRptr;
volatile UINT16  txWptr;
volatile BOOL    txBusy;

#pragma DATA_SEG DEFAULT

/*****************************************************************************/
/* SCI控制器中断服务程序                                                     */
/*****************************************************************************/
#pragma CODE_SEG NON_BANKED
#pragma TRAP_PROC
void SCI_Handler(void)
{
    UINT8 sts = SCI0SR1;

    if (sts & 0x20)                           /*接收中断*/
    {
        UINT8 ch = SCI0DRL;

#if PTL_HMI
        Modbus_recv (&HmiBuf, ch);
#endif
    }
    else if (sts & 0x40)                      /*发送中断*/
    {
        if (txRptr != txWptr)
        {
            SCI0DRL = txBuf[txRptr];
            txRptr = (txRptr + 1) & (BUFSZ - 1);
        }
        else
        {
            txBusy = FALSE;
            
            SCI0CR2_TCIE = 0;                 /*关闭发送中断*/
            PTS_PTS4 = 0;                     /*485收发器 - 接收*/
        }
    }
}
#pragma CODE_SEG DEFAULT


/*****************************************************************************/
/* SCI发送数据数据                                                           */
/*****************************************************************************/
void  SCI_write (const void *buf, unsigned bytes)
{
    unsigned i;

    SCI0CR2_TCIE = 0;                         /*关闭发送中断*/

    for (i = 0; i < bytes; i++)
    {
        UINT8 ch = *((UINT8 *)buf + i);

        if (!txBusy)
        {
            txBusy = TRUE;
    
            PTS_PTS4 = 1;                         /* 485收发器 - 发送 */
    
            SCI0DRL = ch;
        }
        else
        {
            txBuf[txWptr] = ch;
            txWptr = (txWptr + 1) & (BUFSZ - 1);
        }
    }

    SCI0CR2_TCIE = 1;                         /*打开发送中断*/
}

/*****************************************************************************/
/* SCI控制器初始化                                                           */
/*****************************************************************************/
void  SCI_Init (void)
{
    txRptr = 0;
    txWptr = 0;
    txBusy = FALSE;

    SCI0CR2 = 0x2c;   //enable Receive Full Interrupt,RX enable,Tx enable
    SCI0BDH = 0x00;   //busclk  8MHz,19200bps,
    //SCI0BDL = 0x34;   //38400
    SCI0BDL = 0xd0;   //9600
    //SCI0BDL = 0x68;   //19200

    PTS_PTS4 = 0;     /* 485收发器 - 接收 */
}
