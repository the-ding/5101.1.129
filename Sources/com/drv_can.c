#include "sys.h"
#include "drv_can.h"
#include "ptl.h"
#include "db.h"

#define BTR0_500K   0x43  /* 16M/(1+3) = 4M*/
#define BTR1_500K   0x14  /* 4M/[1+(1+1)+(1+4)] = 500K*/

#define BTR0_250K   0x47  /* 16M/(1+7) = 2M*/
#define BTR1_250K   0x14  /* 2M/[1+(1+1)+(1+4)] = 250K*/

#pragma DATA_SEG SHARED_DATA
volatile UINT8  CAN0_RxRPtr = 0;
volatile UINT8  CAN0_RxWPtr = 0;
CanbusFrame     CAN0_RxBuf[CAN0_RXBSZ];

volatile UINT8  CAN1_RxRPtr=0;
volatile UINT8  CAN1_RxWPtr=0;
CanbusFrame     CAN1_RxBuf[CAN1_RXBSZ];

volatile UINT8  CAN4_RxRPtr=0;
volatile UINT8  CAN4_RxWPtr=0;
CanbusFrame     CAN4_RxBuf[CAN4_RXBSZ];

volatile UINT8  OTA_can_buf[200];

#pragma DATA_SEG DEFAULT

#pragma CONST_SEG NON_BANKED
extern const char _Startup[];
#pragma CONST_SEG DEFAULT

void SoftUpdate (int can, CanbusFrame *cf)
{
    UINT8   tbsel;
    volatile UINT8 *reg;

    if (*(UINT16 *)0xFFFE == (UINT16)&_Startup)         // 调试版本
        return;

    switch (can)
    {
    case 0: reg = (volatile UINT8 *)0x140; break;
    case 1: reg = (volatile UINT8 *)0x180; break;
    case 2: reg = (volatile UINT8 *)0x1c0; break;
    case 3: reg = (volatile UINT8 *)0x200; break;
    case 4: reg = (volatile UINT8 *)0x280; break;
    default: return;
    }

    if (cf->len == 8 && cf->data[0] == 0xde && cf->data[1] == 0xad && cf->data[2] == 'H' && cf->data[3] == 'C')
    {
        if (*(UINT16 *)&cf->data[6] != SWVer &&
            ((cf->data[4] == '1' && cf->data[5] == '2') || (cf->data[4] == 'x' && cf->data[5] == 'x')))
        {
            DisableInterrupts;

            *(volatile unsigned int *)0x3FF0 = 0xDEAD;
            *(volatile unsigned int *)0x3FF2 = 0xBEEF;

            *(volatile long *far)0x200001 = 0;  //lint !e511
            *(volatile long *far)0x300001 = 0;  //lint !e511
        }
        else if (cf->data[4] == '1' && cf->data[5] == '2' && *(UINT16 *)&cf->data[6] == SWVer)
        {
            cf->id = 0xaaa;
            cf->len = 0;

            goto reply;
        }
        else if (cf->data[4] == 'x' && cf->data[5] == 'x' && *(UINT16 *)&cf->data[6] == SWVer)
        {
            cf->id = *(UINT32 *)0xFFA0 & 0x3fffff;
            cf->len = 8;
            *(UINT32 *)&cf->data[0] = *(UINT32 *)0xFFA0;
            *(UINT16 *)&cf->data[4] = *(UINT16 *)0xdc00;     //SWVer
            *(UINT16 *)&cf->data[6] = *(UINT16 *)0xdc02;     //HWVer

reply:      reg[0xA]/*CANxTBSEL*/ = reg[0x6]/*CANxTFLG*/;
            tbsel = reg[0xA]/*CANxTBSEL*/;

            reg[0x30]/*CANxTXIDR0*/ = (UINT8)(cf->id >> 21);
            reg[0x31]/*CANxTXIDR1*/ = (UINT8)(((cf->id >> 18) & 0x07) << 5) | ((cf->id >> 15) & 0x07) | 0x18;
            reg[0x32]/*CANxTXIDR2*/ = (UINT8)(cf->id >> 7);
            reg[0x33]/*CANxTXIDR3*/ = (UINT8)(cf->id << 1);

            *(UINT16 *)&reg[0x34]/*CANxTXDSR0*/ = *(UINT16 *)&cf->data[0];
            *(UINT16 *)&reg[0x36]/*CANxTXDSR2*/ = *(UINT16 *)&cf->data[2];
            *(UINT16 *)&reg[0x38]/*CANxTXDSR4*/ = *(UINT16 *)&cf->data[4];
            *(UINT16 *)&reg[0x3a]/*CANxTXDSR6*/ = *(UINT16 *)&cf->data[6];

            reg[0x3C]/*CANxTXDLR*/  = cf->len;
            reg[0x3D]/*CANxTXTBPR*/ = 0;
            reg[0x6]/*CANxTFLG*/   = tbsel;
            while (reg[0x6] != 0x7);
        }
    }
}

/*****************************************************************************/
/* 查询CAN0控制器发送缓冲区是否空                                            */
/*****************************************************************************/
BOOL CAN0_IsBusy (void)
{
    return CAN0TFLG == 0x0;
}

/*****************************************************************************/
/* 查询CAN1控制器发送缓冲区是否空                                            */
/*****************************************************************************/
BOOL CAN1_IsBusy (void)
{
    return CAN1TFLG == 0x0;
}

/*****************************************************************************/
/* 查询CAN4控制器发送缓冲区是否空                                            */
/*****************************************************************************/
BOOL CAN4_IsBusy (void)
{
    return CAN4TFLG == 0x0;
}

/*****************************************************************************/
/* CAN0控制器发送数据帧                                                      */
/*****************************************************************************/
void CAN0_write (const CanbusFrame *cf)
{
    UINT8   tbsel;

    CAN0TBSEL = CAN0TFLG;
    tbsel = CAN0TBSEL;

    CAN0TXIDR0 = (UINT8)(cf->id >> 21);
    CAN0TXIDR1 = (UINT8)(((cf->id >> 18) & 0x07) << 5) | ((cf->id >> 15) & 0x07) | 0x18;
    CAN0TXIDR2 = (UINT8)(cf->id >> 7);
    CAN0TXIDR3 = (UINT8)(cf->id << 1);

    CAN0TXDSR0 = cf->data[0];
    CAN0TXDSR1 = cf->data[1];
    CAN0TXDSR2 = cf->data[2];
    CAN0TXDSR3 = cf->data[3];
    CAN0TXDSR4 = cf->data[4];
    CAN0TXDSR5 = cf->data[5];
    CAN0TXDSR6 = cf->data[6];
    CAN0TXDSR7 = cf->data[7];

    CAN0TXDLR  = cf->len;
    CAN0TXTBPR = cf->pri;

    CAN0TFLG   = tbsel;
//    while (CAN0TFLG != 0x7);
}

/*****************************************************************************/
/* CAN0控制器发送标准数据帧                                                  */
/*****************************************************************************/
void CAN0_write_std(const CanbusFrame *cf)
{
    UINT8   tbsel;

    CAN0TBSEL = CAN0TFLG;
    tbsel = CAN0TBSEL;

    CAN0TXIDR0 = (UINT8)(cf->id >> 3);
    CAN0TXIDR1 = (UINT8)(cf->id & 0x7) << 5;

    CAN0TXDSR0 = cf->data[0];
    CAN0TXDSR1 = cf->data[1];
    CAN0TXDSR2 = cf->data[2];
    CAN0TXDSR3 = cf->data[3];
    CAN0TXDSR4 = cf->data[4];
    CAN0TXDSR5 = cf->data[5];
    CAN0TXDSR6 = cf->data[6];
    CAN0TXDSR7 = cf->data[7];

    CAN0TXDLR  = cf->len;
    CAN0TXTBPR = cf->pri;

    CAN0TFLG   = tbsel;

}

/*****************************************************************************/
/* CAN1控制器发送数据帧                                                      */
/*****************************************************************************/
void CAN1_write (const CanbusFrame *cf)
{
    UINT8   tbsel;

    CAN1TBSEL = CAN1TFLG;
    tbsel = CAN1TBSEL;

    CAN1TXIDR0 = (UINT8)(cf->id >> 21);
    CAN1TXIDR1 = (UINT8)(((cf->id >> 18) & 0x07) << 5) | ((cf->id >> 15) & 0x07) | 0x18;
    CAN1TXIDR2 = (UINT8)(cf->id >> 7);
    CAN1TXIDR3 = (UINT8)(cf->id << 1);

    CAN1TXDSR0 = cf->data[0];
    CAN1TXDSR1 = cf->data[1];
    CAN1TXDSR2 = cf->data[2];
    CAN1TXDSR3 = cf->data[3];
    CAN1TXDSR4 = cf->data[4];
    CAN1TXDSR5 = cf->data[5];
    CAN1TXDSR6 = cf->data[6];
    CAN1TXDSR7 = cf->data[7];

    CAN1TXDLR  = cf->len;
    CAN1TXTBPR = cf->pri;

    CAN1TFLG   = tbsel;
}
/*****************************************************************************/
/* CAN1控制器发送标准数据帧                                                  */
/*****************************************************************************/
void CAN1_write_std(const CanbusFrame *cf)
{

    UINT8   tbsel;

    CAN1TBSEL = CAN1TFLG;
    tbsel = CAN1TBSEL;


    CAN1TXIDR0 = (UINT8)(cf->id >> 3);
    CAN1TXIDR1 = (UINT8)(cf->id & 0x7) << 5;


    CAN1TXDSR0 = cf->data[0];
    CAN1TXDSR1 = cf->data[1];
    CAN1TXDSR2 = cf->data[2];
    CAN1TXDSR3 = cf->data[3];
    CAN1TXDSR4 = cf->data[4];
    CAN1TXDSR5 = cf->data[5];
    CAN1TXDSR6 = cf->data[6];
    CAN1TXDSR7 = cf->data[7];

    CAN1TXDLR  = cf->len;
    CAN1TXTBPR = cf->pri;

    CAN1TFLG   = tbsel;

}


/*****************************************************************************/
/* CAN4控制器发送数据帧                                                      */
/*****************************************************************************/
void CAN4_write (const CanbusFrame *cf)
{
    UINT8   tbsel;

    CAN4TBSEL = CAN4TFLG;
    tbsel = CAN4TBSEL;

    CAN4TXIDR0 = (UINT8)(cf->id >> 21);
    CAN4TXIDR1 = (UINT8)(((cf->id >> 18) & 0x07) << 5) | ((cf->id >> 15) & 0x07) | 0x18;
    CAN4TXIDR2 = (UINT8)(cf->id >> 7);
    CAN4TXIDR3 = (UINT8)(cf->id << 1);

    CAN4TXDSR0 = cf->data[0];
    CAN4TXDSR1 = cf->data[1];
    CAN4TXDSR2 = cf->data[2];
    CAN4TXDSR3 = cf->data[3];
    CAN4TXDSR4 = cf->data[4];
    CAN4TXDSR5 = cf->data[5];
    CAN4TXDSR6 = cf->data[6];
    CAN4TXDSR7 = cf->data[7];

    CAN4TXDLR  = cf->len;
    CAN4TXTBPR = cf->pri;

    CAN4TFLG   = tbsel;
}


/*****************************************************************************/
/* CAN4控制器发送标准数据帧                                                  */
/*****************************************************************************/
void CAN4_write_std(const CanbusFrame *cf)
{
    UINT8   tbsel;

    CAN4TBSEL = CAN4TFLG;
    tbsel = CAN4TBSEL;

    CAN4TXIDR0 = (UINT8)(cf->id >> 3);
    CAN4TXIDR1 = (UINT8)(cf->id & 0x7) << 5;

    CAN4TXDSR0 = cf->data[0];
    CAN4TXDSR1 = cf->data[1];
    CAN4TXDSR2 = cf->data[2];
    CAN4TXDSR3 = cf->data[3];
    CAN4TXDSR4 = cf->data[4];
    CAN4TXDSR5 = cf->data[5];
    CAN4TXDSR6 = cf->data[6];
    CAN4TXDSR7 = cf->data[7];

    CAN4TXDLR  = cf->len;
    CAN4TXTBPR = cf->pri;

    CAN4TFLG   = tbsel;

}



/*****************************************************************************/
/* CAN0控制器初始化                                                          */
/*****************************************************************************/
static void CAN0_Init (void)
{
    CAN0CTL0 = 0x01;            /* Enter Initialization Mode
                                 *
                                 *  0b00000001
                                 *    ||||||||__ Enter Initialization Mode
                                 *    |||||||___ Sleep Mode Request bit
                                 *    ||||||____ Wake-Up disabled
                                 *    |||||_____ Time stamping disabled
                                 *    ||||______ Synchronized Status
                                 *    |||_______ CAN not affected by Wait
                                 *    ||________ Receiver Active Status bit
                                 *    |_________ Received Frame Flag bit
                                 */
    while(CAN0CTL1_INITAK!=1) {}// Wait for Initialization Mode acknowledge INITRQ bit = 1
    CAN0CTL1 = 0x80;            /* Enable MSCAN module and not LoopBack Mode
                                 *
                                 *  0b10100000
                                 *    ||||||||__ Initialization Mode Acknowledge
                                 *    |||||||___ Sleep Mode Acknowledge
                                 *    ||||||____ Wake-up low-pass filter disabled
                                 *    |||||_____ Unimplemented
                                 *    ||||______ Listen Only Mode disabled
                                 *    |||_______ not Loop Back Mode enabled
                                 *    ||________ Ext Osc/Xtal as Clock Source
                                 *    |_________ MSCAN Module enabled
                                 */
    //#ifdef demo
    CAN0BTR0 = BTR0_250K;

                               /* Synch Jump = 2 Tq clock Cycles
                                *
                                 *  0b01000011
                                 *    ||||||||__
                                 *    |||||||___\
                                 *    ||||||____ |
                                 *    |||||_____ |_ CAN Clock Prescaler = 4
                                 *    ||||______ |
                                 *    |||_______/
                                 *    ||________
                                 *    |_________>- SJW = 2   //0
                                 */
     CAN0BTR1 = BTR1_250K;
                  /* Set Number of samples per bit, TSEG1 and TSEG2
                                 * bit rate=Fclk/pres v/(1+TSEG1+TSEG2)=16M/4/(1+2+5)=500kbps
                                 *  0b00010100
                                 *    ||||||||__
                                 *    |||||||___\
                                 *    ||||||____ |- TSEG1 = 5
                                 *    |||||_____/
                                 *    ||||______
                                 *    |||_______\_ TSEG2 = 2
                                 *    ||________/
                                 *    |_________ One sample per bit
                                 */
     CAN0IDAC = 0x00;           /* Set two 32-bit Filters
                                 *
                                 *  0b00000000
                                 *    ||||||||__
                                 *    |||||||___\_ Filter Hit Indicator
                                 *    ||||||____/
                                 *    |||||_____ Unimplemented
                                 *    ||||______
                                 *    |||_______>- two 32-bit Acceptance Filters
                                 *    ||________
                                 *    |_________>- Unimplemented
                                 */
    /* note Acceptance Filters neither Acceptance Filter is accorded with,message in receivebuffer will pass  */
    CAN0IDAR0 = 1;   // |\    32 bit Filter 0
    CAN0IDAR1 = 2;   // | \__ Accepts Extended Data Frame Msg
    CAN0IDAR2 = 3;   // | /   with ID: CAN0_ID
    CAN0IDAR3 = 4;   // |/
    CAN0IDAR4 = 1;   // |\    32 bit Filter 0
    CAN0IDAR5 = 2;   // | \__ Accepts Extended Data Frame Msg
    CAN0IDAR6 = 3;   // | /   with ID: CAN0_ID
    CAN0IDAR7 = 4;   // |/
    CAN0IDMR0 = 0XBF;
    CAN0IDMR1 = 0XFF;
    CAN0IDMR2 = 0XFF;
    CAN0IDMR3 = 0XFF;
    CAN0IDMR4 = 0XBF;
    CAN0IDMR5 = 0XFF;
    CAN0IDMR6 = 0XFF;
    CAN0IDMR7 = 0XFF;

    CAN0CTL0 = 0x00;                /* Exit Initialization Mode Request */
    while ((CAN0CTL1&0x01)!=0) {}   // 等待，直到MSCAN正常运行
    //while(!(CAN0CTL0&0x10)) {}      // 等待，直到MSCAN与CAN总线同步
    CAN0RFLG = 0xC3;          /* Reset Receiver Flags
                               *
                               *  0b11000011
                               *    ||||||||__ Receive Buffer Full Flag
                               *    |||||||___ Overrun Interrupt Flag
                               *    ||||||____
                               *    |||||_____>- Transmitter Status Bits
                               *    ||||______
                               *    |||_______>- Receiver Status Bits
                               *    ||________ CAN Status Change Interrupt Flag
                               *    |_________ Wake-Up Interrupt Flag
                               */
    CAN0RIER = 0x01;
                               /* Enable Receive Buffer Full Interrupt
                               *
                               *  0b00000001
                               *    ||||||||__ Receive Buffer Full Int enabled
                               *    |||||||___ Overrun Int disabled
                               *    ||||||____
                               *    |||||_____>- Tx Status Change disabled
                               *    ||||______
                               *    |||_______>- Rx Status Change disabled
                               *    ||________ Status Change Int disabled
                               *    |_________ Wake-Up Int disabled
                               */
}

/*****************************************************************************/
/* CAN1控制器初始化                                                          */
/*****************************************************************************/
static void CAN1_Init(void)
{
    CAN1CTL0 = 0x01;            /* Enter Initialization Mode
                                 *
                                 *  0b00000001
                                 *    ||||||||__ Enter Initialization Mode
                                 *    |||||||___ Sleep Mode Request bit
                                 *    ||||||____ Wake-Up disabled
                                 *    |||||_____ Time stamping disabled
                                 *    ||||______ Synchronized Status
                                 *    |||_______ CAN not affected by Wait
                                 *    ||________ Receiver Active Status bit
                                 *    |_________ Received Frame Flag bit
                                 */
    while(CAN1CTL1_INITAK!=1) {}// Wait for Initialization Mode acknowledge INITRQ bit = 1
    CAN1CTL1 = 0x80;            /* Enable MSCAN module and not LoopBack Mode
                                 *
                                 *  0b10100000
                                 *    ||||||||__ Initialization Mode Acknowledge
                                 *    |||||||___ Sleep Mode Acknowledge
                                 *    ||||||____ Wake-up low-pass filter disabled
                                 *    |||||_____ Unimplemented
                                 *    ||||______ Listen Only Mode disabled
                                 *    |||_______ not Loop Back Mode enabled
                                 *    ||________ Ext Osc/Xtal as Clock Source
                                 *    |_________ MSCAN Module enabled
                                 */
   
      //250kbps
      CAN1BTR0 = BTR0_250K;

                                 /* Synch Jump = 2 Tq clock Cycles
                                  *
                                   *  0b01000011
                                   *    ||||||||__
                                   *    |||||||___\
                                   *    ||||||____ |
                                   *    |||||_____ |_ CAN Clock Prescaler = 4
                                   *    ||||______ |
                                   *    |||_______/
                                   *    ||________
                                   *    |_________>- SJW = 2
                                   */
      CAN1BTR1 = BTR1_250K;       /* Set Number of samples per bit, TSEG1 and TSEG2
                                   * bit rate=Fclk/pres v/(1+TSEG1+TSEG2)=16M/4/(1+2+5)=500kbps
                                   *  0b00010100
                                   *    ||||||||__
                                   *    |||||||___\
                                   *    ||||||____ |- TSEG1 = 5
                                   *    |||||_____/
                                   *    ||||______
                                   *    |||_______\_ TSEG2 = 2
                                   *    ||________/
                                   *    |_________ One sample per bit
                                   */

    
     

    
     CAN1IDAC = 0x00;           /* Set two 32-bit Filters
                                 *
                                 *  0b00000000
                                 *    ||||||||__
                                 *    |||||||___\_ Filter Hit Indicator
                                 *    ||||||____/
                                 *    |||||_____ Unimplemented
                                 *    ||||______
                                 *    |||_______>- two 32-bit Acceptance Filters
                                 *    ||________
                                 *    |_________>- Unimplemented
                                 */
    /* note Acceptance Filters neither Acceptance Filter is accorded with,message in receivebuffer will pass  */
/*   
    CAN1IDMR0 = 0;
    CAN1IDMR1 = 0;
    CAN1IDMR2 = 0;
    CAN1IDMR3 = 0;
    CAN1IDMR4 = 0;
    CAN1IDMR5 = 0;
    CAN1IDMR6 = 0;
    CAN1IDMR7 = 0; 
    
  
    CAN1IDAR0 = 0xc0;   // |\    32 bit Filter 0
    CAN1IDAR1 = 0x1b;   // | \__ Accepts Extended Data Frame Msg
    CAN1IDAR2 = 0xc1;   // | /   with ID: CAN0_ID
    CAN1IDAR3 = 0x82;   // |/
    
    CAN1IDMR0 = 0;
    CAN1IDMR1 = 0x18;
    CAN1IDMR2 = 0;
    CAN1IDMR3 = 0; 
    
    CAN1IDAR4 = 0xe0;   // |\    32 bit Filter 0
    CAN1IDAR5 = 0x3A;   // | \__ Accepts Extended Data Frame Msg
    CAN1IDAR6 = 0x00;   // | /   with ID: CAN0_ID
    CAN1IDAR7 = 0x1e;   // |/
    
    CAN1IDMR4 = 0;
    CAN1IDMR5 = 0x18;
    CAN1IDMR6 = 0;
    CAN1IDMR7 = 0x1F;   
 //*/   
// /*    
    CAN1IDAR0 = 0xFF;   // |\    32 bit Filter 0
    CAN1IDAR1 = 0XFF;   // | \__ Accepts Extended Data Frame Msg
    CAN1IDAR2 = 0XFF;   // | /   with ID: CAN0_ID
    CAN1IDAR3 = 0XFF;   // |/
    CAN1IDAR4 = 0xFF;   // |\    32 bit Filter 0
    CAN1IDAR5 = 0XFF;   // | \__ Accepts Extended Data Frame Msg
    CAN1IDAR6 = 0XFF;   // | /   with ID: CAN0_ID
    CAN1IDAR7 = 0XFF;   // |/
    CAN1IDMR0 = 0xFF;//0XBF;
    CAN1IDMR1 = 0XFF;
    CAN1IDMR2 = 0XFF;
    CAN1IDMR3 = 0XFF;
    CAN1IDMR4 = 0xFF;//0XBF;
    CAN1IDMR5 = 0XFF;
    CAN1IDMR6 = 0XFF;
    CAN1IDMR7 = 0XFF;  
//   */
    CAN1CTL0 = 0x00;                /* Exit Initialization Mode Request */
    while ((CAN1CTL1&0x01)!=0) {}   // 等待，直到MSCAN正常运行
    //while(!(CAN1CTL0&0x10)) {}      // 等待，直到MSCAN与CAN总线同步
    CAN1RFLG = 0xC3;          /* Reset Receiver Flags
                               *
                               *  0b11000011
                               *    ||||||||__ Receive Buffer Full Flag
                               *    |||||||___ Overrun Interrupt Flag
                               *    ||||||____
                               *    |||||_____>- Transmitter Status Bits
                               *    ||||______
                               *    |||_______>- Receiver Status Bits
                               *    ||________ CAN Status Change Interrupt Flag
                               *    |_________ Wake-Up Interrupt Flag
                               */
    CAN1RIER = 0x01;
                               /* Enable Receive Buffer Full Interrupt
                               *
                               *  0b00000001
                               *    ||||||||__ Receive Buffer Full Int enabled
                               *    |||||||___ Overrun Int disabled
                               *    ||||||____
                               *    |||||_____>- Tx Status Change disabled
                               *    ||||______
                               *    |||_______>- Rx Status Change disabled
                               *    ||________ Status Change Int disabled
                               *    |_________ Wake-Up Int disabled
                               */
}

/*****************************************************************************/
/* CAN4控制器初始化                                                          */
/*****************************************************************************/
static void CAN4_Init (void)
{
    MODRR |= 0x8;       //CAN4  PM6 ,PM7

    CAN4CTL0 = 0x01;
    while(CAN4CTL1_INITAK!=1) {} // Wait for Initialization Mode acknowledge INITRQ bit = 1

    CAN4CTL1 = 0x80;

    CAN4BTR0 = BTR0_250K;
    CAN4BTR1 = BTR1_250K;

    CAN4IDAC_IDAM0 = 0;                 // 2 32-bit Acceptance Filters
    /* note Acceptance Filters neither Acceptance Filter is accorded with,message in receivebuffer will pass  */

    CAN4IDAR0 = 0Xff;   // |
    CAN4IDAR1 = 0Xff;   //  CAN ID 0X1XX
    CAN4IDAR2 = 0Xff;   // |
    CAN4IDAR3 = 0Xff;   //  CAN ID 0X2xx

    CAN4IDAR4 = 0xff;   // |
    CAN4IDAR5 = 0Xff;   //  CAN ID 0x4xx & 0x5xx & 0x6xx
    CAN4IDAR6 = 0xff;   // |
    CAN4IDAR7 = 0Xff;   //  CAN ID  ALL

    CAN4IDMR0 = 0XFF;
    CAN4IDMR1 = 0XFF;
    CAN4IDMR2 = 0XFF;
    CAN4IDMR3 = 0XFF;

    CAN4IDMR4 = 0XFF;
    CAN4IDMR5 = 0XFF;
    CAN4IDMR6 = 0XFF;
    CAN4IDMR7 = 0XFF;

    CAN4CTL0 = 0x00;                /* Exit Initialization Mode Request */
    while ((CAN4CTL1&0x01)!=0) {}   // 等待，直到MSCAN正常运行
    //while(!(CAN4CTL0&0x10)) {}      // 等待，直到MSCAN与CAN总线同步
    CAN4RFLG = 0xC3;
    CAN4RIER = 0x01;
}

/*****************************************************************************/
/* CAN控制器初始化                                                           */
/*****************************************************************************/
void CAN_Init (void)
{
    UINT32  i;

    CAN0_RxRPtr = 0;
    CAN1_RxRPtr = 0;
    CAN4_RxRPtr = 0;
    CAN0_RxWPtr = 0;
    CAN1_RxWPtr = 0;
    CAN4_RxWPtr = 0;

    CAN0_Init ();
    CAN1_Init ();
    CAN4_Init ();

    /*防止CAN总线接在一起导致初始化失败*/
    i = 0;
    while(!(CAN0CTL0&0x10)) // 等待，直到MSCAN与CAN总线同步
    {
        if (i++ > 300000UL)
        {
            while (!(CAN0CTL0&0x10))
            {
                PTP_PTP1 = 0;
                mdelay (150);
                PTP_PTP1 = 1;
                mdelay (800);
            }
        }
    }
    
    i = 0;
    while(!(CAN1CTL0&0x10)) // 等待，直到MSCAN与CAN总线同步
    {
        if (i++ > 300000UL)
        {
            while (!(CAN1CTL0&0x10))
            {
                PTP_PTP1 = 0;
                mdelay (150);
                PTP_PTP1 = 1;
                mdelay (150);
                PTP_PTP1 = 0;
                mdelay (150);
                
                PTP_PTP1 = 1;
                mdelay (800);
            }
        }
    }
    
    i = 0;
    while(!(CAN4CTL0&0x10)) // 等待，直到MSCAN与CAN总线同步
    {
        if (i++ > 300000UL)
        {
            while (!(CAN4CTL0&0x10))
            {
                PTP_PTP1 = 0;
                mdelay (150);
                PTP_PTP1 = 1;
                mdelay (150);
                PTP_PTP1 = 0;
                mdelay (150);
                PTP_PTP1 = 1;
                mdelay (150);
                PTP_PTP1 = 0;
                mdelay (150);
                
                PTP_PTP1 = 1;
                mdelay (800);
            }
        }
    }
}

/*****************************************************************************/
/* CAN通信帧处理                                                             */
/*****************************************************************************/
void CAN_main (void)
{
    if (CAN1_RxRPtr != CAN1_RxWPtr)
    {
        CanbusFrame *cf = &CAN1_RxBuf[CAN1_RxRPtr];

        if (++cf->pri > 2)        /*没有其他模块处理该报文，删除*/
          CAN1_RxRPtr = (CAN1_RxRPtr + 1) & (CAN1_RXBSZ - 1);
    }

    if (CAN0_RxRPtr != CAN0_RxWPtr)
    {
        CanbusFrame *cf = &CAN0_RxBuf[CAN0_RxRPtr];

        //if(Read_flag !=1) {//互斥锁-防止实时读取时进入更新模式
            if (cf->id == 0x555){
                SoftUpdate (0, cf);
            }
        //}
        if (++cf->pri > 2)        /*没有其他模块处理该报文，删除*/
          CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);
    }

    if (CAN4_RxRPtr != CAN4_RxWPtr)
    {
        CanbusFrame *cf = &CAN4_RxBuf[CAN4_RxRPtr];

        if (++cf->pri > 6)        /*没有其他模块处理该报文，删除*/
          CAN4_RxRPtr = (CAN4_RxRPtr + 1) & (CAN4_RXBSZ - 1);
    }
}
