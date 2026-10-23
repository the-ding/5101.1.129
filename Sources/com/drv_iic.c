#include "sys.h"
#include "drv_iic.h"

 #define ACK             0
#define NACK            1
/*IIC9 - ADS1115*/  //PS0 ---SDA PS1 ---SCL
#define SCL9_LOW()      PORTB_PB2=0;
#define SCL9_HIGH()    PORTB_PB2=1; 

#define SDA9_LOW()     PORTB_PB3=0;
#define SDA9_HIGH()    PORTB_PB3=1;
#define SDA9_READ()     !!PORTB_PB3

#define SDA9_INPUT()    DDRB_DDRB3 =0;
#define SDA9_OUTPUT()   DDRB_DDRB3 =1;

#define IIC_Wait1       asm NOP
#define IIC_Wait2       asm NOP
#define IIC_Wait4       asm NOP;asm NOP
#define IIC_Wait5       asm NOP;asm NOP;asm NOP ;
#define IIC_Wait15      asm NOP;asm NOP;asm NOP ;asm NOP;asm NOP;asm NOP;asm NOP ;





/*****************************************************************************/
/* IIC控制器初始化                                                           */
/*****************************************************************************/
void  IIC_Init (void)
{
    MODRR |= 0x8;    //CAN4  PM6 ,PM7

    IIC0_IBFD = 0x09;      //baud rates = bus clock/(scl devider * 2) ;该处的波特率为32MHZ/32/2=500KHZ
    IIC0_IBAD = 0x00;      //D7-D0位是MCU作为从机时的地址,最低位不使用
    IIC0_IBCR_IBEN = 1;
}

/*****************************************************************************/
/* 轮询IIC状态寄存器                                                         */
/*****************************************************************************/
BOOL  IIC_Wait (UINT8 msk, UINT8 val)
{
    int   i;

    for (i = 0; i < 5000; i++)
    {
      if ((IIC0_IBSR & msk) == val)
      {
          if (IIC0_IBSR & IIC0_IBSR_IBIF_MASK)
            IIC0_IBSR_IBIF = 1;
          return TRUE;
      }
    }
    return FALSE;
}

/*****************************************************************************/
/* IIC发送一个字节数据，等待发送完成、等待ACK                                */
/*****************************************************************************/
BOOL IIC_Write (UINT8 ch)
{
    IIC0_IBDR = ch;
    
    return IIC_Wait (IIC0_IBSR_RXAK_MASK | IIC0_IBSR_IBIF_MASK, IIC0_IBSR_IBIF_MASK);
}




/*
 * S
 */
void IIC9_Start (void)
{
    DisableInterrupts;               /* Disable interrupts */
    SDA9_INPUT();
    SCL9_HIGH();
    
    IIC_Wait5;
    
    SDA9_LOW();
    SDA9_OUTPUT();
    
    IIC_Wait5;
    
    SCL9_LOW();
    
    //if (gSysInitDone)
    
    IIC_Wait15;
          EnableInterrupts;             /* Enable interrupts */

}

/*
 * P
 */
void IIC9_End (void)
{
    DisableInterrupts;               /* Disable interrupts */
    
    SDA9_LOW();
    SDA9_OUTPUT();
    
    SCL9_HIGH();
    
    IIC_Wait5;

    SDA9_INPUT();
    IIC_Wait5;
    
   // if (gSysInitDone)
      EnableInterrupts;             /* Enable interrupts */
    
    IIC_Wait5;
          EnableInterrupts;             /* Enable interrupts */

}

/*****************************************************************************/
/* IIC控制器初始化                                                           */
/*****************************************************************************/
void IIC9_Init (void)
{

     DDRB_DDRB2 =1;
     

    
}

/*****************************************************************************/
/* IIC发送一个字节数据，等待发送完成、等待ACK                                */
/*****************************************************************************/
BOOL IIC9_Write (UINT8 ch)
{
    UINT8   i;

    DisableInterrupts;               /* Disable interrupts */
	  SCL9_LOW();
    SDA9_OUTPUT();

    for(i = 0; i < 8; i++)
    {
        if(ch & 0x80)
        {
            SDA9_HIGH();
        }
        else
        {
            SDA9_LOW();
            
        }

        SCL9_HIGH();     /* 数据有效 */
        IIC_Wait15;
        
        ch <<= 1;

        SCL9_LOW();
        IIC_Wait5;
    }
    
    /* 发送完一个字节后检验设备的应答信号 */
    SDA9_INPUT();

    SCL9_HIGH();         /* 停止驱动，数据有效 */
    IIC_Wait5;

    i = SDA9_READ();
    SCL9_LOW();

    IIC_Wait15;

  //  if (gSysInitDone)
      EnableInterrupts;             /* Enable interrupts */
    return i == ACK;
}

/*****************************************************************************/
/* IIC接收一个字节数据                                                       */
/*****************************************************************************/
UINT8 IIC9_Read (BOOL Ack)
{
    UINT8   ch = 0;
    UINT8   i;

    DisableInterrupts;               /* Disable interrupts */
    SCL9_LOW();
    SDA9_INPUT();
    
    for(i = 0; i < 8; i++)
    {
        IIC_Wait15;

        SCL9_HIGH();                 /* 停止驱动，数据有效 */
        
        IIC_Wait5;

        ch <<= 1;
        if(SDA9_READ())
            ch |= 1;

        SCL9_LOW();
    }

    if (Ack == ACK)
    {
        SDA9_LOW();
        SDA9_OUTPUT();
    }
    else
        SDA9_INPUT();

    SCL9_HIGH();
    IIC_Wait5;

	  SCL9_LOW();
	  IIC_Wait15;

   // if (gSysInitDone)
	    EnableInterrupts;             /* Enable interrupts */
    return ch;
}



















