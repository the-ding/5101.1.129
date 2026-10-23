#include <string.h>
#include "sys.h"
#include "xgate.h"
#include "drv_iic.h"
#include "drv_ads11x.h"

#include "drv_spi.h"
#include "drv_rtc.h"
#include "drv_fm.h"
#include "drv_sci.h"
#include "drv_can.h"
#include "drv_adc.h"
#include "drv_flash.h"
#include "ptl.h"
#include "db.h"



#define ROUTE_INTERRUPT(vec_adr, cfdata)                \
    INT_CFADDR= (vec_adr) & 0xF0;                       \
    INT_CFDATA_ARR[((vec_adr) & 0x0F) >> 1]= (cfdata)   //lint !e415

/*****************************************************************************/
/* XGATE协处理器、中断向量初始化                                             */
/*****************************************************************************/
static void SetupXGATE (void)
{
    /* initialize the XGATE vector block and set the XGVBR register to its start address */
    XGVBR = (unsigned int)(void*__far)(XGATE_VectorTable - XGATE_VECTOR_OFFSET);

    //ROUTE_INTERRUPT(0x90, 0x82);                          //can4 Tx
    ROUTE_INTERRUPT(0x92, 0x82); /* RQST=1 and PRIO=1  l*/  //can4 rx

    //ROUTE_INTERRUPT(0xa8, 0x83);                          //can1 tx
    ROUTE_INTERRUPT(0xaa, 0x83);                            //can1 rx

    //ROUTE_INTERRUPT(0xb0, 0x83);                          //can0 tx
    ROUTE_INTERRUPT(0xb2, 0x83);                            //can0 rx

    ROUTE_INTERRUPT(0x72, 0x81); /* RQST=1 and PRIO=2  h*/  //soft int0

    ROUTE_INTERRUPT(0x74, 0x82); /* RQST=1 and PRIO=2  h*/  //pit3
    ROUTE_INTERRUPT(0x76, 0x82); /* RQST=1 and PRIO=2  h*/  //pit2   proc

    /* enable XGATE mode and interrupts */
    XGMCTL = 0xFBC1; /* XGE | XGFRZ | XGIE */

    /* force execution of software trigger 0 handler */
    //XGSWT= 0x0101;
}

/*****************************************************************************/
/* IO端口初始化                                                              */
/*****************************************************************************/
static void IO_Init (void)
{
   
    DDRT_DDRT1 = 1;     // OUT0
    DDRT_DDRT2 = 1;     // OUT1
    DDRT_DDRT4 = 1;     // OUT2
    DDRT_DDRT6 = 1;     // OUT3
    DDRT_DDRT7 = 1;     // OUT4
    DDRB_DDRB1 = 1;     // OUT5

    DDRP_DDRP0 = 0;     // key
    DDRP_DDRP1 = 0;     // hvil
    DDRB_DDRB0 = 0;     // cc2
    DDRT_DDRT0 = 0;     // IN2
    DDRT_DDRT3 = 0;     // IN1
    DDRT_DDRT5 = 0;     // in0
    
    
    RDRP_RDRP0 = 1;     // key
    RDRP_RDRP1 = 1;     // hvil
    
/////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

    PTT_PTT1 = 1;       // OUT0
    PTT_PTT2 = 1;       // OUT1
    PTT_PTT4 = 1;       // OUT2
    PTT_PTT6 = 1;       // OUT3
    PTT_PTT7 = 1;       // OUT4
    PORTB_PB1 = 1;      // OUT5

    
    /*if(!OUT_chgnum[OUT4])
        OUTchg_reson |= 0x4000;      // 上电流程
     */

    DDRS_DDRS4= 1;      //RS485  D/R   UART0
    PTS_PTS4  = 0;      //RECEVE MODE
   
    DDRP_DDRP2=1;   
    PTP_PTP2 = 1;       //run led
    
    
    
    DDR1AD0  = 0;   //PAD 0---7 
    
    
    
}

/*****************************************************************************/
/* 设置系统、总线时钟频率                                                    */
/*****************************************************************************/
static void SetBusCLK_32M(void)
{
    CLKSEL=0x00;        //disengage PLL to system
    PLLCTL_PLLON=1;     //turn on PLL
    SYNR=3;
    REFDV=1;            //pllclock=2*osc*(1+SYNR)/(1+REFDV)=64MHz;
    _asm(nop);          //BUS CLOCK=32M
    _asm(nop);
    while(!(CRGFLG_LOCK==1)) {} //when pll is steady ,then use it;
    CLKSEL_PLLSEL =1;           //engage PLL to system;
}

/*****************************************************************************/
/* 设置系统滴答中断频率                                                      */
/*****************************************************************************/
static void SystemTick_Init (void)     //16m clk
{
  //  RTICTL  = 0xBF;     //160*10^3  ==  10MS
     RTICTL  = 0x9F;    //32*10^3  ==  2MS

    _asm(nop);

    CRGINT_RTIE = 1;    //ENABLE INT                                   
    _asm(nop);          //BUS CLOCK=32M
}
 
/*****************************************************************************/
/* 初始化周期定时器4，由XGATE周期招测采集板的实时数据                        */
/*****************************************************************************/
/*static void Timer_2ms (void)
{
    //PITCFLMT_PITE=1;          //使能PIT 
    PITCE_PCE2=1;             //使能16位通道3
    PITMUX_PMUX2=0;           //选择8位Counter base通道)
                              //计时总时长=8位counter  *  16位counter
                              //time-out period = (PITMTLD + 1) * (PITLD + 1) / fBUS.
    PITMTLD0 = 2;
    PITLD2 = 1000;              // 31.25ns * 200 * 4000 ~= 25 ms    (fBUS = 32MHz) 
                              // 2ms
    PITFLT_PFLT2=1;           //强制Load 16位counter值进入DOWN-COUNTER
    PITCFLMT_PFLMT0=1;        //强制Load 8  位counter值进入DOWN-COUNTER
}*/

/*****************************************************************************/
/* 配置PWM                                                                   */
/*****************************************************************************/
static void PWM_Init(void)
{
    PWME      =0X00;                     //禁止PWM输出
    PWMCTL    =0X00;                   //通道不级联
    PWMPRCLK  =0X44;                   //BUS clock预分频系数16,即ClockA=ClockB=Bus/16 = 2000 000 Hz
    PWMCLK    = 0;//0Xff;                   //时钟来源选择ClockSA、ClockSB
    PWMPOL    =0;//0XFF;                   //PWM所有通道输出起始为高电平
    PWMCAE    =0X00;                   //PWM所有通道输出为左对齐方式
    PWMPER0   =0xfa;//0XC8;                     //输出周期=(1/20000)*2*PWMPER*1000(s/ms)=20 ms

    PWMDTY0 = 0;
    PWME = 0x01;
}

/*****************************************************************************/
/* 系统时间初始化                                                            */
/*****************************************************************************/
static void Time_Init (void)
{
    struct tm t;

    (void) memset (&t, 0, sizeof(t));

    if (!RTC_read (&t) ||
      t.tm_mon > 12 ||
      t.tm_mday > 31 ||
      t.tm_hour > 23 ||
      t.tm_min > 59 ||
      t.tm_sec > 59)
    {
        gSysErrFlags |= SYSERR_TIME_LOST_ERROR;

        /*设置系统缺省时间*/
        t.tm_year = 2011 - 1900;
        t.tm_mon  = 9;
        t.tm_mday = 1;
        t.tm_wday = 5;
        t.tm_hour = 0;
        t.tm_min  = 0;
        t.tm_sec  = 0;
    }

  /*gSysSec = 0x4E85E780; */    /* 2011-10-1 0:0:0 */
    gSysSec = mktime (&t);
}

/*****************************************************************************/
/* 初始化周期定时器4，由XGATE周期招测采集板的实时数据                        */
/*****************************************************************************/
static void Protec_Timer (void)
{
    //PITCFLMT_PITE=1;          /*使能PIT */
    PITCE_PCE2=1;             /*使能16位通道3*/
    PITMUX_PMUX2=0;           /*选择8位Counter base通道)*/
                              /*计时总时长=8位counter  *  16位counter*/
                              /*time-out period = (PITMTLD + 1) * (PITLD + 1) / fBUS. */
    PITMTLD0 = 250;
    PITLD2 = 60000;              /* 31.25ns * 200 * 4000 ~= 25 ms    (fBUS = 32MHz) */

    PITFLT_PFLT2=1;           /*强制Load 16位counter值进入DOWN-COUNTER*/
    PITCFLMT_PFLMT0=1;        /*强制Load 8  位counter值进入DOWN-COUNTER*/
}

 /*****************************************************************************/
/* 内部ad采样，用于获取直流枪温度      */  
/*   pad0---7是atd0，pad8--15 是atd1                                          */
/*****************************************************************************/

void AD_in_Init(void)
{
   ATD0CTL0 = 0x4;           
   ATD0CTL1 = 0x0;
   ATD0CTL2 = 0x80;
   ATD0CTL3 = 0x20;
   ATD0CTL4 = 0x0f;
   ATD0CTL5 = 0x31;
   
 
   /*   ATD1CTL0 = 0x4;   //转换多少个通道        
   ATD1CTL1 = 0x0;
   ATD1CTL2 = 0x80;
   ATD1CTL3 = 0x20;  //结果保存2个
   ATD1CTL4 = 0x0f;
   ATD1CTL5 = 0x31;  //起始通道
*/
 
}

/* 运行灯闪烁                                                                */
/*****************************************************************************/
static void run (void)
{
    if (gSleepMode)
    {
        static UINT16 cnt = 0;
        static UINT32 back_ticks = 0;

        if(gSysTicks < back_ticks + 3)
            return;
        back_ticks = gSysTicks;

     
    }
    else
    {
        if (Sys_getFlags (SYSFLAG_RUN))
        {
            
            PTP_PTP2 = ~ PTP_PTP2;
        }
    }
}

void main (void)
{
/* MC9S12XDP512RMV2.PDF */
/*   PAGE 601 - IVBR */
/*   PAGE 75  - Vector layout */

    IVBR = 0x7F;
    gSleepMode = FALSE;

    /*XGATE协处理器初始化*/
    SetupXGATE();

    /*主处理器初始化*/
    SetBusCLK_32M();
    SystemTick_Init ();
    Protec_Timer ();
    IO_Init ();
    PWM_Init();
    // Timer_2ms();
    /*SPI通讯需要初始化延时*/
    mdelay (100);

    /*外设初始化 */
    SCI_Init ();
    SPI_Init ();
    AD_in_Init();
    ADC_Init ();
    CAN_Init ();
    Flash_Init ();
    
    /*Why! Why! Why!*/
    {
      long i = 0;
      while (i++ < 1000000)
        asm NOP;
    }

    IIC_Init ();
    IIC9_Init();
    RTC_Init ();
    Time_Init ();
    TmpAdc_Init();

    /*应用层初始化*/
    DB_Init ();
    PTL_Init ();
    PITINTE_PINTE2 = 1;       /*中断使能*/

    /*开外部中断*/
    EnableInterrupts;

   
     Sys_Rt.charge_sts = 0;//初始化为没有充电状态  

    //初始化故障码数组（也包括了发送给TCU的部分，为Bat_Fault_Code[13],[14]）
    Bat_Fault_Code_init();
    for(;;)
    {
        run ();

        DB_main ();
        PTL_main ();
    }
}
