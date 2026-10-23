#include <string.h>
#include "sys.h"
#include "drv_can.h"
#include "drv_fm.h"
#include "db.h"
#include "ptl.h"
#include "modbus.h"

/*冠拓BMS内部协议*/
#if PTL_LECU

#pragma DATA_SEG SHARED_DATA
DBIsoMsg        ISO_message;                 /*绝缘模块信息*/
DBDevRt         LECU_Rt[DEV_MAX];       /*每个采集板的实时数据*/
//UINT8           VTemp_Middle[DEV_MAX][6];
UINT8           VTemp_Middle[40*DEV_MAX];
#if TC_MAX
UINT8           Temp_Middle[TC_MAX][24];
#endif
UINT8           LECU_Age[DEV_MAX];      /*采集板老化年龄*/
UINT32          LECU_SnList[5+DEV_MAX]; /*系统当前所有采集板SN列表*/
UINT8           LECU_IdList[5+DEV_MAX];
UINT8           LECU_SnNum;             /*系统当前采集板连接个数*/
UINT8           LECU_Work;              /*工作阶段：(1)第一次请求SN列表，(2)第二次请求SN列表，(3)采集器下载程序模式，(0)正常工作*/
UINT8           LECU_IdCur;             /*当前轮询采集板*/
UINT32          LECU_Valids;
extern BOOL     BCU_Transport;
#pragma DATA_SEG DEFAULT

static UINT32   collisions[5+DEV_MAX];  /*冲突采集模块列表*/
static UINT8    cno;
static UINT16   LECU_BTC;               /*广播时间计数器*/
static UINT16   LECU_SC;                /*休眠计数器*/
static UINT8    manual_mode = 0;         /*手动更新采集板编号*/

/*****************************************************************************/
/* 请求SN列表                                                                */
/*****************************************************************************/
static void AskSN (void)
{
    CanbusFrame cf;

    cf.id       = 0xfef6;
    cf.len      = 8;
    cf.pri      = 0;
    cf.data[0]  = 0x0;
    cf.data[1]  = 0x0;
    cf.data[2]  = 0x0;
    cf.data[3]  = 0x0;
    cf.data[4]  = 0x0;

    CAN4_write (&cf);     /*通知所有采集板：上报出厂编号*/
}

/*****************************************************************************/
/* 分析SN列表                                                                */
/*****************************************************************************/
static void ParseOne (UINT32 sn, UINT8 id)
{
    /*地址冲突*/
    if (id == 0 || id > Sys_Para.lecu_num || (LECU_Valids & (1UL << id)) || LECU_Para[id].serial_num != sn)
    {
        collisions[cno++] = sn;
        return;
    }

    LECU_Valids |= 1UL << id;
}

/*****************************************************************************/
/* 分析SN列表，为新加入的采集板分配ID                                        */
/*****************************************************************************/
static void ParseSN (void)
{
    UINT8 i;

    /*无冲突*/
    cno = 0;
    
    for (i = 0; i < LECU_SnNum; i++)
        ParseOne (LECU_SnList[i], LECU_IdList[i]);
    
    /*处理冲突列表*/
    for (i = 0; i < cno; i++)
    {
        UINT8 id = DEV_new (collisions[i]);

        LECU_Valids |= 1UL << id;
    }
}

/*****************************************************************************/
/* 给每个采集板下发CAN通讯ID                                                 */
/*****************************************************************************/
static void Assign (void)
{
    UINT32      loops;
    UINT8       j;
    CanbusFrame cf;

    for (j = 0; j < DEV_MAX; j++)
    {
        if (!(LECU_Valids & (1UL << j)))
          continue;

        cf.id   = 0xfef6;
        cf.len  = 8;
        cf.pri  = 0;
        (void) memcpy (cf.data, &LECU_Para[j].serial_num, 4);
        cf.data[4] = 0x81 + j;

        CAN4_write (&cf);

        /**/
        udelay (100);

        loops = 100000;
        while (CAN4TFLG != 0x7 && loops--)
          asm NOP;

        if (!loops)
        {
            /*CANCEL TX*/
            CAN4TARQ = 0x7;

            for (loops = 0; loops < 1000; loops++)
              asm NOP;

            CAN4MISC = 0x0;

            Sys_setFlags (SYSERR_CAN4_ERROR);
        }
    }
}

/*****************************************************************************/
/* 每秒检查采集板老化状态                                                    */
/*****************************************************************************/
static void Aging (void)
{
    CanbusFrame cf;
    UINT16  i, j;

    /*老化采集板状态*/
    for (i = 0; i < DEV_MAX; i++)
    {
        if (LECU_Age[i] < 200 && ++LECU_Age[i] == 20)
        {
            UINT8   bodys = LECU_Rt[i].bodys & 0x7f;   // 0111 1111
            UINT16 *md = RT_Volts ((UINT8) i);
            
            if (md != NULL && bodys != 0)
                (void)memset ( md , 0 , bodys * 2);

            (void) memset (&LECU_Rt[i], 0, sizeof(LECU_Rt[0]));

            LECU_Rt[i].cmd = 0x10;  /*请求发送数据 */
        }

        if (LECU_Rt[i].set_confirm == 0x00)
          continue;

        for (j = 0; j < 6; j++)
        {
            if ((LECU_Rt[i].set_confirm & (1U << j)) == 0)
              continue;

            cf.id   = 0x311f6+(UINT32)i*0x1000+(UINT32)j*0x100;
            cf.len  = 8;
            cf.pri  = 0;

            (void) memcpy (&cf.data[0], (UINT8*)&LECU_Para[i].coltd + j*8, 8);  //lint !e662 !e670

            CAN4_write(&cf);
            while (CAN4TFLG != 0x7) {}
        }
    }
    
#if PTL_HAVE_ISO    // ZYY 有绝缘模块才处理,防止无绝缘模块误报
    {
        if (Sys_Shared.iso_age < 200 && ++Sys_Shared.iso_age == 10)
        {
            Sys_Rt.pos_res = 0;
            //Sys_Rt.neg_res = 0;
            Sys_Rt.total_vol_res = 0;
            Sys_Rt.relayPRE = 0;
            
            (void) memset (&ISO_message, 0, sizeof(ISO_message));
        }    
    }
#endif    
    
}

/*****************************************************************************/
/* 在内部总线上广播当前系统时间                                              */
/*****************************************************************************/
static void BroadcastSysTime (void)
{
    CanbusFrame cf;
    
    cf.id = 0x789;
    cf.pri = 0;
    cf.len = 7;
    cf.data[0] = 0x00 | 6;  /* SF */
    cf.data[1] = 0xF2;
    cf.data[6] = 0xAA;
    *(UINT32 *)&cf.data[2] = gSysSec;
    
    CAN4_write_std (&cf);
}

/*****************************************************************************/
/* 初始化周期定时器3，由XGATE周期招测采集板的实时数据                        */
/*****************************************************************************/
static void StartTimer (void)
{
    PITCFLMT_PITE=1;          /*使能PIT */
    PITCE_PCE3=1;             /*使能16位通道3*/
    PITMUX_PMUX3=0;           /*选择8位Counter base通道)*/
                              /*计时总时长=8位counter  *  16位counter*/
                              /*time-out period = (PITMTLD + 1) * (PITLD + 1) / fBUS. */
    PITMTLD0=200;
    PITLD3=12000;              /* 31.25ns * 200 * 4000 ~= 25 ms    (fBUS = 32MHz) */

    PITFLT_PFLT3=1;           /*强制Load 16位counter值进入DOWN-COUNTER*/
    PITCFLMT_PFLMT0=1;        /*强制Load 8  位counter值进入DOWN-COUNTER*/
    PITINTE_PINTE3 = 1;       /*中断使能*/
}

/*****************************************************************************/
/* 采集板通讯协议初始化函数                                                  */
/*****************************************************************************/
void  LECU_Init (void)
{
    UINT8 i;
    
    /*准备初始化实际连接采集板出厂编号列表*/
    LECU_SnNum  = 0;
    LECU_IdCur  = 1;
    LECU_Valids = 0x0;
    LECU_BTC = 70;
    LECU_SC = 0;
    
    (void) memset (LECU_Age, 0, sizeof(LECU_Age));
    (void) memset (LECU_Rt, 0, sizeof(LECU_Rt));
    
    for (i = 0; i < DEV_MAX; i++)
    {
        LECU_Rt[i].cmd = 0x10;  /*请求发送数据 */
    }

    LECU_Work   = 1;      /*第一次请求SN列表*/

    AskSN ();
}

/*****************************************************************************/
/* 采集板通讯协议重新初始化函数                                              */
/*****************************************************************************/
void LECU_ReInit (void)
{
    PITINTE_PINTE3 = 0;     /*关闭请求实时数据定时器*/
    gSysTicks = 0;
    
    /*等待XGATE收完最后一批数据*/
    mdelay (50);


    /*准备初始化实际连接采集板出厂编号列表*/
    LECU_SnNum  = 0;
    LECU_IdCur  = 1;
    LECU_Valids = 0x0;
    (void) memset (LECU_Age, 0, sizeof(LECU_Age));
    (void) memset (LECU_Rt, 0, sizeof(LECU_Rt));
 //   (void) memset (&Cell_Indexs, 0xff, sizeof(Cell_Indexs));
 //   (void) memset (&Cell_Temp_Indexs, 0xff, sizeof(Cell_Temp_Indexs));    
    (void) memset (&Cell_Volts, 0, sizeof(Cell_Volts));

    LECU_Work   = 1;        /*第一次请求SN列表*/
    
    AskSN ();

    manual_mode = 1;
}

/*****************************************************************************/
/* 采集板通讯协议入口函数                                                    */
/*****************************************************************************/
void LECU_main (void)
{
    if (LECU_Work == 0)
    {
        SINT16  cur = Sys_Rt.current / 10;  // 0.1A => A
        
        if (cur < 0)
            cur = -cur;
        
        if (gSleepMode)
        {
            /*判断能否退出休眠模式*/
            if (cur >= Sys_Para.shutcur)
            {
                PTT_PTT7 = 0;               /*打开采集模块电源*/
                
                mdelay (200);
                
                LECU_ReInit ();
                
                gSleepMode = TRUE;
                return;
            }
        }
        else if (Sys_getFlags (SYSFLAG_AGE))
        {
            /*每分钟广播一次系统时间*/
            if (++LECU_BTC >= 60)
            {
                LECU_BTC = 0;
                BroadcastSysTime ();//调试QDWX时这里需要特别注意
            }
            
            if (!BCU_Transport)
            {
                Aging ();
            }

            /*判断能否进入休眠模式*/
           /* if (0 && cur < Sys_Para.shutcur)
            {
                LECU_SC++;

                if (LECU_SC > (UINT16)Sys_Para.shutmin * 60)        // 10分钟
                {
                    PITINTE_PINTE3 = 0;     //关闭请求实时数据定时器
                    PTT_PTT7 = 1;           //关闭采集模块电源
                    PTP_PTP1 = 1;
                    
                    gSleepMode = TRUE;
                    return;
                }
            }
            else
                LECU_SC = 0;*/
        }
    }
    else
    {
        if (LECU_Work == 1 && gSysTicks > 150)
        {
            LECU_Work = 2;        /*第二次请求SN列表*/

            AskSN ();

            return;
        }

        if (gSysTicks < 300)      /*3秒钟*/
          return;

        ParseSN ();               /*分析当前连接采集板的SN列表*/

        if (manual_mode == 1)
        {
            Assign ();            /*下发所有ID*/
            Assign ();            /*下发所有ID*/
            Assign ();            /*下发所有ID*/
            Assign ();            /*下发所有ID*/
            Assign ();            /*下发所有ID*/

            manual_mode = 0;
        }
        
        StartTimer ();            /*启动招测定时器中断*/

        LECU_Work = 0;            /*初始化完成*/
    }
}

#endif
