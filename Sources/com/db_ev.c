#include <string.h>
#include <stddef.h>
#include "sys.h"
#include "drv_fm.h"
#include "drv_flash.h"
#include "db.h"
#include "db_ev.h"
#include "db_para_cfg.h"
#include "db_para_struct.h"
#include "ptl.h"
#include "modbus.h"

/*报警记录在铁电中的保存地址*/
#define REC_ADDR(i)     (ALARM_REC_START + (i)*16)             //每条报警记录最大占16Byte
/*Flash报警记录索引在铁电中的保存地址*/
#define FLA_ADDR(i)     (FLA_INDEX_START + (i)*FLA_INDEX_SIZE) //每条报警记录最大占16Byte


/*LOCALS*/
static UINT32   alarm1_prev = 0;         //监视报警状态变化
static UINT32   alarm2_prev = 0;
#ifdef HAVE_L3_PARAMS
static UINT32   alarm3_prev = 0;
#ifdef HAVE_L4_PARAMS
static UINT32   alarm4_prev = 0;
#endif
#endif

static UINT32   fault_prev  = 0;         //监视故障状态变化

static UINT8    para_lecu_prev = 0;
static UINT32   exist_prev = 0xffffffff;

volatile UINT8  ASSERT_EV_SIZE[sizeof(DBEvent) <= 16];

#pragma DATA_SEG __GPAGE_SEG PAGED_RAM
static DBEvent  cache[ALARM_REC_NUM];     // 存储本次上电最近125条报警记录，防止某些报警平凡出现塞满存储空间。
#pragma DATA_SEG DEFAULT

static BOOL     ptrs_modified;
volatile UINT16   cache_rptr;
volatile UINT16   cache_wptr;     // 铁电中报警记录的写位置
static UINT32   flash_wptr;       // FLASH中报警记录的写位置
static UINT8    flash_stage;

#ifdef HAVE_PWDNEV
static UINT32   this_time;
#endif

#ifdef HAVE_IOEV
UINT16   bms_in;
UINT16   bms_out;
#endif

static UINT8 heat_prev[DEV_MAX];   //上次加热状态

#define OFFST(X)    offsetof(DBEvent,u.X.level)
static UINT8    Lv_Offs[] =
{0,                 //0
 0,                 //1       上电事件
 0,                 //2       IO事件
 0,                 //3       模块丢失报警
 0,                 //4       参数设置事件（修改参数标识，新设置值）
 0,                 //5       充电事件（开始，结束，异常结束）
 OFFST(tmpunba),    //6       温度不均衡
 OFFST(tmp),        //7       温度过高
 OFFST(tmp),        //8       温度过低
 OFFST(tmpunba),    //9       极柱温度不均衡
 OFFST(tmp),        //10      极柱温度过高
 OFFST(vol),        //11      单体过压
 OFFST(vol),        //12      单体欠压
 OFFST(volunba),    //13      单体电压不均衡
 OFFST(tovol),      //14      总电压过压
 OFFST(tovol),      //15      总电压欠压
 OFFST(cur),        //16      充电过流
 OFFST(cur),        //17      回馈过流
 OFFST(cur),        //18      放电过流
 OFFST(soc),        //19      SOC高
 OFFST(soc),        //20      SOC低
 OFFST(iso),        //21      绝缘报警
 OFFST(tovol),      //22      充电电压大
 0,                 //23      清除报警记录
 0,                 //24      清除历史数据
 OFFST(tmp),        //25      极注温度过低
 0,                 //26      铁电读错误
 0,                 //27      铁电写错误
 0,                 //28      FLASH读错误
 0,                 //29      FLASH写错误
 0,                 //30      时钟错误
 0,                 //31      参数丢失
 0,                 //32      时间丢失
 0,                 //33      与采集板通讯错误
 0,                 //34      电流ADC错误
 0,                 //35      绝缘ADC错误
 0,                 //36      绝缘失效
 0,                 //37      加热事件
};

#define DEF_CHK(lv) static BOOL Check##lv (UINT32 msk) { return (alarm##lv##_prev & msk) == 0UL && (Sys_Alarm.warning##lv & msk) != 0UL; }

DEF_CHK (1)
DEF_CHK (2)
#ifdef HAVE_L3_PARAMS
DEF_CHK (3)
#ifdef HAVE_L4_PARAMS
DEF_CHK (4)
#endif
#endif

static UINT16            month_fm;              //month在铁电索引中的存储位置
volatile UINT8 warm_state;
static BOOL warn_sta[DEV_MAX];


/*****************************************************************************/
/* 检查有无初次系统故障函数                                                  */
/*****************************************************************************/
static BOOL Fault_chk (UINT32 x) 
{
    return ((fault_prev & x) == 0UL && (gSysErrFlags & x) != 0UL);
}


/*****************************************************************************/
/* 检查IN状态函数                                                            */
/*****************************************************************************/
UINT16  BMS_InSts (void)
{
    return 0xffc0 | (UINT16)IO_IN0 | ((UINT16)IO_IN1 << 1) | ((UINT16)IO_IN2 << 2) | ((UINT16)CC2<< 3) | ((UINT16)ACC_ON<< 4) | ((UINT16)HVIL<< 5);
}


/*****************************************************************************/
/* 检查OUT状态函数                                                          */
/*****************************************************************************/
UINT16  BMS_OutSts (void)
{
    return 0xffc0 | (UINT16)IO_OUT0 | ((UINT16)IO_OUT1 << 1) | ((UINT16)IO_OUT2 << 2) | ((UINT16)IO_OUT3 << 3) | ((UINT16)IO_OUT4 << 4)| ((UINT16)IO_OUT5 << 5);
}

/*****************************************************************************/
/* 事件报警生成函数                                                          */
/*****************************************************************************/
void EV_Make (DBEvent *ev)
{
    DBEvent *far newer;
    DBEvent *far elder;
    UINT16  i, p = cache_wptr;
    UINT16  repx;
    UINT8   lv = 0, lvo = Lv_Offs[ev->type];

    // 不判断重复，无条件存储报警类
    switch (ev->type)
    {
    case 1:      /*上电事件,记录上次停电事件*/
    case 2:      /*IO事件,记录IO变化事件*/
    case 3:      /*模块丢失事件*/
    case 4:      /*参数设置事件,暂无处理*/
    case 5:      /*充电事件,暂无处理*/
    case 23:     /*清除报警记录事件,暂无处理*/
    case 24:     /*清除历史数据事件,暂无处理*/
    case 26:     /*铁电读错误*/
    case 27:     /*铁电写错误*/
    case 28:     /*FLASH读错误*/
    case 29:     /*FLASH写错误*/
    case 30:     /*时钟错误*/
    case 31:     /*参数丢失*/
    case 32:     /*时间丢失*/
    case 33:     /*与采集板通讯错误*/
    case 34:     /*电流ADC错误*/
    case 35:     /*绝缘ADC错误*/
    case 36:     /*绝缘失效*/
    case 37:     /*加热事件*/
    case 39:     /*温度失效*/
        goto wb;
    }
    
    if (lvo != 0)
        lv = *((UINT8 *)ev + lvo);

    // 判断是否在一段时间内重复
    for (i = 0; i < ALARM_REC_NUM - 1; i++)
    {
        // 逆顺序查询
        p = p ? p-1 : ALARM_REC_NUM-1;
        newer = &cache[p];

        // 类型不同
        if (newer->type != ev->type)
            continue;
        
        // 等级不同
        if (lvo != 0 && lv != *((UINT8 *far)newer + lvo))
            continue;
        
        if (ev->time - newer->time >= OEM_Para.almintv)
            goto wb;

        repx = p;

        // 继续查找时间段内的 "开始记录"
        for (; i < ALARM_REC_NUM - 1; i++)
        {
            p = p ? p-1 : ALARM_REC_NUM-1;
            elder = &cache[p];

            // 类型不同
            if (elder->type != ev->type)
                continue;
            
            // 等级不同
            if (lvo != 0 && lv != *((UINT8 *far)elder + lvo))
                continue;
            
            if (ev->time - elder->time >= OEM_Para.almintv)
                goto wb;

            // 更新重复计数
            ev->rep = newer->rep + 1;
            if (ev->rep == 0)
                ev->rep = 0xFF;

            ev->crc = CRC8 ((UINT8 *) ev, offsetof(DBEvent, crc));
            *newer = *ev;

            Fm_write (REC_ADDR (repx), ev, sizeof(DBEvent));
            return;
        }

        // 没有时间段内没有找到"开始记录"
        break;
    }

wb: ev->rep = 0;
    ev->crc = CRC8 ((UINT8 *) ev, offsetof(DBEvent, crc));
    Fm_write (REC_ADDR (cache_wptr), ev, sizeof(DBEvent));
    
    cache[cache_wptr] = *ev;
    cache_wptr = (cache_wptr + 1) % ALARM_REC_NUM;
    
    ptrs_modified = TRUE;
}


/*****************************************************************************/
/* 生成充电状态事件                                                          */
/*****************************************************************************/
void EV_Make_Charger (UINT8 type)
{
    DBEvent ev;
    
    (void) memset (&ev, 0, sizeof (ev));
    ev.time = gSysSec;

    ev.type = 5;
    ev.u.cha.volt = Sys_Rt.to_vol;
    ev.u.cha.maxv = Sys_Rt.max_v;
    ev.u.cha.maxt = (UINT8) Sys_Rt.max_t;
    ev.u.cha.type = type;

    EV_Make (&ev);
}


/*****************************************************************************/
/* 生成参数设置事件                                                          */
/*****************************************************************************/
void EV_Make_ParaChange (UINT8 way, UINT8 which, UINT16 old, UINT16 val)
{
    DBEvent ev;
    
    (void) memset (&ev, 0, sizeof (ev));
    ev.time = gSysSec;

    ev.type = 4;
    ev.u.para.old = old;
    ev.u.para.value = val;
    ev.u.para.which = which;
    ev.u.para.way = way;

    EV_Make (&ev);
}

/*****************************************************************************/
/* 清除所有报警信息                                                          */
/*****************************************************************************/
void EV_Reset (void)
{
    UINT16  i;
    DBEvent ev;
    DBFlashIndexRec index;

    // Bug04
    alarm1_prev = 0;
    alarm2_prev = 0;
#ifdef HAVE_L3_PARAMS    
    alarm3_prev = 0;
#ifdef HAVE_L4_PARAMS    
    alarm4_prev = 0;
#endif
#endif    
    fault_prev  = 0;
    exist_prev = 0xFFFFFFFF;
    para_lecu_prev = 0;
    
#if PTL_QDWX
    for (i = 0; i < DEV_MAX; i++)
    {
        heat_prev[i] = 0;
        warn_sta[i] = 0;
    }    
#endif        
#if PTL_LECU
    for (i = 0; i < DEV_MAX; i++)
    {
        heat_prev[i] = 1;
        warn_sta[i] = 0;
    }    
#endif
    warm_state = 0; 
    
    ptrs_modified = TRUE;
    cache_rptr = cache_wptr = 0;
    flash_wptr = FLASH_ALARM_START;
    month_fm = FLA_INDEX_START;
    MEMSET ((UINT8 * far)cache, 0, sizeof(cache));

    (void) memset (&ev, 0, sizeof (ev));
    for (i = 0; i < ALARM_REC_NUM; i++)
        Fm_write (REC_ADDR (i), &ev, sizeof (ev));
    
    (void) memset(&index, 0, sizeof(index));
    for (i = 0; i < FLA_INDEX_MAX; i++)
        Fm_write (FLA_ADDR (i), &index, sizeof(index));    

    Flash_erase (flash_wptr);
    flash_stage = 1;    
    /*生成 "清除报警记录" 事件*/
    ev.time = gSysSec;
    ev.type = 23;
    EV_Make (&ev);
}

/*****************************************************************************/
/* 事件模块初始化                                                            */
/*****************************************************************************/
void EV_Init (void)
{
    BOOL    valid = TRUE;
    UINT16  i;
    UINT16  cs[7];  
    DBEvent ev;        

    para_lecu_prev = 0;
    exist_prev = 0xffffffff;
    flash_stage = 0;
    MEMSET ((UINT8 * far)cache, 0, sizeof(cache));
    
    
#if PTL_QDWX
    for (i = 0; i < DEV_MAX; i++)
    {
        heat_prev[i] = 0;
        warn_sta[i] = 0;
    }    
#endif        
#if PTL_LECU
    for (i = 0; i < DEV_MAX; i++)
    {
        heat_prev[i] = 1;
        warn_sta[i] = 0;
    }    
#endif
    warm_state = 0;                    

    /*恢复报警记录指针信息*/
    Fm_read (ALARM_PTR_START, cs, 7*2); 
    if (Modbus_crc ((UINT8 *) cs, 6*2) == cs[6] && cs[0] == 0xBEEF) 
    {
        ptrs_modified = FALSE;
        cache_wptr = cs[1];
        cache_rptr = cs[2];
        month_fm = cs[3];                  //上一次Flash索引在铁电中的位置
        flash_wptr = *(UINT32 *) &cs[4];
    }
    else
    {
        /*尝试读备份区域信息*/
        Fm_read (ALARM_PTR2_START, cs, 7*2);
        if (Modbus_crc ((UINT8 *) cs, 6*2) == cs[6] && cs[0] == 0xBEEF)
        {
            ptrs_modified = FALSE;
            cache_wptr = cs[1];
            cache_rptr = cs[2];
            month_fm = cs[3];
            flash_wptr = *(UINT32 *) &cs[4];
        }
        else
        {
            valid = FALSE;
        }
    }

    if (!valid)
    {
        ptrs_modified = TRUE;
        cache_wptr = cache_rptr = 0;
        flash_wptr = FLASH_ALARM_START;
        month_fm = FLA_INDEX_START;
    }
    else
    {
        /*读出铁电中现有的报警记录*/
        for (i = 0; i < ALARM_REC_NUM; i++)
        {
            Fm_read (REC_ADDR (i), &ev, sizeof(DBEvent));

            if (ev.time != 0 && ev.crc == CRC8 ((UINT8 *) &ev, offsetof(DBEvent, crc)))
                cache[i] = ev;
        }
    } 
    
#ifdef HAVE_PWDNEV
    // 补充一个停电事件
    Fm_read (ALARM_PWDN_START, cs, 8);
    if (Modbus_crc ((UINT8 *) cs, 6) == cs[3] && cs[2] == 0x2BAD)
    {
        ev.time = gSysSec;             // 上电时刻
        ev.type = 1;                   // 停电事件
        ev.u.pwr.pwdn = *(UINT32 *) cs;// 停电时刻
        
        EV_Make (&ev);
    }
    this_time = 0;
#endif

#ifdef HAVE_IOEV
    bms_in = BMS_InSts ();
    bms_out = BMS_OutSts ();
#endif
}

/*****************************************************************************/
/* 判断事件报警条件，生成报警记录                                            */
/*****************************************************************************/
static void EV_Check (void)
{
#ifdef HAVE_IOEV
    UINT16  in,out;
#endif
    UINT16  i;
    UINT8 j;
    int     err;
    DBEvent ev;
    UINT8 heat_now[DEV_MAX];
    
    (void) memset (&ev, 0, sizeof (ev));
    ev.time = gSysSec;


#ifdef HAVE_IOEV
    in = BMS_InSts ();
    out = BMS_OutSts ();
    if (bms_in != in || bms_out != out)
    {
        bms_in = in;
        bms_out = out;
        
        ev.type = 2;
        ev.u.io.serial_num = *(UINT32 *)0xFFA0;
        ev.u.io.in = in;
        ev.u.io.out = out;
        
        EV_Make (&ev);
    }
#endif

//3
    if (Sys_Para.lecu_num != para_lecu_prev)
    {
        exist_prev = 0xFFFFFFFF;
        para_lecu_prev = Sys_Para.lecu_num;
    }
    
    if (Sys_Rt.lecu_actives != exist_prev)
    {
        ev.type = 3;
        ev.u.lost.lecunum = Sys_Para.lecu_num;
        ev.u.lost.actives = Sys_Rt.num_lecu;
        
        for (i = 0; i < DEV_MAX && i < Sys_Para.lecu_num; i++)
        {
            if (!LECU_Rt[i].bodys && (exist_prev & (1UL << i)))
            {
                ev.u.lost.id = i + 1;
                EV_Make (&ev);
            }
        }
        exist_prev = Sys_Rt.lecu_actives;
    }

//6
//#ifdef L1_TMP_UNBA
    err = (int)(Sys_Rt.max_t - Sys_Rt.min_t);
    
    if (Check1 (WARN_TMP_UNBA))
    {
        ev.u.tmpunba.level = 1;
mk6:    ev.type = 6;
        ev.u.tmpunba.err = (UINT8) err;
        
        EV_Make (&ev);
    }
//#ifdef L2_TMP_UNBA
    else if (Check2 (WARN_TMP_UNBA))
    {
        ev.u.tmpunba.level = 2;
        goto mk6;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_TMP_UNBA))
    {
        ev.u.tmpunba.level = 3;
        goto mk6;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_TMP_UNBA))
    {
        ev.u.tmpunba.level = 4;
        goto mk6;
    }
#endif
#endif

// 7
//#ifdef L1_TMP_HIGH
    if (Check1 (WARN_TMP_HIGH))
    {
        ev.u.tmp.level = 1;
mk7:    ev.type = 7;
        ev.u.tmp.tmp = (UINT8) Sys_Rt.max_t;
        ev.u.tmp.batt = Sys_Rt.maxt_batt;
        if (Sys_Rt.maxt_lecu > DEV_MAX)
            ev.u.tmp.serial_num = 0;
        else
            ev.u.tmp.serial_num = *(UINT32 *) &LECU_Rt[Sys_Rt.maxt_lecu - 1].serial_num[0];
        
        EV_Make (&ev);
    }
//#ifdef L2_TMP_HIGH
    else if (Check2 (WARN_TMP_HIGH))
    {
        ev.u.tmp.level = 2;
        goto mk7;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_TMP_HIGH))
    {
        ev.u.tmp.level = 3;
        goto mk7;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_TMP_HIGH))
    {
        ev.u.tmp.level = 4;
        goto mk7;
    }
#endif
#endif
//#endif
//#endif



// 8
//#ifdef L1_TMP_LOW
    if (Check1 (WARN_TMP_LOW))
    {
        ev.u.tmp.level = 1;
mk8:    ev.type = 8;
        ev.u.tmp.tmp = (UINT8) Sys_Rt.min_t;
        ev.u.tmp.batt = Sys_Rt.mint_batt;
        if (Sys_Rt.mint_lecu > DEV_MAX)
            ev.u.tmp.serial_num = 0;
        else
            ev.u.tmp.serial_num = *(UINT32 *) &LECU_Rt[Sys_Rt.mint_lecu - 1].serial_num[0];
        
        EV_Make (&ev);
    }
//#ifdef L2_TMP_LOW
    else if (Check2 (WARN_TMP_LOW))
    {
        ev.u.tmp.level = 2;
        goto mk8;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_TMP_LOW))
    {
        ev.u.tmp.level = 3;
        goto mk8;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_TMP_LOW))
    {
        ev.u.tmp.level = 4;
        goto mk8;
    }
#endif
#endif
//#endif
//#endif



//9
#ifdef L1_EP_TMP_UNBA
    err = (int)(Sys_Rt.epmax - Sys_Rt.epmin);
    
    if (Check1 (WARN_EP_TMP_UNBA))
    {
        ev.u.tmpunba.level = 1;
mk9:    ev.type = 9;
        ev.u.tmpunba.err = (UINT8) err;
        
        EV_Make (&ev);
    }
#ifdef L2_EP_TMP_UNBA
    else if (Check2 (WARN_EP_TMP_UNBA))
    {
        ev.u.tmpunba.level = 2;
        goto mk9;
    }
#ifdef L3_EP_TMP_UNBA
    else if (Check3 (WARN_EP_TMP_UNBA))
    {
        ev.u.tmpunba.level = 3;
        goto mk9;
    }
#ifdef L4_EP_TMP_UNBA
    else if (Check4 (WARN_EP_TMP_UNBA))
    {
        ev.u.tmpunba.level = 4;
        goto mk9;
    }
#endif
#endif
#endif
#endif



// 10
#ifdef HAVE_L1_EP_TMP_HIGH
    if (Check1 (WARN_EP_TMP_HIGH))
    {
        ev.u.tmp.level = 1;
mk10:   ev.type = 10;
        ev.u.tmp.tmp = (UINT8) Sys_Rt.epmax;
        ev.u.tmp.batt = Sys_Rt.epmax_lecu;
        if (Sys_Rt.epmax_lecu > DEV_MAX)
            ev.u.tmp.serial_num = 0;
        else
            ev.u.tmp.serial_num = *(UINT32 *) &LECU_Rt[Sys_Rt.epmax_lecu - 1].serial_num[0];
        
        EV_Make (&ev);
    }
#ifdef HAVE_L2_EP_TMP_HIGH
    else if (Check2 (WARN_EP_TMP_HIGH))
    {
        ev.u.tmp.level = 2;
        goto mk10;
    }
#if defined(HAVE_L3_PARAMS)      
#ifdef HAVE_L3_EP_TMP_HIGH
    else if (Check3 (WARN_EP_TMP_HIGH))
    {
        ev.u.tmp.level = 3;
        goto mk10;
    }
#if defined(HAVE_L4_PARAMS)     
#ifdef HAVE_L4_EP_TMP_HIGH
    else if (Check4 (WARN_EP_TMP_HIGH))
    {
        ev.u.tmp.level = 4;
        goto mk10;
    }
#endif
#endif
#endif
#endif
#endif
#endif


// 25
#ifdef HAVE_L1_EP_TMP_LOW
    if (Check1 (WARN_EP_TMP_LOW))
    {
        ev.u.tmp.level = 1;
mk25:   ev.type = 25;
        ev.u.tmp.tmp = (UINT8) Sys_Rt.epmin;
        ev.u.tmp.batt = Sys_Rt.epmin_lecu;
        if (Sys_Rt.epmin_lecu > DEV_MAX)
            ev.u.tmp.serial_num = 0;
        else
            ev.u.tmp.serial_num = *(UINT32 *) &LECU_Rt[Sys_Rt.epmin_lecu - 1].serial_num[0];
        
        EV_Make (&ev);
    }
#ifdef HAVE_L2_EP_TMP_LOW
    else if (Check2 (WARN_EP_TMP_LOW))
    {
        ev.u.tmp.level = 2;
        goto mk25;
    }
#if defined(HAVE_L3_PARAMS)    
#ifdef HAVE_L3_EP_TMP_LOW
    else if (Check3 (WARN_EP_TMP_LOW))
    {
        ev.u.tmp.level = 3;
        goto mk25;
    }
#if defined(HAVE_L4_PARAMS)    
#ifdef HAVE_L4_EP_TMP_LOW
    else if (Check4 (WARN_EP_TMP_LOW))
    {
        ev.u.tmp.level = 4;
        goto mk25;
    }
#endif    
#endif
#endif
#endif
#endif
#endif


// 11
//#ifdef L1_VOL_HIGH
    if (Check1 (WARN_VOL_HIGH))
    {
        ev.u.vol.level = 1;
mk11:   ev.type = 11;
        ev.u.vol.volt = Sys_Rt.max_v;
        ev.u.vol.batt = Sys_Rt.maxv_batt;
        if (Sys_Rt.maxv_lecu > DEV_MAX)
            ev.u.vol.serial_num = 0;
        else
            ev.u.vol.serial_num = *(UINT32 *) &LECU_Rt[Sys_Rt.maxv_lecu - 1].serial_num[0];
        
        EV_Make (&ev);
    }
//#ifdef L2_VOL_HIGH
    else if (Check2 (WARN_VOL_HIGH))
    {
        ev.u.vol.level = 2;
        goto mk11;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_VOL_HIGH))
    {
        ev.u.vol.level = 3;
        goto mk11;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_VOL_HIGH))
    {
        ev.u.vol.level = 4;
        goto mk11;
    }
//#endif
//#endif
#endif
#endif



// 12
//#ifdef L1_VOL_LOW
    if (Check1 (WARN_VOL_LOW))
    {
        ev.u.vol.level = 1;
mk12:   ev.type = 12;
        ev.u.vol.volt = Sys_Rt.min_v;
        ev.u.vol.batt = Sys_Rt.minv_batt;
        if (Sys_Rt.minv_lecu > DEV_MAX)
            ev.u.vol.serial_num = 0;
        else
            ev.u.vol.serial_num = *(UINT32 *) &LECU_Rt[Sys_Rt.minv_lecu - 1].serial_num[0];
        
        EV_Make (&ev);
    }
//#ifdef L2_VOL_LOW
    else if (Check2 (WARN_VOL_LOW))
    {
        ev.u.vol.level = 2;
        goto mk12;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_VOL_LOW))
    {
        ev.u.vol.level = 3;
        goto mk12;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_VOL_LOW))
    {
        ev.u.vol.level = 4;
        goto mk12;
    }
//#endif
//#endif
#endif
#endif



//13
//#ifdef L1_VOL_UNBA
    err = (int)(Sys_Rt.max_v - Sys_Rt.min_v);
    
    if (Check1 (WARN_VOL_UNBA))
    {
        ev.u.volunba.level = 1;
mk13:   ev.type = 13;
        ev.u.volunba.err = err;
        
        EV_Make (&ev);
    }
//#ifdef L2_VOL_UNBA
    else if (Check2 (WARN_VOL_UNBA))
    {
        ev.u.volunba.level = 2;
        goto mk13;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_VOL_UNBA))
    {
        ev.u.volunba.level = 3;
        goto mk13;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_VOL_UNBA))
    {
        ev.u.volunba.level = 4;
        goto mk13;
    }
#endif
#endif
//#endif
//#endif



// 14
//#ifdef L1_TVOL_HIGH
    if (Check1 (WARN_TVOL_HIGH))
    {
        ev.u.tovol.level = 1;
mk14:   ev.type = 14;
        ev.u.tovol.volt = Sys_Rt.to_vol;
        
        EV_Make (&ev);
    }
//#ifdef L2_TVOL_HIGH
    else if (Check2 (WARN_TVOL_HIGH))
    {
        ev.u.tovol.level = 2;
        goto mk14;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_TVOL_HIGH))
    {
        ev.u.tovol.level = 3;
        goto mk14;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_TVOL_HIGH))
    {
        ev.u.tovol.level = 4;
        goto mk14;
    }
#endif
#endif
//#endif
//#endif



// 15
//#ifdef L1_TVOL_LOW
    if (Check1 (WARN_TVOL_LOW))
    {
        ev.u.tovol.level = 1;
mk15:   ev.type = 15;
        ev.u.tovol.volt = Sys_Rt.to_vol;
        
        EV_Make (&ev);
    }
//#ifdef L2_TVOL_LOW
    else if (Check2 (WARN_TVOL_LOW))
    {
        ev.u.tovol.level = 2;
        goto mk15;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_TVOL_LOW))
    {
        ev.u.tovol.level = 3;
        goto mk15;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_TVOL_LOW))
    {
        ev.u.tovol.level = 4;
        goto mk15;
    }
#endif
#endif
//#endif
//#endif



// 16
//#ifdef L1_CHG_CUR_HIGH
    if (Check1 (WARN_CHG_CUR_HIGH))
    {
        ev.u.cur.level = 1;
mk16:   ev.type = 16;
        ev.u.cur.volt = Sys_Rt.to_vol;
        ev.u.cur.current = Sys_Rt.current;
        
        EV_Make (&ev);
    }
//#ifdef L2_CHG_CUR_HIGH
    else if (Check2 (WARN_CHG_CUR_HIGH))
    {
        ev.u.cur.level = 2;
        goto mk16;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_CHG_CUR_HIGH))
    {
        ev.u.cur.level = 3;
        goto mk16;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_CHG_CUR_HIGH))
    {
        ev.u.cur.level = 4;
        goto mk16;
    }
#endif
#endif
//#endif
//#endif



// 17
//#ifdef L1_REGEN_CUR_HIGH
    if (Check1 (WARN_REGEN_CUR_HIGH))
    {
        ev.u.cur.level = 1;
mk17:   ev.type = 17;
        ev.u.cur.volt = Sys_Rt.to_vol;
        ev.u.cur.current = Sys_Rt.current;
        
        EV_Make (&ev);
    }
//#ifdef L2_REGEN_CUR_HIGH
    else if (Check2 (WARN_REGEN_CUR_HIGH))
    {
        ev.u.cur.level = 2;
        goto mk17;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_REGEN_CUR_HIGH))
    {
        ev.u.cur.level = 3;
        goto mk17;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_REGEN_CUR_HIGH))
    {
        ev.u.cur.level = 4;
        goto mk17;
    }
#endif
#endif
//#endif
//#endif



// 18
//#ifdef L1_DISCHG_CUR_HIGH
    if (Check1 (WARN_DISCHG_CUR_HIGH))
    {
        ev.u.cur.level = 1;
mk18:   ev.type = 18;
        ev.u.cur.volt = Sys_Rt.to_vol;
        ev.u.cur.current = Sys_Rt.current;
        
        EV_Make (&ev);
    }
//#ifdef L2_DISCHG_CUR_HIGH
    else if (Check2 (WARN_DISCHG_CUR_HIGH))
    {
        ev.u.cur.level = 2;
        goto mk18;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_DISCHG_CUR_HIGH))
    {
        ev.u.cur.level = 3;
        goto mk18;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_DISCHG_CUR_HIGH))
    {
        ev.u.cur.level = 4;
        goto mk18;
    }
#endif
#endif
//#endif
//#endif



// 19
//#ifdef L1_SOC_HIGH
    if (Check1 (WARN_SOC_HIGH))
    {
        ev.u.soc.level = 1;
mk19:   ev.type = 19;
        ev.u.soc.volt = Sys_Rt.to_vol;
        ev.u.soc.soc = (UINT8) Sys_Rt.soc;
        
        EV_Make (&ev);
    }
//#ifdef L2_SOC_HIGH
    else if (Check2 (WARN_SOC_HIGH))
    {
        ev.u.soc.level = 2;
        goto mk19;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_SOC_HIGH))
    {
        ev.u.soc.level = 3;
        goto mk19;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_SOC_HIGH))
    {
        ev.u.soc.level = 4;
        goto mk19;
    }
#endif
#endif
//#endif
//#endif



// 20
//#ifdef L1_SOC_LOW
    if (Check1 (WARN_SOC_LOW))
    {
        ev.u.soc.level = 1;
mk20:   ev.type = 20;
        ev.u.soc.volt = Sys_Rt.to_vol;
        ev.u.soc.soc = (UINT8) Sys_Rt.soc;
        
        EV_Make (&ev);
    }
//#ifdef L2_SOC_LOW
    else if (Check2 (WARN_SOC_LOW))
    {
        ev.u.soc.level = 2;
        goto mk20;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_SOC_LOW))
    {
        ev.u.soc.level = 3;
        goto mk20;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_SOC_LOW))
    {
        ev.u.soc.level = 4;
        goto mk20;
    }
#endif
#endif
//#endif
//#endif



// 21
//#ifdef L1_ISO_LOW
    if (Check1 (WARN_POSRES_LOW | WARN_NEGRES_LOW))
    {
        ev.u.iso.level = 1;
mk21:   ev.type = 21;
        ev.u.iso.volt = Sys_Rt.total_vol_res;
        ev.u.iso.res = Sys_Rt.pos_res ;
        
        EV_Make (&ev);
    }
//#ifdef L2_ISO_LOW
    else if (Check2 (WARN_POSRES_LOW | WARN_NEGRES_LOW))
    {
        ev.u.iso.level = 2;
        goto mk21;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_POSRES_LOW | WARN_NEGRES_LOW))
    {
        ev.u.iso.level = 3;
        goto mk21;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_POSRES_LOW | WARN_NEGRES_LOW))
    {
        ev.u.iso.level = 4;
        goto mk21;
    }
#endif
#endif
//#endif
//#endif



// 22
//#ifdef L1_CHG_VOLT_HIGH
    if (Check1 (WARN_CHG_VOLT_HIGH))
    {
        ev.u.tovol.level = 1;
mk22:   ev.type = 22;
        ev.u.tovol.volt = Sys_Rt.total_vol_res;
        
        EV_Make (&ev);
    }
//#ifdef L2_CHG_VOLT_HIGH
    else if (Check2 (WARN_CHG_VOLT_HIGH))
    {
        ev.u.tovol.level = 2;
        goto mk22;
    }
#ifdef HAVE_L3_PARAMS
    else if (Check3 (WARN_CHG_VOLT_HIGH))
    {
        ev.u.tovol.level = 3;
        goto mk22;
    }
#ifdef HAVE_L4_PARAMS
    else if (Check4 (WARN_CHG_VOLT_HIGH))
    {
        ev.u.tovol.level = 4;
        goto mk22;
    }
#endif
#endif
//#endif
//#endif

//36 绝缘失效
    if (Check1 (WARN_ISO_FAULT))
    {
        ev.u.iso.level = 1;
        ev.type = 36;
        ev.u.iso.volt = Sys_Rt.total_vol_res;
        ev.u.iso.res = (UINT16)Sys_Rt.pos_res ;
        
        EV_Make (&ev);
    }
    
    
//37 加热事件
    for (j = 0; j < DEV_MAX; j++)
    {
        heat_now[j] = LECU_Rt[j].cmd & 0x1;
#if PTL_QDWX    
        if ((LECU_Rt[j].cmd&0x10)&&(heat_now[j]!=heat_prev[j]))
#endif        
#if PTL_LECU
        if ((LECU_Rt[j].cmd&0x4)&&(heat_now[j]!=heat_prev[j]))
#endif                
        {
            ev.type = 37;
            warn_sta[j] = ~warn_sta[j];
            ev.u.warm.state = warn_sta[j] > 0?1:0;
            ev.u.warm.id = j+1;
            ev.u.warm.min_t = LECU_Rt[j].mintmp;
            ev.u.warm.max_t = LECU_Rt[j].maxtmp;
            ev.u.warm.min_v = LECU_Rt[j].minv & 0x7fff;
#if PTL_QDWX            
            if (heat_now[j] == 1)
#endif             
#if PTL_LECU            
            if (heat_now[j] == 0)
#endif            
                ev.u.warm.reason = 1;
            else
            {
                if (LECU_Rt[j].mintmp > Sys_Para.temp_heat_off)
                    ev.u.warm.reason |= 0b1;
                if (LECU_Rt[j].maxtmp > 60)
                    ev.u.warm.reason |= 0b10;
                if ((LECU_Rt[j].minv & 0x7fff) < 2900)
                    ev.u.warm.reason |= 0b100;
                if (warm_state)
                    ev.u.warm.reason |= 0b1000;
                if (!ev.u.warm.reason)
                    ev.u.warm.reason = 0x10;     
            }
            EV_Make (&ev);
            heat_prev[j] = heat_now[j];
            ev.u.warm.reason = 0;
        }  
    }
    
//39 温度失效
    if (Check1 (WARN_TMP_FAULT))
    {
        ev.type = 39;
        ev.u.temp.maxt = (UINT8)Sys_Rt.max_t;
        ev.u.temp.mint = (UINT8)Sys_Rt.min_t;
        ev.u.temp.avgt = (UINT8)Sys_Rt.avg_t;
        
        EV_Make (&ev);
    }
     


    alarm1_prev = Sys_Alarm.warning1;
    alarm2_prev = Sys_Alarm.warning2;
#ifdef HAVE_L3_PARAMS
    alarm3_prev = Sys_Alarm.warning3;
#ifdef HAVE_L4_PARAMS
    alarm4_prev = Sys_Alarm.warning4;
#endif
#endif

/*故障信息*/
//26 铁电读错误
    if (Fault_chk (SYSERR_FRAM_READ_ERROR))
    {
        ev.type = 26;
        EV_Make (&ev);
    }

//27 铁电写错误
    if (Fault_chk (SYSERR_FRAM_WRITE_ERROR))
    {
        ev.type = 27;
        EV_Make (&ev);
    } 

//28 FLASH读错误
    if (Fault_chk (SYSERR_FLASH_READ_ERROR))
    {
        ev.type = 28;
        EV_Make (&ev);
    }
    
//29 FLASH写错误
    if (Fault_chk (SYSERR_FLASH_WRITE_ERROR))
    {
        ev.type = 29;
        EV_Make (&ev);
    }
    
//30 时钟错误
    if (Fault_chk (SYSERR_RTC_ERROR))
    {
        ev.type = 30;
        EV_Make (&ev);
    }
    
//31 参数丢失
    if (Fault_chk (SYSERR_PARA_LOST_ERROR))
    {
        ev.type = 31;
        EV_Make (&ev);
    }
    
//32 时间丢失
    if (Fault_chk (SYSERR_TIME_LOST_ERROR))
    {
        ev.type = 32;
        EV_Make (&ev);
    }
    
//33 与采集板通讯错误
    if (Fault_chk (SYSERR_CAN4_ERROR))
    {
        ev.type = 33;
        EV_Make (&ev);
    }
    
//34 电流ADC错误
    if (Fault_chk (SYSERR_CURADC_ERROR))
    {
        ev.type = 34;
        EV_Make (&ev);
    }
    
//35 绝缘ADC错误
    if (Fault_chk (SYSERR_ISOADC_ERROR))
    {
        ev.type = 35;
        EV_Make (&ev);
    }
    
    fault_prev = gSysErrFlags; 
}

/*****************************************************************************/
/* 将铁电中的报警记录转存到FLASH中                                           */
/*****************************************************************************/
static void EV_Dump (void)
{
    DBEvent ev;
    UINT16  n;
    UINT8 tmp[256];
    UINT16 i;
    UINT32 fla_writeAddr;
    DBFlashIndexRec fla_index, tmp_flaIndex;
    DBFlashIndexRec tmp_flash;
    time_t tmp_time, old_time = 0;                 
    struct tm t;
    struct tm *tim; 
    
    if (cache_wptr >= cache_rptr)
        n = cache_wptr - cache_rptr;
    else
        n = ALARM_REC_NUM - cache_rptr + cache_wptr;

    switch (flash_stage)
    {
    default:
        flash_stage = 0;
        break;

    case 0:/*空闲*/
        if (n < ALARM_FULL_NUM)
            return;

        if ((flash_wptr & (FLASH_ERASE_SIZE - 1)) != 0x0)    /*不在FLASH擦除边界*/
        {
            flash_stage = 2;
            break;
        }
        
        if (!Flash_busy ())
        {
            Flash_erase (flash_wptr);                   /*下发擦除命令*/
            /*擦除索引Flash中报警记录的索引*/ 
            for (i = 0; i < FLA_INDEX_MAX; i++)
            {
                Fm_read (FLA_ADDR(i), &tmp_flash, sizeof(DBFlashIndexRec));
                
                if ((tmp_flash.flash >= flash_wptr) && (tmp_flash.flash <= (flash_wptr+(FLASH_ERASE_SIZE-1))))
                {                    
                    (void) memset (&tmp_flash, 0, sizeof(tmp_flash));
                    Fm_write (FLA_ADDR(i), &tmp_flash, sizeof(tmp_flash)); 
                }                    
            } 
            flash_stage = 1;                            /*等待擦除完成*/
        }
        break;
    
    case 1:/*等待擦除完成*/
        if (!Flash_busy ())                         /*WIP == 0*/
        {
            flash_stage = 2;                        /*等待擦除完成*/
        }
        break;
    
    case 2:/*可编程*/
        if (Flash_busy ())
            break;
        
        if (n < ALARM_FULL_NUM)
        {
            flash_stage = 0;
            break;
        }
        
        (void) memset (&tmp, 0, sizeof(tmp));
        fla_writeAddr = flash_wptr;
        /*读Flash索引*/
        Fm_read (month_fm, &fla_index, sizeof(fla_index));
        for (i = 0; i < 256;)
        {
            Fm_read (REC_ADDR (cache_rptr), &ev, sizeof(DBEvent));
            (void) memcpy (&tmp[i], (UINT8 *)&ev, 16);
            (void) memset (&t, 0, sizeof(t));
            tim = localtime ((time_t *)&ev.time);
            t.tm_year = tim->tm_year;
            t.tm_mon = tim->tm_mon;
            tmp_time = mktime(&t);
            if (!fla_index.mon && month_fm == FLA_INDEX_START)  //第一次建立索引
            {
                if (old_time != 0 && old_time != tmp_time)      //跨月建立索引 
                {                   
                    fla_index.mon = 0x1;                        //无意义的值
                    goto nextIndex;
                }       
                if (old_time != tmp_time)
                {
                    old_time = tmp_time;
                    tmp_flaIndex.mon = tmp_time;
                    tmp_flaIndex.flash = flash_wptr;
                    tmp_flaIndex.magic = 'y';
                }
                if (old_time == tmp_time)
                {
                    flash_wptr += 16;
                    tmp_flaIndex.end = flash_wptr;
                    Fm_write (month_fm, &tmp_flaIndex, sizeof(DBFlashIndexRec));
                }    
                   
            }
            else 
            {
                if (fla_index.mon == tmp_time)
                {
                    if (old_time != tmp_time)
                    {
                        old_time = tmp_time;
                        (void) memcpy (&tmp_flaIndex, &fla_index, sizeof(DBFlashIndexRec));
                    }
                    if (old_time == tmp_time)
                    {
                        flash_wptr += 16;
                        if (flash_wptr >= FLASH_ALARM_END)
                            flash_wptr = 0x1000000;
                            //flash_wptr = FLASH_ALARM_START+(flash_wptr-FLASH_ALARM_END);         

                        tmp_flaIndex.end = flash_wptr;
                        Fm_write (month_fm, &tmp_flaIndex, sizeof(DBFlashIndexRec));
                    }    
                }
                else
                {
nextIndex:          if (old_time != tmp_time)
                    {
                        old_time = tmp_time;
                        month_fm += FLA_INDEX_SIZE;
                        if (month_fm > FLA_INDEX_START + FLA_INDEX_MAX*FLA_INDEX_SIZE)
                            month_fm = FLA_INDEX_START; 
                        tmp_flaIndex.mon = tmp_time;
                        tmp_flaIndex.flash = flash_wptr;
                        tmp_flaIndex.magic = 'y';
                    }
                    if (old_time == tmp_time)
                    {
                        flash_wptr += 16;
                        if (flash_wptr >= FLASH_ALARM_END)
                            flash_wptr = 0x1000000;
                            //flash_wptr = FLASH_ALARM_START+(flash_wptr-FLASH_ALARM_END); 
                        tmp_flaIndex.end = flash_wptr;
                        Fm_write (month_fm, &tmp_flaIndex, sizeof(DBFlashIndexRec));
                    }
                }             
            }
            
            /*清除铁电和cache中的报警记录*/
            (void) memset (&ev, 0, sizeof (ev));
            Fm_write (REC_ADDR (cache_rptr), &ev, sizeof (ev));
            cache[cache_rptr].time = 0;
            cache[cache_rptr].type = 0;    
            cache_rptr = (cache_rptr+1)%ALARM_REC_NUM;
            
            i += 16;
        }
        
        /*写入FLASH*/
        Flash_write (fla_writeAddr, tmp, 256);
        
        ptrs_modified = TRUE;

        if ((flash_wptr & (FLASH_ERASE_SIZE - 1)) == 0x0)   /*在FLASH擦除边界*/
          flash_stage = 0;
        break;
    }
}

/*****************************************************************************/
/* 事件报警模块入口                                                          */
/*****************************************************************************/
void EV_main (void)
{
    if (ptrs_modified)
    {
        UINT16  cs[7];

        cs[0] = 0xBEEF;
        cs[1] = cache_wptr;
        cs[2] = cache_rptr;
        cs[3] = month_fm;
        *(UINT32 *) &cs[4] = flash_wptr;
        cs[6] = Modbus_crc ((UINT8 *) cs, 6*2);

        Fm_write (ALARM_PTR_START, cs, 7*2);
        Fm_write (ALARM_PTR2_START, cs, 7*2);

        ptrs_modified = FALSE;
    }

#ifdef HAVE_PWDNEV
    if (this_time != gSysSec)
    {
        UINT16  cs[4];
        
        this_time = gSysSec;
        
        *(UINT32 *) &cs[0] = this_time;
        cs[2] = 0x2BAD;
        cs[3] = Modbus_crc ((UINT8 *) cs, 6);
        
        Fm_write (ALARM_PWDN_START, cs, 8);
    }
#endif

    if (gSleepMode || gSysTicks < 700)
        return;

    if (Sys_getFlags (SYSFLAG_EVCHK))
    {
        EV_Check ();   //温度信息7s内不稳定
        
        EV_Dump ();
    }
}
