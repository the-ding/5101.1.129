#include <string.h>
#include <stddef.h>
#include "sys.h"
#include "drv_fm.h"
#include "db.h"
#include "db_ev.h"
#include "db_para_cfg.h"
#include "db_para_struct.h"
#include "modbus.h"
#include "ntc_tbl.h"
#include "drv_ads11x.h"


#define SET_SEM(x) (XGSEM = 0x0101 << (x)) 
#define TST_SEM(x) (XGSEM & 0x0001 << (x)) 
#define REL_SEM(x) (XGSEM = 0x0100 << (x)) 
/*
  do {SET_SEM(0);       // 试图锁定信号量 /      
      } while (!TST_SEM(0));   
      REL_SEM(0); */  
      
 static UINT32   AD_ticks = 0;  //
 static UINT8    BMU_add  = 0;  //
     
 UINT8          BMU_addn[2]; 
 UINT8    Equ_EN          = 0;
#if PTL_BALANCE

     UINT32   ulBal_Sec       = 0;
     UINT8    ucBal_stage     = 0; 
     UINT16   uiBal_arv       = 0;
     UINT8    ucBal_maxv_lecu = 0;
     UINT8    ucBal_maxv_batt = 0;


#endif 

    
#pragma DATA_SEG SHARED_DATA 
UINT16          Cell_Indexs[DEV_MAX];


DBSysRt         Sys_Rt;                 /*整车实时数据*/
DBSysShared     Sys_Shared;
DBDevRt     Sys_Lecu; //itrb            采集数据
#pragma DATA_SEG DEFAULT

DBSysAlarm      Sys_Alarm;
UINT16          Cell_Temp_Indexs[DEV_MAX];
UINT16          Cell_Temp_Volts[SYS_CELL_MAX];
UINT16          Cell_Volts[SYS_CELL_MAX];

#if TC_MAX
//    UINT8           Temp_Filt_Middle[7][TC_MAX*24];  //滤波温度20241107
    UINT8       Temp_Filt_Middle[5][TC_MAX*24];  //滤波温度20241107
#endif

UINT8           Temp_special[DEV_MAX]; 
UINT8           Temp_point[TEM_VALID_MAX];
UINT8           Rt_num    = 0;
UINT16          HPU_sta     = 0;
UINT16          loc_sta     = 0;
UINT16          SOC_k       = 0; 
UINT16          O_Voltage   = 0;
UINT16          T_Voltage   = 0;

TEST_PARAM      test_param = {0};
//根据db.h报警标志位个数修改定时器数量
#define ALARMS              19

#ifdef HAVE_L4_PARAMS
#define TIMER_ID_MAX        4*ALARMS
#elif defined(HAVE_L3_PARAMS)
#define TIMER_ID_MAX        3*ALARMS
#else
#define TIMER_ID_MAX        2*ALARMS
#endif

#define EXPIRED(id, ms) ((gSysTicks - sticks[id]) > MS2TICKS(ms))
#define EXPIREDS(id, s) ((gSysTicks - sticks[id]) > (s)*100UL)
#define TIMER_RESET(id) sticks[id] = gSysTicks;

volatile UINT32  sticks[TIMER_ID_MAX + DEV_MAX+1];//增加了温度的1个定时器

#define CHK1(v,P,T,W) \
if (v Sys_Alarm_Para.L1.P) \
{ if (EXPIREDS (T, Sys_Alarm_Para.L1.##P##_keep)) w1 |= W; \
} else { TIMER_RESET (T); }

#define CHK2(v,P,T,W) \
if(!(w1 & W)) \
if (v Sys_Alarm_Para.L2.P)\
{ if (EXPIREDS (ALARMS + T, Sys_Alarm_Para.L2.##P##_keep)) w2 |= W; \
} else { TIMER_RESET (ALARMS + T); }

#define CHK3(v,P,T,W) \
if((!(w1 & W)) && (!(w2 & W))) \
if (v Sys_Alarm_Para.L3.P)\
{ if (EXPIREDS (2*ALARMS + T, Sys_Alarm_Para.L3.##P##_keep)) w3 |= W; \
} else { TIMER_RESET (2*ALARMS + T);  }

#define CHK4(v,P,T,W) \
if((!(w1 & W)) && (!(w2 & W)) && (!(w3 & W)))  \
if (v Sys_Alarm_Para.L4.P) \
{ if (EXPIREDS (3*ALARMS + T, Sys_Alarm_Para.L4.##P##_keep)) w4 |= W; \
} else { TIMER_RESET (3*ALARMS + T); } //{ { {
//    CHK1 (Sys_Rt.max_v >=, vol_high, 0, WARN_VOL_HIGH)  

void ZJVX_alarm_act(void);
void SZYM_alarm_act(void);

/*****************************************************************************/
/* 实时数据模块－获取采集模块的单体电压值                                    */
/*****************************************************************************/
UINT16 *RT_Volts (UINT8 dev)
{
    UINT16  idx;

    if (dev >= DEV_MAX)
        return NULL;

    idx = Cell_Indexs[dev];
    if (idx == 0xffff)
        return NULL;

    return &Cell_Volts[idx];
}

/*****************************************************************************/
/* 实时数据模块－获取系统第N节电池单体电压值，返回0表示无效                  */
/*****************************************************************************/
UINT16 RT_Voltage (UINT16 num, UINT8 *devNo, UINT8 *cellNo)
{
    UINT16 v = 0;
    UINT16 pos = 0;
    UINT8 i;

    if (num < Sys_Rt.bodys)
    {
        for (i = 0; i < DEV_MAX; i++)
        {
            UINT8 bodys = LECU_Rt[i].bodys & 0x7f;

            if (bodys == 0)
                continue;

            pos += bodys;

            if (pos > num)
            {
                UINT8 cn = bodys - (pos - num);

                v = Cell_Volts[Cell_Indexs[i] + cn];
                if (devNo != NULL) *devNo = i;
                if (cellNo != NULL) *cellNo = cn;
                break;
            }
        }
    }
    return v;
}

/*****************************************************************************/
/* 实时数据模块－设置 "安/秒" (同步到XGATE)                                  */
/*****************************************************************************/
void RT_SetAms (UINT32 as)
{
    UINT32 ams = as * 1000UL;   /* 单位换算As -> Ams */

  /*锁定信号量*/ 
     do  { SET_SEM(0);}      /* 试图锁定信号量 */     
     while (!TST_SEM(0)); 
   /*完成锁定*/  
       
   Sys_Rt.Ams_back = Sys_Rt.TKAms = ams;
    
    REL_SEM(0); /*释放信号量*/ 



    /*

    *(volatile UINT32 *) &Sys_Rt.Ams_back = ams;

    for(;;)
    {
        *(volatile UINT32 *) &Sys_Rt.Ams = ams;

        ams = *(volatile UINT32 *) &Sys_Rt.Ams_back;

        if (ams == *(volatile UINT32 *) &Sys_Rt.Ams)
            break;
    }
    */
    
}
/*****************************************************************************/
/* 实时数据模块－设置SOC百分比 k                                              */
/*****************************************************************************/
void RT_SetSOC_K (UINT16 percent)
{
    UINT32    ah;
    DBSysSoc  soc;
    UINT16    rated_tvol = 0;
    
    rated_tvol = (UINT16)((UINT32)BattColtd_volmax[BattType-1] * Sys_Rt.bodys / 100);
 
    //Sys_Rt.soc = percent;
    SOC_k     = percent;
    Sys_Rt.soc= percent/10; 
   
    soc.TKAms = (UINT32)percent  * Sys_Para.cap_stand * 18/10;     // 百分比 X 额定电量 X 3600秒 
    //soc.TKAms = percent;     
    //soc.TKAms *= Sys_Para.cap_stand ;
    //soc.TKAms *= 180 ;       
    if (rated_tvol >= Sys_Rt.to_vol)   
        soc.wh = soc.TKAms * 2 * Sys_Rt.to_vol /10 /3600 ;// soc.wh = soc.TKAms * Sys_Rt.to_vol /10  /3600 /1000 *2000;
    else
        soc.wh = soc.TKAms * 2 * rated_tvol /10 /3600 ;// soc.wh = soc.TKAms * rated_tvol/10  /3600 /1000 *2000;
    

    soc.crc = 0;
    soc.crc = 0 - Chksum (&soc, sizeof(soc));

   
     do 
     {
        SET_SEM(0);           
     } 
     while (!TST_SEM(0));   
    
    Sys_Rt.TKAms = soc.TKAms;
    Sys_Rt.Ams_back = soc.TKAms;
    REL_SEM(0);     
   

    soc.crc = Modbus_crc ((UINT8 *)&soc, sizeof(soc) - 2);

    Fm_write (SOCINFO_START, &soc, sizeof(soc));
    Fm_write (SOCINFO_START + 24, &soc, sizeof(soc));

    Sys_Rt.wh = soc.wh;
    Sys_Rt.ws = soc.wh * 3600;

   ////////////////////////////////////////// RT_SetAms (soc.as); 
    
}
/*****************************************************************************/
/* 实时数据模块－设置SOC百分比                                               */
/*****************************************************************************/
void RT_SetSOC (UINT16 percent)
{
    UINT32    ah;
    DBSysSoc  soc;
    UINT16    rated_tvol = 0;
    
    rated_tvol = (UINT16)((UINT32)BattColtd_volmax[BattType-1] * Sys_Rt.bodys / 100);
 
    Sys_Rt.soc = percent;
    SOC_k      = percent*10; 
   
    soc.TKAms = (UINT32)percent  * Sys_Para.cap_stand * 3600 / 100 / 2;     // 百分比 X 额定电量 X 3600秒 
/*    if (rated_tvol >= Sys_Rt.to_vol)   
        soc.wh = soc.TKAms * 2 * Sys_Rt.to_vol /10 /3600 ;// soc.wh = soc.TKAms * Sys_Rt.to_vol /10  /3600 /1000 *2000;
    else
        soc.wh = soc.TKAms * 2 * rated_tvol /10 /3600 ;// soc.wh = soc.TKAms * rated_tvol/10  /3600 /1000 *2000;
*/    
    soc.wh  = (UINT32)percent  * Sys_Para.cap_stand * 1035 / 1000;
    soc.crc = 0;
    soc.crc = 0 - Chksum (&soc, sizeof(soc));

   
     do 
     {
        SET_SEM(0);           
     } 
     while (!TST_SEM(0));   
    
    Sys_Rt.TKAms =
    Sys_Rt.Ams_back = soc.TKAms;
    REL_SEM(0);     
   

    soc.crc = Modbus_crc ((UINT8 *)&soc, sizeof(soc) - 2);

    Fm_write (SOCINFO_START, &soc, sizeof(soc));
    Fm_write (SOCINFO_START + 24, &soc, sizeof(soc));

    Sys_Rt.wh = soc.wh;
    Sys_Rt.ws = soc.wh * 3600;

   ////////////////////////////////////////// RT_SetAms (soc.as); 
    
}

/*****************************************************************************/
/* 实时数据模块初始化                                                        */
/*****************************************************************************/
void RT_Init (void)
{
    BOOL    valid = TRUE;
    UINT8   i;
    DBSysSoc soc;
    UINT16  soctmp;

    /*整车实时数据清零*/
    (void) memset (&Sys_Rt, 0, sizeof(Sys_Rt));
    (void) memset (&Sys_Alarm, 0, sizeof(Sys_Alarm));
    (void) memset (&Cell_Indexs, 0xff, sizeof(Cell_Indexs));
    (void) memset (&Cell_Temp_Indexs, 0xff, sizeof(Cell_Temp_Indexs));
    (void) memset (&Cell_Volts, 0, sizeof(Cell_Volts));
/*    Cell_Temp_Indexs[0]=0;
    Cell_Temp_Indexs[1]=36;
    Cell_Temp_Indexs[2]=72;
    Cell_Temp_Indexs[3]=108;
    Cell_Temp_Indexs[4]=144;
    Cell_Temp_Indexs[5]=180;
    /**/
    Sys_Shared.ask = 0;
    Sys_Shared.iso_age = 0;
    Sys_Shared.iso_req = TRUE;

    for (i = 0; i < TIMER_ID_MAX; i++)
        TIMER_RESET (i);

 /*尝试恢复“安时”数据*/
    Fm_read (SOCINFO_START, &soc, sizeof(soc));

    if (soc.crc != Modbus_crc ((UINT8 *)&soc, sizeof(soc) - 2))
    {
        Fm_read (SOCINFO_START + 24, &soc, sizeof(soc));

        if (soc.crc != Modbus_crc ((UINT8 *)&soc, sizeof(soc) - 2))
          valid = FALSE;
    }


      
    /*“安时”数据有效*/
    if (valid)
    {
      do{
          SET_SEM(0);        /* 试图锁定信号量 */      
        } 
      while (!TST_SEM(0)); 
          
      Sys_Rt.TKAms =
      Sys_Rt.Ams_back = soc.TKAms;//////////////////////////////////
        
      REL_SEM(0);     


      Sys_Rt.wh = soc.wh;
      Sys_Rt.ws = Sys_Rt.wh*3600 ;
        //Sys_Rt.soc = (UINT16)(((soc.Ams + 4500000)/9000000) * 100 / Sys_Para.cap_stand);
      soctmp = (UINT16)((soc.TKAms * 2/36) * 10 / Sys_Para.cap_stand);//soctmp = (UINT16)((soc.TKAms * 2000/3600/1000) * 1000 / Sys_Para.cap_stand);
      SOC_k = soctmp;  
      if(SOC_k>1000)  SOC_k =1000;    
        
      if((soctmp%10)>=5)
        {
          soctmp = soctmp/10+1;
        }
      else
        {
          soctmp = soctmp/10;
        }
        
      Sys_Rt.soc = soctmp;
        
      if(Sys_Rt.soc > 102)
          Sys_Rt.soc = 0;
        
      if(Sys_Rt.soc > 100)
          Sys_Rt.soc = 100;
        
    }
    else   /*“安时”数据wu效*/
    {
        do {
            SET_SEM(0);        /* 试图锁定信号量 */      
           }
        while (!TST_SEM(0)); 
        
        Sys_Rt.TKAms = (UINT32)Sys_Para.cap_stand * 3600 * 1000 /2;      //new
     
        REL_SEM(0);     
        
        Sys_Rt.Ams_back =  Sys_Rt.TKAms;

        Sys_Rt.wh = soc.wh;
        Sys_Rt.ws = Sys_Rt.wh*3600;
       
        //Sys_Rt.soc = (UINT16)(((soc.Ams + 4500000)/9000000) * 100 / Sys_Para.cap_stand);
        soctmp = 50;
        SOC_k = soctmp*10; 
        Sys_Rt.soc = soctmp;
       
    }

/*测试*/
//Sys_Para.cap_stand = 300;
//Sys_Rt.to_vol = 5600;
//RT_SetSOC (100);
#if TC_MAX
    for(i=0;i<45;i++) {        //      温度滤波数据初始化 20241107  
        Temp_Filt_Middle[0][i]=25;
    }
#endif
}

/*****************************************************************************/
/* 更新电池电压实时数据                                                      */
/*****************************************************************************/
static void UpdateVolRt (void)     
{
    UINT16      i;
    UINT8       j,k;
    UINT16      Bat_A_num=0;         //电池总数量计数
    UINT8       Bat_L_num=0;         //电池模块数量计数    
    UINT16      bodys = 0;
    UINT32      to_vol = 0;
    UINT32      avv=0;
    UINT8       maxv_lecu =0;       //最大电压lecu号
    UINT8       maxv_batt =0;       //最大电压电池号
    UINT16      max_v =0;
    UINT16      to_max_v=0;
    UINT16      to_min_v=0xffff;

    UINT8       minv_lecu =0;       // 最小电压lecu号
    UINT8       minv_batt =0;       //最小电压电池号
    UINT16      min_v =0xffff;
    UINT32      sumv = 0;
    UINT8       maxnr, minnr;
    
    //O_Voltage = (Cell_Temp_Volts[60]&0x7fff);
////////////////////////////中转屏蔽数据//////////////////////////////////////////
    Cell_Indexs[0]=0;                                         //初始计数0
    for(i=0;i<VC_MAX;i++){                                   //判断板数量
      if(i<OEM_Para.lecu_num){                                //小于设定数量
        for(j=0;j<2;j++){
          for(k=0;k<18;k++){
            if(((LECU_NVT[i].Cell_Valid[j]>>k)&0x0001)==1){
                if(Cell_Temp_Volts[i*36+j*18+k] == 0xFF) {
                    Cell_Temp_Volts[i*36+j*18+k] = 0;
                }
              Cell_Volts[Bat_A_num++]=Cell_Temp_Volts[i*36+j*18+k];
              Bat_L_num++;
            }
          }
        }
        LECU_Rt[i].bodys=(Bat_L_num|0x80); 
        Cell_Indexs[i+1]=Cell_Indexs[i]+Bat_L_num;
        Bat_L_num=0;
      }
    }
////////////////////////////////////////////////////////////////////////// 
    for (i = 0; i < VC_MAX; i++)
    {
        if (!LECU_Rt[i].bodys)
        {
            continue;
        }
        max_v     = 0;
        min_v     = 0xffff;
        maxv_batt = 0;
        minv_batt = 0;
        sumv      = 0;
        for(j=0;j<(LECU_Rt[i].bodys & 0x7f);j++) {
            sumv +=Cell_Volts[j+Cell_Indexs[i]]&0x7fff; 
            if(max_v<Cell_Volts[j+Cell_Indexs[i]]) {
                max_v=Cell_Volts[j+Cell_Indexs[i]];
                maxv_batt = j+1;
            }
            if(min_v>Cell_Volts[j+Cell_Indexs[i]]) {
                min_v=Cell_Volts[j+Cell_Indexs[i]];
                minv_batt = j+1;
            }
        }
        LECU_Rt[i].maxv   = max_v;
        LECU_Rt[i].minv   = min_v;
        LECU_Rt[i].maxvnr = maxv_batt;
        LECU_Rt[i].minvnr = minv_batt;
        LECU_Rt[i].volt   = sumv/100;
        LECU_Rt[i].avv    = sumv/(LECU_Rt[i].bodys & 0x7f);
       
        bodys   += LECU_Rt[i].bodys & 0x7f;
        to_vol  += LECU_Rt[i].volt & 0x7fff;
        avv     += LECU_Rt[i].avv & 0x7fff;
         
        if (to_max_v < (LECU_Rt[i].maxv&0x7fff)){
            maxv_lecu = i;
            to_max_v  = LECU_Rt[i].maxv & 0x7fff;
        }
        if (to_min_v > (LECU_Rt[i].minv&0x7fff)){
            minv_lecu = i;
            to_min_v  = LECU_Rt[i].minv & 0x7fff;
        }
    }
    Sys_Rt.bodys    = bodys;
    if(!bodys){

        Sys_Rt.to_vol = 0;  //精度转换为0.1v

        Sys_Rt.max_v = 0;
        Sys_Rt.maxv_lecu  = 0;
        Sys_Rt.maxv_batt  = 0;

        Sys_Rt.min_v = 0;
        Sys_Rt.minv_lecu = 0;
        Sys_Rt.minv_batt = 0;
        Sys_Rt.avg_v  = 0;
      

    }else{
        Sys_Rt.to_vol     = to_vol;
        Sys_Rt.avg_v      = avv/Sys_Rt.num_lecu;
        Sys_Rt.max_v      = to_max_v;
        Sys_Rt.maxv_lecu  = maxv_lecu + 1;
        Sys_Rt.maxv_batt  = LECU_Rt[maxv_lecu].maxvnr;

        Sys_Rt.min_v      = to_min_v;
        Sys_Rt.minv_lecu  = minv_lecu + 1;
        Sys_Rt.minv_batt  = LECU_Rt[minv_lecu].minvnr;
    }
}
/*****************************************************************************/
/* 更新电池SOC实时数据                                                      */
/*****************************************************************************/
static void UpdateSOCRt (void) {
     
    UINT8       y;
    SINT32      diff;
    UINT32      TKams;
    UINT32      TKams1;
    UINT32      TKAmsMax;
    UINT16      soctmp=0;
    UINT16      rated_tvol = 0;  //标称最大电压
    
     TKAmsMax = Sys_Para.cap_stand ;
                          //* 3600000UL / 2000;
     TKAmsMax = TKAmsMax * 1800;
      
       //锁定信号量// 
     do  { SET_SEM(0);}      // 试图锁定信号量 //     
     while (!TST_SEM(0)); 
      //完成锁定//  
      if(Sys_Rt.TKAms > 0xefffffff)  //1118ah最大
      {
         Sys_Rt.Ams_back = Sys_Rt.TKAms =0;
      }   

      if(Sys_Rt.TKAms > TKAmsMax)  //防止减到0后益处
      {
          Sys_Rt.TKAms = TKAmsMax;
      }
    TKams = Sys_Rt.TKAms;
    
    REL_SEM(0); 
    
    diff = (TKams - Sys_Rt.Ams_back);

    if (diff != 0)
    {
        if (diff >= 0) 
        {
          
          Sys_Rt.ws += (diff*(Sys_Rt.to_vol/10)) * 2; //△t=1s //Sys_Rt.ws += (diff*(Sys_Rt.to_vol/10)) * 2000 /1000; //△t=1s 
          
          
         }
        else
        {
               if((-diff*(Sys_Rt.to_vol/10)) * 2  > Sys_Rt.ws)
               {
                 Sys_Rt.ws = 0;
               }
               else   Sys_Rt.ws += (diff*(Sys_Rt.to_vol/10)) * 2; //△t=1s
        }
       
       
       
       
        Sys_Rt.wh = Sys_Rt.ws/3600;
        Sys_Rt.Ams_back = TKams;
    }
      //  soctmp = (UINT16)((TKams * 2000 / 3600 /1000)/ Sys_Para.cap_stand) *1000;
    soctmp  = (UINT16)(((TKams  *10 / 18 )/ Sys_Para.cap_stand));
    SOC_k   = soctmp; 
    if(SOC_k>1000) SOC_k =1000;
          
    if((soctmp%10)>=5)
    {
      soctmp=soctmp/10+1;
    } 
    else
    {
      soctmp=soctmp/10;
    }
    
    Sys_Rt.soc=soctmp;

    if(Sys_Rt.soc > 100)
      Sys_Rt.soc = 100;
}
/*****************************************************************************/
/* 电流计算                                                        */
/*****************************************************************************/
void Cur_Count(void) {
    UINT8 i = 0;
    //电流转移至，系统故障检测函数
    Sys_Rt.current = Sys_Shared.cur;
    for(i=0;i<DEV_MAX;i++) {
        loc_sta |= ((LECU_Rt[i].in_stauts&0x0001)<<i); 
        if(loc_sta > HPU_sta){
            HPU_sta = loc_sta; 
        }
    }

}
/*****************************************************************************/
/* 更新电池温度实时数据                                                      */
/*****************************************************************************/
//450温度采集重构，变成跟电压一样的每次36个温度
static void UpdateTempRt (void)
{
    UINT16  i;          // 板号索引（0~8），使用 UINT16 因为后面要用作索引计算
    UINT8   j, k;       // j=组号(0~2)，k=位号(0~11)
    UINT16  T_A_num = 0;    // 全局有效温度总数
    UINT16  T_L_num = 0;    // 单块板的有效温度数（临时变量）
    UINT32  sumt = 0;       // 温度累加和（使用32位无符号，无溢出风险）
    UINT8   max_t = 0;      // 临时记录当前扫描到的最高温度
    UINT8   min_t = 0xff;   // 临时记录当前扫描到的最低温度
    UINT8   first_valid = 0; // 标记是否已找到第一个有效温度

    //第一步：清空 Temp_point
    for(i = 0; i < TEM_VALID_MAX; i++) {
        Temp_point[i] = 0;   // 全部清零，防止残留旧数据
    }
    //内部自己用的温度点
    for(i = 0; i < DEV_MAX; i++) {
        Temp_special[i] = 0x00;   // 默认无效
    }
    //第二步：遍历所有板卡
    for(i = 0; i < DEV_MAX; i++) {
        T_L_num = 0;    // 重置本板有效温度计数器
        
        //先清空本板的 tmps 数组（防止残留上一轮的旧数据）
        (void) memset(LECU_Rt[i].tmps, 0, sizeof(LECU_Rt[i].tmps));

        //检查板卡是否有效：i 必须在 VC_MAX 范围内，且小于配置的板卡数量
        if(i >= VC_MAX || i >= OEM_Para.lecu_num) {
            // 无效板卡：将 tmpnum 设为 0（bit7=0 表示板不存在）
            LECU_Rt[i].tmpnum = 0;
            continue;   // 跳过本板，不处理温度数据
        }
        //取特殊温度点
        Temp_special[i] = VTemp_Middle[(UINT16)i * 40 + 19];
        //处理有效板卡：同时完成提取、赋值、统计
        for(j = 0; j < 2; j++) {          // 3组
            for(k = 0; k < 20; k++) {     // 每组16位，统一按16位扫，无效位跳过
                UINT16 phys_slot;
                UINT16 phys_index;
                UINT8  temp_val;

                // 检查 T_Valid[j] 的第 k 位是否为1（有效）
                if(((LECU_NVT[i].T_Valid[j] >> k) & 0x0001) == 0) {
                    continue;   // 无效位直接跳过
                }

                // 物理槽位：组号*20 + 位号
                phys_slot = (UINT16)j * 20 + k;

                // 超出本板物理点数则跳过（每板40点）
                if(phys_slot >= 40) {
                    continue;
                }

                // 计算在 VTemp_Middle 一维数组中的全局索引
                phys_index = (UINT16)i * 40 + phys_slot;
                // 取出温度值
                temp_val = VTemp_Middle[phys_index];

                // 无效温度过滤：0xFF 视为采集异常，直接跳过，不参与统计
                // 考虑到屏幕显示先不过滤，等后续再处理
                if(temp_val == 0xFF) {
                    temp_val = 55;
                }

                // 1. 存入 Temp_point（供外部使用）
                Temp_point[(UINT16)i * 36 + T_L_num] = temp_val;
                // 2. 存入本板 tmps 数组（结构体存储）
                LECU_Rt[i].tmps[T_L_num] = temp_val;
                
                // 3. 累加统计信息
                T_L_num++;          // 本板有效数+1
                T_A_num++;          // 全局有效数+1
                sumt += temp_val;   // 累加和

                if(first_valid == 0) {
                    // 第一个有效温度，同时初始化最大/最小，防止
                    first_valid = 1;
                    max_t = temp_val;
                    min_t = temp_val;
                    Sys_Rt.maxt_lecu = (UINT8)i+1;            // 板号 0-based
                    Sys_Rt.maxt_batt = (UINT8)T_L_num;       // 槽位号 1-based
                    Sys_Rt.mint_lecu = (UINT8)i+1;
                    Sys_Rt.mint_batt = (UINT8)T_L_num;
                } else{
                    // 更新最大值（使用临时变量，最后才写入 Sys_Rt）
                    if(temp_val > max_t) {
                        max_t = temp_val;
                        Sys_Rt.maxt_lecu = (UINT8)i+1;
                        Sys_Rt.maxt_batt = (UINT8)T_L_num;
                    }
                    // 更新最小值（使用临时变量，最后才写入 Sys_Rt）
                    if(temp_val < min_t) {
                        min_t = temp_val;
                        Sys_Rt.mint_lecu = (UINT8)i+1;
                        Sys_Rt.mint_batt = (UINT8)T_L_num;
                    }
                }
            }
        }

        //将本板有效温度数存入结构体（保留实际有效数量，不强制覆盖）
        // bit7=1 表示板存在，低7位存有效数量
        LECU_Rt[i].tmpnum = (UINT8)(T_L_num | 0x80);
    }

    // ========== 第三步：循环结束后，统一更新系统极值 ==========
    if(T_A_num > 0) {
        // 将临时变量 max_t 写入系统全局变量
        Sys_Rt.max_t = max_t;
        Sys_Rt.min_t = min_t;
    } else {
        // 没有有效温度时，极值设为0
        Sys_Rt.max_t = 0;
        Sys_Rt.min_t = 0;
    }

    // ========== 第四步：计算平均温度 ==========
    if(T_A_num > 0) {
        // 结果强转为 UINT8（温度值范围 0~255）
        Sys_Rt.avg_t = (UINT8)(sumt / T_A_num);
    } else {
        Sys_Rt.avg_t = 0;
    }
}


/*****************************************************************************/
/* 计算整车实时数据                                                          */
/*****************************************************************************/
static void update_rt (void)
{
  UINT8 i=0;
  UINT8 lecus = 0;
  reg_err2_code = 0;    
  for (i = 0; i < DEV_MAX; i++)
    {
        if (!LECU_Rt[i].bodys)
        {
            Sys_Rt.lecu_actives &= ~(1UL << i);
            continue;
        }
        Sys_Rt.lecu_actives |= 1UL << i;
        //用来判断采集板状态 
        if(LECU_Rt[i].soh_1 != LECU_Rt[i].soh){
          LECU_Rt[i].soh_1=LECU_Rt[i].soh;
          lecus++; 
          reg_err2_code |= (1<<i);      
        }
     }
     Sys_Rt.num_lecu = lecus;
}

/*****************************************************************************/
/*            计算ALARM信息                                                  */
/*****************************************************************************/
static void alarm_proc (void)
{
    int     err;
    UINT8   i;
    UINT8   epmax = 0;
    UINT8   epmaxnr = 0;
    UINT8   epmin = 0xFF;
    UINT8   epminnr = 0;
    UINT32  w1 = 0, w2 = 0, w3 = 0, w4 = 0;


    //电压高
    CHK1 (Sys_Rt.max_v >=, vol_high, 0, WARN_VOL_HIGH)
    CHK2 (Sys_Rt.max_v >=, vol_high, 0, WARN_VOL_HIGH)
#ifdef HAVE_L3_PARAMS
    CHK3 (Sys_Rt.max_v >=, vol_high, 0, WARN_VOL_HIGH)
#ifdef HAVE_L4_PARAMS
    CHK4 (Sys_Rt.max_v >=, vol_high, 0, WARN_VOL_HIGH)
#endif
    //}
#endif
    //}}


    //电压过低
    CHK1 (Sys_Rt.min_v <=, vol_low, 1, WARN_VOL_LOW)
    CHK2 (Sys_Rt.min_v <=, vol_low, 1, WARN_VOL_LOW)
#ifdef HAVE_L3_PARAMS
    CHK3 (Sys_Rt.min_v <=, vol_low, 1, WARN_VOL_LOW)
#ifdef HAVE_L4_PARAMS
    CHK4 (Sys_Rt.min_v <=, vol_low, 1, WARN_VOL_LOW)
#endif
    //}
#endif
    //}}


    //互差阈值过大
    err = Sys_Rt.max_v - Sys_Rt.min_v;

    CHK1 (err >=, vol_unba, 2, WARN_VOL_UNBA)
    CHK2 (err >=, vol_unba, 2, WARN_VOL_UNBA)
#ifdef HAVE_L3_PARAMS
    CHK3 (err >=, vol_unba, 2, WARN_VOL_UNBA)
#ifdef HAVE_L4_PARAMS
    CHK4 (err >=, vol_unba, 2, WARN_VOL_UNBA)
#endif
    //}
#endif
    //}}


    //总电压高
    CHK1 (Sys_Rt.to_vol >=, tvol_high, 3, WARN_TVOL_HIGH)
    CHK2 (Sys_Rt.to_vol >=, tvol_high, 3, WARN_TVOL_HIGH)
#ifdef HAVE_L3_PARAMS
    CHK3 (Sys_Rt.to_vol >=, tvol_high, 3, WARN_TVOL_HIGH)
#ifdef HAVE_L4_PARAMS
    CHK4 (Sys_Rt.to_vol >=, tvol_high, 3, WARN_TVOL_HIGH)
#endif
    //}
#endif
    //}}


    //总电压过低
    CHK1 (Sys_Rt.to_vol <=, tvol_low, 4, WARN_TVOL_LOW)
    CHK2 (Sys_Rt.to_vol <=, tvol_low, 4, WARN_TVOL_LOW)
#ifdef HAVE_L3_PARAMS
    CHK3 (Sys_Rt.to_vol <=, tvol_low, 4, WARN_TVOL_LOW)
#ifdef HAVE_L4_PARAMS
    CHK4 (Sys_Rt.to_vol <=, tvol_low, 4, WARN_TVOL_LOW)
#endif
    //}
#endif
    //}}

    //温度高
    CHK1 (Sys_Rt.max_t >, tmp_high, 5, WARN_TMP_HIGH)
    CHK2 (Sys_Rt.max_t >, tmp_high, 5, WARN_TMP_HIGH)
#ifdef HAVE_L3_PARAMS
    CHK3 (Sys_Rt.max_t >, tmp_high, 5, WARN_TMP_HIGH)
#ifdef HAVE_L4_PARAMS
    CHK4 (Sys_Rt.max_t >, tmp_high, 5, WARN_TMP_HIGH)
#endif
    //}
#endif
    //}}


    //温度低
    CHK1 (Sys_Rt.min_t <, tmp_low, 6, WARN_TMP_LOW)
    CHK2 (Sys_Rt.min_t <, tmp_low, 6, WARN_TMP_LOW)
#ifdef HAVE_L3_PARAMS
    CHK3 (Sys_Rt.min_t <, tmp_low, 6, WARN_TMP_LOW)
#ifdef HAVE_L4_PARAMS
    CHK4 (Sys_Rt.min_t <, tmp_low, 6, WARN_TMP_LOW)
#endif
    //}
#endif
    //}}


    //温差大 (BS111/BS113温度采集周期为8*6s)
    err = (int)(Sys_Rt.max_t - Sys_Rt.min_t);

    CHK1 (err >=, tmp_unba, 7, WARN_TMP_UNBA)
    CHK2 (err >=, tmp_unba, 7, WARN_TMP_UNBA)
#ifdef HAVE_L3_PARAMS
    CHK3 (err >=, tmp_unba, 7, WARN_TMP_UNBA)
#ifdef HAVE_L4_PARAMS
    CHK4 (err >=, tmp_unba, 7, WARN_TMP_UNBA)
#endif
    //}
#endif
    //}}


skip_temp_chk:;



//////////////////////////////////////////////////////////////////////////////////////
  if(OEM_Para.cur_dir&0x40)   ///调换报警方向 0x40  正电流充电报警  
{

  //放电电流大
    if(Sys_Rt.current <= 0 ) //放电电流为负
    {
    
         UINT16  C_TREMP = 0;
          C_TREMP = 0 -Sys_Rt.current ;
    
         
        CHK1 (C_TREMP >, dischg_cur_high, 11, WARN_DISCHG_CUR_HIGH)
        CHK2 (C_TREMP >, dischg_cur_high, 11, WARN_DISCHG_CUR_HIGH)
#ifdef HAVE_L3_PARAMS
        CHK3 (C_TREMP >, dischg_cur_high, 11, WARN_DISCHG_CUR_HIGH)
#ifdef HAVE_L4_PARAMS
        CHK4 (C_TREMP >, dischg_cur_high, 11, WARN_DISCHG_CUR_HIGH)
#endif
        //}
#endif
        TIMER_RESET(12);
        TIMER_RESET(12+19);
        TIMER_RESET(12+2*19);
        //}}
    }
    else
    {
          
            UINT16  C_TREMP = 0;
            C_TREMP = Sys_Rt.current ;//正电流
          
        //if (Sys_Rt.charge_sts != 0) //充电状态
        {
            //充电电流大
           
            CHK1 (C_TREMP >, chg_cur_high, 12, WARN_CHG_CUR_HIGH) //Sys_Rt.current
            CHK2 (C_TREMP >, chg_cur_high, 12, WARN_CHG_CUR_HIGH)
             #ifdef HAVE_L3_PARAMS
            CHK3 (C_TREMP >, chg_cur_high, 12, WARN_CHG_CUR_HIGH)
             #ifdef HAVE_L4_PARAMS
            CHK4 (C_TREMP >, chg_cur_high, 12, WARN_CHG_CUR_HIGH)
             #endif
            //}
             #endif
              TIMER_RESET(11);
              TIMER_RESET(11+19);
              TIMER_RESET(11+2*19);
            //}}

            //}}
        }
       
    }

  

}
else//原来的默认的报警放方向
{

    //放电电流大
    if(Sys_Rt.current >= 0 ) //chong dian zheng
    {
        CHK1 (Sys_Rt.current >, dischg_cur_high, 11, WARN_DISCHG_CUR_HIGH)
        CHK2 (Sys_Rt.current >, dischg_cur_high, 11, WARN_DISCHG_CUR_HIGH)
#ifdef HAVE_L3_PARAMS
        CHK3 (Sys_Rt.current >, dischg_cur_high, 11, WARN_DISCHG_CUR_HIGH)
#ifdef HAVE_L4_PARAMS
        CHK4 (Sys_Rt.current >, dischg_cur_high, 11, WARN_DISCHG_CUR_HIGH)
#endif
        //}
#endif
        TIMER_RESET(12);
        TIMER_RESET(12+19);
        TIMER_RESET(12+2*19);
        //}}
    }
    else
    {
          
            UINT16  C_TREMP = 0;
            C_TREMP = 0 -Sys_Rt.current ;
          
      //  if (Sys_Rt.charge_sts != 0) //充电状态
        {
            //充电电流大
           
            CHK1 (C_TREMP >, chg_cur_high, 12, WARN_CHG_CUR_HIGH)
            CHK2 (C_TREMP >, chg_cur_high, 12, WARN_CHG_CUR_HIGH)
             #ifdef HAVE_L3_PARAMS
            CHK3 (C_TREMP >, chg_cur_high, 12, WARN_CHG_CUR_HIGH)
             #ifdef HAVE_L4_PARAMS
            CHK4 (C_TREMP >, chg_cur_high, 12, WARN_CHG_CUR_HIGH)
             #endif
            //}
             #endif
                TIMER_RESET(11);
                TIMER_RESET(11+19);
                TIMER_RESET(11+2*19);
            //}}


            //}}
        }
        
    }

  
  
  
  
  
  
 } 
  
  
  
 
  
    
    
    // SOC过高
    CHK1 (Sys_Rt.soc >=, soc_high, 15, WARN_SOC_HIGH)
    CHK2 (Sys_Rt.soc >=, soc_high, 15, WARN_SOC_HIGH)
#ifdef HAVE_L3_PARAMS
    CHK3 (Sys_Rt.soc >=, soc_high, 15, WARN_SOC_HIGH)
#ifdef HAVE_L4_PARAMS
    CHK4 (Sys_Rt.soc >=, soc_high, 15, WARN_SOC_HIGH)
#endif
    //}
#endif
    //}}
    
    // SOC过低
    CHK1 (Sys_Rt.soc <, soc_low, 16, WARN_SOC_LOW)
    CHK2 (Sys_Rt.soc <, soc_low, 16, WARN_SOC_LOW)
#ifdef HAVE_L3_PARAMS
    CHK3 (Sys_Rt.soc <, soc_low, 16, WARN_SOC_LOW)
#ifdef HAVE_L4_PARAMS
    CHK4 (Sys_Rt.soc <, soc_low, 16, WARN_SOC_LOW)
#endif
    //}
#endif
    //}}

    


#if PTL_HAVE_ISO
    
     
    if(Sys_Rt.charge_sts == 0 ) 
    {
      
       if(Sys_Shared.iso_opv&0x80000000)
       {
        
          w1 &= ~WARN_ISO_FAULT;

         Sys_Shared.iso_opv &= 0x7fffffff;
     
        if( Sys_Shared.iso_age < 5 && Sys_Rt.total_vol_res > 50) //
         {
   
      
      
        //总正失效
        CHK1 (Sys_Shared.iso_opv <, iso_low, 17, WARN_POSRES_LOW)
        CHK2 (Sys_Shared.iso_opv <, iso_low, 17, WARN_POSRES_LOW)
       #ifdef HAVE_L3_PARAMS
        CHK3 (Sys_Shared.iso_opv <, iso_low, 17, WARN_POSRES_LOW)
       #ifdef HAVE_L4_PARAMS
        CHK4 (Sys_Shared.iso_opv <, iso_low, 17, WARN_POSRES_LOW)
       #endif
        //}
       #endif
        //}}

        //总负失效
        CHK1 (Sys_Shared.iso_opv <, iso_low, 18, WARN_NEGRES_LOW)
        CHK2 (Sys_Shared.iso_opv <, iso_low, 18, WARN_NEGRES_LOW)
       #ifdef HAVE_L3_PARAMS
        CHK3 (Sys_Shared.iso_opv <, iso_low, 18, WARN_NEGRES_LOW)
        #ifdef HAVE_L4_PARAMS
        CHK4 (Sys_Shared.iso_opv <, iso_low, 18, WARN_NEGRES_LOW)
       #endif
        //}
        #endif
        //}}
      } 
      
        
     } 
     else
     {
     
         w1 |= WARN_ISO_FAULT;  // 绝缘失效
      
     }
        
    }
    
    
   
    
    
    
      
  
  
    
#endif

  
    // 温度失效
    if (Sys_Rt.max_t >= (150+55))
       {
        if (EXPIREDS (TIMER_ID_MAX, 10*1000))
            w1 |= WARN_TMP_FAULT;
        else
            w1 &= ~WARN_TMP_FAULT;
       }
    else
        TIMER_RESET (TIMER_ID_MAX);

    BMU_add++;
    if(BMU_add>5){        //1s12个模块只够轮询一次增加到5s
      BMU_add=0;
      //模块丢失(节点失效)
      if( (Sys_Rt.num_lecu != Sys_Para.lecu_num)) 
        w1 |=  WARN_LECU_LOST;
   //   else
   //     w1 &= ~WARN_LECU_LOST; 
      
      
    }
    
   
    
    
    
    
    
    
    
    

    Sys_Alarm.warning1 = w1;
    Sys_Alarm.warning2 = w2;
    Sys_Alarm.warning3 = w3;
    Sys_Alarm.warning4 = w4;

    ////////////////////////////////////////////
    //此局代码仅用于测试,屏蔽报警
    //Sys_Alarm.warning=0;
    ////////////////////////////////////////////
}

/*****************************************************************************/
/* 保存“安时”数据                                                            */
/*****************************************************************************/
static void save_energy (void)
{
    DBSysSoc  soc;
    UINT32    TKAmsMax;
    UINT16    rated_tvol = 0;
    
     rated_tvol = (UINT16)((UINT32)BattColtd_volmax[BattType-1] * Sys_Rt.bodys / 100);
   // AmsMax = (UINT32)(Sys_Para.cap_stand)*9000000;
     TKAmsMax = (UINT32)(Sys_Para.cap_stand)*3600*1000/2000;

    //AmsMax += 4500000;

    do {SET_SEM(0);        /* 试图锁定信号量 */      
      } while (!TST_SEM(0));
      
    if( Sys_Rt.TKAms > 0xefffffff)
    {//防止下溢  txz1
      Sys_Rt.TKAms = 0;
      Sys_Rt.Ams_back = 0;
    } 
    else
    {
       if(Sys_Rt.TKAms > TKAmsMax)
        {//防止上溢
           Sys_Rt.TKAms = TKAmsMax;            
        }
    }

    /*保存ah*/
    soc.TKAms = Sys_Rt.TKAms;

    REL_SEM(0);     
    
    soc.wh = Sys_Rt.wh;
    //////////////////////////////////////////soc.magic = 1983;
    soc.crc = Modbus_crc ((UINT8 *)&soc, sizeof(soc) - 2);

    Fm_write (SOCINFO_START, &soc, sizeof(soc));
    Fm_write (SOCINFO_START + 24, &soc, sizeof(soc));
}



/*****************************************************************************/


 /*****************************************************************************/
/* 将LT6803温度转换结果换算成温度值                                          */
/*****************************************************************************/
static UINT8 CalcTemp (UINT16 w)
{
    UINT16    i, Rnor;
  volatile  UINT32    pp;

     //w*4096/32768             
     
     
     w   /=  8;
     pp  =  w;
    //pp *=1000;// 
     pp  *=  270;//  2.7k
    //Rnor = ( pp/(4096-w));   
     pp  =  pp/(3300-w);
     if(pp >=0xfffff) pp  =  0xfffff;   //越界错误，会导致温度查询出错
     
     Rnor  =  pp; //电路错误 2048 基准
  
    /*防错*/
     if (Rnor > NTC_TBL[0] || Rnor < NTC_TBL[sizeof(NTC_TBL)/sizeof(NTC_TBL[0]) - 1])
        return 0xff;

     i = 0;
     while (Rnor <(NTC_TBL[i]/2 +NTC_TBL[i + 1]/2))  
           i++; 
     
     if(i > 164)
            return  0xff ;
     else
         return i ;
}


/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
static UINT32   temp_ticks = 0;  //
static UINT8    temp_stage = 0;  //
volatile SINT16   ttttwww[4];


 static void temp_adc (void)
{

 
    switch(temp_stage)
    {
      case 0: 
                
                  
                  temp_ticks = gSysTicks;
                  temp_stage++;
                  break;
//////////////////////////////////////////////////////////////////////////////////////////////////////
//Uag
     case 1:
                  if (gSysTicks - temp_ticks < 10)
                      break;
               
                  TmpAdc_setreg (0x01, 0xc340);// in0 0xc780
                  
                  temp_ticks = gSysTicks;
                  temp_stage++;
                 

                  break; 
                  
        
     case  2:                                    
                  if (gSysTicks - temp_ticks < 10)
                      break; 
                
                ///////scope ///////////
                 
                  ttttwww[0] = TmpAdc_getreg (0x00);// + offsetp;
                 
                   Sys_Rt.Sys_tz1 =  ttttwww[0];
                
                  temp_ticks = gSysTicks;
                  
                  temp_stage++;
                  
                  break;
              
    case 3:
                  if (gSysTicks - temp_ticks < 10)
                      break;
               
                 TmpAdc_setreg (0x01, 0xd340);// in0 
                
                  
                  temp_ticks = gSysTicks;
                  temp_stage++;
                 

                  break; 
                  
        
      case  4:            
                  if (gSysTicks - temp_ticks < 10)
                      break; 
                
                ///////scope ///////////
                 
                  ttttwww[1] = TmpAdc_getreg (0x00);// + offsetp;
                 
                    Sys_Rt.Sys_tz2 =  CalcTemp(ttttwww[1]);
                
                  temp_ticks = gSysTicks;
                  
                  //temp_stage++;
                  temp_stage = 0;//260716
                  
                  break;
              
        case 5:
                  if (gSysTicks - temp_ticks < 10)
                      break;
               
                 TmpAdc_setreg (0x01, 0xe340);// in0 
             
                  temp_ticks = gSysTicks;
                  temp_stage++;
                 

                  break; 
                  
        
     case  6:            
                  if (gSysTicks - temp_ticks < 10)
                      break; 
                
                ///////scope ///////////
                 
                  ttttwww[2] = TmpAdc_getreg (0x00);// + offsetp;
                 
                   Sys_Rt.Sys_tj1 =  CalcTemp(ttttwww[2]);
                
                  temp_ticks = gSysTicks;
                  
                  temp_stage++;
                  
                  break;
       case 7:
                  if (gSysTicks - temp_ticks < 10)
                      break;
               
                  TmpAdc_setreg (0x01, 0xf340);// in0 
                  
                  temp_ticks = gSysTicks;
                  temp_stage++;
                 

                  break; 
                  
        
     case  8:            
                  if (gSysTicks - temp_ticks < 10)
                      break; 
                
                ///////scope ///////////
                 
                  ttttwww[3] = TmpAdc_getreg (0x00);// + offsetp;
                 
                   Sys_Rt.Sys_tj2 =  CalcTemp(ttttwww[3]);
                
                  temp_ticks = gSysTicks;
                  
                  temp_stage = 0;
                  
                  break;    
                        


   default:
                  
                  break;
              

}
}

/*****************************************************************************/
/* 软件测试                                                      */
/*****************************************************************************/
void Check_test(void)
{
    if (TEST_WDG_ACTIVE())
    {
        if (test_param.to_vol_set == 1)
            Sys_Rt.to_vol = test_param.to_vol;

        if (test_param.current_set == 1)
            Sys_Rt.current = test_param.current;

        if (test_param.vol_set == 1)
        {
            Sys_Rt.max_v = test_param.max_v;
            Sys_Rt.min_v = test_param.min_v;
        }

        if (test_param.temp_set == 1)
        {
            Sys_Rt.max_t = test_param.max_t;
            Sys_Rt.min_t = test_param.min_t;
        }

        if (test_param.soc_set == 1)
            Sys_Rt.soc = test_param.soc;
    }
    else
    {
        memset(&test_param, 0, sizeof(test_param));
        test_param.bms_err = 0;
    }
}

/////////////////////////////////////////////////////////////////////////////
UINT8 Equ_num(UINT8 num){
  UINT8 a,b;
  UINT16 max_val=0;
  for(a=0;a<36;a++){
    if(Cell_Temp_Volts[num*36+a]>max_val){
      max_val = Cell_Temp_Volts[num*36+a];
      b = a;
    }
  }
  return b;
}



/*****************************************************************************/
/* 均衡程序                                                                  */
/*****************************************************************************/
void Bat_Equ(void){
  UINT8 i = 0;
  UINT8 j = 0;
  UINT32 TEMP_DATE=0;  
  
 #if PTL_BALANCE
        /*自动开启均衡*/
        Sys_Para.arv_on_vol =25;
//        if(Bat_Fault_flag == 0){
          switch(ucBal_stage){
            case 0:
              uiBal_arv = Sys_Rt.min_v+ Sys_Para.arv_on_vol;
              for(i=0;i<VC_MAX;i++){
                if(((LECU_Rt[i].maxv&0x7fff) > uiBal_arv) && (uiBal_arv >= 2300) &&\
                   (Sys_Rt.current >= -100 )&& (Sys_Rt.current <= 100)){
                  Equ_EN =0x80;
                  j = Equ_num(i);
                  TEMP_DATE = 0x00000001;
                  if(j<32){
                    LECU_Rt[i].balsts   |=  ((UINT32)0x00000001<<j);
                    LECU_Rt[i].balsts2  =  0;
                    LECU_Rt[i].balsts2  |= Equ_EN ;
                  }else{
                    LECU_Rt[i].balsts   =  0;
                    LECU_Rt[i].balsts2  =  (0x01<<(j-32));
                    LECU_Rt[i].balsts2  |= Equ_EN ;
                  }
                  ucBal_stage = 1;
                  ulBal_Sec = gSysSec;  //开始计时
               
                }
              }
              
            break;
            case 1:
              if((gSysSec - ulBal_Sec >= 300))//如果时间超过时间秒
              {
                for(i=0;i<VC_MAX;i++){
                  LECU_Rt[i].balsts = 0; //关闭均衡控制位
                  LECU_Rt[i].balsts2 = 0;
                  Equ_EN &= ~0x80;
                }

                ucBal_stage = 0;
                ulBal_Sec = 0;//也可以不清零 
                                  
              }
            break;
            default:break;
          }
         
 //       }else{
 //         for(i=0;i<VC_MAX;i++){
 //           LECU_Rt[i].balsts = 0;  
 //           LECU_Rt[i].balsts2 = 0;                  
 //         }
 //       }

#endif
}









/*****************************************************************************/
/* 实时数据模块入口                                                          */
/*****************************************************************************/
void RT_main (void)
{
    if (Sys_getFlags (SYSFLAG_SYSRT))     /*每秒计算一次整车实时数据*/
    {
        if (gSleepMode)
            return;

        /* 每秒招测一次绝缘信息 */
        Sys_Shared.iso_req = TRUE;

        UpdateVolRt();    
        UpdateTempRt();
        UpdateSOCRt();
        Cur_Count();
        Bat_Equ();
        Rt_num++;
        if(Rt_num>2){
            update_rt();        
            Rt_num = 0;
        }

        Check_test();

        if (gSysTicks > 1000 )  //    STARTUP_DELAY
        {
          alarm_proc ();   //温度信息7s内不稳定
        }else{
          Sys_Alarm.warning1 = 0;
          Sys_Alarm.warning2 = 0;
          Sys_Alarm.warning3 = 0;
          Sys_Alarm.warning4 = 0;
        }
        save_energy ();
    } 
     temp_adc();
}
