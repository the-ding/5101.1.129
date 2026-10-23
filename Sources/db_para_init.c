#include <string.h>
#include <stddef.h>
#include "sys.h"
#include "drv_fm.h"
#include "db.h"
#include "db_ev.h"
#include "db_para_cfg.h"
#include "db_para_struct.h"
//#include "oem_para.h"  //每个客户的默认参数表



void Sys_Alarm_Para_Save (void)
{

    Sys_Shared.Sys_Alarm_Para_L1_vol_high = Sys_Alarm_Para.L1.vol_high;


    Sys_Alarm_Para.size = sizeof(Sys_Alarm_Para);
    Sys_Alarm_Para.chksum = 0;
    Sys_Alarm_Para.chksum = 0 - Chksum (&Sys_Alarm_Para, sizeof(Sys_Alarm_Para));

    Fm_write (ALARM_PARA_START, &Sys_Alarm_Para, sizeof(Sys_Alarm_Para));
}

void PARA_Init (void)
{
    Fm_read (ALARM_PARA_START, &Sys_Alarm_Para, sizeof(Sys_Alarm_Para));
    
//    if (Sys_Alarm_Para.size != sizeof(Sys_Alarm_Para)          || 
//        Chksum (&Sys_Alarm_Para, sizeof(Sys_Alarm_Para)) != 0  ||
//        ProChg)
    {
        // 使用缺省参数
   //     (void) memset (&Sys_Alarm_Para, 0xfe, sizeof(Sys_Alarm_Para));




#define SET(x, v, kp)   Sys_Alarm_Para.##x##_keep = kp; Sys_Alarm_Para.x = v;
#define OEMSET(x, v)    Sys_Alarm_Para.x = v;


 /////////////////注意！！！以上内容请勿动/////////////////////////////////
 /////////////////注意！！！以上内容请勿动/////////////////////////////////
 /////////////////注意！！！以上内容请勿动/////////////////////////////////
 /////////////////注意！！！以上内容请勿动/////////////////////////////////
 /////////////////注意！！！以上内容请勿动/////////////////////////////////
 /////////////////注意！！！以上内容请勿动/////////////////////////////////
 /////////////////注意！！！以上内容请勿动/////////////////////////////////
 /////////////////注意！！！以上内容请勿动/////////////////////////////////
 /////////////////注意！！！以上内容请勿动/////////////////////////////////
 /////////////////注意！！！以上内容请勿动/////////////////////////////////
 /////////////////注意！！！以上内容请勿动/////////////////////////////////










#ifdef HAVE_WX_PARAMS
        OEMSET (WX.blnc_ctr, 0);
        OEMSET (WX.blnc_openv, 150);//0.02分辨率--3000mv
        OEMSET (WX.blnc_endtmp, 30+40);
        OEMSET (WX.fan_avgtmp, 10);//无偏移
        OEMSET (WX.fan_detatmp, 5);//无偏移
        OEMSET (WX.batt_sort, 0);
        OEMSET (WX.add_mlt, 0);
        OEMSET (WX.cur_type, 0);
        OEMSET (WX.volt_crr1, 0);
        OEMSET (WX.volt_crr2, 0);
        OEMSET (WX.cur_crr, 0);
        OEMSET (WX.volt_type, 0);
        OEMSET (WX.vmin, 140);//0.2分辨率--2800mv
        OEMSET (WX.vmax, 170);//0.2分辨率--3400mv
#endif




#if OEM == OEM_Pro     //修改
        SET (L1.vol_high, 2800, 2)              //单体电压高 一级对应BMS三级   
        SET (L1.vol_low, 1750, 2)               //单体电压低
        SET (L1.dischg_cur_high, 3500, 2)         //放电电流高
        SET (L1.chg_cur_high, 700, 2)            //充电电流高
        SET (L1.tmp_high, 65+55, 2)               //温度高
        SET (L1.tmp_low, -30+55, 2)               //温度低
        SET (L1.vol_unba, 700, 2)                 //电压互差
        SET (L1.soc_low, 5, 2)                   //SOC过低
        SET (L1.tmp_unba, 20, 2)                  //温度互差
        SET (L1.tvol_high, 9072, 2)              //总压高
        SET (L1.tvol_low, 5670, 2)               //总压低

        SET (L1.regen_cur_high, 3500, 10)       //
        SET (L1.soc_high, 105, 5)               //
        SET (L1.iso_low, 100, 30)               //
        SET (L1.ep_tmp_high, 100+55, 10)
        SET (L1.ep_tmp_low, -20+55, 10)
        SET (L1.chg_tvol_high, 8000, 2) 
        

        SET (L2.vol_high, 2750, 5)        
        SET (L2.vol_low, 1800, 5)
        SET (L2.dischg_cur_high,3100, 5)
        SET (L2.chg_cur_high, 700, 5)
        SET (L2.tmp_high, 60+55, 5)
        SET (L2.tmp_low, -25+55, 5)        
        SET (L2.vol_unba, 500, 5)
        SET (L2.soc_low, 10, 5)
        SET (L2.tmp_unba, 15, 5)
        SET (L2.tvol_high, 8910, 5)
        SET (L2.tvol_low, 5830, 5)

        SET (L2.regen_cur_high, 3200, 10)
        SET (L2.soc_high, 101, 5)
        SET (L2.iso_low, 500, 5)
        SET (L2.ep_tmp_high, 90+55, 5)
        SET (L2.ep_tmp_low, -15+55, 5)
        SET (L2.chg_tvol_high, 7500, 5)
        
        
        SET (L3.vol_high, 2650, 5)   
        SET (L3.vol_low, 1900, 5)
        SET (L3.dischg_cur_high, 3100, 5)
        SET (L3.chg_cur_high, 700, 5)
        SET (L3.tmp_high, 55+55, 5)
        SET (L3.tmp_low, -25+55, 5)
        SET (L3.vol_unba, 350, 5)
        SET (L3.soc_low, 15, 5)
        SET (L3.tmp_unba, 10, 5)
        SET (L3.tvol_high, 8586, 5) 
        SET (L3.tvol_low,6150, 5)

        SET (L3.regen_cur_high, 5700, 10)
        SET (L3.soc_high, 105, 10)
        SET (L3.iso_low, 500, 10)
        SET (L3.ep_tmp_high, 80+55, 10)
        SET (L3.ep_tmp_low, -10+55, 10)
        SET (L3.chg_tvol_high, 7500, 10)
        
        
        
        SET (L4.vol_high, 5000, 5)   
        SET (L4.vol_low, 0, 5)
        SET (L4.dischg_cur_high, 9999, 5)
        SET (L4.chg_cur_high, 9999, 5)
        SET (L4.tmp_high, 80+55, 5)
        SET (L4.tmp_low, -40+55, 5)
        SET (L4.vol_unba, 3000, 5)
        SET (L4.soc_low, 0, 5)
        SET (L4.tmp_unba, 50, 5)
        SET (L4.tvol_high, 10000, 5) 
        SET (L4.tvol_low,0, 5)

        SET (L4.regen_cur_high, 5700, 10)
        SET (L4.soc_high, 105, 10)
        SET (L4.iso_low, 500, 10)
        SET (L4.ep_tmp_high, 80+55, 10)
        SET (L4.ep_tmp_low, -10+55, 10)
        SET (L4.chg_tvol_high, 7500, 10)   
        
        
#endif 






 /////////////////注意！！！以下内容请勿动/////////////////////////////////
 /////////////////注意！！！以下内容请勿动/////////////////////////////////
 /////////////////注意！！！以下内容请勿动/////////////////////////////////
 /////////////////注意！！！以下内容请勿动/////////////////////////////////
 /////////////////注意！！！以下内容请勿动/////////////////////////////////
 /////////////////注意！！！以下内容请勿动/////////////////////////////////
 /////////////////注意！！！以下内容请勿动/////////////////////////////////
 /////////////////注意！！！以下内容请勿动/////////////////////////////////
 /////////////////注意！！！以下内容请勿动/////////////////////////////////
 /////////////////注意！！！以下内容请勿动/////////////////////////////////
















/*OEM没有设置的参数，使用缺省参数*/
#define DEFAULT(x, v, kp) if (Sys_Alarm_Para.x##_keep == 0xfe) {Sys_Alarm_Para.x##_keep = kp; Sys_Alarm_Para.x = v;}

#ifdef L1_VOL_HIGH
        DEFAULT (L1.vol_high, 3000, 10)
#endif

#ifdef L1_VOL_LOW
        DEFAULT(L1.vol_low, 1800, 10)
#endif

#ifdef L1_VOL_UNBA
        DEFAULT (L1.vol_unba, 200, 10)
#endif

#ifdef L1_TVOL_HIGH
        DEFAULT (L1.tvol_high, 7200, 10)
#endif

#ifdef L1_TVOL_LOW
        DEFAULT(L1.tvol_low, 4800, 10)
#endif

#ifdef L1_TMP_HIGH
        DEFAULT (L1.tmp_high, 55+55, 10)
#endif

#ifdef L1_TMP_LOW
        DEFAULT(L1.tmp_low, -39+55, 10)
#endif

#ifdef L1_TMP_UNBA
        DEFAULT (L1.tmp_unba, 20, 10)
#endif

#ifdef L1_EP_TMP_HIGH
        DEFAULT (L1.ep_tmp_high, 80+55, 10)
#endif

#ifdef L1_EP_TMP_LOW
        DEFAULT (L1.ep_tmp_low, -30+55, 10)  /*ZZZ*/
#endif

#ifdef L1_EP_TMP_UNBA
        DEFAULT (L1.ep_tmp_unba, 50, 10)     /*ZZZ*/
#endif

#ifdef L1_DISCHG_CUR_HIGH
        DEFAULT (L1.dischg_cur_high, 4000, 10)
#endif

#ifdef L1_CHG_CUR_HIGH
        DEFAULT (L1.chg_cur_high, 3000, 10)
#endif

#ifdef L1_CHG_TVOL_HIGH
        DEFAULT (L1.chg_tvol_high, 8000, 10)
#endif

#ifdef L1_REGEN_CUR_HIGH
        DEFAULT (L1.regen_cur_high, 4000, 10)
#endif

#ifdef L1_SOC_HIGH
        DEFAULT (L1.soc_high, 105, 10)
#endif

#ifdef L1_SOC_LOW
        DEFAULT (L1.soc_low, 5, 10)
#endif

#ifdef L1_ISO_LOW
        DEFAULT (L1.iso_low, 100, 10)
#endif



#ifdef L2_VOL_HIGH
        DEFAULT (L2.vol_high, 2920, 10)
#endif

#ifdef L2_VOL_LOW
        DEFAULT(L2.vol_low, 2380, 10)
#endif

#ifdef L2_VOL_UNBA
        DEFAULT (L2.vol_unba, 60, 10)
#endif

#ifdef L2_TVOL_HIGH
        DEFAULT (L2.tvol_high, 6960, 10)
#endif

#ifdef L2_TVOL_LOW
        DEFAULT(L2.tvol_low, 5700, 10)
#endif

#ifdef L2_TMP_HIGH
        DEFAULT (L2.tmp_high, 50+55, 10)
#endif

#ifdef L2_TMP_LOW
        DEFAULT(L2.tmp_low, -30+55, 10)
#endif

#ifdef L2_TMP_UNBA
        DEFAULT (L2.tmp_unba, 15, 10)
#endif

#ifdef L2_EP_TMP_HIGH
        DEFAULT (L2.ep_tmp_high, 70+55, 10)
#endif

#ifdef L2_EP_TMP_LOW
        DEFAULT (L2.ep_tmp_low, -25+55, 10)  /*ZZZ*/
#endif

#ifdef L2_EP_TMP_UNBA
        DEFAULT (L2.ep_tmp_unba, 45, 10)     /*ZZZ*/
#endif

#ifdef L2_DISCHG_CUR_HIGH
        DEFAULT (L2.dischg_cur_high, 3000, 10)
#endif

#ifdef L2_CHG_CUR_HIGH
        DEFAULT (L2.chg_cur_high, 2800, 10)
#endif

#ifdef L2_CHG_TVOL_HIGH
        DEFAULT (L2.chg_tvol_high, 7500, 10)
#endif

#ifdef L2_REGEN_CUR_HIGH
        DEFAULT (L2.regen_cur_high, 3000, 10)
#endif

#ifdef L2_SOC_HIGH
        DEFAULT (L2.soc_high, 104, 10)
#endif

#ifdef L2_SOC_LOW
        DEFAULT (L2.soc_low, 10, 10)
#endif

#ifdef L2_ISO_LOW
        DEFAULT (L2.iso_low, 500, 10)
#endif



#ifdef L3_VOL_HIGH
        DEFAULT (L3.vol_high, 4000, 10)
#endif

#ifdef L3_VOL_LOW
        DEFAULT(L3.vol_low, 1500, 10)
#endif

#ifdef L3_VOL_UNBA
        DEFAULT (L3.vol_unba, 300, 10)
#endif

#ifdef L3_TVOL_HIGH
        DEFAULT (L3.tvol_high, 7500, 10)
#endif

#ifdef L3_TVOL_LOW
        DEFAULT(L3.tvol_low, 6200, 10)
#endif

#ifdef L3_TMP_HIGH
        DEFAULT (L3.tmp_high, 60+55, 10)
#endif

#ifdef L3_TMP_LOW
        DEFAULT(L3.tmp_low, -39+55, 10)
#endif

#ifdef L3_TMP_UNBA
        DEFAULT (L3.tmp_unba, 25, 10)
#endif

#ifdef L3_EP_TMP_HIGH
        DEFAULT (L3.ep_tmp_high, 90+55, 10)
#endif

#ifdef L3_EP_TMP_LOW
        DEFAULT (L3.ep_tmp_low, -30+55, 10)  /*ZZZ*/
#endif

#ifdef L3_EP_TMP_UNBA
        DEFAULT (L3.ep_tmp_unba, 50, 10)     /*ZZZ*/
#endif

#ifdef L3_DISCHG_CUR_HIGH
        DEFAULT (L3.dischg_cur_high, 5000, 10)
#endif

#ifdef L3_CHG_CUR_HIGH
        DEFAULT (L3.chg_cur_high, 4000, 10)
#endif

#ifdef L3_CHG_TVOL_HIGH
        DEFAULT (L3.chg_tvol_high, 8000, 10)
#endif

#ifdef L3_REGEN_CUR_HIGH
        DEFAULT (L3.regen_cur_high, 5000, 10)
#endif

#ifdef L3_SOC_HIGH
        DEFAULT (L3.soc_high, 103, 10)
#endif

#ifdef L3_SOC_LOW
        DEFAULT (L3.soc_low, 1, 10)
#endif

#ifdef L3_ISO_LOW
        DEFAULT (L3.iso_low, 1000, 10)
#endif



#ifdef L4_VOL_HIGH
        DEFAULT (L4.vol_high, 4000, 10)
#endif

#ifdef L4_VOL_LOW
        DEFAULT(L4.vol_low, 1500, 10)
#endif

#ifdef L4_VOL_UNBA
        DEFAULT (L4.vol_unba, 300, 10)
#endif

#ifdef L4_TVOL_HIGH
        DEFAULT (L4.tvol_high, 7500, 10)
#endif

#ifdef L4_TVOL_LOW
        DEFAULT(L4.tvol_low, 6200, 10)
#endif

#ifdef L4_TMP_HIGH
        DEFAULT (L4.tmp_high, 60+55, 10)
#endif

#ifdef L4_TMP_LOW
        DEFAULT(L4.tmp_low, -39+55, 10)
#endif

#ifdef L4_TMP_UNBA
        DEFAULT (L4.tmp_unba, 25, 10)
#endif

#ifdef L4_EP_TMP_HIGH
        DEFAULT (L4.ep_tmp_high, 90+55, 10)
#endif

#ifdef L4_EP_TMP_LOW
        DEFAULT (L4.ep_tmp_low, -35+55, 10)  /*ZZZ*/
#endif

#ifdef L4_EP_TMP_UNBA
        DEFAULT (L4.ep_tmp_unba, 55, 10)     /*ZZZ*/
#endif

#ifdef L4_DISCHG_CUR_HIGH
        DEFAULT (L4.dischg_cur_high, 5000, 10)
#endif

#ifdef L4_CHG_CUR_HIGH
        DEFAULT (L4.chg_cur_high, 4000, 10)
#endif

#ifdef L4_CHG_TVOL_HIGH
        DEFAULT (L4.chg_tvol_high, 8000, 10)
#endif

#ifdef L4_REGEN_CUR_HIGH
        DEFAULT (L4.regen_cur_high, 5000, 10)
#endif

#ifdef L4_SOC_HIGH
        DEFAULT (L4.soc_high, 102, 10)
#endif

#ifdef L4_SOC_LOW
        DEFAULT (L4.soc_low, 1, 10)
#endif

#ifdef L4_ISO_LOW
        DEFAULT (L4.iso_low, 2000, 10)
#endif

        Sys_Alarm_Para_Save ();
    }
     Sys_Shared.Sys_Alarm_Para_L1_vol_high = Sys_Alarm_Para.L1.vol_high;
         
}