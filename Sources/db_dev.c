#include <string.h>
#include "sys.h"
#include "drv_fm.h"
#include "db.h"
#include "ptl.h"
#include "modbus.h"
#include "db_para_cfg.h"
#include "db_para_struct.h"

/*每个采集板的参数*/
#if PTL_LECU 
DBDevPara       LECU_Para[DEV_MAX];
DBSysNum        LECU_NVT[DEV_MAX];  //DEV_MAX:从控数量
#endif

/*整车参数*/
#pragma DATA_SEG SHARED_DATA
DBSysPara       Sys_Para;
DBBattPara      Batt_Para;
DBOEMPara       OEM_Para;
DBChgRt         CHG_Rt;
#pragma DATA_SEG DEFAULT

volatile BOOL ProChg = FALSE;//定制号修改标志

#define SET_224(x, v, kp)              \
        Sys_Alarm_Para.##x##_keep = kp; \
        Sys_Alarm_Para.x = v            //


////////每次根据项目需求修改电池类型对应的额定单体电压///////
//////电池类型分别为铅酸,镍氢,磷酸铁锂,锰酸钾,钴酸钾,三元材料电池(镍钴锰),聚合物电池,钛酸锂
volatile UINT16 BattColtd_volmax[8] = {2400,1400,3650,4200,4200,4200,4200,3000};
volatile UINT8 BattType = 7;    // 数值对应上表中的电池类型        


static UINT16 dischg_cur[6][6]={10, 15, 15, 15, 15, 15,
                                15, 20, 20, 20, 20, 20,
                                15, 20, 20, 30, 30, 30,
                                10, 15, 15, 25, 25, 25,
                                5,  8,  10, 15, 15, 15,
                                5,  5,  5,  5,  5,  5};
static UINT16 Regen_cur[5][4] ={0,  0,  0,  0,
                                0,  0,  10, 0,
                                0,  10, 10, 0,
                                0,  10, 15, 0,
                                0,  15, 20, 0};
static UINT8  Regen_time[5][4]={1,  1,  1,  1,
                                1,  1,  5,  1,
                                1,  5,  10, 1,
                                1,  10, 10, 1,
                                1,  10, 10, 1};  

UINT16 Rated_recur = 0;
UINT16 Rated_discur = 0;

//#if PTL_LECU   20140813 @yyzz

/*****************************************************************************/
/* 新增一个采集器模块                                                        */
/*****************************************************************************/
UINT8 DEV_new (UINT32 sn)
{
    UINT8   j, best;
    
    /*以前是否注册过*/
    for (j = 0; j < Sys_Para.lecu_num; j++)
    {
        if (LECU_Para[j].serial_num == sn && !(LECU_Valids & (1UL << j)))
          return j;
    }

    /*分配ID*/
    for (best = 0; best < Sys_Para.lecu_num; best++)
    {
        if (!(LECU_Valids & (1UL << best)))
          break;
    }

    /*模块数太多*/
    if (best >= DEV_MAX)
        best = DEV_MAX - 1;
    
    /*使用缺省参数*/
    LECU_Para[best].serial_num = sn;
    LECU_Para[best].RegCnt = 1;
    LECU_Para[best].crc = Modbus_crc ((UINT8 *)&LECU_Para[best], sizeof(LECU_Para[0]) - 2);

    /*更新所有字段*/
    Fm_write (LECU_PARA_START + best*sizeof(LECU_Para[0]), &LECU_Para[best], sizeof(LECU_Para[0]));
    
    return best;
}
//#endif  20140813 @yyzz

/*****************************************************************************/
/* 采集器管理模块初始化                                                      */
/*****************************************************************************/
void DEV_Init (void)
{
    BOOL        sysModified = FALSE;
    UINT16      crc;
#if PTL_LECU
    UINT16      i;
#endif
  
    
    /*从铁电RAM导入OEM参数*/
    Fm_read (OEMPARA_START, &OEM_Para, sizeof(OEM_Para));
    if (OEM_Para.oem_special != OEM)
        ProChg = TRUE;
    else
    {
       ProChg = FALSE;                                          
    }    
    
    crc = Modbus_crc ((UINT8 *)&OEM_Para, sizeof(OEM_Para) - 2);
    if (OEM_Para.crc != crc || ProChg)
    {
        //(void) memset (&OEM_Para, 0, sizeof(OEM_Para));
        
        /*缺省OEM参数*/
        OEM_Para.bms_addr = 1;    /*主BMS*/
        OEM_Para.lecu_num = DEV_MAX;
        OEM_Para.volt_delta = 1;  /*5mV*/
        OEM_Para.curr_delta = 30; /*3A*/
        OEM_Para.hallpara = 3500;//2000;
        OEM_Para.almintv = 5*60;    // 5分钟
        OEM_Para.oem_special = OEM;
        OEM_Para.crc = Modbus_crc ((UINT8 *)&OEM_Para, sizeof(OEM_Para) - 2);
        
        /*保存缺省参数*/
        Fm_write (OEMPARA_START, &OEM_Para, sizeof(OEM_Para));
    }
    
     if (OEM_Para.crc != crc )  //有错误时恢复摩恩校准系数
    {
       
        OEM_Para.Raw = 1000;                                
        OEM_Para.Ref = 1000; // 0.1mv   缺省霍尔系数:4
        OEM_Para.c0 = 0;    
 
        OEM_Para.crc = Modbus_crc ((UINT8 *)&OEM_Para, sizeof(OEM_Para) - 2);
        
        /*保存缺省参数*/
        Fm_write (OEMPARA_START, &OEM_Para, sizeof(OEM_Para));
    }

    if (OEM_Para.almintv < 1*60) OEM_Para.almintv = 5*60;
    else if (OEM_Para.almintv > 60*60) OEM_Para.almintv = 10*60;

    /*从铁电RAM导入系统参数*/
    Fm_read (SYSPARA_START, &Sys_Para, sizeof(Sys_Para));
    
    crc = Modbus_crc ((UINT8 *)&Sys_Para, sizeof(Sys_Para) - 2);
   
    //if (Sys_Para.crc != crc || ProChg)
    {
       
/////////////////////////////// /*缺省系统参数*/
        Sys_Para.hist_prd = 2;    /*2秒*/
        Sys_Para.lecu_num = OEM_Para.lecu_num;
        Sys_Para.hallpara = OEM_Para.hallpara;
        Sys_Para.temp_fan_on = 40+55;
        Sys_Para.temp_fan_off = 35+55;
        Sys_Para.temp_heat_on = 0+55;
        Sys_Para.temp_heat_off = 5+55;
        
        Sys_Para.cap_stand = 69;
        Sys_Para.shutcur = 4;           // 4安培
        
        Sys_Para.full_cha_c = 600;
        Sys_Para.full_cha_v = 9790;
        
        Sys_Para.coltd = BattType;
       
       
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
 
        /**/
        sysModified = TRUE;
    }
   
    if (Sys_Para.hist_prd == 0)
        Sys_Para.hist_prd = 2;      // 2秒
    //if (Sys_Para.shutcur == 0)
    //    Sys_Para.shutcur = 4;     // 4安培
    if (Sys_Para.shutmin < 1)
        Sys_Para.shutmin = 3;       // 3分钟
    
    /*同步只读参数*/
    if (Sys_Para.lecu_num != OEM_Para.lecu_num ||
        Sys_Para.hallpara != OEM_Para.hallpara)
    {
        Sys_Para.lecu_num = OEM_Para.lecu_num;
        Sys_Para.hallpara = OEM_Para.hallpara;
        
        /**/
        sysModified = TRUE;
    }

    if (sysModified == TRUE)
    {
        Sys_Para.crc = Modbus_crc ((UINT8 *)&Sys_Para, sizeof(Sys_Para) - 2);
        
        /*保存缺省参数*/
        Fm_write (SYSPARA_START, &Sys_Para, sizeof(Sys_Para));
    }
    /*从铁电RAM导入电池及车辆部分参数*/
    Fm_read (BATTPARA_START, &Batt_Para, sizeof(Batt_Para));
    
    crc = Modbus_crc ((UINT8 *)&Batt_Para, sizeof(Batt_Para) - 2);
    if (Batt_Para.crc != crc || ProChg)
    {
       
       
        Batt_Para.chg_times = 0;  //

        Batt_Para.batt_manu1 = 0;
        Batt_Para.batt_manu2 = 'H';
        Batt_Para.batt_manu3 = 'T';
        Batt_Para.batt_manu4 = 'G';
        
        Batt_Para.batt_year  = 30;
        Batt_Para.batt_month = 1;
        Batt_Para.batt_date  = 1;
        
        Batt_Para.batt_mark = 1;// 电池组产权标示 0:租赁 1:自有
        
        
        for (i = 0; i < 30; i++)
            Batt_Para.rsvd[i] = 0;    
            
        Batt_Para.crc = Modbus_crc ((UINT8 *)&Batt_Para, sizeof(Batt_Para) - 2);
        
        //保存缺省参数
        Fm_write (BATTPARA_START, &Batt_Para, sizeof(Batt_Para));    
    }

#if PTL_LECU

    /*从铁电RAM导入当前采集板配置参数*/
    Fm_read (LECU_PARA_START, LECU_Para, sizeof(LECU_Para));

    for (i = 0; i < DEV_MAX; i++)
    {
        crc = Modbus_crc ((UINT8 *)&LECU_Para[i], sizeof(LECU_Para[0]) - 2);

        if (LECU_Para[i].crc != crc)
          LECU_Para[i].serial_num = 0;
    }
#endif
}

/*****************************************************************************/
/* 采集器管理模块入口                                                        */
/*****************************************************************************/
void DEV_main (void)
{
}
