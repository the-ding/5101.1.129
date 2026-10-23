#include "sys.h"
#include "db.h"
#include "ptl.h" 
#include "drv_fm.h" 
#include "modbus.h" 


 /*客户协议软件版本信息*/
#pragma CONST_SEG OEM_VER_VARS
const volatile   UINT8   SOFT_ver  = OEM_ver;//屏蔽了充电及    // V2.28//只是升级了一个版本号 无任何修改
//volatile   UINT8   SOFT_ver  = OEM_ver;//屏蔽了充电及    // V2.28//只是升级了一个版本号 无任何修改

#pragma CONST_SEG DEFAULT


/*版本信息*/
#pragma CONST_SEG VER_VARS
const volatile UINT16  OEMVer = OEM;
#pragma CONST_SEG DEFAULT  

static volatile Save_flg = FALSE; 

#define  TIMER_DELAY_30s     0 
#define  TIMER_DELAY_MAX     1

#define EXPIRED_PTL(id, ms) ((gSysTicks - sticksPTL[id]) >= MS2TICKS(ms))
#define TIMER_RESET_PTL(id) sticksPTL[id] = gSysTicks;

static volatile UINT32  sticksPTL[TIMER_DELAY_MAX]; 


/*协议初始化函数*/
void  HMI_Init (void);
void  PC_Init (void);
void  LECU_Init (void);
void  WX_Init (void);
void  CHARGE_Init(void);
void  Control_Init (void);
void  Protocol_Init (void);
void  plt_Init(void);
void  Bat_Fault_Code_init(void);

/*协议入口函数*/
void  HMI_main (void);
void  PC_main (void);
void  CAN_main (void);
void  LECU_main (void);
void  WX_main (void);
void  CHARGE_main(void);
void  Control_main (void);
void  Protocol_main (void);
void  plt_main(void);

/*****************************************************************************/
/* 协议模块初始化                                                            */
/*****************************************************************************/
void PTL_Init (void)
{
    service = FALSE;
    (void) SOFT_ver ;
    
    SWVer =   CORE_ver<<8;
    SWVer |= OEM_ver ;

#if PTL_LECU
    LECU_Init ();
#endif


#if PTL_HMI
    HMI_Init ();
#endif

#if PTL_Pro
    plt_Init();
#endif

    PC_Init ();   





#if PTL_CHARGE
//    CHARGE_Init ();
#endif
}

/*****************************************************************************/
/* 协议模块总入口                                                            */
/*****************************************************************************/
void PTL_main (void)
{
    PC_main ();

#if PTL_LECU
    LECU_main ();
#endif

if((!BCU_Update)&&(!service)) //进入维护模式后不要发报文，会造成上位机失败！
{



#if PTL_HMI
    //HMI_main ();
#endif

#if PTL_Pro
    plt_main();
#endif
   
   

#if PTL_CHARGE
//    CHARGE_main ();    
#endif

    }
    /*考虑删除不可识别的CAN帧*/
    CAN_main ();
    
    if (EXPIRED_PTL (TIMER_DELAY_30s, 30*1000UL) && !Save_flg)
    {
        Save_flg = TRUE;
        Batt_Para.max_tmp = (UINT8)Sys_Rt.max_t;
        Batt_Para.min_tmp = (UINT8)Sys_Rt.min_t;
        Batt_Para.avg_tmp = Sys_Rt.avg_t;
        Batt_Para.crc = Modbus_crc ((UINT8 *)&Batt_Para, sizeof(Batt_Para) - 2);
        
        /*保存缺省参数*/
        Fm_write (BATTPARA_START, &Batt_Para, sizeof(Batt_Para));
    }
}

