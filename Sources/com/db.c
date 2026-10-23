#include "sys.h"
#include "db.h"
#include "ptl.h"

  
//硬件版本号:
//  class(主分类): 1-采集模块类，2-通讯维护类，3-电源类，4-主控类，5-全功能类
//  series(主分类内系列号): 例如 111表示防水版主控BC111 
//  hwver(主分类内该系列的硬件版本)
/*#define MAKE_HWVER(class,series,hwver)  (((UINT16)(class) << 13) | ((UINT16)(series) << 3) | ((hwver) & 7))


硬件版本信息
#pragma CONST_SEG HW_VARS
 const volatile UINT16  HWVer  = MAKE_HWVER(4, 111, 2);
#pragma CONST_SEG DEFAULT
*/



//硬件版本号:
//  class(主分类): 1-采集模块类，2-通讯维护类，3-电源类，4-主控类，5-全功能类
#define MAKE_HWVER(series,hwver)   (((UINT16)(series) << 3) | ((hwver) & 7))

/*硬件版本信息*/
#pragma CONST_SEG HW_VARS
 const volatile UINT16  HWVer  = MAKE_HWVER(1141,1);  //软件程序文件名5101.X.036.mcp   X:1,2,3
#pragma CONST_SEG DEFAULT
/*核心软件版本信息*/



/*核心软件版本信息*/
#pragma CONST_SEG CORE_VER_VARS
const volatile   UINT8   CORE_ver  = 1; ////SYSRT_SOC XIUGAIWei INT
#pragma CONST_SEG DEFAULT


/*
简易说明，109是合成了4级报警到flash中的版本
*/



void DEV_Init (void);
void HST_Init (void);
void RT_Init (void);
void PARA_Init (void);
void EV_Init (void);

void DEV_main (void);
void HST_main (void);
void RT_main (void);
void EV_main (void);

/*****************************************************************************/
/* 数据库模块初始化                                                          */
/*****************************************************************************/
void DB_Init (void)
{

    (void) HWVer;
    (void) CORE_ver;


    DEV_Init ();
    RT_Init ();
    PARA_Init ();
    EV_Init ();
    HST_Init ();
}

/*****************************************************************************/
/* 数据库模块总入口                                                          */
/*****************************************************************************/
void DB_main (void)
{
    DEV_main ();
    RT_main ();
    EV_main ();
    HST_main ();
}
