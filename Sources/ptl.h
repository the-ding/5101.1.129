#ifndef _PTL_H
#define _PTL_H 


/*定制号*/
#define OEM_DEBUG      0   /*调试*/
#define OEM_JSYG      5555   /*STANDARD*/
#define OEM_Pro       288   /*STANDARD*/
/*当前定制号*/
#define OEM       OEM_Pro


/*    #define HAVE_WX_PARAMS  1 没有额外的参数会导致参数读取和设置失败！！！！
注意，在  #define PTL_LECU    1时必须同时包含该定义
*/

#if OEM == OEM_Pro 
#define OEM_ver        0
#define PTL_Pro        1
#define PTL_LECU       1  /*冠拓BMS内部协议*/
#define PTL_HMI        0 /*彩屏通讯*/ 
#define PTL_BALANCE    1   //均衡启动标志位
#define PTL_TEST       0   //测试标志位
#define PTL_PHY_TEST       0   //测试箱测试标志位

#define PTL_SDHY       0  /*山东恒宇沂兴7米考斯特电动大巴项目控制文件*/
#define PTL_CHARGE     0  /*充电控制流程*/                   
#define PTL_GB         0  /*GBT 27930-2011充电机协议*/
#define PTL_HAVE_ISO   0  /*配置绝缘检测模块*/
#define PTL_CHG_KEY    1  /*继电器控制开关*/ 
#define HAVE_PWDNEV       /*允许生成停送电事件*/
#define HAVE_IOEV         /*允许生成IO事件*/
#define HAVE_L3_PARAMS
#define HAVE_L4_PARAMS

#endif



   


/*缺省协议配置选项*/
#ifndef OEM_YXWTS
#define OEM_YXWTS     0
#endif
#ifndef PTL_WTSXY
#define PTL_WTSXY     0
#endif
#ifndef PTL_SDHY
#define PTL_SDHY      0
#endif
#ifndef PTL_YXTY4_0
#define PTL_YXTY4_0   0
#endif
#ifndef PTL_YXGL
#define PTL_YXGL      0
#endif
#ifndef PTL_AKZBGL
#define PTL_AKZBGL    0
#endif
#ifndef PTL_AKCAN
#define PTL_AKCAN     0
#endif
#ifndef PTL_YX213
#define PTL_YX213     0
#endif
#ifndef PTL_YXLY
#define PTL_YXLY      0
#endif
#ifndef PTL_GXNT
#define PTL_GXNT      0
#endif
#ifndef PTL_NJJL
#define PTL_NJJL      0
#endif
#ifndef PTL_TJSL
#define PTL_TJSL      0
#endif
#ifndef PTL_JY
#define PTL_JY        0
#endif
#ifndef PTL_JNHW_1_2
#define PTL_JNHW_1_2  0
#endif
#ifndef PTL_JNHW
#define PTL_JNHW      0
#endif 
#ifndef PTL_YXHD_4_0
#define PTL_YXHD_4_0  0
#endif
#ifndef PTL_XJ
#define PTL_XJ        0
#endif
#ifndef PTL_QDHD206
#define PTL_QDHD206   0
#endif
#ifndef PTL_YXWH12
#define PTL_YXWH12    0
#endif
#ifndef PTL_YX4_0_MODIFIED
#define PTL_YX4_0_MODIFIED  0
#endif
#ifndef PTL_XCGD
#define PTL_XCGD      0
#endif
#ifndef PTL_YIXING213
#define PTL_YIXING213 0
#endif
#ifndef PTL_CHARGE
#define PTL_CHARGE    0
#endif  
#ifndef PTL_GB
#define PTL_GB        0
#endif
#ifndef PTL_GW
#define PTL_GW        0
#endif
#ifndef PTL_CUSTOMIZE
#define PTL_CUSTOMIZE 0
#endif
#ifndef PTL_LECU
#define PTL_LECU      0
#endif
#ifndef PTL_HMI
#define PTL_HMI       0
#endif
#ifndef PTL_BALANCE
#define PTL_BALANCE   0
#endif
#ifndef PTL_METER
#define PTL_METER     0
#endif
#ifndef PTL_ZIBO
#define PTL_ZIBO      0
#endif
#ifndef PTL_LINYI
#define PTL_LINYI     0
#endif
#ifndef PTL_MGL
#define PTL_MGL       0
#endif
#ifndef PTL_FG
#define PTL_FG        0
#endif
#ifndef PTL_ZJVX
#define PTL_ZJVX      0
#endif
#ifndef PTL_EGV
#define PTL_EGV       0
#endif
#ifndef PTL_QDWX
#define PTL_QDWX      0
#endif
#if     PTL_QDWX
#define HAVE_WX_PARAMS  1
#endif
#ifndef PTL_ZZRC
#define PTL_ZZRC      0
#endif
#ifndef PTL_SZYM      
#define PTL_SZYM      0
#endif
#ifndef PTL_METEROLD
#define PTL_METEROLD  0
#endif
#ifndef PTL_CHG_KEY
#define PTL_CHG_KEY   0
#endif
#ifndef PTL_LINYI_180
#define PTL_LINYI_180 0
#endif
#ifndef PTL_YUYAO_182
#define PTL_YUYAO_182 0
#endif
#ifndef PTL_K13_DELAY
#define PTL_K13_DELAY 0
#endif
#ifndef PTL_TC_CHA
#define PTL_TC_CHA    0
#endif
#ifndef PTL_SZAJ
#define PTL_SZAJ      0
#endif
#ifndef PTL_JSJY_CHA
#define PTL_JSJY_CHA  0
#endif
#ifndef PTL_WZL
#define PTL_WZL       0
#endif
#ifndef PTL_YXMETER30
#define PTL_YXMETER30 0
#endif
#ifndef PTL_YXMETER31
#define PTL_YXMETER31 0
#endif
#ifndef PTL_YXMETER40
#define PTL_YXMETER40 0
#endif
#ifndef PTL_ZYZLP
#define PTL_ZYZLP     0
#endif
#ifndef PTL_HAVE_ISO
#define PTL_HAVE_ISO  0
#endif  
#ifndef PTL_YX
#define PTL_YX        0
#endif  
#ifndef PTL_GBT_CHA
#define PTL_GBT_CHA   0
#endif 
#ifndef PTL_GLYB
#define PTL_GLYB      0
#endif   
#ifndef PTL_HGD_ECU
#define PTL_HGD_ECU   0
#endif    
#ifndef PTL_JYNP
#define PTL_JYNP      0
#endif 
#ifndef PTL_FOTON_BUS
#define PTL_FOTON_BUS 0
#endif  
#ifndef PTL_SENS
#define PTL_SENS      0
#endif


#ifndef PTL_BQ_MIX
#define PTL_BQ_MIX    0
#endif
#ifndef PTL_YXZBGL
#define PTL_YXZBGL    0
#endif
#ifndef PTL_SXLSY
#define PTL_SXLSY     0
#endif
#ifndef PTL_SLKC
#define PTL_SLKC      0
#endif

/*****************************************************************************/
/*****************************************************************************/
/* 协议模块总入口                                                            */
/*****************************************************************************/
void PTL_main (void);

/*****************************************************************************/
/* 协议模块初始化                                                            */
/*****************************************************************************/
void PTL_Init (void);

#endif
