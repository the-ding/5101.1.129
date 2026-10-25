/*最大可接采集板个数（最大31）*/
#define DEV_MAX               1         //从控数量  注3.0单独指BMU，CR450指的HMPU
#define VC_MAX                1
//#define TC_MAX                2
#define DEV_IN                1

/*采集板最大可接电池支数*/
#define DEV_CELL_MAX          36

/*系统总共最大可接电池支数*/
#define SYS_CELL_MAX          DEV_MAX*36

#define BAT_VALID_MAX         36   //324 采集电压数
#define TEM_VALID_MAX         36  //324 采集温度数

#define DAY_INDEX_MAX         100     /*最多保存100天的历史数据*/
#define DAY_INDEX_SIZE        32      /*每天的摘要数据长度 > sizeof(DBDayIndexRec)*/

#define FLA_INDEX_MAX         100     /*最多保存100月的报警记录*/
#define FLA_INDEX_SIZE        16      /*每月的摘要数据长度 > sizeof(DBFlashIndexRec)*/

#define EV_SIZE               64      /*每条事件记录长度 > sizeof(DBEvRec)*/

#define HIST_RSVD             2000    /*保留2000字节，防止FLASH中最后一条记录空间跨段*/

//报警标志位
#define WARN_VOL_HIGH         (1UL << 0)
#define WARN_VOL_LOW          (1UL << 1)
#define WARN_VOL_UNBA         (1UL << 2)
#define WARN_TVOL_HIGH        (1UL << 3)
#define WARN_TVOL_LOW         (1UL << 4)
#define WARN_TMP_HIGH         (1UL << 5)
#define WARN_TMP_LOW          (1UL << 6)
#define WARN_TMP_UNBA         (1UL << 7)
#define WARN_EP_TMP_HIGH      (1UL << 8)
#define WARN_EP_TMP_LOW       (1UL << 9)
#define WARN_EP_TMP_UNBA      (1UL << 10)
#define WARN_DISCHG_CUR_HIGH  (1UL << 11)
#define WARN_CHG_CUR_HIGH     (1UL << 12)
#define WARN_CHG_VOLT_HIGH    (1UL << 13)    // 充电总电压过高
#define WARN_REGEN_CUR_HIGH   (1UL << 14)    // 回收电流过高
#define WARN_SOC_HIGH         (1UL << 15)
#define WARN_SOC_LOW          (1UL << 16)
#define WARN_POSRES_LOW       (1UL << 17)
#define WARN_NEGRES_LOW       (1UL << 18)
#define WARN_TVOLT_COMPARE      (1UL << 19)

#define WARN_TMP_FAULT        (1UL << 29)    // 温度失效
#define WARN_ISO_FAULT        (1UL << 30)    //绝缘失效
#define WARN_LECU_LOST        (1UL << 31)
#define WARN_LECU_ZT          (1UL << 32)


//将IO和线束统一
#define IO_OUT0 PTT_PTT1
#define IO_OUT1 PTT_PTT2
#define IO_OUT2 PTT_PTT4
#define IO_OUT3 PTT_PTT6
#define IO_OUT4 PTT_PTT7
#define IO_OUT5 PORTB_PB1

#define ACC_ON    PTP_PTP0
#define HVIL      PTP_PTP1
#define CC2     PORTB_PB0
#define IO_IN0  PTIT_PTIT5
#define IO_IN1  PTIT_PTIT3
#define IO_IN2  PTIT_PTIT0

#define OUT0   0 
#define OUT1   1
#define OUT2   2
#define OUT3   3
#define OUT4   4
#define OUT5   5
#define OUT6   6
#define OUT7   7
#define OUT8   8
#define OUT9   9
#define OUT10  10
#define OUT11  11
#define OUT12  12
#define OUT13  13
#define OUT14  14
#define OUT15  15

// OUT受控原因
#define VOL_HIGH           ((UINT16)1 << 0)  // 单体电压高
#define VOL_LOW            ((UINT16)1 << 1)  // 单体电压低
#define TMP_HIGH           ((UINT16)1 << 2)  // 单体温度高
#define TMP_LOW            ((UINT16)1 << 3)  // 单体温度低
#define TVOL_HIGH          ((UINT16)1 << 4)  // 总电压高
#define TVOL_LOW           ((UINT16)1 << 5)  // 总电压低
#define CUR_HIGH           ((UINT16)1 << 6)  // 电流过大
#define ISO_LOW            ((UINT16)1 << 7)  // 绝缘低
#define STA_RCOV           ((UINT16)1 << 8)  // 状态恢复
#define LECU_LOST          ((UINT16)1 << 9)  // 模块丢失
#define IN_CON             ((UINT16)1 << 10) // IN信号
#define VEH_ASK            ((UINT16)1 << 11) // 整车请求
#define CHG_START          ((UINT16)1 << 12) // 充电开始
#define CHG_STOP           ((UINT16)1 << 13) // 充电结束
#define POWER_ON           ((UINT16)1 << 14) // 上电流程
#define OTHERS             ((UINT16)1 << 15) // 其他



/*整车参数*/
typedef struct db_syspara
{
    UINT16      hanzi;          // 车牌 汉字
    UINT8       zimu;           // 字母
    UINT8       zimu1;          // 字母
    UINT8       shuzi1;         // 数字
    UINT8       shuzi2;
    UINT8       shuzi3;
    UINT8       shuzi4;

    UINT16      max_dischg_curX; // 0.1A  最大放电电流    未使用
    UINT16      full_cha_v;     // 0.1V  满充电电压
    UINT16      full_cha_c;     // 0.1A  满充电电流
    UINT16      precharge_cur;  // 0.1A  预充电       未使用
    UINT16      max_regen_curX;  // 0.1A  下调电流 --- 最大回收电流    未使用
    UINT16      max_rechg_vX;    // 0.1V  截止电流 --- 充电总电压过高   未使用
    UINT16      max_rechg_curX;  // 0.1A  最大充电电流

    UINT16      hist_prd;       // 历史数据保存间隔
    UINT8       lecu_num;       // 采集板个数 (READONLY)
    UINT8       soc_low;        // SOC过低     未使用
    UINT8       soc_2low;       // SOC超低     未使用
    UINT8       shutcur;        // 1A, 10分钟低于该电流系统进入低功耗模式
    UINT8       shutmin;        // 进入低功耗模式时间 (单位：分钟)
    UINT8       max_deta_tX;     // 温度互差超大阈值  未使用

    UINT8       coltd;          // 生产厂家&产品类型
    UINT8       soh;            // 模块SOH
    UINT16      cap_stand;      // 电池额定容量
    UINT16      arv_on_vol;     // 均衡开启阀值
    UINT16      max_deta_vX;     // 电压互差阀值1级报警

    
    UINT8       temp_highX;      // 模块温度过高     未使用
    UINT8       temp_2highX;     // 模块温度超高    未使用
    UINT8       temp_lowX;       // 模块温度过低     未使用
    UINT8       temp_2lowX;      // 模块温度超低     未使用
    UINT8       temp_fan_on;    // 风扇开启温度
    UINT8       temp_fan_off;   // 风扇关闭温度
    UINT8       temp_heat_on;   // 加热管开启温度
    UINT8       temp_heat_off;  // 加热管关闭温度

    SINT16      RawX;            // 去零漂后计算的电流             
    SINT16      RefX;            // 实测值计算的电流
    SINT16      c0X;             // 零漂值
    //UINT16      vol_highX;       // 单体电压过高
    //UINT16      vol_2highX;      // 单体电压超高
    //UINT16      vol_lowX;        // 单体电压过低
    UINT16      vol_2lowX;       // 单体电压超低    未使用
    
    UINT8       ep_highX;        // 端子温度过高   未使用
    UINT8       ep_2highX;       // 端子温度超高   未使用

    UINT16      max_deta_v2X;    // 电压互差阀值2级报警  未使用
    UINT8       soc_highX;       // SOC过高              未使用
    UINT8       soc_2highX;      // SOC超高              未使用

    UINT16      hallpara;       // 霍尔系数 (READONLY)

    UINT8       rsvd1[36];        //


    UINT16      crc;            //校验和
} DBSysPara;

/*电池及车辆部分参数*/
typedef struct db_battpara
{
    UINT8       batt_manu1;       // 电池生产厂商名称 ASCII码
    UINT8       batt_manu2;
    UINT8       batt_manu3;
    UINT8       batt_manu4;
    UINT8       batt_year;        // 电池组生产日期
    UINT8       batt_month;
    UINT8       batt_date;
    UINT8       batt_mark;        // 电池组产权标示
    /*8*/
    
    UINT8       VIN[17];          // 车辆识别码
    UINT8       max_tmp;        // 上次下电保存的系统最高温度
    UINT8       min_tmp;        // 上次下电保存的系统最低温度
    UINT8       avg_tmp;        // 上次下电保存的系统平均温度
    UINT8       rsvd1[2];         // 保留
    UINT16      chg_times;      // 动力电池充电次数
     /*24*/
    
    UINT8       rsvd[30];         // 保留
      /*8*/
    UINT16      crc;              //校验和      
} DBBattPara;   /*40*/

typedef struct db_oempara
{
    UINT8       bms_addr;
    UINT8       lecu_num;       // 采集板个数
    UINT8       volt_delta;     // 电池电压突变超过该值时自动存储一条记录
    UINT8       curr_delta;     // 总电流突变超过该值时自动存储一条记录
    UINT16      hallpara;       // 霍尔系数
    UINT8       cur_dir;        //电流方向和充放电正负1；；01 放电为正 00 放电为负 \\0x80   调换霍尔方向
    UINT8       rsvd[1];        //开始为12
    
    SINT16      Raw;            // 去零漂后计算的电流             
    SINT16      Ref;            // 实测值计算的电流
    SINT16      c0;             // 零漂值
    
    UINT16      oem_special;     //OEM定制号 用于修改默认参数，定值号不同则启动默认参数一次
    UINT16      rescede;         // 
    
    UINT16      almintv;        // 报警记录防重复间隔时间
    UINT8       rsvd1[6];        //
    UINT16      crc;
} DBOEMPara;                                   


/*采集板配置参数结构*/
typedef struct db_devpara
{
    UINT32      serial_num;     // 采集板出厂编号
    UINT32      RegCnt;         // 采集板注册次数

    //GROUP 1
    UINT8       coltd;          // 生产厂家&产品类型
    UINT8       soh;            // 模块SOH
    UINT16      cap_stand;      // 电池额定容量
    UINT16      arv_on_vol;     // 均衡开启阀值
    UINT16      max_deta_v;     // 电压互差阀值

    //GROUP 2
    UINT8       temp_high;      // 模块温度过高
    UINT8       temp_2high;     // 模块温度超高
    UINT8       temp_low;       // 模块温度过低
    UINT8       temp_2low;      // 模块温度过低
    UINT8       temp_fan_on;    // 风扇开启温度
    UINT8       temp_fan_off;   // 风扇关闭温度
    UINT8       temp_heat_on;   // 加热管开启温度
    UINT8       temp_heat_off;  // 加热管关闭温度

    //GROUP 3
    UINT16      vol_high;       // 单体电压过高
    UINT16      vol_2high;      // 单体电压超高
    UINT16      vol_low;        // 单体电压过低
    UINT16      vol_2low;       // 单体电压超低

    //GROUP 4
    UINT16      amp_mav;        // 最大充电电流
    UINT16      amp_pre_chg;    // 预充电流值
    UINT8       amp_dow;        // 下调ΔA
    UINT8       pause_time;     // 暂停时间
    UINT8       amp_stop;       // 截止电流
    UINT8       ep_high;        // 端子温度过高

    //GROUP 5
    UINT16      yesrs;          // 年
    UINT8       month;          // 月
    UINT8       date;           // 日
    UINT8       hours;          // 时
    UINT8       minutes;        // 分
    //UINT8       resv9;          // 保留
    UINT8       arv_byte4;      // 均衡控制位36-33只
    //UINT8       resv10;         // 保留
    UINT8       cell3_num;      // 模块3采用单体个数
    //GROUP 6
    UINT8       arv_byte0;      // 均衡控制位8-1只
    UINT8       arv_byte1;      // 均衡控制位16-9只
    UINT8       arv_byte2;      // 均衡控制位24-17只
    UINT8       arv_byte3;      // 均衡控制位32-25只
    UINT8       temp_num;       // 模块采样温度个数
    UINT8       cell_num;       // 模块采样单体个数
    UINT8       cell2_num;      // 模块2采用单体个数(该字节此前为均衡状态位)
    UINT8       ep_2high;       // 端子温度超高

    UINT16      crc;            //校验和
} DBDevPara;

/*采集板实时数据*/
typedef struct db_devrt
{
    UINT8       bodys;      // 模块内单体个数
    UINT8       tmpnum;     // 模块内温度采样点个数
    UINT16      volt;       // 模块总电压
    UINT16      soc;        // 模块SOC
    UINT8       soh;        // 修改为定制号，原为模块SOH
    UINT8       soh_1;
    
    UINT16      itmp;       // LECU内部温度
    UINT8       ists;       // 模块内状态(风扇开关状态, 模块内风扇开关状态)
    UINT8       itrb;       // 故障 (内部通信故障, 均衡故障, 线束故障, 温度传感器故障, 内部风扇故障)
    UINT8       rsvd5;      //换电站中，用作故障信息
    UINT8       rsvd6;
    UINT8       rsvd7;
    UINT8       rsvd8;

    UINT16      maxv;       // 电池单体电压最大值
    UINT16      minv;       // 电池单体电压最小值
    UINT16      avv;        // 电池单体电压平均值
    UINT8       maxvnr;     // 最大单体电压编号
    UINT8       minvnr;     // 最小单体电压编号

    UINT8       maxtmp;     // 电池温度最大值
    UINT8       maxtmpnr;   // 最大温度采样点编号
    UINT8       mintmp;     // 电池温度最小值
    UINT8       mintmpnr;   // 最小温度采样点编号
    UINT8       avtmp;      // 电池温度平均值
    UINT8       set_confirm; //bit0----bit5  表示  设置包有没有成功收到，收到为0   未收到为1
    UINT16      answer ;     // 收到的主机请求数据帧发送的次数

    UINT16      swver;      // LECU软件版本号信息
    UINT16      hwver;      // LECU硬件版本号信息
    UINT8       serial_num[4];      // LECU唯一编号

    UINT8       tmps[36];   // 模块温度采样点温度


    UINT32      balsts;     // 均衡状态
    UINT8       cmd;        // 每次给采集模块下发的命令
    //UINT8       rsvd2;
    UINT8       balsts2;     // 均衡状态,33-36电压均衡控制
    UINT8       resved[2];     // 20151217
    

    
    UINT32       in_stauts   ;
    UINT32       out_stauts   ;

    
} DBDevRt;// itrb

/*整车信息*/
typedef struct db_sysrt
{
    SINT16      soc;
    UINT16      to_vol;     //总电压 (0.1V)
    SINT16      current;    //总电流 (0.1A)
    UINT8       maxv_lecu ; //最大电压lecu号
    UINT8       maxv_batt ; //最大电压电池号
    
    UINT16      max_v ;
    UINT8       minv_lecu ; // 最小电压lecu号
    UINT8       minv_batt ; //最小电压电池号
    UINT16      min_v ;
    UINT8       maxt_lecu ; // 最大温度lecu号
    UINT8       maxt_batt ; //最大温度电池号
    
    UINT16      max_t ;
    UINT8       mint_lecu ; //最小温度lecu号
    UINT8       mint_batt ; //最小温度电池号
    UINT16      min_t ;
    UINT8       num_lecu;   // 当前系统lecu数量
    UINT8       avg_t;   //平均温度，端子温度不参与计算 //平均温度，添加于20150407@yyzz
     
     
    UINT16      bodys;      // 当前系统单体个数
    UINT32      wh;         // 瓦时(彩屏/1000=KWH)
    UINT8       epmax;
    UINT8       epmax_lecu;


   /* UINT32      pos_res;    //绝缘电阻--总正(单位为10K欧姆)
    UINT32      neg_res;    //绝缘电阻--总负
    UINT32      total_vol_res; //绝缘电阻总电压(单位为0.1V)
   */
   
    UINT16      relayFAST; //绝缘模块继电器正1电压
    UINT16      pos_res;    //绝缘电阻--总正(单位为10K欧姆)
    UINT16      relaySLOW; //绝缘模块继电器正2电压 
    
    
    
   // UINT16      neg_res;    //绝缘电阻--总负
    UINT8      Sys_tj1; //qiang wendu1
    UINT8      Sys_tj2; // qiang wendu2
    
    
    UINT16      NCNC; //绝缘模块继电器负1电压 
    UINT16      total_vol_res; //绝缘电阻总电压(单位为0.1V)
    
    UINT32      TKAms;        //2000Ams
    
    UINT32      Ams_back;    
    SINT32      ams_2000;       //ams累计时的数据，为了取消系统中的64位运算，同时
    
    UINT8       charge_sts; //充电状态
    UINT8       resvd9;
    UINT16      charge_cycle;//未使用
    
    
    //UINT64      ws;        //瓦.秒
    UINT32      ws;        //10瓦.秒
    
   
    UINT8       Sys_cc;
    UINT8       Sys_cp; //
    //UINT16     reseved1111;        //
    UINT8       Sys_tz1; //qiang wendu1
    UINT8       Sys_tz2; // qiang wendu2
    UINT32      lecu_actives;
    UINT8       epmin;
    UINT8       epmin_lecu;
    
    
   
     UINT16      avg_v ;//整车平均电压
     UINT16      relayPRE; //与充电电压(单位为0.1V) 
 
     UINT16     reseved1111;        //
    
    
    
} DBSysRt;
/*充电及信息*/
typedef struct db_chgrt
{
    UINT16      chg_v_set;  //设置充电机输出电压 (0.1V)
    UINT16      chg_i_set;  //设置充电机输出电流 (0.1A)    
    UINT16      chg_v_out;  //充电机输出电压 (0.1V)
    UINT16      chg_i_out;  //充电机输出电流 (0.1A) 
    UINT8       chg_stage ;  //充电流程
    UINT8       chg_err ;  //充电流程
    
} DBChgRt;

 /*从控电池和温度设定*/
typedef struct
{
    UINT8       Cell_num;      // 模块电池总数量
    UINT32      Cell_Valid[2]; // 电池有效位，2 组，每组 32 位，用前 18 位
    UINT8       T_num;         // 模块温度总数量
    UINT32      T_Valid[2];    // 温度有效位，2 组，每组 32 位，用前 20 位
} DBSysNum;

/*SOC计算信息*/
typedef struct
{
    UINT32      TKAms;     // 单位实际是 ams/2000
    UINT32      wh;        // 真宗的瓦时
    UINT16      crc;
} DBSysSoc;

/*系统报警状态信息*/
typedef struct
{
    UINT32      warning1;
    UINT32      warning2;
    UINT32      warning3;
    UINT32      warning4;
}DBSysAlarm;

/*HC12和XGATE共享的杂类信息*/
typedef struct
{
    SINT16      cur;
    UINT16      ask;
    UINT16      iso_age;  /*绝缘模块老化年龄*/
    BOOL        iso_req;  /*请求绝缘模块发送信息*/
    UINT8       iso_rsvd;
    UINT32      iso_opv;   //绝缘电阻值V/o
    BOOL        INT_cutOUT;
    BOOL        cvb;  
    /*****************************************************************************/
    /* 电流测量值 （系数整定之用）                                               */
   /*****************************************************************************/
    SINT16      C0_Raw;
    SINT16      CurrentRaw;
    UINT16      Sys_Alarm_Para_L1_vol_high; 
    
    
} DBSysShared;
//Sys_Shared


/*电流及绝缘校准*/
typedef struct
{
    //SINT16      Raw[2];         // 电流校准系数
   // SINT16      Ref[2];         // 电流校准系数
   // SINT16      c0;             // 电流零漂
   
    UINT8       vcoe[DEV_IN][12];
    UINT16      IsoVpCoe;
    UINT16      IsoVnCoe;
    UINT16      IsoVppCoe;
    UINT16      IsoVnnCoe;
    UINT16      crc;            //校验和
}DBAdjpara;

/*绝缘模块信息*/
typedef struct
{
    UINT32      iso_sn;    //绝缘模块SN号
    UINT16      iso_oem;   //绝缘模块定制号
    UINT8       iso_core;  //绝缘模块核心版本号
    UINT8       iso_swer;  //绝缘模块软件版本号
    UINT16      iso_hwer;  //绝缘模块硬件版本号
}DBIsoMsg;


/*历史数据头部*/
typedef struct
{
    time_t      time;     /* 日期时间 */
    UINT16      length;   /* 本记录数据长度 */
    UINT8       lecu;     /* 采集板个数 (最高位为1表示上电运行第一条记录)*/
    UINT8       type;     /* 记录类型：A(大记录), a(小记录) */

    SINT16      current;  /*整车电流*/
    UINT8       soc;
    UINT8       soh;
    //UINT8       com_qua;  /*通信质量0-100,取代soh位置*/
    UINT16      isovolt;  /*绝缘总电压*/
    UINT16      posres;
    //UINT16      negres;   /*与总正值相同,去掉*/
    UINT16      relayN1;  /*继电器负1电压,取代总负绝缘电阻*/ 
    UINT16      vol_pre;  /*预充电压*/
    //UINT32      warning; 
    UINT16      relayP1;  /*继电器正1电压,取代warning位置*/
    UINT16      relayP2;  /*继电器正2电压,取代warning位置*/  
    
    UINT8       maxt;     /*最高温度*/
    UINT8       mint;     /*最低温度*/
    UINT16      in_sts;   /*IN状态*/
    UINT16      out_sts;  /*OUT状态*/
    UINT16      out_reson;/*OUT变化原因*/
} DBHistRecHdr;

/*日历史数据摘要信息*/
typedef struct
{
    time_t      time;       /* 日期时间 */
    UINT32      flash;      /* FLASH起始地址 */
    UINT32      end;        /* FLASH结束地址 */
    UINT32      cursor:24;  /* 本日结束时间 */
    UINT32      magic: 8;   /* 本记录有效标记 */
} DBDayIndexRec;

/*Flash中报警记录摘要信息*/
typedef struct
{
    time_t      mon;         /* 精确到月 */
    UINT32      flash;       /* 报警记录在FLASH中的起始地址 */
    UINT32      end;         /* 报警记录在FLASH中的结束地址 */
    UINT32      magic: 8;    /* 本记录有效标记 */
} DBFlashIndexRec;


/*每个采集板的参数*/
extern DBDevPara      LECU_Para[DEV_MAX];
extern DBSysNum       LECU_NVT[DEV_MAX];
extern DBSysAlarm     Sys_Alarm;

/*每个采集板的实时数据*/
#pragma DATA_SEG SHARED_DATA
extern DBSysPara      Sys_Para;
extern DBBattPara     Batt_Para;
extern DBOEMPara      OEM_Para;
extern DBChgRt        CHG_Rt;
extern DBSysRt        Sys_Rt;
extern DBSysShared    Sys_Shared;
extern DBDevRt        LECU_Rt[DEV_MAX];
extern UINT8          Temp_point[TEM_VALID_MAX];
extern UINT8          Temp_special[DEV_MAX]; 
//extern UINT8          VTemp_Middle[DEV_MAX][6];
extern UINT8          VTemp_Middle[40*DEV_MAX];
#if TC_MAX
    extern UINT8          Temp_Middle[TC_MAX][24];
#endif

extern DBIsoMsg       ISO_message; 



extern UINT16         Cell_Volts[SYS_CELL_MAX];
extern UINT16         Cell_Temp_Volts[SYS_CELL_MAX];
extern UINT16         Cell_Indexs[DEV_MAX];
extern UINT16         Cell_Temp_Indexs[DEV_MAX];
extern UINT8          LECU_Age[DEV_MAX];      /*采集板老化年龄*/
extern UINT32         LECU_Valids;          /*采集板上线标志*/


#pragma DATA_SEG DEFAULT













   /*核心软件版本信息*/
#pragma CONST_SEG CORE_VER_VARS
extern const volatile   UINT8   CORE_ver;
#pragma CONST_SEG DEFAULT


//extern volatile UINT32   sticks[];
extern volatile BOOL     ProChg;
extern volatile UINT16   OUTchg_reson;        // OUT受控原因
extern volatile UINT16   IN_pre;              // IN之前的状态
extern volatile UINT16   OUT_pre;             // OUT之前的状态
extern volatile BOOL     OUT_chgnum[16];      // OUT变化的序列号
//extern volatile UINT16 ASK_num;             // 主控招测从控和绝缘的次数
//extern volatile UINT16 ANS_num[DEV_MAX];    // 从控和绝缘回应主控招测的次数
//extern volatile UINT8  COM_rate[DEV_MAX];   // 通信质量 ANS_num/ASK_num*100%
extern volatile UINT16   BattColtd_volmax[8]; // 电池类型对应的满充单体电压最大值
extern volatile UINT8    BattType;            // 电池类型

extern volatile SINT16   ttttwww[4];
extern volatile UINT16   bms_in;
extern volatile UINT16   bms_out;
extern volatile UINT16   SOC_k;
extern volatile UINT8    BCU2_heart; 
extern volatile UINT8    BCU2_sys_sts;
extern volatile UINT8    Kin_BX[2];
extern volatile UINT8    Bat_Fault_Code[15];
//extern volatile UINT8    KOUT_flag;
extern volatile UINT8    Com_485_Err;

extern volatile UINT16  O_Voltage;
extern volatile UINT16  T_Voltage;
//extern volatile UINT16  I_Voltage; 
extern volatile UINT8   Bat_Fault_flag; 
//extern volatile UINT16  SOF_down_flag;  //软件下载标志
//extern volatile UINT8   Cmd_Stop_TRDP ;
//extern volatile UINT16  SOC_Tenths;
//extern volatile UINT8   Read_flag;
extern volatile UINT8   Equ_EN;
extern volatile UINT16  HPU_sta;
extern volatile UINT32  reg_err2_code;
extern volatile UINT8   rst_bmu_save;  
///////////////OTA////
extern volatile UINT16  SOF_down_flag;  //软件下载标志
extern volatile UINT8   OTA_Rec_flag;
extern volatile UINT8   OTA_RecStop_flag;
extern volatile UINT8   Cmd_Stop_TRDP ;
extern volatile UINT16  R_STP1_len;
extern volatile UINT16  R_STP1_add;
////////test///////////
typedef struct
{
    UINT32 test_wdg_tick;
    // 设置标记
    UINT8 to_vol_set;
    UINT8 current_set;
    UINT8 vol_set;
    UINT8 temp_set;
    UINT8 soc_set;
    UINT8 O_Voltage_set;
    // 参数
    UINT16 to_vol;          // 总电压
    SINT16 current;         // 总电流
    UINT16 max_v;           // 最高电压
    UINT16 min_v;           // 最低电压
    UINT16 max_t;           // 最高温度
    UINT16 min_t;           // 最低温度
    SINT16 soc;             // SOC
    UINT16 bms_err;         // 硬件故障状态
    UINT16 O_Voltage;       // 外总压
} TEST_PARAM;

extern TEST_PARAM test_param;

#define TEST_WDG_ACTIVE() (gSysTicks - test_param.test_wdg_tick < 5000 / 10)
/////
/*
#define CELL_NUM        384U      // 
#define BAT_CAP_AH      23U      // 23Ah
#define SOC_TBL_SIZE    20U      // 5%~100% 共20点

typedef enum {
    TEMP_N30 = 0,   // -30℃
    TEMP_N20,       // -20℃
    TEMP_N10,       // -10℃
    TEMP_0,         // 0℃
    TEMP_25,        // 25℃
    TEMP_45,        // 45℃
    TEMP_CNT
} TempIndexTypeDef;


// 顺序：5% 10% 15% 20% 25% 30% 35% 40% 45% 50% 55% 60% 65% 70% 75% 80% 85% 90% 95% 100%
static const UINT16 OCV_TABLE[TEMP_CNT][SOC_TBL_SIZE] = {
    // -30℃
    {2080,2110,2130,2150,2160,2180,2190,2210,2230,2250,2280,2320,2350,2360,2380,2410,2450,2490,2530,2540},
    // -20℃
    {2080,2110,2130,2140,2160,2180,2190,2210,2230,2250,2280,2320,2350,2360,2380,2410,2450,2490,2530,2540},
    // -10℃
    {2080,2110,2130,2140,2160,2180,2190,2210,2230,2250,2280,2320,2350,2360,2380,2410,2450,2490,2530,2540},
    // 0℃
    {2080,2110,2130,2140,2160,2180,2190,2210,2230,2250,2290,2320,2350,2360,2380,2410,2450,2490,2530,2540},
    // 25℃
    {2080,2110,2130,2150,2160,2180,2190,2210,2230,2250,2290,2320,2340,2350,2380,2410,2450,2490,2530,2540},
    // 45℃
    {2080,2110,2130,2150,2160,2180,2190,2210,2230,2260,2290,2320,2330,2350,2380,2410,2450,2490,2530,2540}
}; */


/*****************************************************************************/
/* 数据库模块初始化                                                          */
/*****************************************************************************/
void DB_Init (void);

/*****************************************************************************/
/* 数据库模块总入口                                                          */
/*****************************************************************************/
void DB_main (void);

/*****************************************************************************/
/* 新增一个采集器模块                                                        */
/*****************************************************************************/
UINT8 DEV_new (UINT32 sn);

/*****************************************************************************/
/* 实时数据模块－设置SOC百分比                                               */
/*****************************************************************************/
void RT_SetSOC (UINT16 percent);

/*****************************************************************************/
/* 实时数据模块－获取采集模块的单体电压值                                    */
/*****************************************************************************/
UINT16 *RT_Volts (UINT8 dev);

/*****************************************************************************/
/* 实时数据模块－获取系统第N节电池单体电压值，返回0表示无效                  */
/*****************************************************************************/
UINT16 RT_Voltage (UINT16 num, UINT8 *devNo, UINT8 *cellNo);

/*****************************************************************************/
/* 根据时间查找历史数据                                                      */
/*****************************************************************************/
BOOL HST_find (time_t from, DBDayIndexRec *rec, DBHistRecHdr *hdr, UINT32 *faddr);

/*****************************************************************************/
/* 根据历史数据头判断记录是否有效                                            */
/*****************************************************************************/
BOOL HST_IsValidHdr (DBHistRecHdr *hdr);

/*****************************************************************************/
/* 判断历史数据记录头尺寸                                                    */
/*****************************************************************************/
UINT16 HST_HdrSize (DBHistRecHdr *hdr);

/*****************************************************************************/
/* 读历史数据                                                                */
/*****************************************************************************/
void HST_read (UINT32 faddr, void *buf, UINT16 bytes);

/*****************************************************************************/
/* 历史数据清零                                                              */
/*****************************************************************************/
void HST_reset (void);
/*****************************************************************************/
/* 检查IN状态函数                                                            */
/*****************************************************************************/
UINT16  BMS_InSts (void);

/*****************************************************************************/
/* 检查OUT状态函数                                                          */
/*****************************************************************************/
UINT16  BMS_OutSts (void);

