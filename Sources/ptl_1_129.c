#include <string.h>
#include "sys.h"
#include "drv_can.h"
#include "drv_fm.h"
#include "drv_rtc.h"
#include "db.h"
#include "ptl.h"
#include "db_para_cfg.h"             
#include "db_para_struct.h"
#include "modbus.h"

#if PTL_Pro

#define VL                  OEM_ver            //版本低    10机制显示   12>>>>>>>0x12
#define VH                  CORE_ver            //版本高    10机制显示
 
#define START_YEAR                  26                  //出厂年
 
#define VER_TIME_Y                  26                  //内部版本
#define VER_TIME_M                  10                   //内部版本
#define VER_TIME_D                  23                  //内部版本
#define VER_YG                      1                   //内部版本



///////////////////////BCU1///////////////////
#if !PTL_TEST
    #define KM15_OS       IO_IN0   //KM15主正接触器辅助触点反馈信号，常开辅助触点
    #define KA12_OS       IO_IN1   //牵引回路切除信号，常开辅助触点
    #define KM10_OS       IO_IN2   //KM10主负接触器辅助触点反馈信号，常开辅助触点
    #define CHARGE_EN_OS       CC2      //充电机使能信号（预留）
    #define KA9_OS          ACC_ON   //硬件保护反馈中继状态检测,常开辅助触点
    #define KM14_OS           HVIL     //预充接触器辅助触点反馈信号，常开辅助触点
#endif

#if PTL_TEST
    #define KM15_OS       ino_test   //KM15主正接触器辅助触点反馈信号，常开辅助触点
    #define KA12_OS       in1_test   //牵引回路接触器切除信号中继状态检测，常开辅助触点
    #define KM10_OS       in2_test   //KM10主负接触器辅助触点反馈信号，常开辅助触点
    #define CHARGE_EN_OS       cc2_test      //充电机使能信号（预留）
    #define KA9_OS          acc_on_test   //硬件保护反馈中继状态检测,常开辅助触点
    #define KM14_OS           hvil_test     //预充接触器辅助触点反馈信号，常开辅助触点
#endif
#define FU2_OS         ((LECU_Rt[8].in_stauts)&0x01)  //主正熔断器状态检测，常闭触点
#define FU1_OS         ((LECU_Rt[0].in_stauts)&0x01)  //主负熔断器状态检测，常闭触点

#define KA15            IO_OUT0 //主正接触器线圈控制
#define KA10            IO_OUT1 //主负接触器线圈控制
#define KA14            IO_OUT2 //预充接触器线圈控制
#define KA16            IO_OUT3 //KA16继电器线圈控制，控制输出动力电池故障状态
#define KA8             IO_OUT4 //KA8继电器线圈控制，控制KM8加热负接触器
#define KA7             IO_OUT5 //KA7继电器线圈控制，控制KM7正加热接触器

////////////////BCU2//////////////////////
#define KA11_OS       ((Kin_BX[0]>>0)&0x01)  //KA11牵引变流器检修开关状态，常开触点
#define KA13_OS       ((Kin_BX[0]>>1)&0x01)  //KA13应急牵引投入信号中继状态检测，常开触点
#define FU3_FU4_OS    ((Kin_BX[0]>>2)&0x01)  //加热熔断器状态检测,常闭触点；加热熔断器状态检测，常开触点
#define KM7_KM8_OS    ((Kin_BX[0]>>3)&0x01)  //加热接触器状态检测,常开触点；加热接触器状态检测，常开触点
////////////////////

#define CLOSE                       0x00
#define OPEN                        0x01

#define NC_CLOSE                    0x01
#define NC_OPEN                     0x00

#define BAT_KWH                      514   //51.4kWh

/////////////////////////自检状态///////////////////////////////
#define NSC                         0               //未自检
#define SCR                         3               //自检中
#define SCS                         1               //自检成功
#define SCF                         2               //自检故障
//////////////////////////水冷机&加热//////////////////////////////////////
#define C_ST                        (33+55)
#define C_SP                        (28+55) 
#define C_KO                        (28+55) 
#define H_ST                        (5+55) 
#define H_SP                        (10+55) 
#define TEMP_VALUE_MIN              (55-25)   // 有效温度下限
#define TEMP_MAX                    (15+55)   //最大温度



#define PHM_OD                      130  //PHM增加报文    固定70+4+电压*2+温度+1+PHM_OD
#define CHVMIN                      2530


/////////////////////////定时器///////////////////////////////
#define TIMER_TURN_ON               0       //
#define TIMER_CAN0                  1       //CAN0发送定时器
#define TIMER_CAN1                  2       //CAN1发送定时器
#define TIMER_RS485                 3  
#define TIMER_BMU                   4
#define TIMER_C                     5
#define TIMER_CHA_DELAY             6
#define TIMER_CHA_1                 7
#define TIMER_PRE_CHA_DELAY             8
#define TIMER_PRE_CHA_COUNT_DELAY       9
#define TIMER_PRE_CHA_KM14_DELAY       10
#define TIMER_HOT_DELAY             11
#define TIMER_HOT_UP_DELAY             12
#define TIMER_OKM10                  13
#define TIMER_OKM14                  14
#define TIMER_OKM15                  15
#define TIMER_OKM7_KM8                 16
#define TIMER_CCU_COM               17
#define TIMER_BAT_OFF               18
#define TIMER_SOC                   19
#define TIMER_SOC_SET               20
#define TIMER_FFC                   21
#define TIMER_FFS                   22
#define TIMER_TEST                  23
#define TIMER_CKM10                 24
#define TIMER_CKM14                 25
#define TIMER_CKM15                 26
#define TIMER_CKM7_KM8              27
#define TIMER_CCU_FIRE              28
#define TIMER_BAT_FIRE              29
#define TIMER_TA1_ERR               30
#define TIMER_TV1_ERR               31
#define TIMER_FU1_FU2_ERR           32
#define TIMER_FU3_FU4_ERR           33
#define TIMER_CAN4                  34
#define TIMER_SOC_L                 35
#define TIMER_BAT_4H                36
#define TIMER_BAT_1MIN              37
#define TIMER_BAT_CHA_CUR_ERR         38
#define TIMER_VOL_ERR               39
#define TIMER_TEMP_ERR              40
#define TIMER_TCU_ERR               41
#define TIMER_KA9_ERR               42
#define TIMER_CAN0_HB               43  //CAN0心跳定时器
#define TIMER_CKM15_ERR                 44
#define TIMER_CKM14_ERR                 45
#define TIMER_CKM10_ERR                 46
//箱体测试用，防止接触器快速切换
#define TIMER_PRE_CHA_DELAY_TEST     47
#define TIMER_RATE_BASE             48
#define TIMER_FFC_RATE              49
#define TIMER_OVOL_H_ERR            50
#define TIMER_OVOL_L_ERR            51
#define TIMER_OVOL_L_KM15_ERR       52

#define TIMER_ID_TMAX               53  //最大计数个数

#define TIMER_RESET_T(id)           sticks_PTL[id] = gSysTicks_2ms;           //计数器复位


/////////////////////////////////故障Bat_Fault_Code[X]///////////////// 
//Bat_Fault_Code[0]~Bat_Fault_Code[6]为动力电池协议字节1004~1009
//Bat_Fault_Code[0,1,2]
#define TVOL_H                      0x80 //总压高
#define TVOL_L                      0x40 //总压低
#define VOL_H                       0x20 //单体电压高
#define VOL_L                       0x10 //单体电压低
#define V_DIFF                      0x08 //压差过大
#define T_DIFF                      0x04 //温差过大
#define TMP_H                       0x02 //温度过高
#define TMP_L                       0x01 //温度过低

//Bat_Fault_Code[3]
#define CUR_H3                      0x80  //充电过流
#define FUR_H3                      0x40  //放电过流
#define SOC_L3                      0x20  //SOC过低
#define BMS_ERR                     0x10  //动力电池BMS硬件故障
#define BMS_COM_ERR                 0x08  //动力电池内部通讯故障
#define IWD_ERR                     0x04  //动力电池正极绝缘过低故障
#define VLNC_ERR                    0x02 //动力电池电压过低无法充电故障
#define KM7_KM8_CERR                0x01  //动力电池热接触器无法闭合故障 C：Close
//Bat_Fault_Code[4]
#define CUR_H2                      0x80  //充电过流
#define FUR_H2                      0x40  //放电过流
#define SOC_L2                      0x20  //SOC过低
#define KM7_KM8_OERR                0x10  //动力电池热接触器无法断开故障 O：Open
#define HFUS_ERR                    0x08  //动力电池加热熔断器故障 H:Heat up
#define FUS_ERR                     0x04  //动力电池内部熔断器故障
#define KM15_CERR                   0x02  //动力电池总正接触器无法闭合故障
#define KM14_CERR                   0x01  //动力电池预充接触器无法闭合故障
//Bat_Fault_Code[5]
#define CUR_H1                      0x80  //充电过流
#define FUR_H1                      0x40  //放电过流
#define SOC_L1                      0x20  //SOC过低
#define KM10_CERR                   0x10  //动力电池总负接触器无法闭合故障
#define PCH_ERR                     0x08  //动力电池充电机预充电故障 PCH：Pre Charge
#define HPU_ERR                     0x04  //动力电池BMPU触发过压硬件冗余保护
#define OVOL_H_ERR                  0x02  //动力电池牵引回路电压/外总压过高故障
#define OVOL_L_ERR                  0x01  //动力电池牵引回路电压/外总压过低故障
//Bat_Fault_Code[6]为动力电池协议字节1041
#define FFC_S_ERR1                  0x01 //动力电池系统的电池包序列1-8的灭火探头状态
#define FFC_S_ERR2                  0x02 
#define FFC_S_ERR3                  0x04 
#define FFC_S_ERR4                  0x08 
#define FFC_S_ERR5                  0x10
#define FFC_S_ERR6                  0x20 
#define FFC_S_ERR7                  0x40 
#define FFC_S_ERR8                  0x80 

//Bat_Fault_Code[7]为动力电池协议字节1040 
#define FFC_S_ERR9                  0x40 //动力电池系统的电池包序列9-10的灭火探头状态
#define FFC_S_ERR10                 0x80

//Bat_Fault_Code[8]为动力电池协议字节1042
#define HPU_TEMP_ERR                0x80 //动力电池BPU触发过温硬件冗余保护
#define KM15_OERR                   0x40 //动力电池总正接触器无法断开故障
#define KM14_OERR                   0x20 //动力电池预充接触器无法断开故障
#define KM10_OERR                   0x10 //动力电池总负接触器无法断开故障
#define BMU_VOL_ERR                 0x08 //动力电池BMU单体电压采集异常
#define TV1_ERR                     0x04 //动力电池电压传感器TV1故障
#define BMS_BC_CURRENT_ERR          0x02 //动力电池与充电机电流不一致故障
#define TA1_ERR                     0x01 //动力电池电流传感器TA1故障

//Bat_Fault_Code[9]为动力电池协议字节1043
#define BAT_CABIN_TEMP_OVER         0x80 //动力电池舱温度过高故障
#define BAT_CABIN_TEMP_DIFF_OVER    0x40 //动力电池舱温差过高故障
#define HOT_ERR                     0x20 //加热功能异常
#define BMU_TEMP_ERR                0x10 //动力电池BMU温度采集异常
#define DATA_RECORD_ERR             0x08 //动力电池数据记录异常

//Bat_Fault_Code[10]为动力电池协议字节1044
#define BAT_FIRE_ALM                0x80 //动力电池灭火报警
#define FIRE_BOTTLE_TRIG            0x40 //动力电池灭火钢瓶触发
#define FIRE_VALVE_SPRAY_TRIG       0x20  // 动力电池舱电磁阀喷放触发
#define FIRE_BOTTLE1_FAULT          0x10  // 灭火系统钢瓶1故障
#define FIRE_BOTTLE2_FAULT          0x08  // 灭火系统钢瓶2故障
#define FIRE_DETECT_ERR             0x04  // 动力电池灭火探测器故障
#define FIRE_SYS_OFFLINE            0x02  // 电池灭火系统离线
#define FIRE_THERMAL_SUPPRESS_FAIL  0x01  // 动力电池热失控抑制失效/严重热失控

//Bat_Fault_Code[11]为动力电池协议字节1045
#define TBMS_OK_STATUS               0x01 //动力电池BMS准备就绪

//Bat_Fault_Code[12]为动力电池协议字节1046
#define TBMS_SOC_LOW                 0x01 //动力电池SOC过低保护

//Bat_Fault_Code[13]为动力电池协议字节1051
#define CHG_STOP_REQ                 0x80 //动力电池故障请求充电机停机
#define CHG_PULSE_LOCK_REQ           0x40 //动力电池请求封锁脉冲
#define KM14_STATUS                  0x20 //动力电池预充接触器状态
#define KM15_STATUS                  0x10 //动力电池主回路正接触器状态
#define KM10_STATUS                  0x08 //动力电池主回路负接触器状态
#define KM7_STATUS                   0x04 //动力电池加热正接触器状态
#define KM8_STATUS                   0x02 //动力电池加热负接触器状态
#define BAT_FAULT_STATUS             0x01 //动力电池故障状态

//Bat_Fault_Code[14]为动力电池协议字节1052
#define PRECHG_PULSE_LOCK            0x80 //预充电封锁
#define MAIN_FUSE_STATUS             0x40 //动力电池主熔断器状态
#define HEAT_MODE_STATUS             0x20 //加热模式
#define HEAT_FUSE_STATUS             0x10 //动力电池加热熔断器状态


//self_err_code
#define BDC_NET_ERR                 0x01
#define CCU_NET_ERR                 0x02
//RxCAN_count[0]  CAN0通信异常
#define BCU1_COM_ERR        0x01
#define BCU2_COM_ERR        0x02
#define BMS_OUTCOM_ERR1     0x04
#define BMS_OUTCOM_ERR2     0x08

#define BMS_OUTCOM_CAN_ERR  0x40
#define BMS_OUTCOM_ERR      0x80
//RxCAN_count[1]  CAN1通信异常
#define FIRE_COM_ERR        0x01 
#define BTMS_COM_ERR        0x02
#define IWD_COM_ERR         0x04

////////////////////////////BCU1,BCU2/////////////////////////////////////////////////////////
UINT8       Kout_BX[2]                          = {0xff,0xff};  //输入引脚信息交换
UINT8       Kin_BX[2]                           = {0xff,0xff};  //输出引脚信息交换
UINT8       BCU2_heart                          = 0;
UINT8       BCU2_sys_sts                        = 0;
/////////////////////////////////定时器/////////////////////////////////
static volatile UINT32  sticks_PTL[TIMER_ID_TMAX]; 
/////////////////////////////////AC/DC/////////////////////////////////
static volatile UINT8       Charger_heart       = 0;
static volatile UINT8       C_loc_heart         = 0;
static volatile UINT8       Charger_sts         = 0;
static volatile UINT16      Charger_Volt        = 0;  
static volatile UINT16      Charger_Cur         = 0;
static volatile UINT16      charge_voltage      = 0;                //充电机电压
static volatile UINT16      charge_current      = 0;                //充电机电流
static volatile UINT8       Hot_mode            = 0;
static volatile UINT16      Hot_Vol             = 745;
static volatile UINT16      Hot_Cur             = 0;
static volatile UINT8       stop_charge         = 0x00;                //0X80停机
static volatile UINT8       BDC_Sys_sts         = 0;
static volatile UINT8       BDC_KMreback        = 0;
static volatile UINT8       bdc_err_num         = 0;
static volatile UINT8       Ch_Hot_mode         = 0;
static volatile UINT8       Ch_Hot_cmd          = 0;
static volatile UINT8       Ch_Hot_over         = 0;
static volatile UINT8       Ch_Hot_yc           = 0;
static volatile UINT8       Ch_hands_sts        = 0;
static volatile UINT8       Ch_sts_err          = 0;
static volatile UINT8       Ch_run_sts          = 0;
static volatile UINT8       Ch_up_sts           = 0;
static volatile UINT8       BtoDC_sts           = 0;
static volatile UINT16      loc_date[4]         = {0,0,0,0};
static volatile UINT8       loc_ffc_flag        = 0;
//static volatile UINT8       BtoDC_sts1          = 0;
/////////////////////////////////CCU_R///////////////////////////////////
struct tm                   CCU_T;
static volatile UINT8       CCU_Time_en         = 0;
static volatile UINT16      CCU_life            = 0;
static volatile UINT8       CCU_Bat_off         = 0;
static volatile UINT8       TR_Work_mode        = 0;
static volatile UINT8       TR_up_sts           = 0;
static volatile UINT8       TR_date             = 0;
static volatile UINT8       TR_Bat_reset        = 0;
static volatile UINT8       TR_Aux_one          = 0;
static volatile UINT8       TR_Bat_stop         = 0;
static volatile UINT8       TR_KM_sts           = 0;
static volatile UINT8       TR_BK_cmd           = 0;
static volatile UINT8       CCU_DC_heart        = 0;
static volatile UINT8       CCU_DC_len          = 0;
static volatile UINT8       CCU_DC_sts          = 0;
static volatile UINT16      CCU_DC_vol          = 0;
static volatile UINT16      CCU_DC_cur          = 0;
static volatile UINT16      CCU_loc_heart       = 0;
static volatile UINT8       ccu_ok_count = 0;     //计算心跳正常连续计数
/////////////////////////////////CCU_T///////////////////////////////////
static volatile UINT8       BC_T_Heart          = 0;                //CAN0接收心跳
static volatile UINT8       CCU_com_num         = 0;
static volatile UINT8       CCU_MC_wait         = 0;
static volatile UINT8       CCU_MT_cmd          = 0;
static volatile UINT8       CCU_locmc_cmd       = 0;
static volatile UINT8       CCU_locmt_cmd       = 0;                 //本地模式
/////////////////////////////////CAN0///////////////////////////////////
static volatile UINT16       CAN0_add            = 0;
static volatile UINT8       BDC_T_Heart         = 0;      //CAN0发送心跳
static volatile UINT8       CAN0R_flag          = 0; 
/////////////////////////////////CAN1///////////////////////////////////
static volatile UINT8       CAN1_add            = 0;
static volatile UINT8       Heart_TCAN1         = 0;      //CAN1发送心跳
/////////////////////////////////CAN4///////////////////////////////////
static volatile UINT8       CAN4_add            = 0;
/////////////////////////////////RS485/////////////////////////////////        
static volatile UINT8       RS485_heart         = 0;
static volatile UINT8       bat_heart             = 0;
static volatile UINT8       RS485_heart1         = 0;
extern volatile ModbusFrame     HmiBuf;                             //485接收Buf
static volatile UINT8       rs485_err_num       = 0;
static volatile UINT8       bat_err_num       = 0;
////////////////////////////绝缘检测//////////////////////////////////////////////////////////
static volatile UINT8   	EN_IWD     = 0;                  //绝缘监测使能信号
static volatile UINT8   	STA_IWD    = 0;                 //绝缘监测开始能信号
static volatile UINT8   	FAU_IWD    = 0;                 //绝缘监测故障信号
static volatile UINT8       IWD_cmd             = 0;
static volatile UINT16      IWD_Res             = 0;      //绝缘电阻, 1 KΩ/bit
static volatile UINT16      Bus_Voltage         = 0;      //母线电压, 0.1 V/bit
static volatile UINT8       Fault_Level         = 0;      //故障等级
static volatile UINT8       IWD_sta             = 0;
static volatile UINT8       Alarm_Status        = 0;      //报警状态
static volatile UINT8       IWD_Life            = 0;      //生命信号
static volatile UINT16      P_Res               = 0;      //正端绝缘电阻, 1 KΩ/bit
static volatile UINT16      N_Res               = 0;      //负端绝缘电阻, 1 KΩ/bit
static volatile UINT16      Fault_Voltage       = 0;      //故障点电压, 0.1 V/bit
volatile UINT8 IWDCheck = 0; 


/////////////////////////////////SYS///////////////////////////////////
static volatile UINT8       Bat_Sys_sts         = 0;
static volatile UINT8       Bat_Fault_Code[15]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static volatile UINT8       check_bms           = SCR;              //bms自检状态   3=自检中；1=自检成功；2=自检失败；其余值无效
static volatile UINT8       Bat_Rt_sts          = 0;
static volatile UINT8       Battery_SOH_BMS     = 100;              //蓄电池SOH
static volatile UINT8       self_err_code       = 0;
static volatile UINT8       ks_stage            = 0;                //流程状态
static volatile UINT16      Chg_Cmd             = 0;                //需要停充电机标志
static volatile UINT8       BMU_err_num         = 0;                //
static volatile UINT8       cha_stage           = 0;                //充电流程
static volatile UINT8       pre_cha_stage       = 0;            //预充流程
static volatile UINT8       precharge_fail_count = 0;           //预充封锁计数
static volatile UINT8       hot_stage           = 0;                //加热流程 0:未加热 1:等待充电机加热允许信号 2:加热中
static volatile UINT8       pre_status = 0;     //预充状态
static volatile UINT8       charge_mode_BMS     = 0;
static volatile UINT8       mode_stage[4]       = {0xff,0xff,0xff,0xff};
static volatile UINT32      Q_remain            = 514;               //可用电量初始值
static volatile UINT32      Q_usable            = 514;
static volatile UINT32      Q_lock              = 514;
static volatile UINT8       KOUT_flag           = 0xff;   //继电器状态

static volatile UINT8       Heart_RCAN1         = 0;      //CAN1接收心跳

static volatile UINT8       CAN1_R_flag         = 0;
static volatile UINT16      loc_soc             = 0;
      
static volatile     UINT16  tx0_cell_index = 0;
static volatile     UINT16  tx0_temp_index = 0;
static volatile     UINT16  tx0_dsu_cell_index = 0;
static volatile     UINT16  tx0_dsu_temp_index = 0;

volatile UINT32             reg_err1_code  = 0;  
volatile UINT32             reg_err2_code  = 0;  
static volatile UINT8       rst_bmu_flag   = 0;  
static volatile UINT8       rst_bmu_num    = 0;
UINT8 rst_bmu_save   = 0;  

static volatile UINT16  Bat_Power_Tx = 0;   // 当前输出功率，0.1kW/bit，0~1000
static volatile UINT16      Bat_Time_Tx            = 0;
static UINT8 last_bms_addr = 0;
static UINT8 hb_div = 0;
/////////////////////////////////DSU///////////////////////////////////
static volatile UINT16      DSU_heart           = 0;


/////////////////////////////////自定义状态///////////////////////////////////
static volatile UINT8       Bat_self_code       = 0;
static volatile UINT8       Bat_Fault_flag      = 0;
static volatile UINT8       net_temp            = 65;
static volatile UINT8       Test_num            = 0;  //测试模式编号can报文0字节
static volatile SINT16      Test_date0           = 0; //测试模式can报文1~2字节
static volatile SINT16      Test_date1           = 0; //测试模式can报文3~4字节
static volatile SINT16      Test_date2           = 0; //测试模式can报文5~6字节

static volatile BOOL        Only_L1_Ltemp_falg           = 0;//低温严重故障表示
static volatile UINT8       hot_enter_time_Ltemp            = 0;//加热启动时刻最低温度
//static volatile UINT8       only_lspanto_hot_exit            = 0;//加热转过分相的退出
static volatile UINT8       KA15_cerr_time            = 0;//KA15无法闭合异常次数
static volatile UINT8       KA14_cerr_time            = 0;//KA14无法闭合异常次数
static volatile UINT8       KA10_cerr_time            = 0;//KA10无法闭合异常次数
///////////////////SOC&SOP&SOH/////////////-20，0，25，45，////////////////////////////
typedef struct{
  
    UINT16  size;
    UINT16  chksum;
    UINT32  CF_num;                         //充放电次数
    UINT32  C_kw;                           //
    UINT32  F_kw;                           //

    UINT16  C_re_ele;                       //充电剩余电量
    UINT16  F_re_ele;                       //放电剩余电量
}SAVE_DATA;
SAVE_DATA Sys_kwh,Bup_kwh;
static volatile UINT16      ChgSOP              = 488;             //蓄电池放电功率
static volatile UINT16      DchSOP              = 732;
static volatile SINT16      SOC_diff            = 0;      //SOC误差值
static volatile UINT32      sta_ele             = 0;             //启始电量
static volatile UINT32      end_ele             = 0;             //结束电量	
static volatile UINT8       Save_flag           = 0;
volatile SINT16             SOC_Offset          = 0;          //SOC修正值
volatile UINT16             SOC_num             = 0;            //SOC分钟计数
volatile UINT16             Bat_soh2            = 100;
volatile UINT8              SOC_S_flag          = 0; 

//////////////////////////////FFC///////////////////////////////////////////////////
typedef struct{             
    UINT8   sts;           
    UINT8   ver;
    UINT8   bot1_sts;                              
    UINT8   bot2_sts;
    UINT8   bot_sv_sta;                 
    UINT8   cb_sv_sta;              
    UINT8   R_life;
    UINT8   bms_off;

   UINT8   sensor[10];
   UINT8   temp[10];

   UINT8   Smoke[10];
   UINT8   pollute[10];
    UINT8   data[12];//为了给DSU发送数据
    UINT8   sen_num;
    UINT8   T_life;   
    UINT8   max_t1[4]; 
    UINT8   max_t2[4];
    UINT8   num_t1[4]; 
    UINT8   num_t2[4];   

    /* 新增：温升速率标志 */
    UINT8   rate_flag[4];      // 每舱温升速率标志，bit4=1 表示满足
    UINT8   last_max_t[4];     // 上一秒最高温（用于算温升）
    UINT8   rate_cnt[4];       // 连续满足计数
}FIREDATA;

// 定义一个包含所有TCU发送动力电池及相关专属信号的结构体
typedef struct {
    // 1300/1320 字节相关信号
    BOOL cutoff;            // 动力电池切除
    BOOL comm_ccu;          // CCU诊断本车动力电池通信故障
    BOOL ecab_smoke;        // 电气舱烟雾报警
    BOOL tcnv_comm;         // CCU诊断关联牵引变流器通信故障 
    
    // 1301/1321 字节相关信号
    BOOL panto_stat;        // 受电弓状态 (公用信号)
    
} CCUDATA;
typedef struct {
    BOOL tcnv_cool;         // 牵引变流器制检修开关状态
    BOOL heat_en;           // 允许加热模式
    BOOL lspanto;           // 低速过分相模式
    BOOL mc_en;             // 允许主回路接触器闭合(这个信号是过分相专用，如果为1表示不预充闭合接触器，在过分相处理)

    BOOL chg_dis;           // 动力电池充电机放电
    BOOL chg_work;          // 充电机工作状态
    BOOL cont_en;           // 接触器使能(动力电池牵引切除环线已经改名)
    BOOL chg_out;           // 充电机输出接触器状态

    BOOL chg_fault;         // 动力电池充电机故障状态
    BOOL req_mc_cls;        // 请求主接触器闭合
    BOOL req_mc_opn;        // 请求主接触器断开
    BOOL chg_act;           // 充电机充电中
    
} TCUDATA;

// 定义车辆和电池数量的宏，方便后续维护
#define BAT_1_IDX 0
#define BAT_2_IDX 1
#define CAR_COUNT 1
#define BAT_COUNT 2

/*
 * 声明二维数组：[车辆索引][电池索引]
 * 
 * 【关联关系说明】：
 * 1. 受电弓状态 (panto_stat) 为公用信号，统一存在此数组中。
 * 2. 牵引变流器通信故障 (tcnv_comm) 对应关系如下：
 *    - 1 8车动力电池1 (CAR_1_IDX, BAT_1_IDX) 对应 2 7车牵引变流器
 *    - 1 8车动力电池2 (CAR_1_IDX, BAT_2_IDX) 对应 4 5车牵引变流器
 */
volatile CCUDATA CCU_Data[CAR_COUNT][BAT_COUNT];

volatile CCUDATA * volatile temp_ccu_data = NULL;
volatile TCUDATA  temp_tcu_data = {0};


FIREDATA FFCD,CCU;
static volatile UINT8      ffc_life             = 0;
static volatile UINT8      ffc_com_num          = 0;
static volatile UINT8      ffc_com_sts          = 0;
static volatile UINT8      ffc_s_err            = 0;

///////////CCU发TBMS////////////
volatile BOOL Bat_Off  = FALSE;      //蓄电池 OFF
volatile BOOL NetErrorReset  = FALSE; 
             
volatile BOOL HotFlag = FALSE;
              
volatile BOOL TBMS_Traction = FALSE;   //动力电池应急牵引
volatile BOOL TBMS_Standby  = FALSE;   //电池供电 应急等待
volatile BOOL EmemStandby  = FALSE;   //应急等待
volatile BOOL last_TBMS_Traction = FALSE;//用于判断下降沿
volatile BOOL last_TBMS_Standby = FALSE;
volatile BOOL last_EmemStandby = FALSE;
//volatile BOOL Emer_Generate_Power = FALSE;  //动力电池应急牵引微制动发电

volatile UINT8 EmerTraBrakePower_Quit = 0;
volatile UINT8  BAT_Work  = 0;   //电池工作状态
volatile UINT16 Tra_Vol=0;
volatile UINT16 Aux_Vol=0;
static volatile UINT8  Contact_Open=0;   //接触器断开标志
static volatile UINT8  M_Fus  = 1;                    //主回路熔断器状
static volatile UINT8  RxCAN_count[2] = {0,0};       //接收本地心跳

volatile UINT8 Tra_Contact_ERR=0;   
volatile UINT8 BDC_Contact_ERR=0; 
volatile UINT8 Full_Charge_Flag = 0; 

volatile UINT8 BCU2_Out=0xff;
volatile UINT8 AuxBat1_Heart=0;
volatile UINT8 AuxBat1_Out=0xff; 
volatile UINT8 AuxBat1_In=0xff; 
volatile UINT8 AuxBat2_Heart=0;
volatile UINT8 AuxBat2_Out=0xff; 
volatile UINT8 AuxBat2_In=0xff; 
BOOL Bat_Stop_Exit1 = 1;
BOOL Bat_Stop_Exit2 = 1; 
BOOL Bat_Stop_Exit =1;
BOOL Contact_TBMS1 = 1;
BOOL Contact_TBMS2 = 1;
BOOL Contact_TBMS = 1;
volatile UINT8 Out_stock_flag = 0;
volatile UINT8 Slave_Ctrl_En = 0;
volatile UINT8 Slave_Power = 0;
UINT16          O_Voltage1   = 0;
UINT16          T_Voltage1   = 0; 
volatile UINT8 Hot_Flag = 0;
typedef struct{
  
    UINT16  life;
    UINT8   sta;
    UINT16  OutVol;
    UINT16  OutCurrent;
      
}BDCDATA;
BDCDATA BDC,NetBDC,ModbusBDC;
volatile UINT8 chavl =0;
volatile UINT8 chavh =0;
static volatile UINT8 aux_precharge_err_cnt=0;
static volatile UINT8 tra_precharge_err_cnt=0;
volatile UINT8  Test_offset_H = 15;
volatile UINT8  Test_offset_L = 15;
volatile UINT16  temp_ta1_current = 0;
volatile UINT16  temp_tv1_volt = 0;
volatile UINT16  temp_ta1_current_err_count = 0;
volatile UINT16  temp_tv1_volt_err_count = 0;
static volatile UINT8 tcu_cur_over_flag = 0;
static volatile UINT8 tv1_diag_init = 0;      
static volatile UINT8 ta1_diag_init = 0;      
#if PTL_TEST
    volatile UINT8 test_init_flag = 0;
    volatile UINT8  ino_test = 0;
    volatile UINT8  in1_test = 0;
    volatile UINT8  in2_test = 0;
    volatile UINT8  cc2_test = 0;
    volatile UINT8  acc_on_test = 0;
    volatile UINT8  hvil_test = 0;
#endif
/*****************************************************************************/
/* 定时器                                                              */
/*****************************************************************************/
UINT8  EXPIRED_T(UINT8 id,UINT32 ms) {
    if(gSysTicks_2ms >= sticks_PTL[id]) {
        if((gSysTicks_2ms - sticks_PTL[id]) >= (ms/2))    return 1;
        else    return 0;
    }else{
        if((0xffffffff-sticks_PTL[id] + gSysTicks_2ms)>= (ms/2)) return 1;
        else    return 0;
    }
} 

/*****************************************************************************/
/* 本协议初始化函数       采集                                                   */
/*****************************************************************************/
void plt_Init (void) {
    UINT8 i,j;

    gSysTicks = 0;
    gSysTicks_2ms = 0;                   
    //上位机显示电池采样只数，可修改
    for(j = 0;j<9;j++){
        LECU_NVT[j].Cell_num        = 36; //电压采集数
        LECU_NVT[j].Cell_Valid[0]   = 0x0003FFFF;   // 前 18 位有效
        LECU_NVT[j].Cell_Valid[1]   = 0x0003FFFF;    //
        LECU_NVT[j].T_num           = 36; //温度采集数
        LECU_NVT[j].T_Valid[0]      = 0x0003FFFF;   // 前 20 位有效
        LECU_NVT[j].T_Valid[1]      = 0x0003FFFF;
    }


    //初始化KA15、KA10、KA14、KA7、KA8、KA16为断开
    KA15 = OPEN;
    KA10 = OPEN;
    KA14 = OPEN;
    KA7 = OPEN;
    KA8 = OPEN;
    KA16 = OPEN;

    Sys_kwh.F_kw      = 0;
    Sys_kwh.CF_num    = 0;
    Fm_read (SAVE_176_DATA, &Sys_kwh, sizeof(Sys_kwh));
    //Bat_Sys_sts     = 0;
    //FFCD.sen_num = 0;
    for(i = 0; i < 4; i++) {
        FFCD.rate_flag[i]  = 0;
        FFCD.last_max_t[i] = 0;
        FFCD.rate_cnt[i]   = 0;
    } 
    for(i=0;i<TIMER_ID_TMAX;i++)  TIMER_RESET_T(i);  
}
/*****************************************************************************/
/* 保存电量                                                                 */
/*****************************************************************************/
void Sys_kwh_Save (UINT16 FADR ,SAVE_DATA save_data)
{
    save_data.size = sizeof(save_data);
    save_data.chksum = 0;
    save_data.chksum = 0 - Chksum (&save_data, sizeof(save_data));


    Fm_write (FADR, &save_data, sizeof(save_data));
}
/*****************************************************************************/
/* CAN帧发送                                                                 */
/*****************************************************************************/
static void Can1Write(CanbusFrame *cf)
{
    UINT16  i;

    //CAN1_write_std (cf);
    CAN1_write(cf);

    for (i = 0; i < 1000; i++)
    {
        if (CAN1TFLG != 0x0)
            break;
    }

    if(i >= 1000)
    {
        CAN1TARQ = 0x07;
        CAN1MISC = 0X00;
    }
}
static void Can0Write(CanbusFrame *cf)
{
    UINT16  i;

    //CAN1_write_std (cf);
    CAN0_write(cf);

    for (i = 0; i < 1000; i++)
    {
        if (CAN0TFLG != 0x0)
            break;
    }

    if(i >= 1000)
    {
        CAN0TARQ = 0x07;
        CAN0MISC = 0X00;
    }
  //      while (reg[0x6] != 0x7);
}
static void Can4Write(CanbusFrame *cf)
{
    UINT16  i;

    //CAN1_write_std (cf);
    CAN4_write(cf);

    for (i = 0; i < 1000; i++)
    {
        if (CAN4TFLG != 0x0)
            break;
    }

    if(i >= 1000)
    {
        CAN4TARQ = 0x07;
        CAN4MISC = 0X00;
    }
}
////////////////////////////////////////////////////////////////////////////
static void Rx_can0(void)
{
    CanbusFrame *cf;
    UINT32 ID;
    UINT16 temp_data = 0;
    UINT8 temppp1[8];
    UINT8  L_ID = 0;
   // UINT8  chavh = 0;
  //  UINT8  chavl = 0;
    //处理接收到的CAN报文
    if (CAN0_RxRPtr == CAN0_RxWPtr)
        return;

    cf      = &CAN0_RxBuf[CAN0_RxRPtr];
    ID      = cf->id;
   
    switch (ID)
    {
        case 0x0a01:
        case 0x1a01: //偏移0
            CCU_life        = ((UINT16)cf->data[0]<<8) | cf->data[1];
            CCU.data[0]     = cf->data[2];
            Bat_Off         = cf->data[2]&0x01;
            NetErrorReset = (cf->data[2]>>1)&0x01;   //故障复位
            TBMS_Traction = (cf->data[2]>>4)&0x01;   //动力电池应急自走行
            TBMS_Standby = (cf->data[2]>>5)&0x01;   //动力电池应急等待
            //Emer_Generate_Power = (cf->data[2]>>6)&0x01;   //动力电池应急牵引微制动发电预留
            EmemStandby = (cf->data[2]>>7)&0x01;   //动力电池应急救援等待

            CCU.data[1]     = cf->data[3];
            CCU_Time_en          = cf->data[3]&0x01;
    
            CCU_T.tm_year   = cf->data[4];  //系统时间用于校时
            CCU_T.tm_mon    = cf->data[5];
            CCU_T.tm_mday   = cf->data[6];
            CCU_T.tm_hour   = cf->data[7]; 
            CAN0R_flag |= 0x01;  
          
        break;
        case 0x0a02:
        case 0x1a02: //偏移8
            CCU_T.tm_min    = cf->data[0];
            CCU_T.tm_sec    = cf->data[1]; 
            
            CAN0R_flag |= 0x02; 
        break;

        case 0x1a03: //偏移1300
        case 0x0a03:

            CCU.data[2] = cf->data[0];
            CCU_Data[0][0].cutoff     = cf->data[0]&0x01;
            CCU_Data[0][1].cutoff     = (cf->data[0]>>1)&0x01;
            CCU_Data[0][0].comm_ccu   = (cf->data[0]>>2)&0x01;
            CCU_Data[0][1].comm_ccu   = (cf->data[0]>>3)&0x01;
            CCU_Data[0][0].tcnv_comm  = (cf->data[0]>>4)&0x01; // 对应2 7车
            CCU_Data[0][1].tcnv_comm  = (cf->data[0]>>5)&0x01; // 对应4 5车
            CCU_Data[0][0].ecab_smoke = (cf->data[0]>>6)&0x01;
            CCU_Data[0][1].ecab_smoke = (cf->data[0]>>7)&0x01;

            CCU.data[3] = cf->data[1];
            CCU_Data[0][0].panto_stat = cf->data[1]&0x01; // 受电弓公用
            CCU_Data[0][1].panto_stat = cf->data[1]&0x01; // 受电弓公用

            CAN0R_flag |= 0x10;

        break;
        
        case 0x0a04:
        case 0x1a04: 

             
        break;
        case 0x0a05:
        case 0x1a05:


        break;

        case 0x0d01:
        case 0x1d01:
            temp_tcu_data.heat_en = (cf->data[0])&0x01;
            temp_tcu_data.lspanto = (cf->data[0]>>1)&0x01;
            temp_tcu_data.mc_en = (cf->data[0]>>2)&0x01;
            temp_tcu_data.chg_fault = (cf->data[0]>>4)&0x01;
            
            temp_tcu_data.req_mc_cls = (cf->data[1])&0x01;
            temp_tcu_data.req_mc_opn = (cf->data[1]>>1)&0x01;
            temp_tcu_data.chg_act = (cf->data[1]>>2)&0x01;
            temp_tcu_data.chg_dis = (cf->data[1]>>3)&0x01;
            temp_tcu_data.chg_work = (cf->data[1]>>4)&0x01;
            temp_tcu_data.cont_en = (cf->data[1]>>5)&0x01;
            temp_tcu_data.chg_out = (cf->data[1]>>6)&0x01;
            temp_tcu_data.tcnv_cool = (cf->data[1]>>7)&0x01;
        break;
       
        case 0x0d02:
        case 0x1d02:
        
        break;
        case 0x10ffe0d1:
            temppp1[0] = cf->data[0];
            temppp1[1] = cf->data[1];
            temppp1[2] = cf->data[2];
            temppp1[3] = cf->data[3];
            temppp1[4] = cf->data[4];
            temppp1[5] = cf->data[5];
            temppp1[6] = cf->data[6];
            temppp1[7] = cf->data[7]; 
     
            Sys_kwh.F_kw=((UINT32)temppp1[3]<<24)|((UINT32)temppp1[2]<<16)|((UINT32)temppp1[1]<<8)|temppp1[0];
            Sys_kwh.C_kw=((UINT32)temppp1[7]<<24)|((UINT32)temppp1[6]<<16)|((UINT32)temppp1[5]<<8)|temppp1[4];
            Sys_kwh_Save(SAVE_176_DATA,Sys_kwh);
        
            break;
        ///////DSU相关///////////
        case 0x10ffb1a3:
        case 0x10ffb1a4:
            temppp1[0] = cf->data[0];
            temppp1[1] = cf->data[1];
            temppp1[2] = cf->data[2];
            temppp1[3] = cf->data[3];
            temppp1[4] = cf->data[4];
            temppp1[5] = cf->data[5];
            temppp1[6] = cf->data[6];
            temppp1[7] = cf->data[7]; 
            if((cf->data[0]==0x75)&&(cf->data[1]==0x70)&&(cf->data[2]==0x31)&&(cf->data[3]==0x32)) {
                DisableInterrupts;
                *(volatile unsigned int *)0x3FF0 = 0x3134;
                *(volatile unsigned int *)0x3FF2 = 0x1234;
                *(volatile unsigned char *)0x3FF4 = OEM_Para.bms_addr;

                *(volatile long *far)0x200001 = 0;  //lint !e511
                *(volatile long *far)0x300001 = 0;  //lint !e511             

            }
        break;
        case 0x10ffe0d2:
        test_param.test_wdg_tick = gSysTicks;
        switch (cf->data[0])
        {
            case 1:
                test_param.temp_set = 1;
                test_param.max_t = cf->data[1];
                test_param.min_t = cf->data[3];
                break;
            case 2:
                test_param.vol_set = 1;
                test_param.max_v = ((UINT16)cf->data[2] << 8) | cf->data[1];
                test_param.min_v = ((UINT16)cf->data[4] << 8) | cf->data[3];
                break;
            case 3:
                test_param.soc_set = 1;
                test_param.soc = cf->data[1];
                break;
            case 4:
                test_param.to_vol_set = 1;
                test_param.to_vol = ((UINT16)cf->data[2] << 8) | cf->data[1];
                break;
            case 5:
                test_param.current_set = 1;
                test_param.current = (((UINT16)cf->data[2] << 8) | cf->data[1]);
                break;
            case 6:
                test_param.bms_err = ((UINT16)cf->data[2] << 8) | cf->data[1];
                break;
            case 7:
                test_param.O_Voltage_set = 1;
                test_param.O_Voltage = ((UINT16)cf->data[2] << 8) | cf->data[1];
                break;
        }
        break;
        case 0x1001b3a0: // 心跳包
            test_param.test_wdg_tick = gSysTicks;
            break;
        case 0x1002b3a0:
            test_param.to_vol_set = 1;
            test_param.to_vol = ((UINT16)cf->data[0] << 8) | cf->data[1];
            break;
        case 0x1003b3a0:
            test_param.current_set = 1;
            test_param.current = (((UINT16)cf->data[0] << 8) | cf->data[1]) - 9000;
            break;
        case 0x1004b3a0:
            test_param.vol_set = 1;
            test_param.max_v = ((UINT16)cf->data[0] << 8) | cf->data[1];
            test_param.min_v = ((UINT16)cf->data[2] << 8) | cf->data[3];
            break;
        case 0x1005b3a0:
            test_param.temp_set = 1;
            test_param.max_t = cf->data[0];
            test_param.min_t = cf->data[1];
            break;
        case 0x1006b3a0:
            test_param.soc_set = 1;
            test_param.soc = cf->data[0];
            break;
        case 0x1007b3a0:
            test_param.bms_err = cf->data[0];
            break;
       
        default:
            return; 
        break;
    }
    //删除当前帧?
    CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);  
}


void Assign_ccu_Data() {
    switch(OEM_Para.bms_addr){
        case 3:
            temp_ccu_data = &CCU_Data[0][0]; //1车动力1
            break;
        case 4:
            temp_ccu_data = &CCU_Data[0][1]; //1车动力2
            break;
        default:
            temp_ccu_data = NULL;
            break;
    }
}

//判断下降沿
BOOL Is_Falling(BOOL current, BOOL *last)
{
    BOOL falling = ((*last == TRUE) && (current == FALSE));
    *last = current;
    return falling;
}
/* 应急退出接口                                                         */
/*****************************************************************************/
/**
 * @brief   应急退出接口
 * @return  
 * @note    检测应急退出条件是否满足，满足退出应急模式
 */
void Emergency_Exit(void)
{
    UINT8 traction_falling = 0;
    UINT8 standby_falling = 0;
    UINT8 emem_standby_falling = 0;
    //判断下降沿
    // if (Is_Falling(TBMS_Traction, &last_TBMS_Traction)) {
    //     traction_falling = 1;
    // }
    // else{
    //     traction_falling = 0;
    // }
    // if (Is_Falling(TBMS_Standby, &last_TBMS_Standby)) {
    //     standby_falling = 1;
    // }
    // else{
    //     standby_falling = 0;
    // }
    // if (Is_Falling(EmemStandby, &last_EmemStandby)) {
    //     emem_standby_falling = 1;
    // }
    // else{
    //     emem_standby_falling = 0;
    // }


    if((temp_tcu_data.req_mc_opn == 1) || //请求断开1
       (temp_ccu_data->cutoff == 1) || //有切除指令
       (Contact_Open == 1) || //有严重故障（严重故障会断接触器）
       (KA11_OS == OPEN) || //硬线检修开关为断开  
       (temp_ccu_data->tcnv_comm == 1)||//牵引变流器通信故障为1
       (KA12_OS == CLOSE)//牵引回路切除信号为闭合 
    //    (traction_falling == 1) || //应急自走行下降沿
    //    (standby_falling == 1) || //应急等待下降沿
    //    (emem_standby_falling == 1) //救援等待下降沿
    )
    {
        TR_Work_mode = 0;
        KA15 = OPEN;
        KA14 = OPEN;
        KA10 = OPEN;
    }

    //微制动退出

}
/* 应急进入接口                                                         */
/*****************************************************************************/
/**
 * @brief   应急进入接口
 * @return  
 * @note    检测应急进入条件是否满足，满足进入应急模式
 */
void Emergency_Enter(void)
{
    //动力电池准备就绪代表可以进入应急模式
    if(((TBMS_Traction == 1) ||(TBMS_Standby == 1) || (EmemStandby == 1)) &&
        (KM15_OS == CLOSE)&&
        (KM10_OS == CLOSE)&&
        (Contact_Open == 0)&&
        ((Bat_Fault_Code[12]&TBMS_SOC_LOW) == 0)&&
        (pre_status == 1)
    )
    {
        Bat_Fault_Code[11] |= TBMS_OK_STATUS;
        if(TBMS_Traction == 1){
            TR_Work_mode = 2;
        }
        else if(TBMS_Standby == 1){
            TR_Work_mode = 1;
        }
        else if(EmemStandby == 1){
            TR_Work_mode = 3;
        }
    }
    else{
        Bat_Fault_Code[11] &=~TBMS_OK_STATUS;
    }
}

/* 预充投入接口                                                          */
/*****************************************************************************/
/**
 * @brief   预充投入接口
 * @return  UINT8 0-不满足条件不投入预充，1-满足条件已完成预充，2-正在预充
 * @note    检测预充条件是否满足，满足则投入预充
 */
UINT8 Pre_Charge(void)
{
    UINT32 voltage = 0;
    UINT32 battery_voltage = 0;
    pre_status = 0;
    
    if (temp_ccu_data == NULL){
        //预充归零退出
        pre_status = 0;
        pre_cha_stage = 0;
        TIMER_RESET_T(TIMER_PRE_CHA_KM14_DELAY);
        TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
        return pre_status;
    }
    //判断接触器是否闭合，未闭合才预充
    if((KM10_OS == CLOSE) && (KM15_OS == CLOSE) && (KM14_OS == OPEN)){
        pre_status = 1;
        pre_cha_stage = 0;
        TIMER_RESET_T(TIMER_PRE_CHA_KM14_DELAY);
        TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
        return pre_status;
    }

    //判断预充是否被封锁
    if((Bat_Fault_Code[14] & PRECHG_PULSE_LOCK) == PRECHG_PULSE_LOCK){
        //预充归零退出
        pre_status = 0;
        pre_cha_stage = 0;
        TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
        return pre_status;
    }
    //判断5分钟内如果投预充三次，预充被封锁
    if (EXPIRED_T(TIMER_PRE_CHA_COUNT_DELAY, 300000))
    {
        precharge_fail_count = 0;
        TIMER_RESET_T(TIMER_PRE_CHA_COUNT_DELAY);
    }
    else{
        if(precharge_fail_count >= 3){
            Bat_Fault_Code[14] |= PRECHG_PULSE_LOCK;
            //预充归零退出
            pre_status = 0;
            pre_cha_stage = 0;
            precharge_fail_count = 0;
            TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
            return pre_status;
        }
    }
    //判断闭合主正主负接触器条件
    if((temp_tcu_data.req_mc_cls == 1) && //请求闭合1
       (temp_tcu_data.req_mc_opn == 0) && //请求断开0
       (temp_ccu_data->cutoff == 0) && //无切除指令
       ((Bat_Fault_Code[14] & HEAT_MODE_STATUS) == 0) &&
       (KM7_KM8_OS == OPEN) && //加热为断开
       (Contact_Open == 0)&&//无严重故障（严重故障会断接触器）
       ((Bat_Fault_Code[14] & PRECHG_PULSE_LOCK) == 0)&&//无预充封锁
       (KA11_OS == CLOSE)&&//硬线检修开关闭合 
       (temp_ccu_data->tcnv_comm == 0)&&//牵引变流器通信故障为0
       (KA12_OS == OPEN)&&//牵引回路切除信号为断开 
       ((Bat_Fault_Code[13] & CHG_PULSE_LOCK_REQ) == CHG_PULSE_LOCK_REQ)//封锁脉冲请求为1
       //加热转过分相现在正常投入预充
       //(temp_tcu_data.mc_en == 0)&&//允许闭合正负极接触器闭合为0(这个信号是过分相专用，如果为1表示不预充闭合接触器，在过分相处理)
       //(only_lspanto_hot_exit == 0)//加热转过分相的退出为0 
    )
    {

    }
    else{
        pre_status = 0;
        pre_cha_stage = 0;
        TIMER_RESET_T(TIMER_PRE_CHA_KM14_DELAY);
        TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
        return pre_status;
    }
    switch (pre_cha_stage)
    {
        case 0:
            //投入预充，并增加预充次数
            precharge_fail_count++;
            //闭合总负KM10
            KA10 = CLOSE;
            pre_cha_stage = 1;
            pre_status = 2;
            TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
            break;

        case 1:
            //检测5s内总负KM10闭合
            if (KM10_OS == CLOSE)
            {
                //闭合总负KM14
                KA14 = CLOSE;
                TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
                pre_cha_stage = 2;
                pre_status = 2;
            }
            else if (EXPIRED_T(TIMER_PRE_CHA_DELAY, 5000))
            {
                //接触器故障，总负KM10未闭合，预充归零退出
                pre_cha_stage = 0;
                pre_status = 0;
                TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
            }
            break;
        case 2:
            //检测5s内总负KM14闭合
            if (KM14_OS == CLOSE)
            {
                TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
                pre_cha_stage = 3;
                pre_status = 2;
            }
            else if (EXPIRED_T(TIMER_PRE_CHA_DELAY, 5000))
            {
                //接触器故障，总负KM14未闭合，预充归零退出
                TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
                pre_cha_stage = 0;
                pre_status = 0;
            }
            break;
        case 3:
            voltage = O_Voltage;
            battery_voltage = Sys_Rt.to_vol;

            // 立即判定
#if PTL_PHY_TEST
            if(1){
                if (EXPIRED_T(TIMER_PRE_CHA_DELAY_TEST, 5000)){
                    TIMER_RESET_T(TIMER_PRE_CHA_DELAY_TEST);
                }
                else{
                    break;
                }
#endif
#if !PTL_PHY_TEST
            if(((voltage * 10) >= (battery_voltage * 2)) &&
            ((voltage ) <= (battery_voltage + 450))){
#endif
                // 电压正常，预充判定成功，立即退出
                pre_cha_stage = 4;
                pre_status = 2;
                TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
            }else{
                // 电压不正常，看是否超时
                if (EXPIRED_T(TIMER_PRE_CHA_DELAY, 2000)){
                    // 超过2s还没成功，报错退出
                    Bat_Fault_Code[5] |= PCH_ERR;
                    pre_cha_stage = 0;
                    pre_status = 0;
                    TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
                }
                // 没超时就继续等，下次进来再判
            }
            break;
        case 4:
            voltage = O_Voltage;
            battery_voltage = Sys_Rt.to_vol;
#if PTL_PHY_TEST
            if(1){
                if (EXPIRED_T(TIMER_PRE_CHA_DELAY_TEST, 5000)){
                    TIMER_RESET_T(TIMER_PRE_CHA_DELAY_TEST);
                }
                else{
                    break;
                }
#endif
#if !PTL_PHY_TEST
            if(((voltage * 10) >= (battery_voltage * 9)) &&
            ((voltage) <= (battery_voltage + 450))){
#endif
                pre_cha_stage = 5;
                pre_status = 2;
                TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
            }else{
                if (EXPIRED_T(TIMER_PRE_CHA_DELAY, 5000)){   
                    Bat_Fault_Code[5] |= PCH_ERR;
                    pre_cha_stage = 0;
                    pre_status = 0;
                    TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
                }
            }
            break;
        case 5:
            voltage = O_Voltage;
            battery_voltage = Sys_Rt.to_vol;
#if PTL_PHY_TEST
            if(1){
                if (EXPIRED_T(TIMER_PRE_CHA_DELAY_TEST, 5000)){
                    TIMER_RESET_T(TIMER_PRE_CHA_DELAY_TEST);
                }
                else{
                    break;
                }
#endif
#if !PTL_PHY_TEST
            if(((voltage * 10) >= (battery_voltage * 9)) &&
                ((voltage) <= (battery_voltage + 450))){
#endif
                //检测电压正常闭合总正KM15
                KA15 = CLOSE;
                TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
                TIMER_RESET_T(TIMER_PRE_CHA_KM14_DELAY);
                pre_cha_stage = 6;
                pre_status = 2;
            }else{
                //电压超出范围，预充失败
                Bat_Fault_Code[5] |= PCH_ERR;
                //预充归零退出
                pre_cha_stage = 0;
                pre_status = 0;
                TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
                TIMER_RESET_T(TIMER_PRE_CHA_KM14_DELAY);
            }
            break;


        case 6:
            //计时2s后断开预充接触器KM14
            if (EXPIRED_T(TIMER_PRE_CHA_KM14_DELAY, 2000))
            {
                TIMER_RESET_T(TIMER_PRE_CHA_KM14_DELAY);
                KA14 = OPEN;
            }
            //检测5s内总负KM15闭合
            if ((KM15_OS == CLOSE) && (KM14_OS == OPEN))
            {   
                TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
                TIMER_RESET_T(TIMER_PRE_CHA_KM14_DELAY);
                pre_cha_stage = 0;
                pre_status = 2;
            }
            else if (EXPIRED_T(TIMER_PRE_CHA_DELAY, 5000))
            {
                //接触器故障，总负KM15未闭合，预充归零退出
                TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
                TIMER_RESET_T(TIMER_PRE_CHA_KM14_DELAY);
                pre_cha_stage = 0;
                pre_status = 0;
            }
            break;

        default:
            pre_cha_stage = 0;
            pre_status = 0;
            TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
            TIMER_RESET_T(TIMER_PRE_CHA_KM14_DELAY);
            break;
    }

    return pre_status;
}
/*****************************************************************************/
/* CAN1接收                                                                  */
/*****************************************************************************/
static void Rx_can1(void){
    CanbusFrame *cf;
    UINT32 ID;
    UINT16 NUM=0;

    //处理接收到的CAN报文
    if (CAN1_RxRPtr == CAN1_RxWPtr)
        return;

    cf = &CAN1_RxBuf[CAN1_RxRPtr];
    ID=cf->id;
    switch(ID) {
        //接收绝缘监测读取
        case 0x1819A1A4:
            IWD_Res         =  ((UINT16)cf->data[0]<<8) | cf->data[1];//绝缘电阻, 1 KΩ/bit
            Bus_Voltage     =  ((UINT16)cf->data[2]<<8) | cf->data[3];//母线电压, 0.1 V/bit
            Fault_Level     =  cf->data[4];                           //故障等级
            IWD_sta         =  cf->data[5];                           //运行状态
            Alarm_Status    =  cf->data[6];                           //报警状态
            IWD_Life        =  cf->data[7];                           //生命信号        
         break;
         case 0x1819A1A3:
            P_Res         =  ((UINT16)cf->data[0]<<8) | cf->data[1];//正端绝缘电阻, 1 KΩ/bit
            N_Res         =  ((UINT16)cf->data[2]<<8) | cf->data[3];//负端绝缘电阻, 1 KΩ/bit
            Fault_Voltage =  ((UINT16)cf->data[4]<<8) | cf->data[5]; //故障点电压, 0.1 V/bit
         break;
    default:
        return;   //交给其他规约模块处理
        break;
  }
    //删除当前帧
  CAN1_RxRPtr = (CAN1_RxRPtr + 1) & (CAN1_RXBSZ - 1);  
}
/*****************************************************************************/
/* CAN4接收                                                                  */
/*****************************************************************************/
static void Rx_can4(void){

  CanbusFrame *cf;
  UINT32 ID;
  UINT8 NUM=0;                                        
  //处理接收到的CAN报文
  if (CAN4_RxRPtr == CAN4_RxWPtr)
    return;
  
  cf = &CAN4_RxBuf[CAN4_RxRPtr];
  ID=cf->id;
 
  switch(ID) {
    case 0x0C0060A2:

    break;    
    case 0x0C0061A2:

    break;
    
    
         
   default:
      return;   //交给其他规约模块处理
    break;
  }

    //删除当前帧
  CAN4_RxRPtr = (CAN4_RxRPtr + 1) & (CAN4_RXBSZ - 1);  

}
/*****************************************************************************/
/* CAN0发送                                                                  */
/*****************************************************************************/
static void Tx0(void)
{
    CanbusFrame cf;
    CanbusFrame cf1;
    
    UINT8   temy_d  = 0;
    UINT8   i       = 0;
    UINT8   IN_now  = 0xff;
    UINT8   OUT_now = 0xff;
     
    
    if (EXPIRED_T (TIMER_CAN0, 2)) {
        memset(&cf, 0, sizeof(cf));
        memset(&cf1, 0, sizeof(cf1));
        // 每 100次轮询（200ms）插发一次心跳
        hb_div++;
        if (hb_div >= 100) {
            hb_div = 0;

            // 心跳帧1
            memset(&cf, 0, sizeof(cf));
            cf.id  = 0x0B00 + 1;
            cf.pri = 1;
            cf.len = 8;
            BDC_T_Heart++;
            cf.data[0] = BDC_T_Heart >> 8;
            cf.data[1] = BDC_T_Heart;
            cf.data[2] = VH;
            cf.data[3] = VL;
            cf.data[4] = check_bms;
            cf.data[5] = Bat_Sys_sts;
            cf.data[6] = Sys_Rt.soc;
            cf.data[7] = Battery_SOH_BMS;
            Can0Write(&cf);

            cf.id = 0x1B00 + 1;
            Can0Write(&cf);

            TIMER_RESET_T(TIMER_CAN0);
            return;   // 本次只发心跳，不轮询其他帧
        }
        switch(CAN0_add) {
            case 1:
                // cf.id         = 0x0B00+CAN0_add;
                // cf.pri        = 1;
                // cf.len        = 8;

                // BDC_T_Heart++;
                // cf.data[0] = BDC_T_Heart>>8;//生命信号
                // cf.data[1] = BDC_T_Heart;
                // cf.data[2] = VH;//软件版本
                // cf.data[3] = VL;
                // cf.data[4] = check_bms;//自检状态
                // cf.data[5] = Bat_Sys_sts;//电池状态
                // cf.data[6] = Sys_Rt.soc;//SOC
                // cf.data[7] = Battery_SOH_BMS;//SOH
                
                // cf1 = cf;
                // Can0Write (&cf);
               
                // cf1.id         = 0x1B00+CAN0_add;
                // Can0Write (&cf1);
            break;
            case 2:
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;
                cf.data[0] = Sys_Rt.to_vol>>8;//总电压
                cf.data[1] = Sys_Rt.to_vol;
                cf.data[2] = (Sys_Rt.current+9000)>>8;
                cf.data[3] = (Sys_Rt.current+9000);
                cf.data[4] = Sys_Rt.max_v>>8;//最高单体电压 
                cf.data[5] = Sys_Rt.max_v;
                cf.data[6] = Sys_Rt.min_v>>8; //最低单体电压
                cf.data[7] = Sys_Rt.min_v;

                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;
            case 3:
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = (Cell_Indexs[Sys_Rt.maxv_lecu-1]+Sys_Rt.maxv_batt)>>8;//最高单体电压位置
                cf.data[1] = (Cell_Indexs[Sys_Rt.maxv_lecu-1]+Sys_Rt.maxv_batt);
                cf.data[2] = (Cell_Indexs[Sys_Rt.minv_lecu-1]+Sys_Rt.minv_batt)>>8;//最低单体电压位置
                cf.data[3] = (Cell_Indexs[Sys_Rt.minv_lecu-1]+Sys_Rt.minv_batt); 
                cf.data[4] = Sys_Rt.max_t;//动力电池最高温度
                cf.data[5] = Sys_Rt.min_t;//动力电池最低温度
                cf.data[6] = (((Sys_Rt.maxt_lecu-1) *36) + Sys_Rt.maxt_batt) >> 8; //最高温度位置
                cf.data[7] = (((Sys_Rt.maxt_lecu-1) *36) + Sys_Rt.maxt_batt); 
                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;
            case 4:
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;
                
                cf.data[0] = (((Sys_Rt.mint_lecu-1) *36) + Sys_Rt.mint_batt) >> 8;//最低温度位置
                cf.data[1] = (((Sys_Rt.mint_lecu-1) *36) + Sys_Rt.mint_batt);
                cf.data[2] = charge_voltage>>8; //最高允许充电电压
                cf.data[3] = charge_voltage;
                cf.data[4] = charge_current>>8;//最高允许充电电流
                cf.data[5] = charge_current;

                
                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;

            case 126://偏移1000
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;     

                tx0_temp_index = (CAN0_add - 86) * 8;
                for (i = 0; i < 4; i++){
                    cf.data[i] = Temp_point[tx0_temp_index + i];
                }
                //故障
                cf.data[4] = Bat_Fault_Code[2];//偏移1004
                cf.data[5] = Bat_Fault_Code[1];//偏移1005
                cf.data[6] = Bat_Fault_Code[0];//偏移1006
                cf.data[7] = Bat_Fault_Code[3];//偏移1007

                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;

            case 127://偏移1008
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;                

                //故障
                cf.data[0] = Bat_Fault_Code[4];
                cf.data[1] = Bat_Fault_Code[5];


                cf.data[4] = ffc_com_sts;//消防系统状态
                cf.data[5] = (FFCD.ver/10);
                cf.data[6] = CCU.bot1_sts;//消防系统钢瓶1状态
                cf.data[7] = CCU.bot2_sts;//消防系统钢瓶2状态



                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;

            case 128://偏移1016
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;     

                cf.data[0] = FFCD.pollute[0];//消防探头污染值
                cf.data[1] = FFCD.pollute[1];
                cf.data[2] = FFCD.pollute[2];
                cf.data[3] = FFCD.pollute[3];
                cf.data[4] = FFCD.pollute[4];
                cf.data[5] = FFCD.pollute[5];
                cf.data[6] = FFCD.pollute[6];
                cf.data[7] = FFCD.pollute[7];
                
                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;

            case 129://偏移1024
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;                

                cf.data[0] = FFCD.pollute[8];
                cf.data[1] = FFCD.pollute[9];
                cf.data[2] = FFCD.temp[0];//消防探头温度
                cf.data[3] = FFCD.temp[1];
                cf.data[4] = FFCD.temp[2];
                cf.data[5] = FFCD.temp[3];
                cf.data[6] = FFCD.temp[4];
                cf.data[7] = FFCD.temp[5];
                
                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;

            case 130://偏移1032
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;                

                cf.data[0] = FFCD.temp[6];
                cf.data[1] = FFCD.temp[7];
                cf.data[2] = FFCD.temp[8];
                cf.data[3] = FFCD.temp[9];
                cf.data[4] = IWD_Res>>8;
                cf.data[5] = IWD_Res;
                cf.data[6] = Q_usable>>8; //最大可用电量
                cf.data[7] = Q_usable;

                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;

            case 131://偏移1040
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;                

                cf.data[0] =((FFCD.cb_sv_sta&0x1f)|Bat_Fault_Code[7]);   // 1-5内管路电磁阀状态+9-10灭火探头状态;
                cf.data[1] = Bat_Fault_Code[6];
                cf.data[2] = Bat_Fault_Code[8];
                cf.data[3] = Bat_Fault_Code[9];
                cf.data[4] = Bat_Fault_Code[10];
                cf.data[5] = Bat_Fault_Code[11];//动力电池准备就绪
                cf.data[6] = Bat_Fault_Code[12];//电量过低保护
                cf.data[7] = TR_Work_mode;//工作模式

                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;

            case 132://偏移1048
                 cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;                
                //发送给TCU
                cf.data[3] = Bat_Fault_Code[13];
                cf.data[4] = Bat_Fault_Code[14];
                cf.data[5] = Sys_Rt.to_vol>>8;//内总压
                cf.data[6] = Sys_Rt.to_vol;
                cf.data[7] = O_Voltage>>8;//外总压

                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;

            case 133://偏移1056
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;                
                //发送给TCU
                cf.data[0] = O_Voltage;
                cf.data[1] = (Sys_Rt.current+9000)>>8;//总电流
                cf.data[2] = (Sys_Rt.current+9000);
                cf.data[3] = charge_current>>8;//充电电流
                cf.data[4] = charge_current;
                cf.data[5] = charge_voltage>>8;//充电电压
                cf.data[6] = charge_voltage;


                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;

            case 134://偏移1064
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;     

                //补充数据
                cf.data[4] = Bat_Time_Tx>>8;//可用时间预估
                cf.data[5] = Bat_Time_Tx;
                cf.data[6] = 100 - Sys_Rt.soc;// 电池充放电DOD
                cf.data[7] = Bat_Power_Tx>>8;//当前输出功率

                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;

            case 135://偏移1072
                cf.id         = 0x0B00+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;     

                cf.data[0] = Bat_Power_Tx;
                cf.data[1] = Sys_kwh.F_kw>>8;//放电电量累计量
                cf.data[2] = Sys_kwh.F_kw;

                cf1 = cf;
                Can0Write (&cf);
               
                cf1.id         = 0x1B00+CAN0_add;
                Can0Write (&cf1);
            break;

            //PHM
            case 200:
               cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                DSU_heart++;
                cf.data[0] = DSU_heart>>8; //心跳
                cf.data[1] = DSU_heart;
                cf.data[2] = gSysSec>>24;  //时间
                cf.data[3] = gSysSec>>16;
                cf.data[4] = gSysSec>>8;
                cf.data[5] = gSysSec;
                cf.data[6] = Bat_Sys_sts;//电池状态
                cf.data[7] = check_bms;  //电池自检状态
                Can0Write (&cf);   
            break;

            case 201:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;
    
                cf.data[0] = SOC_k/10;      //Sys_Rt.soc;
                cf.data[1] = Battery_SOH_BMS;   //电池健康度SOH
                cf.data[2] = charge_current>>8;  //充电电流
                cf.data[3] = charge_current;
                cf.data[4] = charge_voltage>>8;  //充电电压
                cf.data[5] = charge_voltage;
                cf.data[6] = Sys_Rt.max_v>>8; //最高单体电压
                cf.data[7] = Sys_Rt.max_v;

                Can0Write (&cf);
            break;
            case 202:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

 
                cf.data[0] = Sys_Rt.min_v>>8; //最低单体电压
                cf.data[1] = Sys_Rt.min_v;
                cf.data[2] = (Cell_Indexs[Sys_Rt.maxv_lecu-1]+Sys_Rt.maxv_batt)>>8;  //最高单体电压位置
                cf.data[3] = (Cell_Indexs[Sys_Rt.maxv_lecu-1]+Sys_Rt.maxv_batt);     
                cf.data[4] = (Cell_Indexs[Sys_Rt.minv_lecu-1]+Sys_Rt.minv_batt)>>8;  //最低单体电压位置
                cf.data[5] = (Cell_Indexs[Sys_Rt.minv_lecu-1]+Sys_Rt.minv_batt);     
                cf.data[6] = Sys_Rt.max_t;
                cf.data[7] = Sys_Rt.min_t;       

                Can0Write (&cf);
            break;
            case 203:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

 
                cf.data[0] = (Cell_Indexs[Sys_Rt.maxt_lecu-1]+Sys_Rt.maxt_batt)>>8;  //最高温度位置
                cf.data[1] = (Cell_Indexs[Sys_Rt.maxt_lecu-1]+Sys_Rt.maxt_batt);     
                cf.data[2] = (Cell_Indexs[Sys_Rt.mint_lecu-1]+Sys_Rt.mint_batt)>>8;  //最低温度位置
                cf.data[3] = (Cell_Indexs[Sys_Rt.mint_lecu-1]+Sys_Rt.mint_batt);     
                cf.data[4] = Sys_Rt.to_vol>>8;//总电压
                cf.data[5] = Sys_Rt.to_vol;
                cf.data[6] = (Sys_Rt.current+20000)>>8;//总电流
                cf.data[7] = (Sys_Rt.current+20000);

                Can0Write (&cf);
            break;
           case 204:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = ffc_com_sts;    //火灾状态
                cf.data[1] = 0; //充电机启停指令-450没这个
                cf.data[2] = Bat_Fault_Code[0];
                cf.data[3] = Bat_Fault_Code[1];
                cf.data[4] = Bat_Fault_Code[2];
                cf.data[5] = Bat_Fault_Code[3];
                cf.data[6] = Bat_Fault_Code[4];
                cf.data[7] = Bat_Fault_Code[5];

                Can0Write (&cf);
            break;
            case 205:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

 
                cf.data[0] = Bat_Fault_Code[6]; 
                cf.data[1] = Bat_Time_Tx>>8;   //可用时间预估
                cf.data[2] = Bat_Time_Tx;
                cf.data[3] = 100 - Sys_Rt.soc;  //充电功率
                cf.data[4] = Bat_Power_Tx>>8;   //当前输出功率值高字节
                cf.data[5] = Bat_Power_Tx;
                cf.data[6] = Sys_kwh.F_kw>>24; //放电累计电量
                cf.data[7] = Sys_kwh.F_kw>>16;

                Can0Write (&cf);
            break;
           case 206:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

 
                cf.data[0] = Sys_kwh.F_kw>>8; 
                cf.data[1] = Sys_kwh.F_kw;
                cf.data[2] = Sys_kwh.C_kw>>24; //充电累计电量
                cf.data[3] = Sys_kwh.C_kw>>16;
                cf.data[4] = Sys_kwh.C_kw>>8;
                cf.data[5] = Sys_kwh.C_kw;
                cf.data[6] = Sys_kwh.CF_num>>24; //当日循环次数
                cf.data[7] = Sys_kwh.CF_num>>16;

                Can0Write (&cf);
            break;
            case 207:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

 
                cf.data[0] = Sys_kwh.CF_num>>8; 
                cf.data[1] = Sys_kwh.CF_num;
                cf.data[2] = 131; //补充存储数据个数
                cf.data[3] = CCU_life; //CCU发送心跳
                cf.data[4] = BDC_T_Heart; //CCU接收心跳
                cf.data[5] = 0; //充电机发送心跳
                cf.data[6] = 0; //充电机接收心跳
                cf.data[7] = 0; //BCU2发送心跳

                Can0Write (&cf);
            break;
            case 208:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                IN_now = ((KM15_OS<<5)|(KA12_OS<<4)|(KM10_OS<<3)|(CHARGE_EN_OS<<2)|(KA9_OS<<1)|(KM14_OS));
                OUT_now = BMS_OutSts ();
                tx0_dsu_cell_index = 0;

                cf.data[0] = IN_now; //输入信号
                cf.data[1] = OUT_now;     //输出信号
                cf.data[2] = 324>>8;    // 电压采集数量
                cf.data[3] = 324;   
                cf.data[4] = 324>>8;    // 温度采集数量
                cf.data[5] = 324; 
                cf.data[6] = ((Cell_Volts[tx0_dsu_cell_index]& 0x7fff) >> 8);
                cf.data[7] = ((Cell_Volts[tx0_dsu_cell_index]& 0x7fff));
                Can0Write (&cf);
            break;
            case 289:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                tx0_dsu_cell_index = 321;
                cf.data[0] = ((Cell_Volts[tx0_dsu_cell_index]& 0x7fff) >> 8);
                cf.data[1] = ((Cell_Volts[tx0_dsu_cell_index]& 0x7fff));
                cf.data[2] = ((Cell_Volts[tx0_dsu_cell_index+1]& 0x7fff) >> 8);
                cf.data[3] = ((Cell_Volts[tx0_dsu_cell_index+1]& 0x7fff));   
                cf.data[4] = ((Cell_Volts[tx0_dsu_cell_index+2]& 0x7fff) >> 8);
                cf.data[5] = ((Cell_Volts[tx0_dsu_cell_index+2]& 0x7fff));
                cf.data[6] = 0;
                cf.data[7] = 0;
                Can0Write (&cf);
            break;
            case 304:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                tx0_dsu_temp_index = 0;
                cf.data[0] = 0;
                cf.data[1] = 0;
                cf.data[2] = 0;
                cf.data[3] = 0;   
                cf.data[4] = 0;
                cf.data[5] = 0;
                cf.data[6] = Temp_point[tx0_dsu_temp_index];
                cf.data[7] = Temp_point[tx0_dsu_temp_index+1];
                Can0Write (&cf);
            break;
            case 345:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                tx0_dsu_temp_index = 322;
                cf.data[0] = Temp_point[tx0_dsu_temp_index];
                cf.data[1] = Temp_point[tx0_dsu_temp_index+1];
                cf.data[2] = 0;
                cf.data[3] = 0;
                cf.data[4] = 0;
                cf.data[5] = 0;
                cf.data[6] = 0;
                cf.data[7] = 0;
                Can0Write (&cf);
            break;
            case 352:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0;
                cf.data[1] = 0;
                cf.data[2] = 0;
                cf.data[3] = 0;
                cf.data[4] = 0;
                cf.data[5] = 0;
                cf.data[6] = Bat_Fault_Code[7];
                cf.data[7] = Bat_Fault_Code[8];
                Can0Write (&cf);
            break;

            case 353:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = Bat_Fault_Code[9];
                cf.data[1] = Bat_Fault_Code[10];
                cf.data[2] = Bat_Fault_Code[11];
                cf.data[3] = Bat_Fault_Code[12];
                cf.data[4] = Bat_Fault_Code[13];
                cf.data[5] = Bat_Fault_Code[14];
                cf.data[6] = 0; 
                cf.data[7] = 0; 
                Can0Write (&cf);
            break;

            case 354:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0;
                cf.data[1] = 0;
                cf.data[2] = 0;//充电机电压 没发
                cf.data[3] = 0;
                cf.data[4] = 0;//充电机电流 没发
                cf.data[5] = 0;
                cf.data[6] = 0;//充电机状态 没发
                cf.data[7] = 0; 
                Can0Write (&cf);
            break;

            case 355:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0;//bms发DC模式 没发
                cf.data[1] = 0;
                cf.data[2] = 0;//bms发DC状态 没发
                cf.data[3] = 0;
                cf.data[4] = 0;//bms发DC反馈状态 没发
                cf.data[5] = 0;
                cf.data[6] = 0;//加热模式不再这里处理
                cf.data[7] = 0; 
                Can0Write (&cf);
            break;

            case 356:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = CCU.data[0];//接收CCU数据
                cf.data[1] = CCU.data[1];
                cf.data[2] = CCU.data[2];
                cf.data[3] = CCU.data[3];
                cf.data[4] = CCU.data[4];
                cf.data[5] = CCU.data[5];
                cf.data[6] = O_Voltage>>8;//外总压
                cf.data[7] = O_Voltage; 
                Can0Write (&cf);
            break;
            case 357:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0;
                cf.data[1] = ks_stage;//启动流程信号
                cf.data[2] = 0;//牵引外总压 450没有
                cf.data[3] = 0;
                cf.data[4] = 0;
                cf.data[5] = FFCD.sts;//动力系统消防状态
                cf.data[6] = Q_usable>>8;//最大可用电量
                cf.data[7] = Q_usable;
                Can0Write (&cf);
            break;
            case 358:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0;//BDC软件版本
                cf.data[1] = 0;
                cf.data[2] = 0;//BDC生命信号
                cf.data[3] = 0;
                cf.data[4] = 0;//直流侧电压
                cf.data[5] = 0;
                cf.data[6] = 0;//直流侧电流
                cf.data[7] = 0;
                Can0Write (&cf);
            break;
            case 359:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = CCU.data[6];
                cf.data[1] = CCU.data[7];
                cf.data[2] = CCU.data[8];
                cf.data[3] = CCU.data[9];
                cf.data[4] = CCU.data[10];
                cf.data[5] = CCU.data[11];
                cf.data[6] = 0;
                cf.data[7] = 0;
                Can0Write (&cf);
            break;
            case 360:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0;
                cf.data[1] = 0;
                cf.data[2] = 0;
                cf.data[3] = 0;
                cf.data[4] = test_param.test_wdg_tick;//测试模式信号
                cf.data[5] = 0;//透传模式信号
                cf.data[6] = FFCD.sts; //以下为全部火警相关数据
                cf.data[7] = FFCD.ver;
                Can0Write (&cf);
            break;
            case 361:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = FFCD.bot1_sts;
                cf.data[1] = FFCD.bot2_sts;
                cf.data[2] = FFCD.bot_sv_sta;
                cf.data[3] = FFCD.cb_sv_sta;
                cf.data[4] = FFCD.cb_sv_sta;
                cf.data[5] = FFCD.bms_off;
                cf.data[6] = FFCD.sensor[0];
                cf.data[7] = FFCD.sensor[1];
                Can0Write (&cf);
            break;
            case 362:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = FFCD.sensor[2];
                cf.data[1] = FFCD.sensor[3];
                cf.data[2] = FFCD.temp[0];
                cf.data[3] = FFCD.temp[1];
                cf.data[4] = FFCD.temp[2];
                cf.data[5] = FFCD.temp[3];
                cf.data[6] = FFCD.Smoke[0];
                cf.data[7] = FFCD.Smoke[1];
                Can0Write (&cf);
            break;
            case 363:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = FFCD.Smoke[2];
                cf.data[1] = FFCD.Smoke[3];
                cf.data[2] = FFCD.pollute[0];
                cf.data[3] = FFCD.pollute[1];
                cf.data[4] = FFCD.pollute[2];
                cf.data[5] = FFCD.pollute[3];
                cf.data[6] = FFCD.sensor[4];
                cf.data[7] = FFCD.sensor[5];
                Can0Write (&cf);
            break;
            case 364:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = FFCD.sensor[6];
                cf.data[1] = FFCD.sensor[7];
                cf.data[2] = FFCD.temp[4];
                cf.data[3] = FFCD.temp[5];
                cf.data[4] = FFCD.temp[6];
                cf.data[5] = FFCD.temp[7];
                cf.data[6] = FFCD.Smoke[4];
                cf.data[7] = FFCD.Smoke[5];
                Can0Write (&cf);
            break;
            case 365:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = FFCD.Smoke[6];
                cf.data[1] = FFCD.Smoke[7];
                cf.data[2] = FFCD.pollute[4];
                cf.data[3] = FFCD.pollute[5];
                cf.data[4] = FFCD.pollute[6];
                cf.data[5] = FFCD.pollute[7];
                cf.data[6] = FFCD.sensor[8];
                cf.data[7] = FFCD.sensor[9];
                Can0Write (&cf);
            break;
            case 366:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = FFCD.temp[8];
                cf.data[1] = FFCD.temp[9];
                cf.data[2] = FFCD.Smoke[8];
                cf.data[3] = FFCD.Smoke[9];
                cf.data[4] = FFCD.pollute[8];
                cf.data[5] = FFCD.pollute[9];
                cf.data[6] = 0;
                cf.data[7] = FFCD.T_life;
                Can0Write (&cf);
            break;
            case 367:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = FFCD.max_t1[0];
                cf.data[1] = FFCD.max_t2[0];
                cf.data[2] = FFCD.num_t1[0];
                cf.data[3] = FFCD.num_t2[0];
                cf.data[4] = FFCD.max_t1[1];
                cf.data[5] = FFCD.max_t2[1];
                cf.data[6] = FFCD.num_t1[1];
                cf.data[7] = FFCD.num_t2[1];
                Can0Write (&cf);
            break;
            case 368:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = FFCD.max_t1[2];
                cf.data[1] = FFCD.max_t2[2];
                cf.data[2] = FFCD.num_t1[2];
                cf.data[3] = FFCD.num_t2[2];
                cf.data[4] = FFCD.max_t1[3];
                cf.data[5] = FFCD.max_t2[3];
                cf.data[6] = FFCD.num_t1[3];
                cf.data[7] = FFCD.num_t2[3];
                Can0Write (&cf);
            break;
            case 369:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = (LECU_Rt[0].out_stauts)&0x01; //BMU1故障状态
                cf.data[1] = LECU_Rt[0].soh; //BMU1心跳
                cf.data[2] = (LECU_Rt[1].out_stauts)&0x01; //BMU2故障状态
                cf.data[3] = LECU_Rt[1].soh; //BMU2心跳
                cf.data[4] = (LECU_Rt[2].out_stauts)&0x01; //BMU3故障状态
                cf.data[5] = LECU_Rt[2].soh; //BMU3心跳
                cf.data[6] = (LECU_Rt[3].out_stauts)&0x01; //BMU4故障状态
                cf.data[7] = LECU_Rt[3].soh; //BMU4心跳
                Can0Write (&cf);
            break;
            case 370:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = (LECU_Rt[4].out_stauts)&0x01; //BMU5故障状态
                cf.data[1] = LECU_Rt[4].soh; //BMU5心跳
                cf.data[2] = (LECU_Rt[5].out_stauts)&0x01; //BMU6故障状态
                cf.data[3] = LECU_Rt[5].soh; //BMU6心跳
                cf.data[4] = (LECU_Rt[6].out_stauts)&0x01; //BMU7故障状态
                cf.data[5] = LECU_Rt[6].soh; //BMU7心跳
                cf.data[6] = (LECU_Rt[7].out_stauts)&0x01; //BMU8故障状态
                cf.data[7] = LECU_Rt[7].soh; //BMU8心跳
                Can0Write (&cf);
            break;
            case 371:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = (LECU_Rt[8].out_stauts)&0x01; //BMU9故障状态
                cf.data[1] = LECU_Rt[8].soh; //BMU9心跳
                cf.data[2] = 0;//10
                cf.data[3] = 0;
                cf.data[4] = 0;//11
                cf.data[5] = 0;
                cf.data[6] = 0;//12
                cf.data[7] = 0;
                Can0Write (&cf);
            break;
            case 372:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0; //Bpu1
                cf.data[1] = 0; 
                cf.data[2] = 0; //2
                cf.data[3] = 0;
                cf.data[4] = 0; //3
                cf.data[5] = 0;
                cf.data[6] = 0; //4
                cf.data[7] = 0;
                Can0Write (&cf);
            break;
            case 373:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0; //5
                cf.data[1] = 0; 
                cf.data[2] = 0; //6
                cf.data[3] = 0;
                cf.data[4] = 0; //7
                cf.data[5] = 0;
                cf.data[6] = 0; //8
                cf.data[7] = 0;
                Can0Write (&cf);
            break;
            case 374:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0; //9
                cf.data[1] = 0; 
                cf.data[2] = 0; //10
                cf.data[3] = 0;
                cf.data[4] = 0; //11
                cf.data[5] = 0;
                cf.data[6] = 0; //12
                cf.data[7] = 0;
                Can0Write (&cf);
            break;
            case 375:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0; //13
                cf.data[1] = 0; 
                cf.data[2] = 0; //14
                cf.data[3] = 0;
                cf.data[4] = 0; //15
                cf.data[5] = 0;
                cf.data[6] = 0; //16
                cf.data[7] = 0;
                Can0Write (&cf);
            break;
            case 376:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0; //1启动均衡
                cf.data[1] = 0; //2
                cf.data[2] = 0; //3
                cf.data[3] = 0; //4
                cf.data[4] = 0; //5
                cf.data[5] = 0; //6
                cf.data[6] = 0; //7
                cf.data[7] = 0; //8
                Can0Write (&cf);
            break;
            case 377:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0; //9
                cf.data[1] = 0; //10
                cf.data[2] = 0; //11
                cf.data[3] = 0; //12
                cf.data[4] = 0; //13
                cf.data[5] = 0; //14
                cf.data[6] = 0; //15
                cf.data[7] = 0; //16
                Can0Write (&cf);
            break;
            case 378:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0; 
                cf.data[1] = cha_stage; //充电流程信号
                cf.data[2] = 0; 
                cf.data[3] = Kin_BX[0]; //BCU2输入信号
                cf.data[4] = 0; 
                cf.data[5] = 0; //BCU2输出信号
                cf.data[6] = IWD_Res>>8; //绝缘电阻
                cf.data[7] = IWD_Res; 
                Can0Write (&cf);
            break;
            case 379:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = Temp_special[0]; //特殊温度点1
                cf.data[1] = Temp_special[1]; //特殊温度点2
                cf.data[2] = Temp_special[2]; //特殊温度点3
                cf.data[3] = Temp_special[3]; //特殊温度点4
                cf.data[4] = Temp_special[4]; //特殊温度点5
                cf.data[5] = Temp_special[5]; //特殊温度点6
                cf.data[6] = Temp_special[6]; //特殊温度点7
                cf.data[7] = Temp_special[7]; //特殊温度点8
                Can0Write (&cf);
            break;
            case 380:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = Temp_special[8]; //特殊温度点9
                cf.data[1] = 0;
                cf.data[2] = 0;
                cf.data[3] = 0;
                cf.data[4] = 0;
                cf.data[5] = 0;
                cf.data[6] = 0;
                cf.data[7] = 0;
                Can0Write (&cf);
            break;
            case 381:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = ks_stage;//上电自检阶段
                cf.data[1] = cha_stage; //充电阶段
                cf.data[2] = pre_cha_stage; //预充阶段
                cf.data[3] = hot_stage; //加热阶段
                cf.data[4] = check_bms;//自检状态
                cf.data[5] = pre_status;//预充状态
                cf.data[6] = TR_Work_mode;//工作模式
                cf.data[7] = Contact_Open;//严重故障接触器断开标志
                Can0Write (&cf);
            break;
            case 382:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = 0; //库停标志
                cf.data[1] = Bat_Sys_sts; //电池状态
                cf.data[2] = 0; //库停退出标志
                cf.data[3] = temp_ccu_data->panto_stat;
                cf.data[4] = temp_ccu_data->cutoff;
                cf.data[5] = temp_ccu_data->comm_ccu;
                cf.data[6] = temp_ccu_data->ecab_smoke;
                cf.data[7] = temp_ccu_data->tcnv_comm;
                Can0Write (&cf);
            break;
            case 383:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = temp_tcu_data.tcnv_cool;
                cf.data[1] = temp_tcu_data.heat_en;
                cf.data[2] = temp_tcu_data.lspanto;
                cf.data[3] = temp_tcu_data.mc_en;
                cf.data[4] = temp_tcu_data.chg_dis;
                cf.data[5] = temp_tcu_data.chg_work;
                cf.data[6] = temp_tcu_data.cont_en;
                cf.data[7] = temp_tcu_data.chg_out;
                Can0Write (&cf);
            break;
            case 384:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = temp_tcu_data.chg_fault;
                cf.data[1] = temp_tcu_data.req_mc_cls;
                cf.data[2] = temp_tcu_data.req_mc_opn; 
                cf.data[3] = temp_tcu_data.chg_act;
                cf.data[4] = ((KA11_OS<<3)|(KA13_OS<<2)|(KM7_KM8_OS<<1)|(FU3_FU4_OS));//BCU2输入状态
                cf.data[5] = Sys_Rt.num_lecu;//从控存活数量
                cf.data[6] = LECU_Rt[0].in_stauts; //主熔断器1状态
                cf.data[7] = LECU_Rt[8].in_stauts; //主熔断器2状态
                Can0Write (&cf);
            break;
            case 385:
                cf.id         = 0x12301-200+CAN0_add;
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0] = self_err_code;//通信故障状态
                cf.data[1] = reg_err1_code;//消防系统错误码
                cf.data[2] = SOC_S_flag; //SOC校准标志
                cf.data[3] = 0;
                cf.data[4] = 0;
                cf.data[5] = 0;
                cf.data[6] = 0;
                cf.data[7] = 0; 
                Can0Write (&cf);
            break;

            case 150:
                cf.id         = 0x12401;
                cf.pri        = 1;
                cf.len        = 8;
                if(CCU_Time_en == 1) {
                    cf.data[0] = CCU_T.tm_year;
                    cf.data[1] = CCU_T.tm_mon;
                    cf.data[2] = CCU_T.tm_mday;
                    cf.data[3] = CCU_T.tm_hour;
                    cf.data[4] = CCU_T.tm_min;
                    cf.data[5] = CCU_T.tm_sec;
                }else{
                    cf.data[0] = 0xff;
                    cf.data[1] = 0xff;
                    cf.data[2] = 0xff;
                    cf.data[3] = 0xff;
                    cf.data[4] = 0xff;
                    cf.data[5] = 0xff;
                }
                cf.data[6] = VH;
                cf.data[7] = VL;
                Can0Write (&cf);
            break;

            case 151:
                cf.id         = 0x12402;
                cf.pri        = 1;
                cf.len        = 8;
                
                cf.data[0]    = (VER_TIME_Y / 10) << 4 | (VER_TIME_Y % 10);
                cf.data[1]    = (VER_TIME_M / 10) << 4 | (VER_TIME_M % 10);
                cf.data[2]    = (VER_TIME_D / 10) << 4 | (VER_TIME_D % 10);
                cf.data[3]    = VER_YG;
                cf.data[4]    = 0;
                cf.data[5]    = LECU_Rt[0].in_stauts>>16;                    
                cf.data[6]    = LECU_Rt[0].in_stauts>>8;                 
                cf.data[7]    = LECU_Rt[0].in_stauts;   
                Can0Write (&cf);
            break;

            default:
                if (CAN0_add >= 5 && CAN0_add <= 85)
                {
                    tx0_cell_index = (CAN0_add - 5) * 4;

                    cf.id  = 0x0B00 + CAN0_add;
                    cf.pri = 1;
                    cf.len = 8;

                    for (i = 0; i < 4; i++)
                    {
                        cf.data[i * 2]     = ((Cell_Volts[tx0_cell_index + i]& 0x7fff) >> 8);
                        cf.data[i * 2 + 1] = (Cell_Volts[tx0_cell_index + i]& 0x7fff);
                    }

                    cf1 = cf;
                    Can0Write(&cf);

                    cf1.id = 0x1B00 + CAN0_add;
                    Can0Write(&cf1);
                }
                else if (CAN0_add >= 86 && CAN0_add <= 125)
                {
                    tx0_temp_index = (CAN0_add - 86) * 8;

                    cf.id  = 0x0B00 + CAN0_add;
                    cf.pri = 1;
                    cf.len = 8;

                    for (i = 0; i < 8; i++){
                        cf.data[i] = Temp_point[tx0_temp_index + i];
                    }
                    cf1 = cf;
                    Can0Write(&cf);

                    cf1.id = 0x1B00 + CAN0_add;
                    Can0Write(&cf1);
                }    
                
                if(CAN0_add>= 209 && CAN0_add<=288){
                    tx0_dsu_cell_index = 1 + (CAN0_add - 209) * 4;

                    cf.id  = 0x12301 - 200 + CAN0_add;
                    cf.pri = 1;
                    cf.len = 8;

                    for (i = 0; i < 4; i++)
                    {
                        cf.data[i * 2]     = ((Cell_Volts[tx0_dsu_cell_index + i]& 0x7fff) >> 8);
                        cf.data[i * 2 + 1] = (Cell_Volts[tx0_dsu_cell_index + i]& 0x7fff);
                    }

                    Can0Write(&cf);
                }
                else if (CAN0_add >= 305 && CAN0_add <= 344)
                {
                    tx0_dsu_temp_index = 2 + (CAN0_add - 305) * 8;

                    cf.id  = 0x12301 - 200 + CAN0_add;
                    cf.pri = 1;
                    cf.len = 8;

                    for (i = 0; i < 8; i++){
                        cf.data[i] = Temp_point[tx0_dsu_temp_index + i];
                    }
                    Can0Write(&cf);
                } 
            break;
        }
        CAN0_add++;
        if(CAN0_add>=450) {
            CAN0_add=0;
           
        }
        TIMER_RESET_T(TIMER_CAN0);   
    } 
}
/*****************************************************************************/
/* CAN1发送                                                                  */
/*****************************************************************************/
static void Tx1(void){
    CanbusFrame cf;
    UINT8 temy_data =0;
    UINT8 sum = 0;
    UINT8 i = 0; 
    UINT32 temp_can_id = 0;
    UINT8  cabin_base  = 0; 
    
    if(EXPIRED_T(TIMER_CAN1,20)){
        switch(CAN1_add) {   
            //绝缘监测
            case 2: {
                cf.id         = 0x1CFF0001;                         
                cf.pri        = 1;
                cf.len        = 8;

                cf.data[0]    = 0x46;  
                cf.data[1]    = 0x54;                  
                cf.data[2]    = 0x42;
                cf.data[3]    = 0x4b;
                cf.data[4]    = IWD_cmd;   //0：关闭绝缘监测功能 ,255: 打开绝缘监测功能，持续监测
                cf.data[5]    = 0;                   
                cf.data[6]    = 0;                   
                cf.data[7]    = 0;                   
                CAN1_write (&cf);             
            }break;
            default:
                break;
        }
        CAN1_add++;
        if(CAN1_add>=50) {
            CAN1_add=0;
            
        }
        TIMER_RESET_T(TIMER_CAN1);
    }
    
}
/*****************************************************************************/
/* CAN4发送                                                                  */
/*****************************************************************************/
static void Tx4(void){
    CanbusFrame cf;
    SINT32 P_now        = 0;
    UINT8 temp_bms_addr = 0;
      if (EXPIRED_T (TIMER_CAN4, 5)){
	   switch(CAN4_add){
         case 1:{
               
                Can4Write (&cf);
           }break;
           case 2:{
               
                Can4Write (&cf);
            }break;
            case 3:{
            
                Can4Write (&cf);
            }break;
            case 4:{
                
                Can4Write (&cf);
            }break;
            case 5:{
               
                Can4Write (&cf);
            }break;
         default:break;
	  }
	  CAN4_add++;
	  if(CAN4_add>=20){
	      CAN4_add=0;
	  }
           
      TIMER_RESET_T(TIMER_CAN4);  
    }
}
// }
/*****************************************************************************/
/* 对时                                                                      */
/*****************************************************************************/
void set_time(void) {
    UINT32 temy_time;
    UINT32 a,b;         //对时标志
    UINT32 temy_time1;
    
//    temy_time= mktime (&CCU_T);
    a = (temy_time > gSysSec) ? (temy_time-gSysSec):(gSysSec-temy_time);  
    if( a >= 2){
        gSysSec = temy_time;
    }  

//    RTC_write (&CCU_T);
}


/* 充电投入接口                                                          */
/*****************************************************************************/
/**
 * @brief   充电投入接口
 * @return  无
 * @note    检测充电条件是否满足，满足则投入充电
 */
static void CHAProc (void)
{ 
    if(((Bat_Fault_Code[13] & CHG_PULSE_LOCK_REQ)== CHG_PULSE_LOCK_REQ)||
        (pre_status!=1)){
        charge_voltage = 0;
        charge_current = 0;
    }
    if(pre_status == 1){
        //满足条件已完成预充
        //出现严重故障（断开接触器）
        if(Contact_Open == 1){
            //充电流程归0退出
            TIMER_RESET_T(TIMER_CHA_DELAY);
            cha_stage = 0;
            return;
        }

        if((TBMS_Traction == 0)&& //不处于应急模式
            (TBMS_Standby == 0)&&
            (EmemStandby == 0)&&
            (temp_tcu_data.lspanto == 0)&&//不处于过分相模式
            //(temp_ccu_data->panto_stat == 1)&&//处于升弓状态
            ((Bat_Fault_Code[14] & HEAT_MODE_STATUS) == 0)&&//不处于加热模式
            (KM7_KM8_OS == OPEN)&&
            (((Sys_Rt.max_v <= 2530) && (cha_stage==0))|| (cha_stage!=0))//首次进充电模式最大电压小于2530mv
        )
        {
            
        }
        else{
            //充电流程归0退出
            TIMER_RESET_T(TIMER_CHA_DELAY);
            Bat_Fault_Code[13] |= CHG_PULSE_LOCK_REQ;//将充电脉冲锁位置1
            cha_stage = 0;
            return;
        }

        switch(cha_stage){
            case 0:{
                //开始充电
                Bat_Fault_Code[13] &= ~CHG_PULSE_LOCK_REQ;//将充电脉冲锁位清0
                cha_stage = 1;
                break;
            }
            case 1:{
                //1段充电  
                charge_voltage = 8260;
                charge_current = 350;
                cha_stage = 2;
                TIMER_RESET_T (TIMER_CHA_DELAY);
                break;
            }
            case 2:{
#if PTL_PHY_TEST
            //if(1){
            if((Sys_Rt.max_v >= 2540) || (Sys_Rt.to_vol >= 8230)){
#endif
#if !PTL_PHY_TEST
                if((Sys_Rt.max_v >= 2540) || (Sys_Rt.to_vol >= 8230)){
#endif
                    if (EXPIRED_T (TIMER_CHA_DELAY, 5*1000UL)){
                        TIMER_RESET_T (TIMER_CHA_DELAY);
                        cha_stage = 3;
                    }
                }
                else{
                    TIMER_RESET_T (TIMER_CHA_DELAY);
                }
                break;
            }
            case 3:{
                //2段充电  
                charge_voltage = 8260;
                charge_current = 180;
                cha_stage = 4;
                TIMER_RESET_T (TIMER_CHA_DELAY);
                break;
            }
            case 4:{
#if PTL_PHY_TEST
            //if(1){
            if((Sys_Rt.max_v >= 2540) || (Sys_Rt.to_vol >= 8230)){
#endif
#if !PTL_PHY_TEST
                if((Sys_Rt.max_v >= 2540) || (Sys_Rt.to_vol >= 8230)){
#endif
                    if (EXPIRED_T (TIMER_CHA_DELAY, 5*1000UL)){
                        TIMER_RESET_T (TIMER_CHA_DELAY);
                        cha_stage = 5;
                    }
                }
                else{
                    TIMER_RESET_T (TIMER_CHA_DELAY);
                }
                break;
            }
            case 5:{
                //3段充电
                charge_voltage = 8260;
                charge_current = 90;
                cha_stage = 6;
                TIMER_RESET_T (TIMER_CHA_DELAY);
                break;
            }
            case 6:{
#if PTL_PHY_TEST
            //if(1){
            if((Sys_Rt.max_v >= 2540) || (Sys_Rt.to_vol >= 8230)){
#endif
#if !PTL_PHY_TEST
                if((Sys_Rt.max_v >= 2540) || (Sys_Rt.to_vol >= 8230)){
#endif
                    if (EXPIRED_T (TIMER_CHA_DELAY, 5*1000UL)){
                        TIMER_RESET_T (TIMER_CHA_DELAY);
                        cha_stage = 7;
                    }
                }
                else{
                    TIMER_RESET_T (TIMER_CHA_DELAY);
                }
                break;
            }
            case 7:{
                //4段充电
                charge_voltage = 8260;
                charge_current = 50;
                cha_stage = 8;
                TIMER_RESET_T (TIMER_CHA_DELAY);
                break;
            }
            case 8:{
#if PTL_PHY_TEST
            //if(1){
            if((Sys_Rt.max_v >= 2550) || (Sys_Rt.to_vol >= 8230)){
                if (EXPIRED_T (TIMER_CHA_DELAY, 10000UL)){
#endif
#if !PTL_PHY_TEST
                if((Sys_Rt.max_v >= 2550) || (Sys_Rt.to_vol >= 8230)){
                    if (EXPIRED_T (TIMER_CHA_DELAY, 300000UL)){
#endif
                        Bat_Fault_Code[13] |= CHG_PULSE_LOCK_REQ;//将充电脉冲锁位设置为1
                        TIMER_RESET_T (TIMER_CHA_DELAY);
                        cha_stage = 9;
                    }
                }
                else{
                    TIMER_RESET_T (TIMER_CHA_DELAY);
                }
                break;
            }
            case 9:{
                //检测脉冲封锁是否正常 已转移到CHECK_TCU
                SOC_k = 1000;
                cha_stage = 0;
                TIMER_RESET_T (TIMER_CHA_DELAY);
                break;
            }
            default:{
                cha_stage = 0;
                TIMER_RESET_T (TIMER_CHA_DELAY);
                break;
            }
        }
        
    }
    else if(pre_status == 2){
        //正在预充
        cha_stage = 0;
        TIMER_RESET_T (TIMER_CHA_DELAY);
    }
    else if(pre_status == 0){
        //不满足条件不投入预充
        cha_stage = 0;
        TIMER_RESET_T (TIMER_CHA_DELAY);
    }
}

/*****************************************************************************/
/* 绝缘电阻状态监测                                                            */
/*****************************************************************************/
BOOL Check_IWD(void){
   // return(P_Res<(Sys_Rt.to_vol/100));
    return(IWD_Res < (Sys_Rt.to_vol)/100); //KΩ
}

/*****************************************************************************/
/* BMU故障检测                                                         */
/*****************************************************************************/
void Check_BMU(void){
/////////////////////////////////////BMS故障/////////////////////////////////////
    if ((Sys_Rt.num_lecu != DEV_MAX))
    {
        if (EXPIRED_T(TIMER_BMU, 2500))
        {
            BMU_err_num++;
            TIMER_RESET_T(TIMER_BMU);
        }
    }
    else
    {
        BMU_err_num = 0;
        TIMER_RESET_T(TIMER_BMU);
    }
    if ((BMU_err_num >= 5) || (test_param.bms_err != 0))
    {
        Bat_Fault_Code[3] |= BMS_ERR;
        Bat_Fault_Code[3] |= BMS_COM_ERR;
    }
}

/*****************************************************************************/
/* TCU故障检测                                                         */
/*****************************************************************************/
void Check_TCU(void){
/////////////////////////////////////TCU故障/////////////////////////////////////
    // if((Bat_Fault_Code[13] & CHG_PULSE_LOCK_REQ) == CHG_PULSE_LOCK_REQ){
    //     if(Sys_Rt.current > 50){
    //         if(tcu_cur_over_flag == 0){
    //             tcu_cur_over_flag = 1;
    //             TIMER_RESET_T(TIMER_TCU_ERR);   // 上升沿，开始计时
    //         }
    //         else if(EXPIRED_T(TIMER_TCU_ERR, 5000UL)){
    //             Contact_Open = 1;
    //             TIMER_RESET_T(TIMER_TCU_ERR);
    //             tcu_cur_over_flag = 0;
    //         }
    //     }
    //     else{
    //         tcu_cur_over_flag = 0;
    //         TIMER_RESET_T(TIMER_TCU_ERR);
    //     }
    // }
    // else{
    //     tcu_cur_over_flag = 0;
    //     TIMER_RESET_T(TIMER_TCU_ERR);
    // }
    // //网络与TCU通信异常 先不处理这个
    // if((temp_ccu_data->tcnv_comm) == 1){
    //     Contact_Open = 1;
    // }
}



/*****************************************************************************/
/* 接触器状态监测                                                            */
/*****************************************************************************/
void Check_Contact(void){
    //KM15
    if(((KA15==CLOSE)&&(KM15_OS==CLOSE))||((KA15==OPEN)&&(KM15_OS==OPEN))){    
        if((KA15==CLOSE)&&(KM15_OS==CLOSE)){
            if(KA15_cerr_time < 2){
                Bat_Fault_Code[4] &= ~KM15_CERR;
            }
        }            
        TIMER_RESET_T (TIMER_OKM15);
        TIMER_RESET_T (TIMER_CKM15);          
    }else{
        //无法闭合
        if(((KA15==CLOSE)&&(KM15_OS==OPEN))){
            if(EXPIRED_T(TIMER_CKM15,5000)){ 
                if(KA15_cerr_time < 2){
                    KA15_cerr_time++;
                    KA14 = OPEN;
                    KA15 = OPEN;
                }
                Bat_Fault_Code[4] |= KM15_CERR;
                TIMER_RESET_T (TIMER_CKM15);
                
                // 第1次故障：启动10分钟窗口
                if(KA15_cerr_time == 1){
                    TIMER_RESET_T(TIMER_CKM15_ERR);
                }
                
                // 第2次故障：立即锁死，并标记为已锁死
                if(KA15_cerr_time >= 2){
                    Contact_Open = 1;
                    KA15_cerr_time = 3;   // 防止重复触发
                }
            }
        }
        //无法断开
        else if(((KA15==OPEN)&&(KM15_OS==CLOSE))){
            if(EXPIRED_T(TIMER_OKM15,5000)){ 
                Contact_Open = 1;
                Bat_Fault_Code[8] |= KM15_OERR;
                TIMER_RESET_T (TIMER_OKM15);
            }
        }        
    }

    // 10分钟窗口到期
    if(EXPIRED_T(TIMER_CKM15_ERR,600000)){ 
        if(KA15_cerr_time < 2){
            KA15_cerr_time = 0;    // 没到2次，清零重新计数
        }
        // 计数=3表示已锁死，不再处理
        TIMER_RESET_T (TIMER_CKM15_ERR);
    }

    //KM14
    if(((KA14==CLOSE)&&(KM14_OS==CLOSE))||((KA14==OPEN)&&(KM14_OS==OPEN))){ 
        if((KA14==CLOSE)&&(KM14_OS==CLOSE)){
            if(KA14_cerr_time < 2){
                Bat_Fault_Code[4] &= ~KM14_CERR;
            }
        }                
        TIMER_RESET_T (TIMER_OKM14);
        TIMER_RESET_T (TIMER_CKM14);          
    }else{
        //无法闭合 二级故障直接断开所有接触器
        if(((KA14==CLOSE)&&(KM14_OS==OPEN))){
            if(EXPIRED_T(TIMER_CKM14,5000)){ 
                if(KA14_cerr_time < 2){
                    KA14_cerr_time++;
                    KA14 = OPEN;
                    KA15 = OPEN;
                }
                Bat_Fault_Code[4] |= KM14_CERR;
                TIMER_RESET_T (TIMER_CKM14);
                
                // 第1次故障：启动10分钟窗口
                if(KA14_cerr_time == 1){
                    TIMER_RESET_T(TIMER_CKM14_ERR);
                }
                
                // 第2次故障：立即锁死，并标记为已锁死
                if(KA14_cerr_time >= 2){
                    Contact_Open = 1;
                    KA14_cerr_time = 3;   // 防止重复触发
                }
            }
        }
        //无法断开 二级故障直接断开所有接触器
        else if(((KA14==OPEN)&&(KM14_OS==CLOSE))){
             if(EXPIRED_T(TIMER_OKM14,5000)){ 
                Contact_Open = 1;
                Bat_Fault_Code[8] |= KM14_OERR;
                TIMER_RESET_T (TIMER_OKM14);
            }
        }        
    }
    // 10分钟窗口到期
    if(EXPIRED_T(TIMER_CKM14_ERR,600000)){ 
        if(KA14_cerr_time < 2){
            KA14_cerr_time = 0;    // 没到2次，清零重新计数
        }
        // 计数=3表示已锁死，不再处理
        TIMER_RESET_T (TIMER_CKM14_ERR);
    }

    //KM10
    if(((KA10==CLOSE)&&(KM10_OS==CLOSE))||((KA10==OPEN)&&(KM10_OS==OPEN))){     
        if((KA10==CLOSE)&&(KM10_OS==CLOSE)){
            if(KA10_cerr_time < 2){
                Bat_Fault_Code[5] &= ~KM10_CERR;
            }
        }            
        TIMER_RESET_T (TIMER_OKM10);
        TIMER_RESET_T (TIMER_CKM10);          
    }else{
        //无法闭合 二级故障直接断开所有接触器
        if(((KA10==CLOSE)&&(KM10_OS==OPEN))){
            if(EXPIRED_T(TIMER_CKM10,5000)){ 
                if(KA10_cerr_time < 2){
                    KA10_cerr_time++;
                    KA14 = OPEN;
                    KA15 = OPEN;
                }
                Bat_Fault_Code[5] |= KM10_CERR;
                TIMER_RESET_T (TIMER_CKM10);
                
                // 第1次故障：启动10分钟窗口
                if(KA10_cerr_time == 1){
                    TIMER_RESET_T(TIMER_CKM10_ERR);
                }
                
                // 第2次故障：立即锁死，并标记为已锁死
                if(KA10_cerr_time >= 2){
                    Contact_Open = 1;
                    KA10_cerr_time = 3;   // 防止重复触发
                }
            }
        }
        //无法断开 二级故障直接断开所有接触器
        else if(((KA10==OPEN)&&(KM10_OS==CLOSE))){
            if(EXPIRED_T(TIMER_OKM10,5000)){ 
                Contact_Open = 1;
                Bat_Fault_Code[8] |= KM10_OERR;
                TIMER_RESET_T (TIMER_OKM10);
            }
        }        
    }
    // 10分钟窗口到期
    if(EXPIRED_T(TIMER_CKM10_ERR,600000)){ 
        if(KA10_cerr_time < 2){
            KA10_cerr_time = 0;    // 没到2次，清零重新计数
        }
        // 计数=3表示已锁死，不再处理
        TIMER_RESET_T (TIMER_CKM10_ERR);
    }

    //KM7_KM8 加热接触器（同一回路，一起判断）
    if((((KA7==CLOSE)&&(KA8==CLOSE))&&(KM7_KM8_OS==CLOSE))||
    (((KA7==OPEN)&&(KA8==OPEN))&&(KM7_KM8_OS==OPEN))){                
        TIMER_RESET_T (TIMER_OKM7_KM8);
        TIMER_RESET_T (TIMER_CKM7_KM8);          
    }else{
        //无法闭合 三级故障上报就行
        if(KM7_KM8_OS==OPEN){
            if(EXPIRED_T(TIMER_CKM7_KM8,5000)){ 
                Bat_Fault_Code[3] |= KM7_KM8_CERR;
                TIMER_RESET_T (TIMER_CKM7_KM8);
            }
        }
        //无法断开 二级故障直接断开所有接触器
        else if(KM7_KM8_OS==CLOSE){
            if(EXPIRED_T(TIMER_OKM7_KM8,5000)){ 
                Contact_Open = 1;
                Bat_Fault_Code[4] |= KM7_KM8_OERR;
                TIMER_RESET_T (TIMER_OKM7_KM8);
            }
        }
    }   

    if(KM15_OS ==CLOSE){
        Bat_Fault_Code[13] |= KM15_STATUS;  
    }else{
        Bat_Fault_Code[13] &=~KM15_STATUS;
    }

    if(KM14_OS ==CLOSE){
        Bat_Fault_Code[13] |= KM14_STATUS;  
    }else{
        Bat_Fault_Code[13] &=~KM14_STATUS;
    }

    if(KM10_OS ==CLOSE){
        Bat_Fault_Code[13] |= KM10_STATUS;  
    }else{
        Bat_Fault_Code[13] &=~KM10_STATUS;
    }

    if(KM7_KM8_OS ==CLOSE){
        Bat_Fault_Code[13] |= KM7_STATUS;  
        Bat_Fault_Code[13] |= KM8_STATUS;
    }else{
        Bat_Fault_Code[13] &=~KM7_STATUS;
        Bat_Fault_Code[13] &=~KM8_STATUS;
    }
}   

/***************************************/
/* 检测CCU                     */
/***************************************/
static void Check_CCU(void) {
    if(CCU_life!=CCU_loc_heart){
        CCU_loc_heart =CCU_life;
        CAN0R_flag      &= ~0x01; 

        ccu_ok_count++;
        CCU_com_num      = 0;
 
        // 连续10次也就是10s正常清除故障
        if (ccu_ok_count >= 10) {
            self_err_code &= ~CCU_NET_ERR;
            ccu_ok_count = 10;//限制一下防止一直加溢出
        }
        TIMER_RESET_T (TIMER_CCU_COM); 
    }else{
        //没有心跳，重置计数
        ccu_ok_count = 0;
        CCU_Time_en = 0;
        if(EXPIRED_T(TIMER_CCU_COM,1000)){
            CCU_com_num++;
            TIMER_RESET_T (TIMER_CCU_COM); 
        }
    }
    if(CCU_com_num>=10) {
        self_err_code |= CCU_NET_ERR;
        CCU_com_num = 10;//限制一下防止一直加溢出
    }

    if(Bat_Off == 1){

    }
}
/*****************************************************************************/
/* 灭火检测                                                            */
/*****************************************************************************/
void Check_FFC(void){
    UINT8 i,k;
    UINT16 j;
    UINT8 max1 = 0, max2 = 0;
    UINT8 num1 = 0, num2 = 0;
    UINT16 start = 0;   // 每舱 162 个点
    UINT16 end = 0;

    UINT8 Cabin_temp_max = 0;
    UINT8 Cabin_temp_min = (125+40);
    UINT8 Cabin_temp_diff = 0;

    /* 按 addr 区分：动力1管探头1~4，动力2管探头5~8 */
    UINT8 sen_start = 0;
    UINT8 sen_cnt   = 0;

    UINT8 cur_t   = 0;     // 温升用
    UINT8 last_t  = 0;     // 温升用
    UINT8 t       = 0;     // 舱温统计用
    UINT8 sen_err = 0;     // 探头故障用
    UINT8 phys    = 0;     // 探头故障用
    UINT8 bit     = 0;     // 探头故障用

    if (OEM_Para.bms_addr == 3) {          // 动力BMS1
        sen_start = 0;
        sen_cnt   = 4;
    } else if (OEM_Para.bms_addr == 4) {   // 动力BMS2
        sen_start = 4;
        sen_cnt   = 4;
    } else {
        sen_start = 0;
        sen_cnt   = 0;
    }

    /* ---------- 1. 舱内温度统计：每动力只统计自己管的 2 个舱，每舱 162 点 ---------- */
    if (EXPIRED_T(TIMER_FFC_RATE, 1000)) {
        TIMER_RESET_T(TIMER_FFC_RATE);
        for(i = 0; i < 2; i++) {
            max1 = 0; 
            max2 = 0;
            num1 = 0; 
            num2 = 0;
            start = i * 162;
            end = start + 162;
            // 找最高温度
            for(j = start; j < end; j++) {
                if(Temp_point[j] < (110 + 55)) {
                    if(max1 < Temp_point[j]) {
                        max1 = Temp_point[j];
                        num1 = j;
                    }
                }
            }
            // 找次高温度（排除最高温度位置）
            for(j = start; j < end; j++) {
                if(Temp_point[j] < (110 + 55)) {
                    if((j != num1) && (max2 < Temp_point[j])) {
                        max2 = Temp_point[j];
                        num2 = j;
                    }
                }
            }
            FFCD.max_t1[i] = max1;
            FFCD.num_t1[i] = num1 + 1;
            FFCD.max_t2[i] = max2;
            FFCD.num_t2[i] = num2 + 1;
        }
    }
    /* ---------- 2. 温升速率判断：1s 基准，≥1℃/s 连续 3s → bit4=1，否则立即清 0 ---------- */
    if (EXPIRED_T(TIMER_RATE_BASE, 1000)) {
        TIMER_RESET_T(TIMER_RATE_BASE);
        for (i = 0; i < 2; i++) {
            cur_t  = FFCD.max_t1[i];
            last_t = FFCD.last_max_t[i];

            // 0.5℃/bit，1℃/s 对应差值 >= 2
            if (cur_t > last_t && (cur_t - last_t) >= 2) {
                if (FFCD.rate_cnt[i] < 3) {
                    FFCD.rate_cnt[i]++;
                }
            } else {
                FFCD.rate_cnt[i] = 0;
            }
            FFCD.last_max_t[i] = cur_t;

            if (FFCD.rate_cnt[i] >= 3) {
                FFCD.rate_flag[i] |= 0x10;    // bit4=1
            } else {
                FFCD.rate_flag[i] &= ~0x10;   // 立即清 0
            }
        }
    }

    /* ---------- 3. 舱温高/温差：只统计本动力管的 4 个探头 ---------- */
    Cabin_temp_max = 0;
    Cabin_temp_min = (125 + 40);

    for(k = 0; k < sen_cnt; k++)
    {
        t = FFCD.temp[sen_start + k];
        if(t > (125 + 40)) continue; 
        if(t > Cabin_temp_max)
            Cabin_temp_max = t;
        if(t < Cabin_temp_min)
            Cabin_temp_min = t;
    }

    if (sen_cnt == 0) {
        Cabin_temp_max = 0;
        Cabin_temp_min = 0;
    }

    //FFC温度0为-40度
    if(Cabin_temp_max>=(60 + 40)){  // 动力电池舱温度过高故障
          Bat_Fault_Code[9]|= BAT_CABIN_TEMP_OVER;
    }else if(Cabin_temp_max<=(55 + 40)){
          Bat_Fault_Code[9]&= ~BAT_CABIN_TEMP_OVER;
    }
    if(Cabin_temp_max>=Cabin_temp_min){
          Cabin_temp_diff = Cabin_temp_max - Cabin_temp_min;
    }else{
          Cabin_temp_diff =0;
    }
    
    if(Cabin_temp_diff >=20)    // 动力电池舱温差过高故障
    {
        Bat_Fault_Code[9] |= BAT_CABIN_TEMP_DIFF_OVER;
    }else if(Cabin_temp_diff <15)
    {
        Bat_Fault_Code[9] &= ~BAT_CABIN_TEMP_DIFF_OVER;
    }

    /* ---------- 4. 火警 / 通信状态（不变） ---------- */
    if(FFCD.sts == 1){
        if(EXPIRED_T(TIMER_BAT_FIRE,3000)) {
            Bat_Fault_Code[10] |= BAT_FIRE_ALM;   //火警
            TIMER_RESET_T(TIMER_BAT_FIRE);
        }
    }else{
            TIMER_RESET_T(TIMER_BAT_FIRE);
    }

    if(ffc_life != FFCD.R_life){
        ffc_life = FFCD.R_life;
        ffc_com_num = 0;
        if(FFCD.sts==0){    //消防通信异常复位
             ffc_com_sts=0;
        }
        
        Bat_Fault_Code[10] &= ~FIRE_SYS_OFFLINE;
        TIMER_RESET_T(TIMER_FFC);
    }else{
        if(EXPIRED_T(TIMER_FFC,1000)) {
            ffc_com_num++;
            TIMER_RESET_T(TIMER_FFC);
        }
    }
    if(ffc_com_num>=10){
        ffc_com_sts  = 3;  //消防系统状态（0=正常,1=火警，2=故障，3=灭火离线）
        ffc_com_num  = 11;
        Bat_Fault_Code[10] |= FIRE_SYS_OFFLINE;
        reg_err1_code |= 0x00000002;
    
    }else{
        ffc_com_sts  = FFCD.sts;
    }

    /* ---------- 5. 电磁阀 / 钢瓶状态（不变） ---------- */
    //电磁阀状态
    if((FFCD.cb_sv_sta&0x0f)!=0){
          Bat_Fault_Code[10] |= FIRE_VALVE_SPRAY_TRIG;    //动力电池阀触发
    }else{
          Bat_Fault_Code[10] &= ~FIRE_VALVE_SPRAY_TRIG;
    }
    //
    //钢瓶状态
    if(FFCD.bot1_sts==0){ //1号钢瓶状态（正常=0，喷放=1，故障=2）
         CCU.bot1_sts=0;
          Bat_Fault_Code[10] &= ~FIRE_BOTTLE1_FAULT;
    }else if(FFCD.bot1_sts==1){
         CCU.bot1_sts=1;
         Bat_Fault_Code[10] |= FIRE_BOTTLE_TRIG;
         Contact_Open = 1;
    }else if(FFCD.bot1_sts==2){//欠压
         CCU.bot1_sts=2;
         Bat_Fault_Code[10] |= FIRE_BOTTLE1_FAULT;  //灭火钢1瓶故障
    }else if(FFCD.bot1_sts==4){
         CCU.bot1_sts=2;
         Bat_Fault_Code[10] |= FIRE_BOTTLE1_FAULT;  //灭火钢1瓶故障
    }
        
    
    if(FFCD.bot2_sts==0){ //2号钢瓶状态（正常=0，喷放=1，故障=2）
         CCU.bot2_sts=0;
         Bat_Fault_Code[10] &= ~FIRE_BOTTLE2_FAULT;
    }else if(FFCD.bot2_sts==1){
         CCU.bot2_sts=1;
         Bat_Fault_Code[10] |= FIRE_BOTTLE_TRIG;
         Contact_Open = 1;
    }else if(FFCD.bot2_sts==2){//欠压
         CCU.bot2_sts=2;
         Bat_Fault_Code[10] |= FIRE_BOTTLE2_FAULT;  //灭火钢2瓶故障
    }else if(FFCD.bot2_sts==4){
         CCU.bot2_sts=2;
         Bat_Fault_Code[10] |= FIRE_BOTTLE2_FAULT;  //灭火钢2瓶故障
    } 
    //灭火钢瓶喷发
    if((FFCD.bot1_sts==1)||(FFCD.bot2_sts==1)){
          Bat_Fault_Code[10] |= FIRE_BOTTLE_TRIG;
          Contact_Open = 1;
    }
    //严重热失控
    if((FFCD.bot1_sts==1)&&(FFCD.bot2_sts==1)){
          Bat_Fault_Code[10] |= FIRE_THERMAL_SUPPRESS_FAIL;
          Contact_Open = 1;
    }
    /* ---------- 6. 探头故障判断：只判本动力管的 4 个探头 ---------- */
    {
        sen_err = 0;
        for (k = 0; k < sen_cnt; k++) {
            if (FFCD.sensor[sen_start + k] & 0x39) {
                sen_err = 1;
            }
        }

        if (sen_err) {
            reg_err1_code |= 0x00000001;
            Bat_Fault_Code[10] |= FIRE_DETECT_ERR;  //灭火探测器故障

            // loc_date 存本动力 4 个探头状态
            loc_date[0] = FFCD.sensor[sen_start + 0];
            loc_date[1] = FFCD.sensor[sen_start + 1];
            loc_date[2] = FFCD.sensor[sen_start + 2];
            loc_date[3] = FFCD.sensor[sen_start + 3];

            if(EXPIRED_T(TIMER_FFS,5000)) {
                // 物理探头 1~8 对应 Bat_Fault_Code[6] 的 bit0~bit7
                // 动力1: sensor[0..3] → FFC_S_ERR1..4
                // 动力2: sensor[4..7] → FFC_S_ERR5..8
                for (k = 0; k < 4; k++) {
                    phys = sen_start + k;   // 0~3 或 4~7
                    bit  = 1 << phys;       // 0x01~0x80
                    if (FFCD.sensor[sen_start + k] & 0x39) {
                        Bat_Fault_Code[6] |= bit;
                    } else {
                        Bat_Fault_Code[6] &= ~bit;
                    }
                }
                TIMER_RESET_T(TIMER_FFS);
            }
        } else {
            TIMER_RESET_T(TIMER_FFS);
            Bat_Fault_Code[10] &= ~FIRE_DETECT_ERR;

            // 清本动力对应的 4 个探头故障位
            for (k = 0; k < sen_cnt; k++) {
                phys = sen_start + k;
                Bat_Fault_Code[6] &= ~(1 << phys);
            }
        }
    }
}

/*****************************************************************************/
/* 电池故障检测                                                         */
/*****************************************************************************/
static void StsChk (void)
{

    UINT8 i=0,j=0;
    
    ///////////////////////////状态转换/////////////////////////////////////////////
    
   
    //if(rst_bmu_flag == 0){    
    	
    //单体电压高                                  
	    if(Sys_Alarm.warning3 & WARN_VOL_HIGH){
	        Bat_Fault_Code[2]       |=VOL_H;    
	    }else {
	        if(Sys_Rt.max_v <= (Sys_Alarm_Para.L3.vol_high-50)){        //三级单体电压高故障恢复值为报警值减0.05V
	            Bat_Fault_Code[2]       &=~VOL_H;
	        }
	    }
	    if(Sys_Alarm.warning2 & WARN_VOL_HIGH){ 
	        Bat_Fault_Code[1]       |=VOL_H; 
	    }else {
	        if(Sys_Rt.max_v <= (Sys_Alarm_Para.L2.vol_high-100)){        //二级单体电压高故障恢复值为报警值减0.1V
	             Bat_Fault_Code[1]       &=~VOL_H;
	        }  
	    }
	    if(Sys_Alarm.warning1 & WARN_VOL_HIGH){
	        Bat_Fault_Code[0]       |=VOL_H; 
	      
	    }
	    //单体电压低
	    if(Sys_Alarm.warning3 & WARN_VOL_LOW){
	        Bat_Fault_Code[2]       |=VOL_L;
	    }else {
	        if(Sys_Rt.min_v>=(Sys_Alarm_Para.L3.vol_low+100)){              //三级单体电压低故障恢复值为报警值加0.1V
	            Bat_Fault_Code[2]       &=~VOL_L;
	        }
	    }
	    if(Sys_Alarm.warning2 & WARN_VOL_LOW){
	        Bat_Fault_Code[1]       |=VOL_L;
	    }else {
	        if(Sys_Rt.min_v >= (Sys_Alarm_Para.L2.vol_low+100)){              //二级单体电压低故障恢复值为报警值加0.1V
	             Bat_Fault_Code[1]       &=~VOL_L;
	        }
	    }
	    if(Sys_Alarm.warning1 & WARN_VOL_LOW){
	        Bat_Fault_Code[0]       |=VOL_L;
	        
	    }
	    //电池组压差过大
	    if(Sys_Alarm.warning3&WARN_VOL_UNBA){
	        Bat_Fault_Code[2]       |=V_DIFF;
	    }else {
	        if((Sys_Rt.max_v - Sys_Rt.min_v)<=(Sys_Alarm_Para.L3.vol_unba-100)){  //三级压差故障恢复值为报警值加0.1V
	            Bat_Fault_Code[2]       &=~V_DIFF;
	        }
	    }
	    if(Sys_Alarm.warning2&WARN_VOL_UNBA){
	        Bat_Fault_Code[1]       |=V_DIFF;
	    }else {
	        if((Sys_Rt.max_v - Sys_Rt.min_v)<=(Sys_Alarm_Para.L2.vol_unba-150)){  //二级压差故障恢复值为报警值加0.15V
	            Bat_Fault_Code[1]   &=~V_DIFF;
	        }
	    }
	    if(Sys_Alarm.warning1&WARN_VOL_UNBA){
	        Bat_Fault_Code[0]       |=V_DIFF;
	    }
	    //电池组温差过大
	    if(Sys_Alarm.warning3&WARN_TMP_UNBA){
	        Bat_Fault_Code[2]       |=T_DIFF;
	    }else {
	        if((Sys_Rt.max_t - Sys_Rt.min_t)<= (Sys_Alarm_Para.L3.tmp_unba-2)){        //三级温差故障恢复值为报警值减5℃
	            Bat_Fault_Code[2]       &=~T_DIFF;
	        }
	    }
	    if(Sys_Alarm.warning2 & WARN_TMP_UNBA){
	        Bat_Fault_Code[1]       |=T_DIFF;
	    }else {
	        if((Sys_Rt.max_t - Sys_Rt.min_t) <= (Sys_Alarm_Para.L2.tmp_unba-5)){        //二级温差故障恢复值为报警值减5℃
	            Bat_Fault_Code[1]       &=~T_DIFF;
	        }
	    }
	    if(Sys_Alarm.warning1 & WARN_TMP_UNBA){
	        Bat_Fault_Code[0]       |=T_DIFF;   
        }
	    //温度过高
	    if(Sys_Alarm.warning3 & WARN_TMP_HIGH){
	        Bat_Fault_Code[2]       |=TMP_H;
	    }else {
	        if(Sys_Rt.max_t <= (Sys_Alarm_Para.L3.tmp_high-5)){        //三级温度过高故障恢复值为报警值减5℃
	            Bat_Fault_Code[2]       &=~TMP_H;
	        }
	    }
	    if(Sys_Alarm.warning2 & WARN_TMP_HIGH){
	        Bat_Fault_Code[1]       |=TMP_H;
	    }else {
	        if(Sys_Rt.max_t <= (Sys_Alarm_Para.L2.tmp_high-5)){        //二级温度过高故障恢复值为报警值减5℃
	           Bat_Fault_Code[1]       &=~TMP_H;
	        }
	    }
	    if(Sys_Alarm.warning1 & WARN_TMP_HIGH){
	        Bat_Fault_Code[0]       |=TMP_H;
	    }
	   
	    //电池总压过高
	    if(Sys_Alarm.warning3 & WARN_TVOL_HIGH){
	        
	        Bat_Fault_Code[2]       |=TVOL_H;
	    }else {
	        if(Sys_Rt.to_vol <= (Sys_Alarm_Para.L3.tvol_high-136)){        //三级总压高故障恢复值为报警值减13.6V
	           Bat_Fault_Code[2]       &=~TVOL_H;
	        }
	    }
	    if(Sys_Alarm.warning2 & WARN_TVOL_HIGH){
	        Bat_Fault_Code[1]       |=TVOL_H; 
	    }else {
	        if(Sys_Rt.to_vol  <= (Sys_Alarm_Para.L2.tvol_high-310)){        //二级总压高故障恢复值为报警值减31V
	            Bat_Fault_Code[1]       &=~TVOL_H;
	        }
	       
	    }
	    if(Sys_Alarm.warning1 & WARN_TVOL_HIGH){     
	        Bat_Fault_Code[0]       |=TVOL_H;
	                 
	    }
	    //电池总压过低
	    if(Sys_Alarm.warning3 & WARN_TVOL_LOW){
	        Bat_Fault_Code[2]       |=TVOL_L;
	    }else {
	        if(Sys_Rt.to_vol>=(Sys_Alarm_Para.L3.tvol_low+100)){        //三级总压低故障恢复值为报警值加10V
	          Bat_Fault_Code[2]       &=~TVOL_L;
	        }
	    }
	    if(Sys_Alarm.warning2 & WARN_TVOL_LOW){
	        Bat_Fault_Code[1]       |=TVOL_L;
	    }else {
	        if(Sys_Rt.to_vol>=(Sys_Alarm_Para.L2.tvol_low+320)){        //二级总压低故障恢复值为报警值加32V
		            Bat_Fault_Code[1]       &=~TVOL_L;
	        }
	    }
	    if(Sys_Alarm.warning1 & WARN_TVOL_LOW){
	        Bat_Fault_Code[0]       |=TVOL_L;
	    } 
	    //电池组温度过低
	    if(Sys_Alarm.warning3 & WARN_TMP_LOW){
	        Bat_Fault_Code[2]       |=TMP_L;
	    }else {
	        if(Sys_Rt.min_t >= (Sys_Alarm_Para.L3.tmp_low+5)){        //三级温度过低故障恢复值为报警值加5℃
	          Bat_Fault_Code[2]       &=~TMP_L;
	        }
	    }
        //新保护规范低温没中级故障 先报着
	    if(Sys_Alarm.warning2 & WARN_TMP_LOW){
	        Bat_Fault_Code[1]|=TMP_L;
	    }else {
	        if(Sys_Rt.min_t >=(Sys_Alarm_Para.L2.tmp_low+5)){        //二级温度过低故障恢复值为报警值加5℃
	          Bat_Fault_Code[1]&=~TMP_L;
	        }
	    }
	    if(Sys_Alarm.warning1 & WARN_TMP_LOW){
            Bat_Fault_Code[0]|=TMP_L;

        }
    //}
    
    //电池组SOC过低
    if(Sys_Alarm.warning3 & WARN_SOC_LOW){
        Bat_Fault_Code[3]       |=SOC_L3;
    } else {
        if(Sys_Rt.soc>=(Sys_Alarm_Para.L3.soc_low+2)){        //三级SOC过低故障恢复值为报警值加2%
           Bat_Fault_Code[3]       &=~SOC_L3;
        }
    }
    if(Sys_Alarm.warning2 & WARN_SOC_LOW){
        Bat_Fault_Code[4]       |=SOC_L2;
    } else {
        if(Sys_Rt.soc >=(Sys_Alarm_Para.L2.soc_low+5)){        //二级SOC过低故障恢复值为报警值加5%
            Bat_Fault_Code[4]       &=~SOC_L2;
        }
    }
    if(Sys_Alarm.warning1 & WARN_SOC_LOW){
        Bat_Fault_Code[5]       |=SOC_L1;
    }else{
        Bat_Fault_Code[5]       &=~SOC_L1;
    }
     //电池组充电过流
    if(Sys_Alarm.warning3 & WARN_CHG_CUR_HIGH){
        Bat_Fault_Code[3]       |=CUR_H3;
    }else {
        if(Sys_Rt.current<=(Sys_Alarm_Para.L3.chg_cur_high-50)){        //三级充电过流故障恢复值为报警值减5A
           Bat_Fault_Code[3]&=~CUR_H3;
        }
    }
    if(Sys_Alarm.warning2 & WARN_CHG_CUR_HIGH){
        Bat_Fault_Code[4]|=CUR_H2;
    }else {
        if(Sys_Rt.current<=(Sys_Alarm_Para.L2.chg_cur_high-50)){        //二级充电过流故障恢复值为报警值减20A
           Bat_Fault_Code[4]&=~CUR_H2;
        }
    }
    if(Sys_Alarm.warning1 & WARN_CHG_CUR_HIGH){
        Bat_Fault_Code[5]|=CUR_H1;
       
    }
     //电池组放电过流
    if(Sys_Alarm.warning3 & WARN_DISCHG_CUR_HIGH){
        Bat_Fault_Code[3] |=FUR_H3;
    }else {
        if((0-Sys_Rt.current)<=(Sys_Alarm_Para.L3.dischg_cur_high-200)){        //三级放电过流故障恢复值为报警值减20A
           Bat_Fault_Code[3] &=~FUR_H3;
        }
    }
    if(Sys_Alarm.warning2 & WARN_DISCHG_CUR_HIGH){
        Bat_Fault_Code[4] |=FUR_H2;
    }else {
        if((0-Sys_Rt.current)<=(Sys_Alarm_Para.L2.dischg_cur_high-200)){        //二级放电过流故障恢复值为报警值减20A
           Bat_Fault_Code[4] &=~FUR_H2;
        }
    }
    if(Sys_Alarm.warning1 & WARN_DISCHG_CUR_HIGH){
        Bat_Fault_Code[5]  |=FUR_H1;
    }
        
    //////////////////////////////////////////////////////////////////////////////////////    
    //加热熔断器故障

    if(FU3_FU4_OS == CLOSE) {  // 正常：熔断器闭合
        Bat_Fault_Code[14] |= HEAT_FUSE_STATUS; //加热熔断器正常
        Bat_Fault_Code[4] &= ~HFUS_ERR;  // 清除熔断器故障
        TIMER_RESET_T(TIMER_FU3_FU4_ERR);
    } else {  // 故障：熔断器断开（熔断）
        if(EXPIRED_T(TIMER_FU3_FU4_ERR, 5000)) {
            Bat_Fault_Code[14] &=~HEAT_FUSE_STATUS; //加热熔断器故障
            Bat_Fault_Code[4] |= HFUS_ERR;   // 置位熔断器故障
            TIMER_RESET_T(TIMER_FU3_FU4_ERR);
        }
    }

    //内部熔断器，常闭触点，输入1正常，0故障
    if((FU1_OS == CLOSE)&&(FU2_OS == CLOSE)){   //读第1个模块的BMU输入为主正熔断器状态，读第9个模块的BMU输入为主负熔断器状态
        Bat_Fault_Code[14] |= MAIN_FUSE_STATUS; //内部熔断器正常
        //Bat_Fault_Code[4] &= ~FUS_ERR;  // 清除熔断器故障
        TIMER_RESET_T(TIMER_FU1_FU2_ERR);
            
    }else{
        if(EXPIRED_T(TIMER_FU1_FU2_ERR, 5000)) {
            Bat_Fault_Code[14] &=~MAIN_FUSE_STATUS; //内部熔断器故障
            Bat_Fault_Code[4] |= FUS_ERR;  // 置位熔断器故障
            TIMER_RESET_T(TIMER_FU1_FU2_ERR);
        }
    }  

    //动力电池BMPU触发硬件冗余保护 -todo 确认硬件保护相关能否恢复
    if(KA9_OS==NC_OPEN){
        if(EXPIRED_T(TIMER_KA9_ERR, 5000)){
            Contact_Open = 1;
            //动力电池BMPU触发过温硬件冗余保护
            if(Sys_Rt.max_t>=75 + 55){ //75℃保护
                Bat_Fault_Code[8] |=HPU_TEMP_ERR;
            }

            //动力电池BMPU触发过压硬件冗余保护
            if(Sys_Rt.max_v>=3300) { //3.3V保护
                Bat_Fault_Code[5] |=HPU_ERR;
            }
            TIMER_RESET_T(TIMER_KA9_ERR); 
        }
    }else{
        Bat_Fault_Code[5] &= ~HPU_ERR;
        Bat_Fault_Code[8] &= ~HPU_TEMP_ERR;
        TIMER_RESET_T(TIMER_KA9_ERR); 
    }


    // BMU单体电压采集异常轻微故障 -没按规范todo
    if(((Bat_Fault_Code[3]&BMS_ERR)!=BMS_ERR)&&((Bat_Fault_Code[3]&BMS_COM_ERR)!=BMS_COM_ERR)){
        if((Sys_Rt.max_v>5000)||(Sys_Rt.min_v<=500)){ //mv
                if(EXPIRED_T(TIMER_VOL_ERR, 2000)){
                    Bat_Fault_Code[8] |=BMU_VOL_ERR;
                    self_err_code |=0x40;
                    TIMER_RESET_T(TIMER_VOL_ERR);
                }
        }else{                                           
                Bat_Fault_Code[8] &= ~BMU_VOL_ERR;       
                TIMER_RESET_T(TIMER_VOL_ERR);
        }
    }
    // BMU单体温度采集异常轻微故障 -没按规范todo
    if(((Bat_Fault_Code[3]&BMS_ERR)!=BMS_ERR)&&((Bat_Fault_Code[3]&BMS_COM_ERR)!=BMS_COM_ERR)){
            if((Sys_Rt.max_t>(125+55))||(Sys_Rt.min_t<(-40+55))){ //55=0℃
                if(EXPIRED_T(TIMER_TEMP_ERR, 10000)){
                    Bat_Fault_Code[9] |=BMU_TEMP_ERR;
                    self_err_code |= 0x20;
                    TIMER_RESET_T(TIMER_TEMP_ERR);
                }
        }else{
            Bat_Fault_Code[9] &= ~BMU_TEMP_ERR;
            TIMER_RESET_T(TIMER_TEMP_ERR);
        }
    }else{                                           
            Bat_Fault_Code[9] &= ~BMU_TEMP_ERR;
            TIMER_RESET_T(TIMER_TEMP_ERR);
    }
    //动力电池电压过低无法充电故障
    if(Sys_Rt.to_vol<=5500){
        Bat_Fault_Code[3] |=VLNC_ERR;
    }
    //电压传感器TV1轻微故障-没按规范todo
    if(ks_stage==2){
        TIMER_RESET_T(TIMER_TV1_ERR);
    }
    if(tv1_diag_init == 0){
        // 首次进入诊断，初始化基准和计数
        temp_tv1_volt = O_Voltage;
        temp_tv1_volt_err_count = 0;
        tv1_diag_init = 1;
        TIMER_RESET_T(TIMER_TV1_ERR);
    }
    else if(EXPIRED_T(TIMER_TV1_ERR, 5000)){
        if(temp_tv1_volt_err_count >= 10){
            Bat_Fault_Code[8] |= TV1_ERR;
        }
        else{
            Bat_Fault_Code[8] &= ~TV1_ERR;
        }
        temp_tv1_volt_err_count = 0;
        TIMER_RESET_T(TIMER_TV1_ERR);
    }
    else{
        if(O_Voltage > temp_tv1_volt){
            if(O_Voltage - temp_tv1_volt >= 1000){
                temp_tv1_volt_err_count++;
            }
        }
        else{
            if(temp_tv1_volt - O_Voltage >= 1000){
                temp_tv1_volt_err_count++;
            }
        }
        temp_tv1_volt = O_Voltage;
    }
    //电流传感器TA1轻微故障-没按规范todo
    if(ks_stage==2){
        TIMER_RESET_T(TIMER_TA1_ERR);
    }
    if(ta1_diag_init == 0){
        temp_ta1_current = Sys_Rt.current;
        temp_ta1_current_err_count = 0;
        ta1_diag_init = 1;
        TIMER_RESET_T(TIMER_TA1_ERR);
    }
    else if(EXPIRED_T(TIMER_TA1_ERR, 30000)){
        if(temp_ta1_current_err_count >= 10){
            Bat_Fault_Code[8] |= TA1_ERR;
        }
        else{
            Bat_Fault_Code[8] &= ~TA1_ERR;
        }
        temp_ta1_current_err_count = 0;
        TIMER_RESET_T(TIMER_TA1_ERR);
    }
    else{
        if(Sys_Rt.current > temp_ta1_current){
            if(Sys_Rt.current - temp_ta1_current >= 1000){
                temp_ta1_current_err_count++;
            }
        }
        else{
            if(temp_ta1_current - Sys_Rt.current >= 1000){
                temp_ta1_current_err_count++;
            }
        }
        temp_ta1_current = Sys_Rt.current;
    }
    //蓄电池与充电机电流不一致轻微故障 -没按规范todo
    if((Bat_Sys_sts == 1)&&(Sys_Rt.current>20)){
        if((Sys_Rt.current > charge_current)&&(Sys_Rt.current - charge_current)> 50){
            if(EXPIRED_T(TIMER_BAT_CHA_CUR_ERR, 15000)) {
                Bat_Fault_Code[8] |= BMS_BC_CURRENT_ERR;
                TIMER_RESET_T(TIMER_BAT_CHA_CUR_ERR);
            }
        }else{
            Bat_Fault_Code[8] &=~BMS_BC_CURRENT_ERR;
            TIMER_RESET_T(TIMER_BAT_CHA_CUR_ERR);
        }
    }else{
        Bat_Fault_Code[8] &=~BMS_BC_CURRENT_ERR;
        TIMER_RESET_T(TIMER_BAT_CHA_CUR_ERR);
    }
    //动力电池牵引回路电压/外总压过高故障
    if(O_Voltage > 9280){
        if(EXPIRED_T(TIMER_OVOL_H_ERR, 250)){
            Bat_Fault_Code[5] |= OVOL_H_ERR;
            TIMER_RESET_T(TIMER_OVOL_H_ERR);
        }
    }
    else{
        TIMER_RESET_T(TIMER_OVOL_H_ERR);
    }
    //动力电池牵引回路电压/外总压过低故障
    if(KM15_OS == CLOSE){
        if(EXPIRED_T(TIMER_OVOL_L_KM15_ERR, 500)){
            if(O_Voltage < 4200){
                if(EXPIRED_T(TIMER_OVOL_L_ERR, 250)){
                    Bat_Fault_Code[5] |= OVOL_L_ERR;
                    TIMER_RESET_T(TIMER_OVOL_L_ERR);
                }
            }
            else{
                TIMER_RESET_T(TIMER_OVOL_L_ERR);
            }
        }
    }
    else{
        TIMER_RESET_T(TIMER_OVOL_L_KM15_ERR);
        TIMER_RESET_T(TIMER_OVOL_L_ERR);
    }
    /////////////////////////////屏幕显示////////////////////////////////////////////////////////////
    //动力电池电量过低保护 -非故障 专用屏幕显示
    if((Sys_Rt.to_vol<=5830)||(Sys_Rt.soc<=5)){
            if(EXPIRED_T(TIMER_SOC_L, 1000)) {
                Bat_Fault_Code[12] |= TBMS_SOC_LOW;
                TIMER_RESET_T(TIMER_SOC_L);
            }
    }else{
        Bat_Fault_Code[12] &=~TBMS_SOC_LOW;
        TIMER_RESET_T(TIMER_SOC_L);
    }

    if((Bat_Sys_sts != 3) && (Bat_Sys_sts != 5)) {
        //电池无严重故障（保护规范二级故障）
         if((Bat_Fault_Code[13]&CHG_PULSE_LOCK_REQ) == CHG_PULSE_LOCK_REQ)//无故障，且充电机封锁脉冲为待机状态
        {
            Bat_Sys_sts = 4;     //待机
        }
        else
        {
            if(Sys_Rt.current>10) 
            {
                Bat_Sys_sts = 1;   //充电
            }
            else if(Sys_Rt.current<(-10))
            {
                Bat_Sys_sts = 2;      //放电
            }

            else if((Sys_Rt.current<=10)&&(Sys_Rt.current >=(-10)))
            {
                Bat_Sys_sts = 4;     //待机
            }
        }
    }
    else{
        //电池有严重故障（保护规范二级故障）
        

    }
    
    
}



/*****************************************************************************/
/* SOC校正函数                                                               */
/*****************************************************************************/
static void SOC_correction1(void) {
    SINT16 temy_soc =0;
    UINT8   soc_no_flag = 0;
    UINT16 Soc_bmrk[10] ={6836,6966,7063,7160,7298,7517,7647,7808,8068,8229};

    
    if(Sys_Rt.current==0) {               //0<电流<10A
        if(EXPIRED_T(TIMER_SOC, 1800000)) {
            if(Sys_Rt.to_vol > Soc_bmrk[9]) {
                soc_no_flag = 1;
            
            }else if(Sys_Rt.to_vol > Soc_bmrk[8]) {
                soc_no_flag = 1;
           
            }else if(Sys_Rt.to_vol > Soc_bmrk[7]) {
                SOC_Offset = (Sys_Rt.to_vol - Soc_bmrk[7])*100 / (Soc_bmrk[8] - Soc_bmrk[7]) + 800;
            }else if(Sys_Rt.to_vol > Soc_bmrk[6]) {
                SOC_Offset = (Sys_Rt.to_vol - Soc_bmrk[6])*100 / (Soc_bmrk[7] - Soc_bmrk[6]) + 700;
            }else if(Sys_Rt.to_vol > Soc_bmrk[5]) {
                SOC_Offset = (Sys_Rt.to_vol - Soc_bmrk[5])*100 / (Soc_bmrk[6] - Soc_bmrk[5]) + 600;
            }else if(Sys_Rt.to_vol > Soc_bmrk[4]) {
                SOC_Offset = (Sys_Rt.to_vol - Soc_bmrk[4])*100 / (Soc_bmrk[5] - Soc_bmrk[4]) + 500;
            }else if(Sys_Rt.to_vol > Soc_bmrk[3]) {
                SOC_Offset = (Sys_Rt.to_vol - Soc_bmrk[3])*100 / (Soc_bmrk[4] - Soc_bmrk[3]) + 400;
            }else if(Sys_Rt.to_vol > Soc_bmrk[2]) {
                SOC_Offset = (Sys_Rt.to_vol - Soc_bmrk[2])*100 / (Soc_bmrk[3] - Soc_bmrk[2]) + 300;
            }else if(Sys_Rt.to_vol > Soc_bmrk[1]) {
                SOC_Offset = (Sys_Rt.to_vol - Soc_bmrk[1])*100 / (Soc_bmrk[2] - Soc_bmrk[1]) + 200;
            }else if(Sys_Rt.to_vol > Soc_bmrk[0]) {
                SOC_Offset = (Sys_Rt.to_vol - Soc_bmrk[0])*100 / (Soc_bmrk[1] - Soc_bmrk[0]) + 100;
            }else{
                SOC_Offset = SOC_k;
            }
            temy_soc = SOC_Offset - SOC_k;
            if((temy_soc != 0)&&(soc_no_flag == 0)){
                SOC_diff    = temy_soc;
                SOC_S_flag  = 1;
            }                        
            TIMER_RESET_T(TIMER_SOC);
        }   
    }else {                                                     
        TIMER_RESET_T(TIMER_SOC);
    }
   
    if((TBMS_Traction ==1 )||(TBMS_Standby == 1)){
        SOC_S_flag  = 0;
        SOC_diff = 0;
    }
    if(SOC_S_flag == 1) {
        if(EXPIRED_T(TIMER_SOC_SET, 1000)){
            if(SOC_diff > 0){
                SOC_diff = SOC_diff - 1;
                RT_SetSOC_K (SOC_k + 1);
            }else if(SOC_diff<0){
                SOC_diff=SOC_diff+1;
                RT_SetSOC_K (SOC_k-1);
            }else{
                SOC_S_flag = 0;
            }
            TIMER_RESET_T(TIMER_SOC_SET); 
        }
    
    }else{
        TIMER_RESET_T(TIMER_SOC_SET);
    }

}

/***************************************/
/* 历史记录数据分配                    */
/**************************************/
static UINT16 dv4000_to_voltage_01v(SINT16 adc_val)
{
    if (adc_val < 0)
        adc_val = 0;
    return (UINT32)adc_val * 100 / 39; // 对应关系：4096->32767，50mA->125v；电阻：39；公式：adc_val * 4096 / 32767 * 800 / 39
}

static UINT16 dvl1000_to_voltage_01v(SINT16 adc_val)
{
    if (adc_val < 0)
        adc_val = 0;
    return (UINT32)adc_val * 25 / 39; // 对应关系：4096->32767，50mA->125v；电阻：39；公式：adc_val * 4096 / 32767 * 200 / 39
}

void save_sign_function(void)
{
   
  
    //T_Voltage = dv4000_to_voltage_01v(ttttwww[0]);
    O_Voltage = dvl1000_to_voltage_01v(ttttwww[0]);
    Sys_Rt.current = 100;
    ////绝缘模块继电器正1电压
    Sys_Rt.relayFAST      = IWD_Res;
    ////绝缘模块继电器正2电压 
    Sys_Rt.relaySLOW      = Sys_Rt.current; 
    
    
    //Sys_Rt.NCNC           = O_Voltage;//ks_stage;
    ////绝缘电阻总电压
//    Sys_Rt.total_vol_res  = (Fire_Start2_flag<<1) | Fire_Start1_flag; 
    ////绝缘电阻--总正(单位为10K欧姆)        
   //Sys_Rt.pos_res = ks_stage;//SOC_k; //(Bat_soh1<<8)|Bat_soh2;
}

/*****************************************************************************/
/* 统计电量                                                                  */
/*****************************************************************************/
void Cum_ele(void){
    static const UINT16  Q_k[61] = {
        150,153,155,158,160,163,165,168,170,173,     //-30
        175,178,180,182,184,186,188,190,192,193,     //-20
        195,197,198,199,200,201,202,203,203,204,     //-10
        205,206,207,208,209,209,210,211,212,214,     //0
        215,216,217,217,218,218,219,219,220,220,     //10
        221,221,222,222,223,225,226,227,228,229,     //20
        230,                                         //30
    };
    UINT32  temy_date   = 0;
    UINT8   Q_temp      = 0;
    UINT8   N_use       = 0;

    if(EXPIRED_T(TIMER_C,2000)) {        //充电判断
        end_ele = Sys_Rt.wh;
        if((end_ele<sta_ele)){
            Sys_kwh.F_re_ele   = Sys_kwh.F_re_ele  +(sta_ele-end_ele);

            if(Sys_kwh.F_re_ele>100){
                Sys_kwh.F_kw      += Sys_kwh.F_re_ele/100;
                Sys_kwh.F_re_ele   = Sys_kwh.F_re_ele%100;
                Save_flag=1;
            }
            sta_ele = end_ele;      
        }else if((end_ele>sta_ele)){
            Sys_kwh.C_re_ele   =  Sys_kwh.C_re_ele  + (end_ele-sta_ele);
            if(Sys_kwh.C_re_ele>100) {
                Sys_kwh.C_kw      += Sys_kwh.C_re_ele/100;     //充电电量累加 单位0.1kwh
                Sys_kwh.C_re_ele   = Sys_kwh.C_re_ele%100; 
                Save_flag=1;
            }
            sta_ele = end_ele;      
        }
        TIMER_RESET_T(TIMER_C);   
    }
    Sys_kwh.CF_num = Sys_kwh.F_kw / BAT_KWH ; 
    //保存电量
    if(Save_flag==1) { 
        Sys_kwh_Save(SAVE_176_DATA,Sys_kwh);
        Save_flag=0;
    }

    if (CCU_T.tm_year >= 27)
    {
        Battery_SOH_BMS = 100 - (CCU_T.tm_year - 26) / 2;
    }
    else
    {
        Battery_SOH_BMS = 100;
    }
    if (Battery_SOH_BMS <= 90)
    {
        Battery_SOH_BMS = 90;
    }

    ////////////////////计算初始电量////////////////////////////
    //年计算
    if(CCU_T.tm_year >= 26)   N_use = CCU_T.tm_year - START_YEAR;
    else                      N_use = 0;
    if(Sys_Rt.max_t <= (-30 + 55)){      //-30以下
        Q_temp  = 0;    
    }else if(Sys_Rt.max_t >= (30 + 55)){
        Q_temp  = 60;
    }else{
        Q_temp  = Sys_Rt.max_t - 25;
    }
    //Q 1=（SOC-10%） *102kWh*（A÷23Ah）*（1-（N-1）/100）
    Q_remain = BAT_KWH;
   
    if(SOC_k > 100) {
        Q_remain *= SOC_k-100;  
    }else{
        Q_remain = 0;
    }
    
    Q_remain    *= Q_k[Q_temp];
    Q_remain    /=23; 
      // 
    if(N_use >=1)  Q_remain    *= (100 - (N_use-1));// Q_remain    *= (100 -N_use);
    else            Q_remain    *= 100;
    Q_remain    /= 100000;
   
    if(CCU_locmt_cmd !=TBMS_Traction) {     
        CCU_locmt_cmd = TBMS_Traction;    
        if(TBMS_Traction == 1) {
            loc_soc = SOC_k;
            //loc_soc = Sys_Rt.wh;
        }
    }
    if(CCU_locmc_cmd != TBMS_Standby ) { 
        CCU_locmc_cmd = TBMS_Standby;
        if(TBMS_Standby == 1) {
            loc_soc = SOC_k;
            //loc_soc = Sys_Rt.wh;
        }
    }
    if((TBMS_Traction == 0)&&(TBMS_Standby == 0)){
        Q_lock      =  Q_remain;
        Q_usable    =  Q_remain;     
    }else {
        if(loc_soc >= SOC_k){
            temy_date   = loc_soc - SOC_k;
        }else{
            temy_date   = 0;
        }
        //temy_date   = temy_date*(Sys_Rt.to_vol/10); 
        //temy_date   = temy_date/883; 
        temy_date   = temy_date*BAT_KWH;
        //temy_date   /= 10000;
        temy_date   /= 100;
        if(Q_lock >= temy_date){
            Q_usable = Q_lock - temy_date; 
        }else{
            Q_usable = 0;
        }
    }
}

/***************************************/
/* /保护/                              */
/***************************************/
static void Protect (void) 
{ 
    ////////////////////保护规范2级故障//////////////////////////
    /////////////////单体电压高严重故障//////////////////////////////   
    if(Sys_Alarm.warning1 & (WARN_VOL_HIGH)) {
        Contact_Open = 1;
    }
    if((Bat_Fault_Code[0]&VOL_H)==VOL_H){
        Contact_Open = 1; 
    }
    /////////////////单体电压低严重故障//////////////////////////////
    if(Sys_Alarm.warning1 & (WARN_VOL_LOW)) {
        Contact_Open = 1;
    }
    if((Bat_Fault_Code[0]&VOL_L)==VOL_L){
        Contact_Open = 1; 
    }
    /////////////////总压高严重故障////////////////////////////// 
    if(Sys_Alarm.warning1 & (WARN_TVOL_HIGH)) {
        Contact_Open = 1;
    }
    if((Bat_Fault_Code[0]&TVOL_H)==TVOL_H){
        Contact_Open = 1;  
    }
    /////////////////总压低严重故障////////////////////////////// 
    if(Sys_Alarm.warning1 & (WARN_TVOL_LOW)) {
        Contact_Open = 1;
    }
    if((Bat_Fault_Code[0]&TVOL_L)==TVOL_L){
        Contact_Open = 1;  
    }
    /////////////////压差过大严重故障//////////////////////////////
    if(Sys_Alarm.warning1 & (WARN_VOL_UNBA)) {
        Contact_Open = 1;
    }
    if((Bat_Fault_Code[0]&V_DIFF)==V_DIFF){
        Contact_Open = 1;  
    }
    /////////////////温度过高严重故障//////////////////////////////
    if(Sys_Alarm.warning1 & (WARN_TMP_HIGH)) {
        Contact_Open = 1;
    }
    if((Bat_Fault_Code[0]&TMP_H)==TMP_H){
        Contact_Open = 1;  
    }
    /////////////////温度过低严重故障//////////////////////////////
    if(Sys_Alarm.warning1 & (WARN_TMP_LOW)) {
        Contact_Open = 1;
    }
    if((Bat_Fault_Code[0]&TMP_L)==TMP_L){
        Contact_Open = 1;  
    }
    /////////////////充电电流过高严重故障//////////////////////////
    if(Sys_Alarm.warning1 & (WARN_CHG_CUR_HIGH)) {
        Contact_Open = 1;
    }
    if((Bat_Fault_Code[5]&CUR_H1)==CUR_H1){
        Contact_Open = 1;  
    }
    /////////////////放电电流过高严重故障//////////////////////////
    if(Sys_Alarm.warning1 & (WARN_DISCHG_CUR_HIGH)) {
        Contact_Open = 1;
    }
    if((Bat_Fault_Code[5]&FUR_H1)==FUR_H1){
        Contact_Open = 1;  
    }
    /////////////////动力电池硬件故障//////////////////////////
    if((Bat_Fault_Code[3]&BMS_ERR)==BMS_ERR){
        Contact_Open = 1;  
    } 
    ////////////////动力电池内部通讯故障/////////////////////////
    if((Bat_Fault_Code[3]&BMS_COM_ERR)==BMS_COM_ERR){
        Contact_Open = 1;  
    }
    ///////////////动力电池正极绝缘过低故障////////////////////
    if((Bat_Fault_Code[3]&IWD_ERR)==IWD_ERR){
        Contact_Open = 1;  
    }
    //////////////动力电池电压过低无法充电故障/////////////////
    if((Bat_Fault_Code[3]&VLNC_ERR)==VLNC_ERR){
        Contact_Open = 1;  
    }
    //////////////动力电池KM15接触器无法闭合故障///////////////
    if((Bat_Fault_Code[4]&KM15_CERR)==KM15_CERR){
        if(KA15_cerr_time >=2){
            Contact_Open = 1;
        } 
    }
    //////////////动力电池KM14接触器无法闭合故障///////////////
    if((Bat_Fault_Code[4]&KM14_CERR)==KM14_CERR){
        if(KA14_cerr_time >=2){
            Contact_Open = 1;
        } 
    }
    /////////////动力电池温差过大故障////////////
    if(Sys_Alarm.warning1 & (WARN_TMP_UNBA)) {
        Contact_Open = 1;
    }
    if((Bat_Fault_Code[0]&T_DIFF)==T_DIFF){
        Contact_Open = 1;  
    }
    //////////////动力电池KM10接触器无法闭合故障///////////////
    if((Bat_Fault_Code[5]&KM10_CERR)==KM10_CERR){
        if(KA10_cerr_time >=2){
            Contact_Open = 1;
        }  
    }
    //////////////动力电池预充电故障///////////////
    if((Bat_Fault_Code[5]&PCH_ERR)==PCH_ERR){
        Contact_Open = 1;  
    }
    //////////////动力电池BPU触发过压硬件冗余保护//////////////
    if((Bat_Fault_Code[5]&HPU_ERR)==HPU_ERR){
        Contact_Open = 1;  
    }
    /////////////动力电池BPU触发过温硬件冗余保护//////////////
    if((Bat_Fault_Code[8]&HPU_TEMP_ERR)==HPU_TEMP_ERR){
        Contact_Open = 1;  
    }
    /////////////动力电池KM15接触器无法断开故障//////////////
    if((Bat_Fault_Code[8]&KM15_OERR)==KM15_OERR){
        Contact_Open = 1;  
    }
    /////////////动力电池KM14接触器无法断开故障//////////////
    if((Bat_Fault_Code[8]&KM14_OERR)==KM14_OERR){
        Contact_Open = 1;  
    }
    /////////////动力电池KM10接触器无法断开故障//////////////
    if((Bat_Fault_Code[8]&KM10_OERR)==KM10_OERR){
        Contact_Open = 1;  
    }
    //////////////动力电池热接触器无法断开故障/////////////////
    if((Bat_Fault_Code[4]&KM7_KM8_OERR)==KM7_KM8_OERR){
        Contact_Open = 1;  
    }
    /////////////动力电池与整车网络通讯故障//////////////
    if((self_err_code & CCU_NET_ERR) == CCU_NET_ERR){
        //断开接触器
        KA10 = OPEN;
        KA15 = OPEN;
        KA14 = OPEN;  
        pre_cha_stage = 0;
        pre_status    = 0;
        Bat_Fault_Code[13] |= CHG_STOP_REQ;
    }
    else{
        if(Contact_Open != 1){
            Bat_Fault_Code[13] &= ~CHG_STOP_REQ;
        }
    }
    //电气舱烟雾报警
    if(temp_ccu_data->ecab_smoke == 1){
        Contact_Open = 1;
    }
    /////////////动力电池牵引回路电压/外总压过低故障//////////////
    if((Bat_Fault_Code[5]&OVOL_L_ERR)==OVOL_L_ERR){
        Contact_Open = 1;  
    }
    /////////////动力电池牵引回路电压/外总压过高故障//////////////
    if((Bat_Fault_Code[5]&OVOL_H_ERR)==OVOL_H_ERR){
        Contact_Open = 1;  
    }
    ////////////////////保护规范1级故障//////////////////////////
    //火警
    if((Bat_Fault_Code[10]&BAT_FIRE_ALM)==BAT_FIRE_ALM){
        Contact_Open = 1;  
    }
    //严重热失控
    if((Bat_Fault_Code[10]&FIRE_THERMAL_SUPPRESS_FAIL)==FIRE_THERMAL_SUPPRESS_FAIL){
        Contact_Open = 1;  
    }
    ////////////////////保护规范3级故障//////////////////////////
    //动力电池热接触器无法闭合故障
    if((Bat_Fault_Code[3]&KM7_KM8_CERR)==KM7_KM8_CERR){
        //不保护 
    }
    //动力电池加热熔断器故障
    if((Bat_Fault_Code[4]&HFUS_ERR)==HFUS_ERR){
        //不保护 
    }
    //动力电池主回路熔断器故障
    if((Bat_Fault_Code[4]&FUS_ERR)==FUS_ERR){
        //不保护 
    }
    //动力电池单体电压采集异常故障
    if((Bat_Fault_Code[8]&BMU_VOL_ERR)==BMU_VOL_ERR){
        //不保护 
    }
    //动力电池电压传感器TV1故障
    if((Bat_Fault_Code[8]&TV1_ERR)==TV1_ERR){
        //不保护 
    }
    //动力电池与充电机电流不一致故障
    if((Bat_Fault_Code[8]&BMS_BC_CURRENT_ERR)==BMS_BC_CURRENT_ERR){
        //不保护 
    }
    //动力电池电流传感器TA1故障
    if((Bat_Fault_Code[8]&TA1_ERR)==TA1_ERR){
        //不保护 
    }
    //动力电池舱温度过高故障
    if((Bat_Fault_Code[9]&BAT_CABIN_TEMP_OVER)==BAT_CABIN_TEMP_OVER){
        //不保护 
    }
    //动力电池舱温差过大故障
    if((Bat_Fault_Code[9]&BAT_CABIN_TEMP_DIFF_OVER)==BAT_CABIN_TEMP_DIFF_OVER){
        //不保护 
    }
    //动力电池加热功能异常故障
    if((Bat_Fault_Code[9]&HOT_ERR)==HOT_ERR){
        //不保护 
    }
    //动力电池温度采集异常故障
    if((Bat_Fault_Code[9]&BMU_TEMP_ERR)==BMU_TEMP_ERR){
        //不保护 
    }
    //动力电池数据记录故障
    if((Bat_Fault_Code[9]&DATA_RECORD_ERR)==DATA_RECORD_ERR){
        //不保护 
    }

    //非保护规范保护
    if((temp_tcu_data.chg_fault == 1)){
        Contact_Open = 1;
    }
    //收到切除信号断开所有接触器
    if(temp_ccu_data->cutoff == 1){
        Bat_Sys_sts = 5;
        //断开所有接触器
        KA10 = OPEN;
        KA15 = OPEN;
        KA14 = OPEN;
        KA7 = OPEN;
        KA8 = OPEN;
        pre_cha_stage = 0;
        pre_status    = 0;
        Bat_Fault_Code[13] |= CHG_PULSE_LOCK_REQ;
    }
    if((temp_ccu_data->cutoff == 0) && (Bat_Sys_sts == 5)){
        Bat_Sys_sts = 4;
    }
    if(Contact_Open==1){
        //断开所有接触器
        KA10 = OPEN;
        KA15 = OPEN;
        KA14 = OPEN;
        KA7 = OPEN;
        KA8 = OPEN;
        KA16 = CLOSE;
        //充电机请求停止并封锁脉冲，置为电池故障状态
        Bat_Fault_Code[13] |= CHG_STOP_REQ;
        Bat_Fault_Code[13] |= CHG_PULSE_LOCK_REQ;
        Bat_Fault_Code[13] |= BAT_FAULT_STATUS;
        Bat_Sys_sts = 3;     
    }
}

/***************************************/
/* /流程/                              */
/***************************************/
static void Turn_On(void) {

    switch(ks_stage){
                 //////////////////上电自检 ks_stage==0~9///////////////////////////////////
            case 0:
                if(EXPIRED_T(TIMER_TURN_ON,15000)) {//上电延迟15s，开始自检
                    check_bms=NSC; 
                    sta_ele           = Sys_Rt.wh;
                    end_ele           = Sys_Rt.wh;
                    IWD_cmd = 0xff; //表示未进行绝缘检测
                    ks_stage=1;
                    TIMER_RESET_T(TIMER_TURN_ON);
                }
                break;
            case 1:
                if(EXPIRED_T(TIMER_TURN_ON,15000)){  //自检开始延迟15s，切换到自检状态
                    check_bms=SCR;  
                    ks_stage=2;
                    TIMER_RESET_T(TIMER_TURN_ON);
                }
                break;
            case 2:
                //进行绝缘检测
                if(IWD_sta==0xAA){   //判断绝缘是否正常
                    IWDCheck=(UINT8)Check_IWD(); //检测绝缘电阻
                    if(IWDCheck==0){
                        IWD_cmd  = 0;   
                        FAU_IWD=0; 
                        Bat_Fault_Code[3] &= ~IWD_ERR; 
                        ks_stage = 3;
                        TIMER_RESET_T(TIMER_TURN_ON);
                    }else{
                       FAU_IWD=1;       //绝缘检测故障
                       Bat_Fault_Code[3] |= IWD_ERR;       //绝缘检测故障
                       check_bms=SCF;         //自检失败
                       
                       ks_stage = 3;
  
                    }
                }
                if(Alarm_Status==0x55){ //绝缘内部警告
                    FAU_IWD=1;       //绝缘检测故障
                    Bat_Fault_Code[3] |= IWD_ERR;       //绝缘检测故障
                    check_bms=SCF;         //自检失败
                    Bat_Sys_sts = 0x03;
                }
                ks_stage = 3;
            break;
            case 3:
                //判断是否连接了电池（自检应为全部断开状态）
                if((Contact_Open==1)){//(Sys_Rt.num_lecu!=OEM_Para.lecu_num)||  
                    check_bms=SCF;         //自检失败
                    ks_stage =4;
                }else{
                    ks_stage =4;
                    check_bms=SCS;    //自检成功
                } 
              
             
            
            break;
            case 4:
                ks_stage = 5;
            break;
            case 5:
                  TR_Work_mode = 0;
                  if(Bat_Sys_sts != 3){
                    Bat_Sys_sts = 4;      //待机
                    
                  }

                  TIMER_RESET_T(TIMER_TURN_ON);             
                  ks_stage = 6;
            break;
            case 6:   

            break;
    }
    
}


/* 应急投入接口                                                         */
/*****************************************************************************/
/**
 * @brief   应急投入接口
 * @return 
 * @note    检测应急条件是否满足之后投入应急使用
 */
void Emergency_Mode_Process(){
    Emergency_Exit();
    Emergency_Enter();
}
/* 低速过分相退出接口                                                         */
/*****************************************************************************/
/**
 * @brief   低速过分相退出接口
 * @return  
 * @note    检测退出低速过分相条件是否满足，满足退出低速过分相模式
 */
void Lspanto_Exit(void)
{
    //低速过分相模式退出，恢复工作状态
    if(Contact_Open == 1){
        TR_Work_mode = 0;
    }
    if((temp_tcu_data.lspanto == 0) &&
        (TR_Work_mode == 4)){
            TR_Work_mode = 0;
    }
    // //保护性质防止异常时序导致不投预充
    // if(temp_tcu_data.lspanto == 0){
    //     only_lspanto_hot_exit = 0;
    // }
}
/* 低速过分相进入接口                                                         */
/*****************************************************************************/
/**
 * @brief   低速过分相进入接口
 * @return  
 * @note    检测低速过分相进入条件是否满足，满足进入低速过分相模式
 */
void Lspanto_Enter(void)
{
    UINT32 voltage = 0;
    voltage = O_Voltage;
    if((temp_tcu_data.lspanto == 1) &&
        (KM15_OS == CLOSE)&&
        (KM10_OS == CLOSE)&&
        (Contact_Open == 0)&&
        (pre_status == 1)
    ){
            TR_Work_mode = 4;
    } 
    //加热转过分相不再不投预充
    // if(only_lspanto_hot_exit == 1){
    //     if((temp_tcu_data.mc_en == 1)&&//允许闭合正负极接触器闭合为1
    //         (temp_tcu_data.lspanto == 1)&&//低速过分相信号为1
    //         (temp_tcu_data.req_mc_cls == 1) && //请求闭合1
    //         (temp_tcu_data.req_mc_opn == 0) && //请求断开0
    //         (temp_ccu_data->cutoff == 0) && //无切除指令
    //         ((Bat_Fault_Code[14] & HEAT_MODE_STATUS) == 0) &&
    //         (KM7_KM8_OS == OPEN) && //加热为0
    //         (Contact_Open == 0)&&//无严重故障（严重故障会断接触器）
    //         ((Bat_Fault_Code[14] & PRECHG_PULSE_LOCK) == 0)&&//无预充封锁
    //         (KA11_OS == CLOSE)&&//硬线牵引变流器检修开关闭合 
    //         (temp_ccu_data->tcnv_comm == 0)&&//牵引变流器通信故障为0
    //         (KA12_OS == OPEN)&&//牵引回路切除信号为断开
    //         (((Sys_Rt.to_vol > O_Voltage) ? (Sys_Rt.to_vol - O_Voltage) : (O_Voltage - Sys_Rt.to_vol)) < 200)//内外压差<20V
    //     ){
    //         //加热转过分相 不预充直接闭合接触器
    //         KA15 = CLOSE;
    //         KA14 = OPEN;
    //         KA10 = CLOSE;
    //         only_lspanto_hot_exit = 0;
    //     }
    // }
    // else{
    //     //非加热进过分相走预充流程
    // }
}

/* 低速过分相投入接口                                                         */
/*****************************************************************************/
/**
 * @brief   低速过分相投入接口
 * @return 
 * @note    检测应急条件是否满足之后投入应急使用
 */
void Lspanto_Mode_Process(){
    Lspanto_Exit();
    Lspanto_Enter();
}
void Bat_Fault_Code_init(void)
{
    UINT8 i = 0;
    for(i=0;i<15;i++){
        Bat_Fault_Code[i] = 0;
    }

    Bat_Fault_Code[13] |=  CHG_PULSE_LOCK_REQ;

    return;
}
void Only_L1_Ltemp_falg_Judg(void){

}


/* 加热模式退出接口                                                         */
/*****************************************************************************/
/**
 * @brief   加热模式退出接口
 * @return  
 * @note    检测退出加热条件是否满足，满足退出加热模式
 */
void HOT_Exit(void)
{
    //加热退出
    if(Contact_Open == 1){
        hot_stage = 0;
        Bat_Fault_Code[14] &= ~ HEAT_MODE_STATUS;
        TIMER_RESET_T(TIMER_HOT_DELAY);
        TIMER_RESET_T(TIMER_HOT_UP_DELAY);
    }
    if(hot_stage == 2){
        // if((temp_tcu_data.lspanto == 1) &&          // 过分相条件成立
        //     (Sys_Rt.min_t < (10 + 55)) &&             // 温度未到10度
        //     (Contact_Open == 0) &&                    // 无严重故障
        //     (temp_ccu_data->cutoff == 0) &&           // 无切除指令
        //     (TBMS_Traction == 0) &&                   // 非应急模式
        //     (TBMS_Standby == 0) &&
        //     (EmemStandby == 0) &&
        //     (temp_ccu_data->tcnv_comm == 0)           // 无牵引通信故障
        //     )
        // {
        //     // 条件：只有过分相为1，其他退出条件均为0
        //     only_lspanto_hot_exit = 1;
        //     Bat_Fault_Code[14] &= ~ HEAT_MODE_STATUS;
        // }

        if((Sys_Rt.min_t >= (10 + 55)) ||//最低温度大于10度
            (Contact_Open == 1)||//有严重故障（严重故障会断接触器
            (temp_ccu_data->cutoff == 1)||//动力电池切除指令为1
            (TBMS_Traction == 1)|| //处于应急模式
            (TBMS_Standby == 1)||
            (EmemStandby == 1)||
            (temp_tcu_data.lspanto == 1)||//处于过分相模式
            (temp_ccu_data->tcnv_comm == 1)//牵引变流器通信故障为1
        ){
            
            Bat_Fault_Code[14] &= ~ HEAT_MODE_STATUS;//此时只退出加热模式，不退出加热状态，等待充电机不允许加热信号
        }

        //加热异常
        //4小时还没结束加热，退出
        if(EXPIRED_T(TIMER_HOT_DELAY, 14400000)) {
            hot_stage = 0;
            KA7 = OPEN;
            KA8 = OPEN;
            Bat_Fault_Code[14] &= ~ HEAT_MODE_STATUS;
            Bat_Fault_Code[9] |= HOT_ERR;
            TIMER_RESET_T(TIMER_HOT_DELAY);
            TIMER_RESET_T(TIMER_HOT_UP_DELAY);
        }

        //加热到15度以上还没结束
        if(Sys_Rt.min_t >= (15 + 55)){
            hot_stage = 0;
            KA7 = OPEN;
            KA8 = OPEN;
            Bat_Fault_Code[14] &= ~ HEAT_MODE_STATUS;
            Bat_Fault_Code[9] |= HOT_ERR;
            TIMER_RESET_T(TIMER_HOT_DELAY);
            TIMER_RESET_T(TIMER_HOT_UP_DELAY);
        }

        //温升小于3摄氏度每小时
        if(EXPIRED_T(TIMER_HOT_UP_DELAY, 3600000)){
            if((Sys_Rt.min_t<hot_enter_time_Ltemp)|| //加热1小时，温度提升小于3摄氏度
                ((Sys_Rt.min_t - hot_enter_time_Ltemp) < 3)){
                hot_stage = 0;
                KA7 = OPEN;
                KA8 = OPEN;
                Bat_Fault_Code[14] &= ~ HEAT_MODE_STATUS;
                Bat_Fault_Code[9] |= HOT_ERR;
                TIMER_RESET_T(TIMER_HOT_DELAY);
                TIMER_RESET_T(TIMER_HOT_UP_DELAY);
            }
            else{
                hot_enter_time_Ltemp = Sys_Rt.min_t;
                TIMER_RESET_T(TIMER_HOT_UP_DELAY);
            }
        }
        //加热过程中出现主回路接触器闭合
        if((KM14_OS == CLOSE) || (KM15_OS == CLOSE) || (KM10_OS == CLOSE)){
            hot_stage = 0;
            KA7 = OPEN;
            KA8 = OPEN;
            Bat_Fault_Code[14] &= ~ HEAT_MODE_STATUS;
            Bat_Fault_Code[9] |= HOT_ERR;
            TIMER_RESET_T(TIMER_HOT_DELAY);
            TIMER_RESET_T(TIMER_HOT_UP_DELAY);
        }
        //充电机不允许加热
        if(temp_tcu_data.heat_en == 0){
            hot_stage = 0;
            KA7 = OPEN;
            KA8 = OPEN;
            Bat_Fault_Code[14] &= ~ HEAT_MODE_STATUS;
            TIMER_RESET_T(TIMER_HOT_DELAY);
            TIMER_RESET_T(TIMER_HOT_UP_DELAY);
        }
    }
}
/* 加热模式进入接口                                                         */
/*****************************************************************************/
/**
 * @brief   加热模式进入接口
 * @return  
 * @note    检测加热进入条件是否满足，满足进入加热模式
 */
void HOT_Enter(void)
{
    if((Sys_Rt.min_t <= (5 + 55)) &&//最低温度小于等于5度
        (TBMS_Traction == 0)&& //不处于应急模式
        (TBMS_Standby == 0)&&
        (EmemStandby == 0)&&
        (temp_tcu_data.lspanto == 0)&&//不处于过分相模式
        ((Bat_Fault_Code[14] & HEAT_MODE_STATUS) == 0) &&
        (KM7_KM8_OS == OPEN) && //加热为0
        (temp_ccu_data->cutoff == 0)&&//动力电池切除指令为0
        (Contact_Open == 0)&&//无严重故障（严重故障会断接触器）
        (KA11_OS == CLOSE)&&//硬线牵引变流器检修开关闭合
        (temp_ccu_data->tcnv_comm == 0)&&//牵引变流器通信故障为0
        ((Bat_Fault_Code[13] & CHG_PULSE_LOCK_REQ) == CHG_PULSE_LOCK_REQ)&&//封锁脉冲请求为1
        ((Bat_Fault_Code[9] & HOT_ERR) == 0)//加热异常为0
    ){
        //加热模式进入
        hot_stage = 1;
        TIMER_RESET_T(TIMER_HOT_DELAY);//加热开始重置一下定时器
        TIMER_RESET_T(TIMER_HOT_UP_DELAY);
        Bat_Fault_Code[14] |= HEAT_MODE_STATUS;
        hot_enter_time_Ltemp = Sys_Rt.min_t;
        KA10 = OPEN;
        KA15 = OPEN;
    }

    switch(hot_stage){
        case 1:
            if(temp_tcu_data.heat_en == 1){
                KA7 = CLOSE;
                KA8 = CLOSE;
                hot_stage = 2;
            }
            break;
        case 2:
            break;
        default: 
            break;
    }

}
/* 加热投入接口                                                         */
/*****************************************************************************/
/**
 * @brief   加热投入接口
 * @return 
 * @note    检测加热条件是否满足之后投入加热使用
 */
void HOT_Mode_Process(void)
{
    HOT_Enter();
    HOT_Exit();
}

/*****************************************************************************/
/* 计算电池当前功率 + 可用时间预估                                            */
/* 功率：0.1kW/bit，范围 0~1000，无符号，取绝对值                             */
/* 时间：1=1min，有效范围 0~5000，超出视为无效                                */
/*****************************************************************************/
static void Calc_Bat_Power(void)
{
    SINT32 p;
    UINT32 q;
    UINT32 t;

    /* ---------- 1. 采集异常检查 ---------- */
    if(Sys_Rt.to_vol > 12000 || Sys_Rt.to_vol < 100) {   // 1200V / 10V
        Bat_Power_Tx = 0;
        Bat_Time_Tx  = 0;
        return;
    }
    if(Sys_Rt.current > 20000 || Sys_Rt.current < -20000) { // ±2000A
        Bat_Power_Tx = 0;
        Bat_Time_Tx  = 0;
        return;
    }

    /* ---------- 2. 只算放电功率，充电/静止置0 ---------- */
    if(Sys_Rt.current >= 0) {
        Bat_Power_Tx = 0;
        Bat_Time_Tx  = 0;      // 不放电，时间无效
        return;
    }

    /* ---------- 3. 功率计算 ---------- */
    p = (SINT32)Sys_Rt.to_vol * (SINT32)Sys_Rt.current;
    if(p < 0) p = -p;

    q = (UINT32)p / 10000;          // 0.1kW
    if(q > 1000) q = 1000;          // 上限 100kW
    Bat_Power_Tx = (UINT16)q;

    /* ---------- 4. 可用时间预估 ---------- */
    // Q_usable 单位 0.1kWh，Bat_Power_Tx 单位 0.1kW
    // 时间(min) = Q_usable * 60 / Bat_Power_Tx
    if(Bat_Power_Tx == 0) {
        Bat_Time_Tx = 0;            // 无功率，无效
        return;
    }

    t = (UINT32)Q_usable * 60 / Bat_Power_Tx;

    if(t > 5000) {
        Bat_Time_Tx = 0;            // 超出 5000min，视为无效
    } else {
        Bat_Time_Tx = (UINT16)t;
    }
}
#if PTL_TEST
    //测试模式接触器
    void Connect_Test(void){
        KM14_OS = KA14;
        KM15_OS = KA15;
        KM10_OS = KA10;
        if (KA7 && KA8){
            Kin_BX[0] |=  (1u << 3);
        }
        else{
            Kin_BX[0] &= ~(1u << 3); //KM7_KM8_OS = (KA7 && KA8);
        }
    }
    //预充测试初始化
    void Test_PreCharge_SetReady(void)
    {
        // 硬线
        KA10 = OPEN;
        KA15 = OPEN;
        KA14 = OPEN;
        KA7 = OPEN;
        KA8 = OPEN;
        KM10_OS     = OPEN;
        KM14_OS     = OPEN;
        KM15_OS     = OPEN;
        if (KA7 && KA8){
            Kin_BX[0] |=  (1u << 3);
        }
        else{
            Kin_BX[0] &= ~(1u << 3);
        } //KM7_KM8_OS = (KA7 && KA8);
        KA11_OS     = CLOSE;
        KA12_OS     = OPEN;

        // 故障/标志
        Contact_Open = 0;
        temp_tcu_data.req_mc_cls = 1;   //请求闭合1
        temp_tcu_data.req_mc_opn = 0;   //请求断开0
        temp_ccu_data->cutoff     = 0;   //无切除指令
        temp_ccu_data->tcnv_comm  = 0;   //牵引变流器通信故障为0
        //temp_tcu_data.mc_en      = 0;   //允许闭合正负极接触器为0
        Bat_Fault_Code[14] &= ~HEAT_MODE_STATUS;    //加热模式位清0
        Bat_Fault_Code[14] &= ~PRECHG_PULSE_LOCK;   //预充封锁位清0
        Bat_Fault_Code[13] |=  CHG_PULSE_LOCK_REQ;  //封锁脉冲请求位置1
        Contact_Open = 0;          //无严重故障
        //only_lspanto_hot_exit = 0; //加热转过分相退出为0

        // 电压
        O_Voltage     = 0;
        Sys_Rt.to_vol = 7450;   // 假设电池为额定745V

        // 计时器
        TIMER_RESET_T(TIMER_PRE_CHA_DELAY);
        TIMER_RESET_T(TIMER_PRE_CHA_KM14_DELAY);
        TIMER_RESET_T(TIMER_PRE_CHA_COUNT_DELAY);

        precharge_fail_count = 0;
        pre_cha_stage        = 0;
    }
    //预充测试
    void Test_PreCharge(void)
    {
        switch(pre_cha_stage){
            case 0:
                break;
            case 1:
                break;
            case 2:
                break;
            case 3:
                O_Voltage = 7450;
                break;
            case 4:
                O_Voltage = 7450;
                break;
            case 5:
                O_Voltage = 7450;
                break;
            case 6:
                break;
            default:
                break;
        }
    }
    //充电测试初始化
    void Test_Charge_SetReady(void)
    {
        pre_status = 1;
        KA10 = CLOSE;
        KA15 = CLOSE;
        KA14 = OPEN;
        KA7 = OPEN;
        KA8 = OPEN;
        if (KA7 && KA8){
            Kin_BX[0] |=  (1u << 3);
        }
        else{
            Kin_BX[0] &= ~(1u << 3);
        } 
        TBMS_Traction = 0; //不处于应急模式
        TBMS_Standby = 0; //不处于待机模式
        EmemStandby = 0;
        temp_tcu_data.lspanto = 0; //不处于过分相模式
        temp_ccu_data->panto_stat = 1; //处于升弓状态
        Bat_Fault_Code[14] &= ~HEAT_MODE_STATUS; //不处于加热模式
        Sys_Rt.max_v = 2530; //最大电压小于2530mv

    }
     //充电测试
    void Test_Charge(void)
    {
        switch(cha_stage){
            case 0:
                Sys_Rt.max_v = 2530;
                break;
            case 1:
                Sys_Rt.max_v = 2530;
                break;
            case 2:
                Sys_Rt.max_v = 2540;
                break;
            case 3:
                Sys_Rt.max_v = 2530;
                break;
            case 4:
                Sys_Rt.max_v = 2540;
                break;
            case 5:
                Sys_Rt.max_v = 2530;
                break;
            case 6:
                Sys_Rt.max_v = 2540;
                break;
            case 7:
                Sys_Rt.max_v = 2530;
                break;
            case 8:
                Sys_Rt.max_v = 2550;
                break;
            default:
                break;
        }
    }
    //应急模式测试初始化
    void Test_Emergency_SetReady(void)
    {
        TBMS_Traction = 1; //处于应急自走行
        // TBMS_Standby = 1; //处于应急等待模式
        // EmemStandby = 1; //救援等待模式
        Test_PreCharge_SetReady();
    }
    //应急模式测试
    void Test_Emergency(void)
    {
        if((KM15_OS == CLOSE)&&
            (KM10_OS == CLOSE)){
            if(TR_Work_mode == 2){
                TBMS_Traction = 0;
                temp_tcu_data.req_mc_opn = 1;
            }
            if(TR_Work_mode == 1){
                TBMS_Standby = 0;
                temp_tcu_data.req_mc_opn = 1;
            }
            if(TR_Work_mode == 3){
                EmemStandby = 0;
                temp_tcu_data.req_mc_opn = 1;
            }
        }
    }
    //加热模式测试初始化
    void Test_Hot_SetReady(void)
    {
        TBMS_Traction = 0; //不处于应急模式
        TBMS_Standby = 0; //不处于待机模式
        EmemStandby = 0;
        temp_tcu_data.lspanto = 0; //不处于过分相模式
        temp_ccu_data->panto_stat = 1; //处于升弓状态
    }
    //加热模式测试
    void Test_Hot(void)
    {
        switch(hot_stage){
            case 0:
                break;
            case 1:
                temp_tcu_data.heat_en = 1;
                break;
            case 2:
                if((Bat_Fault_Code[14] & HEAT_MODE_STATUS) == 0){
                    temp_tcu_data.heat_en = 0;
                }
                break;
            default:
                break;
        }
    }
    //过分相模式测试初始化
    void Test_Lspanto_SetReady(void)
    {

    }
    //过分相模式测试
    void Test_Lspanto(void)
    {
        //加热转过分相
        // switch(hot_stage){
        //     case 2:
        //         temp_tcu_data.lspanto = 1;
        //         temp_tcu_data.mc_en = 1;
        //         O_Voltage = Sys_Rt.to_vol;
        //         break;
        // }
        temp_tcu_data.lspanto = 1;
    }
#endif
/*****************************************************************************/
/* 本协议总入口                                                              */
/*****************************************************************************/ 
 void plt_main (void) {
    if (gSysTicks < 30)
    return;

    IWD_cmd = 0xff;
    //Turn_On();
    Rx_can0();
    Rx_can1();
    Tx1();
    save_sign_function(); 
 }
#endif