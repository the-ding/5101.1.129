//
// 通过OEM决定是否选择: L3, L4
//
#include "ptl.h"

//
// 每一种参数都配有1字节的辅助参数：持续时间(秒)
//
#define L1_BASE             1000
#define L2_BASE             2000
#define L3_BASE             3000
#define L4_BASE             4000
#define OEM_BASE            10000   // 保留5000个参数


//一级报警参数 (严重故障)
#define L1_VOL_HIGH         (L1_BASE +  0)  // 单体电压高 [2B电压值(1mV)]
#define L1_VOL_LOW          (L1_BASE +  1)  // 单体电压低
#define L1_VOL_UNBA         (L1_BASE +  2)  // 单体电压不均衡
#define L1_TVOL_HIGH        (L1_BASE +  3)  // 总电压高 [2B电压值(0.1V)]
#define L1_TVOL_LOW         (L1_BASE +  4)  // 总电压低 [2B电压值(0.1V)]
#define L1_TMP_HIGH         (L1_BASE +  5)  // 单体过温 [1B(+40度)]
#define L1_TMP_LOW          (L1_BASE +  6)  // 单体欠温
#define L1_TMP_UNBA         (L1_BASE +  7)  // 单体温度不均衡

//#ifdef HAVE_L1_EP_TMP_HIGH
#define L1_EP_TMP_HIGH      (L1_BASE +  8)  // 端子温度过温
//#endif
//#ifdef HAVE_L1_EP_TMP_LOW
#define L1_EP_TMP_LOW       (L1_BASE +  9)  // 端子温度欠温
//#endif
#ifdef HAVE_L1_EP_TMP_UNBA
#define L1_EP_TMP_UNBA      (L1_BASE + 10)  // 端子温度不均衡
#endif

#define L1_DISCHG_CUR_HIGH  (L1_BASE + 11)  // 放电过流 [2B电流值(0.1A)]
#define L1_CHG_CUR_HIGH     (L1_BASE + 12)  // 充电过流
#define L1_CHG_TVOL_HIGH    (L1_BASE + 13)  // 充电总电压过高
#define L1_REGEN_CUR_HIGH   (L1_BASE + 14)  // 回收过流
#define L1_SOC_HIGH         (L1_BASE + 15)  // SOC高
#define L1_SOC_LOW          (L1_BASE + 16)  // SOC低
#define L1_ISO_LOW          (L1_BASE + 17)  // 绝缘低


// 二级报警参数 (故障)
#define L2_VOL_HIGH         (L2_BASE +  0)  // 单体电压高 [2B电压值(1mV)]
#define L2_VOL_LOW          (L2_BASE +  1)  // 单体电压低
#define L2_VOL_UNBA         (L2_BASE +  2)  // 单体电压不均衡
#define L2_TVOL_HIGH        (L2_BASE +  3)  // 总电压高
#define L2_TVOL_LOW         (L2_BASE +  4)  // 总电压低
#define L2_TMP_HIGH         (L2_BASE +  5)  // 单体过温 [1B(+40度)]
#define L2_TMP_LOW          (L2_BASE +  6)  // 单体欠温
#define L2_TMP_UNBA         (L2_BASE +  7)  // 单体温度不均衡

//#ifdef HAVE_L2_EP_TMP_HIGH
#define L2_EP_TMP_HIGH      (L2_BASE +  8)  // 端子温度过温
//#endif
//#ifdef HAVE_L2_EP_TMP_LOW
#define L2_EP_TMP_LOW       (L2_BASE +  9)  // 端子温度欠温
//#endif
#ifdef HAVE_L2_EP_TMP_UNBA
#define L2_EP_TMP_UNBA      (L2_BASE + 10)  // 端子温度不均衡
#endif

#define L2_DISCHG_CUR_HIGH  (L2_BASE + 11)  // 放电过流 [2B电流值(0.1A)]
#define L2_CHG_CUR_HIGH     (L2_BASE + 12)  // 充电过流
#define L2_CHG_TVOL_HIGH    (L2_BASE + 13)  // 充电总电压过高
#define L2_REGEN_CUR_HIGH   (L2_BASE + 14)  // 回收过流
#define L2_SOC_HIGH         (L2_BASE + 15)  // SOC高
#define L2_SOC_LOW          (L2_BASE + 16)  // SOC低
#define L2_ISO_LOW          (L2_BASE + 17)  // 绝缘低


// 三级报警参数 (异常)
//#ifdef HAVE_L3_PARAMS
#define L3_VOL_HIGH         (L3_BASE +  0)  // 单体电压高 [2B电压值(1mV)]
#define L3_VOL_LOW          (L3_BASE +  1)  // 单体电压低
#define L3_VOL_UNBA         (L3_BASE +  2)  // 单体电压不均衡
#define L3_TVOL_HIGH        (L3_BASE +  3)  // 总电压高
#define L3_TVOL_LOW         (L3_BASE +  4)  // 总电压低
#define L3_TMP_HIGH         (L3_BASE +  5)  // 单体过温 [1B(+40度)]
#define L3_TMP_LOW          (L3_BASE +  6)  // 单体欠温
#define L3_TMP_UNBA         (L3_BASE +  7)  // 单体温度不均衡

//#ifdef HAVE_L3_EP_TMP_HIGH
#define L3_EP_TMP_HIGH      (L3_BASE +  8)  // 端子温度过温
//#endif
//#ifdef HAVE_L3_EP_TMP_LOW
#define L3_EP_TMP_LOW       (L3_BASE +  9)  // 端子温度欠温
//#endif
#ifdef HAVE_L3_EP_TMP_UNBA
#define L3_EP_TMP_UNBA      (L3_BASE + 10)  // 端子温度不均衡
#endif

#define L3_DISCHG_CUR_HIGH  (L3_BASE + 11)  // 放电过流 [2B电流值(0.1A)]
#define L3_CHG_CUR_HIGH     (L3_BASE + 12)  // 充电过流
#define L3_CHG_TVOL_HIGH    (L3_BASE + 13)  // 充电总电压过高
#define L3_REGEN_CUR_HIGH   (L3_BASE + 14)  // 回收过流
#define L3_SOC_HIGH         (L3_BASE + 15)  // SOC高
#define L3_SOC_LOW          (L3_BASE + 16)  // SOC低
#define L3_ISO_LOW          (L3_BASE + 17)  // 绝缘低
//#endif

// 四级报警参数 (轻微异常)
//#ifdef HAVE_L4_PARAMS
#define L4_VOL_HIGH         (L4_BASE +  0)  // 单体电压高 [2B电压值(1mV)]
#define L4_VOL_LOW          (L4_BASE +  1)  // 单体电压低
#define L4_VOL_UNBA         (L4_BASE +  2)  // 单体电压不均衡
#define L4_TVOL_HIGH        (L4_BASE +  3)  // 总电压高
#define L4_TVOL_LOW         (L4_BASE +  4)  // 总电压低
#define L4_TMP_HIGH         (L4_BASE +  5)  // 单体过温 [1B(+40度)]
#define L4_TMP_LOW          (L4_BASE +  6)  // 单体欠温
#define L4_TMP_UNBA         (L4_BASE +  7)  // 单体温度不均衡

//#ifdef HAVE_L4_EP_TMP_HIGH
#define L4_EP_TMP_HIGH      (L4_BASE +  8)  // 端子温度过温
//#endif
//#ifdef HAVE_L4_EP_TMP_LOW
#define L4_EP_TMP_LOW       (L4_BASE +  9)  // 端子温度欠温
//#endif
#ifdef HAVE_L4_EP_TMP_UNBA
#define L4_EP_TMP_UNBA      (L4_BASE + 10)  // 端子温度不均衡
#endif

#define L4_DISCHG_CUR_HIGH  (L4_BASE + 11)  // 放电过流 [2B电流值(0.1A)]
#define L4_CHG_CUR_HIGH     (L4_BASE + 12)  // 充电过流
#define L4_CHG_TVOL_HIGH    (L4_BASE + 13)  // 充电总电压过高
#define L4_REGEN_CUR_HIGH   (L4_BASE + 14)  // 回收过流
#define L4_SOC_HIGH         (L4_BASE + 15)  // SOC高
#define L4_SOC_LOW          (L4_BASE + 16)  // SOC低
#define L4_ISO_LOW          (L4_BASE + 17)  // 绝缘低
//#endif



#ifdef HAVE_WX_PARAMS
#define WX_VMAX             (OEM_BASE + 1)
#define WX_VMIN             (OEM_BASE + 2)
#define WX_HT_HT            (OEM_BASE + 3)
#define WX_CHG_PT           (OEM_BASE + 4)
#define WX_HT_LV            (OEM_BASE + 5)
#define WX_REHT_V           (OEM_BASE + 6)
#define WX_BLNC_CTR         (OEM_BASE + 7)
#define WX_BLNC_OPENV       (OEM_BASE + 8)
#define WX_BLNC_ENDTMP      (OEM_BASE + 9)
#define WX_FAN_AVGTMP       (OEM_BASE + 10)
#define WX_FAN_DETATMP      (OEM_BASE + 11)
#define WX_BATT_SORT        (OEM_BASE + 12)
#define WX_ADD_MLT          (OEM_BASE + 13)
#define WX_CUR_TYPE         (OEM_BASE + 14)
#define WX_VOLT_CRR1        (OEM_BASE + 15)
#define WX_VOLT_CRR2        (OEM_BASE + 16)
#define WX_CUR_CRR          (OEM_BASE + 17)
#define WX_VOLT_TYPE        (OEM_BASE + 18)
#endif

// 参数格式
#define PF_TMP              1   // 1B 温度: +40
#define PF_01mV             2   // 2B 电压: 0.1mV
#define PF_1mV              3   // 2B 电压: 1mV
#define PF_10mV             4
#define PF_100mV            5
#define PF_1V               6
#define PF_10V              7
#define PF_100V             8
#define PF_1kV              9
#define PF_01mA             10  // 2B 电流: 0.1mA
#define PF_1mA              11  // 2B 电流: 1mA
#define PF_10mA             12
#define PF_100mA            13
#define PF_1A               14
#define PF_10A              15
#define PF_100A             16
#define PF_1kA              17
#define PF_AH               18  // 2B 安时
#define PF_PERCENT          19  // 1B SOC: %
#define PF_01PERCENT        20  // 2B SOC: 0.1%
#define PF_001PERCENT       21  // 2B SOC: 0.01%
#define PF_TMP2             22  // 1B 温度: +0
#define PF_OV               23  // 2B 绝缘电阻: Ω/V
#define PF_OM               24  // 2B Ω
#define PF_kO               25  // 2B KΩ

// 未细分参数格式
#define PF_RAW1             101 // 1字节参数
#define PF_RAW2             102 // 2字节参数
#define PF_RAW3             103 // ...
#define PF_RAW4             104
#define PF_RAW5             105
#define PF_RAW6             106
#define PF_RAW7             107
#define PF_RAW8             108
#define PF_RAW9             109
#define PF_RAW10            110
#define PF_RAW11            111
#define PF_RAW12            112
#define PF_RAW13            113
#define PF_RAW14            114
#define PF_RAW15            115
#define PF_RAW16            116
#define PF_RAW17            117
#define PF_RAW18            118
#define PF_RAW19            119
#define PF_RAW20            120
#define PF_RAW21            121
#define PF_RAW22            122
#define PF_RAW23            123
#define PF_RAW24            124
#define PF_RAW25            125
#define PF_RAW26            126
#define PF_RAW27            127
#define PF_RAW28            128
#define PF_RAW29            129
#define PF_RAW30            130
#define PF_RAW31            131
#define PF_RAW32            132

