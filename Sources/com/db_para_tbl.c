#include <string.h>
#include <stddef.h>
#include "sys.h"
#include "db.h"
#include "db_ev.h"
#include "db_para_cfg.h"
#include "db_para_struct.h"

#define _(x,y)   (x) >> 8, (x) & 0xff, y,

//
// 注意：一定要与db_para_struct.h中保持相同的次序
//
const UINT8 para_tbl[] =
{
//一级报警参数 (严重故障)
//#ifdef L1_VOL_HIGH
    _(L1_VOL_HIGH, PF_1mV)
//#endif

//#ifdef L1_VOL_LOW
    _(L1_VOL_LOW, PF_1mV)
//#endif

//#ifdef L1_VOL_UNBA
    _(L1_VOL_UNBA, PF_1mV)
//#endif

//#ifdef L1_TVOL_HIGH
    _(L1_TVOL_HIGH, PF_100mV)
//#endif

//#ifdef L1_TVOL_LOW
    _(L1_TVOL_LOW, PF_100mV)
//#endif

//#ifdef L1_TMP_HIGH
    _(L1_TMP_HIGH, PF_TMP)
//#endif

//#ifdef L1_TMP_LOW
    _(L1_TMP_LOW, PF_TMP)
//#endif

//#ifdef L1_TMP_UNBA
    _(L1_TMP_UNBA, PF_TMP2)
//#endif

//#ifdef L1_EP_TMP_HIGH
    _(L1_EP_TMP_HIGH, PF_TMP)
//#endif

//#ifdef L1_EP_TMP_LOW
    _(L1_EP_TMP_LOW, PF_TMP)
//#endif

#ifdef L1_EP_TMP_UNBA
    _(L1_EP_TMP_UNBA, PF_TMP2)
#endif

//#ifdef L1_DISCHG_CUR_HIGH
    _(L1_DISCHG_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L1_CHG_CUR_HIGH
    _(L1_CHG_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L1_CHG_TVOL_HIGH
    _(L1_CHG_TVOL_HIGH, PF_100mV)
//#endif

//#ifdef L1_REGEN_CUR_HIGH
    _(L1_REGEN_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L1_SOC_HIGH
    _(L1_SOC_HIGH, PF_PERCENT)
//#endif

//#ifdef L1_SOC_LOW
    _(L1_SOC_LOW, PF_PERCENT)
//#endif

//#ifdef L1_ISO_LOW
    _(L1_ISO_LOW, PF_OV)
//#endif


//二级报警参数 (故障)
//#ifdef L2_VOL_HIGH
    _(L2_VOL_HIGH, PF_1mV)
//#endif

//#ifdef L2_VOL_LOW
    _(L2_VOL_LOW, PF_1mV)
//#endif

//#ifdef L2_VOL_UNBA
    _(L2_VOL_UNBA, PF_1mV)
//#endif

//#ifdef L2_TVOL_HIGH
    _(L2_TVOL_HIGH, PF_100mV)
//#endif

//#ifdef L2_TVOL_LOW
    _(L2_TVOL_LOW, PF_100mV)
//#endif

//#ifdef L2_TMP_HIGH
    _(L2_TMP_HIGH, PF_TMP)
//#endif

//#ifdef L2_TMP_LOW
    _(L2_TMP_LOW, PF_TMP)
//#endif

//#ifdef L2_TMP_UNBA
    _(L2_TMP_UNBA, PF_TMP2)
//#endif

//#ifdef L2_EP_TMP_HIGH
    _(L2_EP_TMP_HIGH, PF_TMP)
//#endif

//#ifdef L2_EP_TMP_LOW
    _(L2_EP_TMP_LOW, PF_TMP)
//#endif

#ifdef L2_EP_TMP_UNBA
    _(L2_EP_TMP_UNBA, PF_TMP2)
#endif

//#ifdef L2_DISCHG_CUR_HIGH
    _(L2_DISCHG_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L2_CHG_CUR_HIGH
    _(L2_CHG_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L2_CHG_TVOL_HIGH
    _(L2_CHG_TVOL_HIGH, PF_100mV)
//#endif

//#ifdef L2_REGEN_CUR_HIGH
    _(L2_REGEN_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L2_SOC_HIGH
    _(L2_SOC_HIGH, PF_PERCENT)
//#endif

//#ifdef L2_SOC_LOW
    _(L2_SOC_LOW, PF_PERCENT)
//#endif

//#ifdef L2_ISO_LOW
    _(L2_ISO_LOW, PF_OV)
//#endif


//三级报警参数 (异常)
//#ifdef L3_VOL_HIGH
    _(L3_VOL_HIGH, PF_1mV)
//#endif

//#ifdef L3_VOL_LOW
    _(L3_VOL_LOW, PF_1mV)
//#endif

//#ifdef L3_VOL_UNBA
    _(L3_VOL_UNBA, PF_1mV)
//#endif

//#ifdef L3_TVOL_HIGH
    _(L3_TVOL_HIGH, PF_100mV)
//#endif

//#ifdef L3_TVOL_LOW
    _(L3_TVOL_LOW, PF_100mV)
//#endif

//#ifdef L3_TMP_HIGH
    _(L3_TMP_HIGH, PF_TMP)
//#endif

//#ifdef L3_TMP_LOW
    _(L3_TMP_LOW, PF_TMP)
//#endif

//#ifdef L3_TMP_UNBA
    _(L3_TMP_UNBA, PF_TMP2)
//#endif

//#ifdef L3_EP_TMP_HIGH
    _(L3_EP_TMP_HIGH, PF_TMP)
//#endif

//#ifdef L3_EP_TMP_LOW
    _(L3_EP_TMP_LOW, PF_TMP)
//#endif

#ifdef L3_EP_TMP_UNBA
    _(L3_EP_TMP_UNBA, PF_TMP2)
#endif

//#ifdef L3_DISCHG_CUR_HIGH
    _(L3_DISCHG_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L3_CHG_CUR_HIGH
    _(L3_CHG_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L3_CHG_TVOL_HIGH
    _(L3_CHG_TVOL_HIGH, PF_100mV)
//#endif

//#ifdef L3_REGEN_CUR_HIGH
    _(L3_REGEN_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L3_SOC_HIGH
    _(L3_SOC_HIGH, PF_PERCENT)
//#endif

//#ifdef L3_SOC_LOW
    _(L3_SOC_LOW, PF_PERCENT)
//#endif

//#ifdef L3_ISO_LOW
    _(L3_ISO_LOW, PF_OV)
//#endif


//四级报警参数 (轻微异常)
//#ifdef L4_VOL_HIGH
    _(L4_VOL_HIGH, PF_1mV)
//#endif

//#ifdef L4_VOL_LOW
    _(L4_VOL_LOW, PF_1mV)
//#endif

//#ifdef L4_VOL_UNBA
    _(L4_VOL_UNBA, PF_1mV)
//#endif

//#ifdef L4_TVOL_HIGH
    _(L4_TVOL_HIGH, PF_100mV)
//#endif

//#ifdef L4_TVOL_LOW
    _(L4_TVOL_LOW, PF_100mV)
//#endif

//#ifdef L4_TMP_HIGH
    _(L4_TMP_HIGH, PF_TMP)
//#endif

//#ifdef L4_TMP_LOW
    _(L4_TMP_LOW, PF_TMP)
//#endif

//#ifdef L4_TMP_UNBA
    _(L4_TMP_UNBA, PF_TMP2)
//#endif

//#ifdef L4_EP_TMP_HIGH
    _(L4_EP_TMP_HIGH, PF_TMP)
//#endif

//#ifdef L4_EP_TMP_LOW
    _(L4_EP_TMP_LOW, PF_TMP)
//#endif

#ifdef L4_EP_TMP_UNBA
    _(L4_EP_TMP_UNBA, PF_TMP2)
#endif

//#ifdef L4_DISCHG_CUR_HIGH
    _(L4_DISCHG_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L4_CHG_CUR_HIGH
    _(L4_CHG_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L4_CHG_TVOL_HIGH
    _(L4_CHG_TVOL_HIGH, PF_100mV)
//#endif

//#ifdef L4_REGEN_CUR_HIGH
    _(L4_REGEN_CUR_HIGH, PF_100mA)
//#endif

//#ifdef L4_SOC_HIGH
    _(L4_SOC_HIGH, PF_PERCENT)
//#endif

//#ifdef L4_SOC_LOW
    _(L4_SOC_LOW, PF_PERCENT)
//#endif

//#ifdef L4_ISO_LOW
    _(L4_ISO_LOW, PF_OV)
//#endif
 // _(CUR_L1, PF_100mA)

#ifdef HAVE_WX_PARAMS
    _(WX_VMAX, PF_RAW1)
    _(WX_VMIN, PF_RAW1)
    _(WX_HT_HT, PF_RAW2)
    _(WX_CHG_PT, PF_RAW2)
    _(WX_HT_LV, PF_RAW2)
    _(WX_REHT_V, PF_RAW2)
    _(WX_BLNC_CTR, PF_RAW1)
    _(WX_BLNC_OPENV, PF_RAW1)
    _(WX_BLNC_ENDTMP, PF_RAW1)
    _(WX_FAN_AVGTMP, PF_RAW1)
    _(WX_FAN_DETATMP, PF_RAW1)
    _(WX_BATT_SORT, PF_RAW1)
    _(WX_ADD_MLT, PF_RAW1)
    _(WX_CUR_TYPE, PF_RAW1)
    _(WX_VOLT_CRR1, PF_RAW1)
    _(WX_VOLT_CRR2, PF_RAW1)
    _(WX_CUR_CRR, PF_RAW1)
    _(WX_VOLT_TYPE, PF_RAW1)
#endif
};

const UINT16    para_tbl_size = sizeof(para_tbl)/* / 3*/;

DBAlarmPara  Sys_Alarm_Para;
