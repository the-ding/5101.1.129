typedef struct
{
    UINT16      size;
    UINT16      chksum;

//一级报警参数 (严重故障)
    struct
    {
//#ifdef L1_VOL_HIGH
        UINT16      vol_high;
        UINT8       vol_high_keep;
//#endif

//#ifdef L1_VOL_LOW
        UINT16      vol_low;
        UINT8       vol_low_keep;
//#endif

//#ifdef L1_VOL_UNBA
        UINT16      vol_unba;
        UINT8       vol_unba_keep;
//#endif

//#ifdef L1_TVOL_HIGH
        UINT16      tvol_high;
        UINT8       tvol_high_keep;
//#endif

//#ifdef L1_TVOL_LOW
        UINT16      tvol_low;
        UINT8       tvol_low_keep;
//#endif

//#ifdef L1_TMP_HIGH
        UINT8       tmp_high;
        UINT8       tmp_high_keep;
//#endif

//#ifdef L1_TMP_LOW
        UINT8       tmp_low;
        UINT8       tmp_low_keep;
//#endif

//#ifdef L1_TMP_UNBA
        UINT8       tmp_unba;
        UINT8       tmp_unba_keep;
//#endif

//#ifdef L1_EP_TMP_HIGH
        UINT8       ep_tmp_high;
        UINT8       ep_tmp_high_keep;
//#endif

//#ifdef L1_EP_TMP_LOW
        UINT8       ep_tmp_low;
        UINT8       ep_tmp_low_keep;
//#endif

#ifdef L1_EP_TMP_UNBA
        UINT8       ep_tmp_unba;
        UINT8       ep_tmp_unba_keep;
#endif

//#ifdef L1_DISCHG_CUR_HIGH
        UINT16      dischg_cur_high;
        UINT8       dischg_cur_high_keep;
//#endif

//#ifdef L1_CHG_CUR_HIGH
        UINT16      chg_cur_high;
        UINT8       chg_cur_high_keep;
//#endif

//#ifdef L1_CHG_TVOL_HIGH
        UINT16      chg_tvol_high;
        UINT8       chg_tvol_high_keep;
//#endif

//#ifdef L1_REGEN_CUR_HIGH
        UINT16      regen_cur_high;
        UINT8       regen_cur_high_keep;
//#endif

//#ifdef L1_SOC_HIGH
        UINT8       soc_high;
        UINT8       soc_high_keep;
//#endif

//#ifdef L1_SOC_LOW
        UINT8       soc_low;
        UINT8       soc_low_keep;
//#endif

//#ifdef L1_ISO_LOW
        UINT16      iso_low;
        UINT8       iso_low_keep;
//#endif
    }L1;


//二级报警参数 (故障)
    struct
    {
//#ifdef L2_VOL_HIGH
        UINT16      vol_high;
        UINT8       vol_high_keep;
//#endif

//#ifdef L2_VOL_LOW
        UINT16      vol_low;
        UINT8       vol_low_keep;
//#endif

//#ifdef L2_VOL_UNBA
        UINT16      vol_unba;
        UINT8       vol_unba_keep;
//#endif

//#ifdef L2_TVOL_HIGH
        UINT16      tvol_high;
        UINT8       tvol_high_keep;
//#endif

//#ifdef L2_TVOL_LOW
        UINT16      tvol_low;
        UINT8       tvol_low_keep;
//#endif

//#ifdef L2_TMP_HIGH
        UINT8       tmp_high;
        UINT8       tmp_high_keep;
//#endif

//#ifdef L2_TMP_LOW
        UINT8       tmp_low;
        UINT8       tmp_low_keep;
//#endif

//#ifdef L2_TMP_UNBA
        UINT8       tmp_unba;
        UINT8       tmp_unba_keep;
//#endif

//#ifdef L2_EP_TMP_HIGH
        UINT8       ep_tmp_high;
        UINT8       ep_tmp_high_keep;
//#endif

//#ifdef L2_EP_TMP_LOW
        UINT8       ep_tmp_low;
        UINT8       ep_tmp_low_keep;
//#endif

#ifdef L2_EP_TMP_UNBA
        UINT8       ep_tmp_unba;
        UINT8       ep_tmp_unba_keep;
#endif

//#ifdef L2_DISCHG_CUR_HIGH
        UINT16      dischg_cur_high;
        UINT8       dischg_cur_high_keep;
//#endif

//#ifdef L2_CHG_CUR_HIGH
        UINT16      chg_cur_high;
        UINT8       chg_cur_high_keep;
//#endif

//#ifdef L2_CHG_TVOL_HIGH
        UINT16      chg_tvol_high;
        UINT8       chg_tvol_high_keep;
//#endif

//#ifdef L2_REGEN_CUR_HIGH
        UINT16      regen_cur_high;
        UINT8       regen_cur_high_keep;
//#endif

//#ifdef L2_SOC_HIGH
        UINT8       soc_high;
        UINT8       soc_high_keep;
//#endif

//#ifdef L2_SOC_LOW
        UINT8       soc_low;
        UINT8       soc_low_keep;
//#endif

//#ifdef L2_ISO_LOW
        UINT16      iso_low;
        UINT8       iso_low_keep;
//#endif
    }L2;


//三级报警参数 (异常)
//#ifdef HAVE_L3_PARAMS
    struct
    {
//#ifdef L3_VOL_HIGH
        UINT16      vol_high;
        UINT8       vol_high_keep;
//#endif

//#ifdef L3_VOL_LOW
        UINT16      vol_low;
        UINT8       vol_low_keep;
//#endif

//#ifdef L3_VOL_UNBA
        UINT16      vol_unba;
        UINT8       vol_unba_keep;
//#endif

//#ifdef L3_TVOL_HIGH
        UINT16      tvol_high;
        UINT8       tvol_high_keep;
//#endif

//#ifdef L3_TVOL_LOW
        UINT16      tvol_low;
        UINT8       tvol_low_keep;
//#endif

//#ifdef L3_TMP_HIGH
        UINT8       tmp_high;
        UINT8       tmp_high_keep;
//#endif

//#ifdef L3_TMP_LOW
        UINT8       tmp_low;
        UINT8       tmp_low_keep;
//#endif

//#ifdef L3_TMP_UNBA
        UINT8       tmp_unba;
        UINT8       tmp_unba_keep;
//#endif

//#ifdef L3_EP_TMP_HIGH
        UINT8       ep_tmp_high;
        UINT8       ep_tmp_high_keep;
//#endif

//#ifdef L3_EP_TMP_LOW
        UINT8       ep_tmp_low;
        UINT8       ep_tmp_low_keep;
//#endif

#ifdef L3_EP_TMP_UNBA
        UINT8       ep_tmp_unba;
        UINT8       ep_tmp_unba_keep;
#endif

//#ifdef L3_DISCHG_CUR_HIGH
        UINT16      dischg_cur_high;
        UINT8       dischg_cur_high_keep;
//#endif

//#ifdef L3_CHG_CUR_HIGH
        UINT16      chg_cur_high;
        UINT8       chg_cur_high_keep;
//#endif

//#ifdef L3_CHG_TVOL_HIGH
        UINT16      chg_tvol_high;
        UINT8       chg_tvol_high_keep;
//#endif

//#ifdef L3_REGEN_CUR_HIGH
        UINT16      regen_cur_high;
        UINT8       regen_cur_high_keep;
//#endif

//#ifdef L3_SOC_HIGH
        UINT8       soc_high;
        UINT8       soc_high_keep;
//#endif

//#ifdef L3_SOC_LOW
        UINT8       soc_low;
        UINT8       soc_low_keep;
//#endif

//#ifdef L3_ISO_LOW
        UINT16      iso_low;
        UINT8       iso_low_keep;
//#endif
    }L3;
//#endif


//四级报警参数 (轻微异常)
//#ifdef HAVE_L4_PARAMS
    struct
    {
//#ifdef L4_VOL_HIGH
        UINT16      vol_high;
        UINT8       vol_high_keep;
//#endif

//#ifdef L4_VOL_LOW
        UINT16      vol_low;
        UINT8       vol_low_keep;
//#endif

//#ifdef L4_VOL_UNBA
        UINT16      vol_unba;
        UINT8       vol_unba_keep;
//#endif

//#ifdef L4_TVOL_HIGH
        UINT16      tvol_high;
        UINT8       tvol_high_keep;
//#endif

//#ifdef L4_TVOL_LOW
        UINT16      tvol_low;
        UINT8       tvol_low_keep;
//#endif

//#ifdef L4_TMP_HIGH
        UINT8       tmp_high;
        UINT8       tmp_high_keep;
//#endif

//#ifdef L4_TMP_LOW
        UINT8       tmp_low;
        UINT8       tmp_low_keep;
//#endif

//#ifdef L4_TMP_UNBA
        UINT8       tmp_unba;
        UINT8       tmp_unba_keep;
//#endif

//#ifdef L4_EP_TMP_HIGH
        UINT8       ep_tmp_high;
        UINT8       ep_tmp_high_keep;
//#endif

//#ifdef L4_EP_TMP_LOW
        UINT8       ep_tmp_low;
        UINT8       ep_tmp_low_keep;
//#endif

#ifdef L4_EP_TMP_UNBA
        UINT8       ep_tmp_unba;
        UINT8       ep_tmp_unba_keep;
#endif

//#ifdef L4_DISCHG_CUR_HIGH
        UINT16      dischg_cur_high;
        UINT8       dischg_cur_high_keep;
//#endif

//#ifdef L4_CHG_CUR_HIGH
        UINT16      chg_cur_high;
        UINT8       chg_cur_high_keep;
//#endif

//#ifdef L4_CHG_TVOL_HIGH
        UINT16      chg_tvol_high;
        UINT8       chg_tvol_high_keep;
//#endif

//#ifdef L4_REGEN_CUR_HIGH
        UINT16      regen_cur_high;
        UINT8       regen_cur_high_keep;
//#endif

//#ifdef L4_SOC_HIGH
        UINT8       soc_high;
        UINT8       soc_high_keep;
//#endif

//#ifdef L4_SOC_LOW
        UINT8       soc_low;
        UINT8       soc_low_keep;
//#endif

//#ifdef L4_ISO_LOW
        UINT16      iso_low;
        UINT8       iso_low_keep;
//#endif
    }L4;
//#endif

#ifdef HAVE_WX_PARAMS
    struct {
        UINT8       vmin;//最小soc对应电压
	    UINT8       vmax;//最大soc对应电压
        UINT16      ht_ht;//加热过温保护阈值
	    UINT16      chg_pt;//充电低温保护阈值
	    UINT16      ht_lv;//加热欠压保护阈值
	    UINT16      reht_v;//加热欠压释放阈值
	    UINT8       blnc_ctr;//均衡控制开关(开\关)
	    UINT8       blnc_openv;//均衡开启电压
	    UINT8       blnc_endtmp;//均衡终止温度
	    UINT8       fan_avgtmp;//风扇开启时高于平均温度值
	    UINT8       fan_detatmp;//风扇开启时温度差异值
	    UINT8       batt_sort;//电池种类
	    UINT8       add_mlt;//MCU累加倍数
	    UINT8       cur_type;//电流传感器类型
	    UINT8       volt_crr1;//电压校正值1
	    UINT8       volt_crr2;//电压校正值2
	    UINT8       cur_crr;//总电流校正值
	    UINT8       volt_type;//总电压选择方式
    } WX;
#endif
} DBAlarmPara;

extern DBAlarmPara  Sys_Alarm_Para;

void Sys_Alarm_Para_Save (void);
