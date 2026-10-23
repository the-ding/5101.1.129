/*参数在铁电RAM中的起始偏移地址 */


#define FLASH_WINFO_START   (0x0)                           /*FLASH写地址信息*/            //0x0
#define LECU_PARA_START     (FLASH_WINFO_START + 32)        /*采集板参数起始地址*/         //0x20
#define SYSPARA_START       (LECU_PARA_START + DEV_MAX*128) /*电池及车辆部分参数存储地址*/ //0xA20                           
#define OEMPARA_START       (SYSPARA_START + 100)                                          //0xA84
#define SOCINFO_START       (OEMPARA_START + 28)            /*OEM参数存储地址*/            //0xAA0
#define BATTPARA_START      (SOCINFO_START + 128)           /*SOC信息存储地址*/            //0xB20
#define RESERVD_START       (BATTPARA_START + 64)                                          //0xB60  保存的地址空间最大6720bytes
#define SOH_NUML            (RESERVD_START + 32) 
#define SOH_NUMH            (SOH_NUML + 16) 
#define SOH_DATAL           (SOH_NUMH + 16) 
#define SOH_DATAH           (SOH_DATAL + 256) 
#define DAY_INDEX_START     (0x25A0)







#define FG_CHG_START        (FLA_INDEX_START - 32)
#define FLA_INDEX_START     (SAVE_DOWN_START - FLA_INDEX_MAX*FLA_INDEX_SIZE)/*Flash报警记录索引在铁电中的保存地址*/
#define SAVE_DOWN_START     (SAVE_176_DATA - 32)
#define SAVE_176_DATA       (ALARM_PARA_START - 32)
#define ALARM_PARA_START    (ALARM_PWDN_START - 960)         /*订制参数*/
#define ALARM_PWDN_START    (ALARM_PTR_START - 16)            /*停电时刻*/
#define ALARM_PTR_START     (ALARM_PTR2_START - 32)           /*事件池记录指针*/
#define ALARM_PTR2_START    (ALARM_REC_START - 32)            /*事件池记录指针(备份)*/
#define ALARM_REC_START     (FM_SIZE - ALARM_REC_NUM*16)      /*事件池(写满后统一导入FLASH)*/
#define ALARM_REC_NUM       150
#define ALARM_FULL_NUM      80

#define FM_SIZE             (32 * 1024)                       /* <64Kb, 不用UL后缀 */

/*****************************************************************************/
/* 铁电RAM读操作                                                             */
/*****************************************************************************/
BOOL  Fm_read (UINT16 addr, void *buf, UINT16 bytes);

/*****************************************************************************/
/* 铁电RAM写操作                                                             */
/*****************************************************************************/
BOOL  Fm_write (UINT16 addr, const void *buf, UINT16 bytes);
