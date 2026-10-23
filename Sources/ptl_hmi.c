#include <string.h>
#include "sys.h"
#include "drv_sci.h"
#include "drv_rtc.h"
#include "drv_fm.h"
#include "drv_can.h"
#include "db.h"
#include "ptl.h"
#include "modbus.h"



#if PTL_HMI

/*协议全局变量*/
ModbusFrame     HmiBuf;


UINT8 HRT_T485=0;
UINT8 HRT_R485=0;
UINT8    Com_485_Err=0; 

#pragma DATA_SEG SHARED_DATA
extern UINT8    LECU_Work;            /*工作阶段：(1)第一次请求SN列表，(2)第二次请求SN列表，(0)正常工作*/
extern UINT8    zbuf[];               /*临时数组*/

extern DBDevPara        TempDevPara;
extern volatile UINT8   TempDevGot;
#pragma DATA_SEG DEFAULT

/*#if PTL_ZBDRGJ
extern UINT8 mod_Cha;
UINT8 hmi_stopcha = 0;
#endif*/


/*****************************************************************************/
/* 彩屏通讯协议初始化函数                                                    */
/*****************************************************************************/
void  HMI_Init (void)
{
    Modbus_init (&HmiBuf);
}

/*****************************************************************************/
/* 彩屏通讯协议 - 发送MODBUS回复帧                                           */
/*****************************************************************************/
static void HMI_Reply (UINT16 reg, UINT16 regs)
{
    UINT16  bytes = regs*2;
    UINT16  crc;

    if (bytes > 0xff)
        return;

    HmiBuf.buf[2] = (UINT8)bytes;        /*回复字节数*/

    (void) memcpy (&HmiBuf.buf[3], zbuf + reg*2, bytes);

    crc = Modbus_crc (HmiBuf.buf, 3 + bytes);

    HmiBuf.buf[3 + bytes] = crc >> 8;
    HmiBuf.buf[4 + bytes] = crc & 0xFF;

    SCI_write (HmiBuf.buf, 5 + bytes);  /*发送*/
}

/*****************************************************************************/
/* 彩屏通讯协议 - MODBUS读功能                                               */
/*****************************************************************************/
static void HMI_Read (void)
{
/*  if(Bat_Fault_Code[0]&CAN1R_ERR){
    if(OEM_Para.bms_addr== HmiBuf.buf[2]){
       KOUT_flag   =  HmiBuf.buf[4];
    }
  }
  if(HRT_R485!=HmiBuf.buf[3]){
    HRT_R485= HmiBuf.buf[3];
    Com_485_Err=0;
  }else{
    Com_485_Err=1;  
  }   */

}

/*****************************************************************************/
/* 彩屏通讯协议 - MODBUS写功能                                               */
/*****************************************************************************/
static void HMI_Write (void)
{
  UINT8 send_buf[28];
  UINT8 lenth = 26;
  UINT16 crc = 0;
  UINT8 ID = 0,i=0;
  UINT16 IN_now = 0;
  UINT16 OUT_now = 0;
  UINT16 temy_data=0;
  
	ID = 0xC0+(OEM_Para.bms_addr&0x0f);
  IN_now = BMS_InSts ();
  OUT_now = BMS_OutSts ();

  send_buf[0] = 0x2d;
  send_buf[1] = lenth;
  send_buf[2] = ID;
  send_buf[3] = HRT_T485++;
  send_buf[4] = 0;
  send_buf[5] = OUT_now;
  send_buf[6] = IN_now;
  send_buf[7] = Sys_Rt.to_vol>>8;
  send_buf[8] = Sys_Rt.to_vol;
  
  send_buf[9] = Sys_Rt.max_v>>8;
  send_buf[10] = Sys_Rt.max_v;

  temy_data=0;
  for(i=0;i<(Sys_Rt.maxv_lecu-1);i++){            //
    temy_data += LECU_NVT[i].Cell_num;
  }
  temy_data += Sys_Rt.maxv_batt;
  send_buf[11] = temy_data>>8;
  send_buf[12] = temy_data;
  
  send_buf[13] = Sys_Rt.min_v>>8;
  send_buf[14] = Sys_Rt.min_v;
  
  temy_data=0;
  for(i=0;i<(Sys_Rt.minv_lecu-1);i++){
    temy_data += LECU_NVT[i].Cell_num;
  }
  temy_data += Sys_Rt.minv_batt;        
  send_buf[15] = temy_data>>8;
  send_buf[16] = temy_data;  
 
  send_buf[17] = Sys_Rt.max_t;
  send_buf[18] = 0; 
  temy_data=0;
  for(i=0;i<(Sys_Rt.maxt_lecu-1);i++){
    temy_data += LECU_NVT[i].T_num;
  }
  temy_data += Sys_Rt.maxt_batt;    
  send_buf[19] = temy_data>>8;
  send_buf[20] = temy_data; 
  
  send_buf[21] = Sys_Rt.min_t;
  send_buf[22] = 0;
  
  temy_data=0;
  for(i=0;i<(Sys_Rt.mint_lecu-1);i++){
    temy_data += LECU_NVT[i].T_num;
  }
  temy_data += Sys_Rt.minv_batt;  
  send_buf[23] = temy_data>>8;
  send_buf[24] = temy_data; 
  
  send_buf[25] = 0; 
  
  crc = Modbus_crc ((UINT8 *)&(send_buf[0]),lenth); 
  send_buf[26] =(UINT8) (crc>>8)&0xff;
  send_buf[27] =(UINT8) crc&0xff;

  SCI_write(&send_buf,sizeof(send_buf));
  return;     
}

/*****************************************************************************/
/* 彩屏通讯协议入口函数                                                      */
/*****************************************************************************/
void HMI_main (void)
{
 /*   //if (IO_IN0 != 0)
        //hmi_stopcha = 0;
    if (!Modbus_check (&HmiBuf,5)){
      return;
    
    }
    if((HmiBuf.buf[2]&0x0f)==(OEM_Para.bms_addr&0x0f)){
      HMI_Read();
      HMI_Write();
    } */
    
}

#endif
