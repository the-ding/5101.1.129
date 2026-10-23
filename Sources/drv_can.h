#define CAN0_RXBSZ    16
#define CAN1_RXBSZ    16
#define CAN4_RXBSZ    4

/*****************************************************************************/
/* CAN帧结构                                                                 */
/*****************************************************************************/

#define CAN_R_PHM1 0x00000B00 // 接收PHM数据1
#define CAN_R_PHM2 0x00001B00 // 接收PHM数据2
#define CAN_R_ADD1 0x00001300 // 接收地址段1
#define CAN_R_ADD2 0x00002300 // 接收地址段2
#define CAN_R_STP1 0x00001401 // 接收停止命令1
#define CAN_R_STP2 0x00002401 // 接收停止命令2
#define CAN_R_REC1 0x00001411 // 回复命令1
#define CAN_R_REC2 0x00002411 // 回复命令2
#define CAN_R_BMS1 0x1801E0E0 // BMS发送数据

#define CAN_T_DATE1 0x00001500 // BMS发送数据
#define CAN_T_DATE2 0x00002500 // BMS发送数据
#define CAN_T_STP1 0x00001601  // 接收停止命令1
#define CAN_T_STP2 0x00002601  // 接收停止命令2
#define CAN_T_REC1 0x00001611  // 回复命令1
#define CAN_T_REC2 0x00002611  // 回复命令2

#define CAN_SP_TRDP 0x00000802  //停止TRDP转换
#define CAN_VE_TRDP 0x00000801  //确认TRDP转换

typedef struct can_frame
{
    UINT32  id;
    UINT8   data[8];
    UINT8   len;
    UINT8   pri;      /* 0 - 最高 */
} CanbusFrame;

#pragma DATA_SEG SHARED_DATA
extern volatile UINT8  CAN0_RxRPtr;
extern volatile UINT8  CAN0_RxWPtr;
extern CanbusFrame     CAN0_RxBuf[CAN0_RXBSZ];

extern volatile UINT8  CAN1_RxRPtr;
extern volatile UINT8  CAN1_RxWPtr;
extern CanbusFrame     CAN1_RxBuf[CAN1_RXBSZ];

extern volatile UINT8  CAN4_RxRPtr;
extern volatile UINT8  CAN4_RxWPtr;
extern CanbusFrame     CAN4_RxBuf[CAN4_RXBSZ];

extern volatile UINT8  OTA_can_buf[200];

#pragma DATA_SEG DEFAULT

/*****************************************************************************/
/* 查询CAN控制器发送缓冲区是否空                                             */
/*****************************************************************************/
BOOL CAN0_IsBusy (void);
BOOL CAN1_IsBusy (void);
BOOL CAN4_IsBusy (void);

/*****************************************************************************/
/* CAN控制器发送数据帧                                                       */
/*****************************************************************************/
void CAN0_write (const CanbusFrame *cf);
void CAN1_write (const CanbusFrame *cf);
void CAN4_write (const CanbusFrame *cf);

void CAN0_write_std(const CanbusFrame *cf);
void CAN1_write_std(const CanbusFrame *cf);
void CAN4_write_std(const CanbusFrame *cf);

/*****************************************************************************/
/* CAN控制器初始化                                                           */
/*****************************************************************************/
void CAN_Init (void);

