#include <string.h>
#include "sys.h"
#include "db.h"
#include "db_ev.h"
#include "db_para_cfg.h"
#include "db_para_struct.h"
#include "ptl.h"
#include "drv_rtc.h"
#include "drv_fm.h"
#include "drv_flash.h"
#include "drv_can.h"
#include "modbus.h"

static volatile UINT8  Project_Num = OEM;

#pragma DATA_SEG SHARED_DATA
extern UINT32   LECU_Valids;
extern UINT8    zbuf[];                /*压缩输出比特流(临时数组)*/
DBDevPara       TempDevPara;
volatile UINT8  TempDevGot;
BOOL            BCU_Update;
BOOL            BCU_Transport;
//extern SINT16 C0_Raw;
//extern SINT16 CurrentRaw;
#pragma DATA_SEG DEFAULT

static BOOL Modify_flg = FALSE;       // IO操作或均衡操作标志位
static BOOL MSG_txflg = FALSE;        // 发送控制流程记录标志位
static UINT32 MSG_txtime = 0;         // 发送控制路程次数

extern const UINT8 para_tbl[];
extern const UINT16 para_tbl_size;

extern volatile UINT16   cache_rptr;    /*铁电中报警记录读位置*/
extern volatile UINT16   cache_wptr;    /*铁电中报警记录写位置*/

//extern DBIsoMsg ISO_message;            /*绝缘模块信息结构体*/

void    EV_Reset (void);
void    LECU_ReInit (void);

/*****************************************************************************/
/* PC后台通讯CAN发送函数                                                     */
/*****************************************************************************/
static void CanWrite (CanbusFrame *cf)
{
    UINT16  i;

    CAN0_write (cf);

    /*超时等待*/
    for (i = 0; i < (UINT16)60000; i++)
        if (CAN0TFLG == 0x7)
            break;
}

/*****************************************************************************/
/* PC后台通讯协议初始化函数                                                  */
/*****************************************************************************/
void  PC_Init (void)
{
    BCU_Update = FALSE;
    BCU_Transport = FALSE;
}

/*****************************************************************************/
/* PC后台通讯协议 - 读历史数据摘要列表                                       */
/*****************************************************************************/
static void ListRec (CanbusFrame *cf)
{
    UINT8         i, Num = 0;
    struct tm     *t;
    DBDayIndexRec rec;
    DBHistRecHdr  hdr;

    for (i = 0; i < DAY_INDEX_MAX; i++)
    {
        Fm_read (DAY_INDEX_START + i*DAY_INDEX_SIZE, &rec, sizeof(rec));

        if (rec.magic != 'y')
          continue;

        Flash_read (rec.flash, &hdr, sizeof(hdr));

        if (!HST_IsValidHdr(&hdr) || rec.time/86400UL != hdr.time/86400UL) /*检查记录有效性*/
          continue;

        //cf->id =
        cf->len = 8;

        t = localtime ((time_t *)&rec.time);
        cf->data[0] = (UINT8)((t->tm_year + 1900) % 100);
        cf->data[1] = (UINT8)(t->tm_mon + 1);
        cf->data[2] = (UINT8)(t->tm_mday);
        cf->data[3] = (UINT8)(rec.flash >> 24);
        cf->data[4] = (UINT8)(rec.flash >> 16);
        cf->data[5] = (UINT8)(rec.flash >> 8);
        cf->data[6] = (UINT8)(rec.flash >> 0);
        cf->data[7] = i;
        CanWrite (cf);

        cf->len = 5;
        cf->data[0] = (UINT8)(rec.end >> 24);
        cf->data[1] = (UINT8)(rec.end >> 16);
        cf->data[2] = (UINT8)(rec.end >> 8);
        cf->data[3] = (UINT8)(rec.end >> 0);
        cf->data[4] = i;
        CanWrite (cf);

        Num++;
    }

    cf->len = 1;
    cf->data[0] = Num;
    CanWrite (cf);
}

/*****************************************************************************/
/* PC后台通讯协议 - 读指定日所有历史数据                                     */
/*****************************************************************************/
static void ReadHst (CanbusFrame *cf)
{
    DBDayIndexRec rec;
    DBHistRecHdr  hdr;
    UINT32        faddr;
    UINT16        len;
    BOOL          pickOne = FALSE;

    PITINTE_PINTE3 = 0;                       /*PIT中断关闭,停止CAN总线活动*/

    /*转换内部时间格式*/
    if (cf->len == 4)
    {
      struct tm   t2;
      (void) memset (&t2, 0, sizeof(t2));

      t2.tm_year = cf->data[1] + 2000 - 1900;
      t2.tm_mon  = cf->data[2] - 1;
      t2.tm_mday = cf->data[3];
      rec.time = mktime(&t2);
    }
    else
    {
      struct tm   t2;
      (void) memset (&t2, 0, sizeof(t2));

      t2.tm_year = cf->data[1] + 2000 - 1900;
      t2.tm_mon  = cf->data[2] - 1;
      t2.tm_mday = cf->data[3];
      t2.tm_hour = (char)cf->data[4];
      t2.tm_min = (char)cf->data[5];
      t2.tm_sec = (char)cf->data[6];
      rec.time = mktime(&t2);

      pickOne = TRUE;
    }

    /*查找指定日的摘要信息*/
    if (!HST_find (rec.time, &rec, &hdr, &faddr))
      goto Ret;

    /*先发送本日数据区间，方便后台估计操作进度*/
    if (!pickOne)
    {
        cf->id = 0xf20110;
        cf->len = 8;
        cf->data[0] = (UINT8)(rec.flash >> 24);
        cf->data[1] = (UINT8)(rec.flash >> 16);
        cf->data[2] = (UINT8)(rec.flash >> 8);
        cf->data[3] = (UINT8)(rec.flash >> 0);
        cf->data[4] = (UINT8)(rec.end >> 24);
        cf->data[5] = (UINT8)(rec.end >> 16);
        cf->data[6] = (UINT8)(rec.end >> 8);
        cf->data[7] = (UINT8)(rec.end >> 0);
        CanWrite (cf);

        udelay (100);
    }

    /*遍历该日所有历史数据记录*/
    while (hdr.time >= rec.time && hdr.time < rec.time + 86400UL)
    {
        UINT32 ts = hdr.time - rec.time;

        /*先发送记录描述信息*/
        cf->len = 7;
        cf->data[0] = (UINT8)(ts >> 16);
        cf->data[1] = (UINT8)(ts >> 8);
        cf->data[2] = (UINT8)ts;
        cf->data[3] = (UINT8)(hdr.length >> 8);
        cf->data[4] = (UINT8)hdr.length;
        cf->data[5] = hdr.lecu;

        if (hdr.type == 'w')
            cf->data[6] = 'a';
        else if (hdr.type == 'W')
            cf->data[6] = 'A';
        else
            cf->data[6] = hdr.type;

        cf->id = 0xf20111;
        CanWrite (cf);

        udelay (10);

        /*发送整车信息*/
        cf->len = 8;
        cf->id = 0xf20112;
        cf->data[0] = (UINT8)((UINT16)hdr.current >> 8);
        cf->data[1] = (UINT8)hdr.current;
        cf->data[2] = hdr.soc;
        cf->data[3] = hdr.soh;
        cf->data[4] = (UINT8)(hdr.isovolt >> 8);
        cf->data[5] = (UINT8)hdr.isovolt;
        cf->data[6] = (UINT8)(hdr.posres >> 8);
        cf->data[7] = (UINT8)hdr.posres;
        CanWrite (cf);

        udelay (10);

        cf->len = 8;
        cf->id = 0xf20113;
        //cf->data[0] = (UINT8)(hdr.negres >> 8);
        //cf->data[1] = (UINT8)hdr.negres;
        cf->data[0] = (UINT8)(hdr.relayN1 >> 8);
        cf->data[1] = (UINT8)hdr.relayN1;

        /*if (HST_HdrSize(&hdr) > offsetof(DBHistRecHdr, warning))
        {
            cf->len += 4;
            cf->data[2] = (UINT8)(hdr.warning >> 24);
            cf->data[3] = (UINT8)(hdr.warning >> 16);
            cf->data[4] = (UINT8)(hdr.warning >> 8);
            cf->data[5] = (UINT8)hdr.warning;
        }*/
        cf->data[2] = (UINT8)(hdr.relayP1 >> 8);
        cf->data[3] = (UINT8)hdr.relayP1;
        cf->data[4] = (UINT8)(hdr.relayP2 >> 8);
        cf->data[5] = (UINT8)hdr.relayP2;
        cf->data[6] = (UINT8)(hdr.vol_pre >> 8);
        cf->data[7] = (UINT8)hdr.vol_pre;
        CanWrite (cf);

        udelay (10);
        
        /*新添加的历史数据*/
        cf->len = 8;
        cf->id = 0xf20114;
        cf->data[0] = (UINT8)hdr.maxt;
        cf->data[1] = (UINT8)hdr.mint;
        cf->data[2] = (UINT8)(hdr.in_sts >> 8);
        cf->data[3] = (UINT8)(hdr.in_sts & 0xff);
        cf->data[4] = (UINT8)(hdr.out_sts >> 8);
        cf->data[5] = (UINT8)(hdr.out_sts & 0xff);
        cf->data[6] = (UINT8)(hdr.out_reson >> 8);
        cf->data[7] = (UINT8)(hdr.out_reson & 0xff); 
        CanWrite (cf);

        udelay (10);
        

        /*发送压缩数据流。为节约内存，将压缩数据流切块、分帧传输*/
        cf->id = 0xf20115;
        len = hdr.length;
        while (len != 0)
        {
            UINT16 i, cb = len;

            if (cb > 512)
              cb = 512;

            Flash_read (faddr, zbuf, cb);

            /*分帧*/
            for (i = 0; i < cb; /**/)
            {
                if (cb - i > 8)
                  cf->len = 8;
                else
                  cf->len = (UINT8)(cb - i);

                (void) memcpy (cf->data, zbuf + i, cf->len);

                CanWrite (cf);

                i += cf->len;
                cf->id++;
            }

            /*下一个压缩块*/
            faddr += cb;
            len -= cb;
        }

        /*只补招一条记录*/

        if (pickOne)
            goto Ret2;

        /*已经跳过压缩数据，定位到下一个记录*/

        if (faddr + HIST_RSVD >= FLASH_HIST_END)  /*FLASH空间循环存储*/
          faddr = FLASH_HIST_START;

        Flash_read (faddr, &hdr, sizeof(hdr));
        faddr += HST_HdrSize(&hdr);

        /*简单判断记录有效性*/
        if (!HST_IsValidHdr(&hdr) || hdr.time == 0xFFFFFFFF || hdr.length > 4000)
        {
            UINT32  probe = (faddr - HST_HdrSize(&hdr)) + 1;
            UINT32  limit = probe + 20*1024;         // 探测20k
            BOOL    success = FALSE;

            if (limit > FLASH_HIST_END - HIST_RSVD)
              limit = FLASH_HIST_END - HIST_RSVD;

            while (probe < limit)
            {
                Flash_read (probe, &hdr, sizeof(hdr));

                if (HST_IsValidHdr(&hdr) &&
                    hdr.time != 0xFFFFFFFF &&
                    hdr.length < 4000 &&
                    (hdr.time - rec.time) < 86400)
                {
                   faddr = probe + HST_HdrSize(&hdr);

                   success = TRUE;
                   break;
                }

                /*继续探测*/
                if (hdr.current == (SINT16)0xffff && hdr.soc == 0xff/* && hdr.soh == 0xff*/)    /*加速探测,ZYYsoh去掉*/
                    probe += HST_HdrSize(&hdr);
                else
                    probe++;
            }

            if (!success)
              break;
        }
    }


    /*传输结束*/
Ret:
    cf->id = 0xf20110;
    cf->len = 3;
    cf->data[0] = (UINT8)(rec.cursor >> 16);
    cf->data[1] = (UINT8)(rec.cursor >> 8);
    cf->data[2] = (UINT8)rec.cursor;
    CanWrite (cf);


Ret2:
    PITINTE_PINTE3 = 1;       /*PIT中断使能*/
}

/*****************************************************************************/
/* PC后台通讯协议 - 时间维护                                                 */
/*****************************************************************************/
static void TimeExt (CanbusFrame *cf)
{
    if (cf->len == 1 && cf->data[0] == 'R')
    {
        struct tm t2;

        (void)RTC_read (&t2);

        cf->len = 7;
        cf->data[0] = 'r';
        cf->data[1] = (UINT8)((t2.tm_year + 1900) % 100);
        cf->data[2] = (UINT8)(t2.tm_mon + 1);
        cf->data[3] = (UINT8)t2.tm_mday;
        cf->data[4] = (UINT8)t2.tm_hour;
        cf->data[5] = (UINT8)t2.tm_min;
        cf->data[6] = (UINT8)t2.tm_sec;
        CanWrite (cf);
    }
    else if (cf->len == 2 && cf->data[0] == 'T' && cf->data[1] == 'e')
    {
        cf->len = 6;
        cf->data[0] = 'C';
        cf->data[1] = 'O';
        cf->data[2] = para_tbl_size >> 8;
        cf->data[3] = para_tbl_size & 0xff;
        cf->data[4] = 0;
        cf->data[5] = 0;
        CanWrite (cf);
    }
    else if (cf->len == 7 && cf->data[0] == 'W')
    {
        struct tm t;

        (void)memset (&t, 0, sizeof(t));
        t.tm_year = cf->data[1] + 2000 - 1900;
        t.tm_mon  = cf->data[2] - 1;
        t.tm_mday = cf->data[3];
        t.tm_hour = (char)cf->data[4];
        t.tm_min  = (char)cf->data[5];
        t.tm_sec  = (char)cf->data[6];

        gSysSec = mktime (&t);
        (void) RTC_write (&t);
    }
}

/*****************************************************************************/
/* PC后台通讯协议 - 报警数据                                                 */
/*****************************************************************************/
static void sendev (DBEvent *ev)
{
    CanbusFrame cf;
    
    cf.id = 0xe21000;
    cf.pri = 0;
    cf.len = 8;

    (void) memcpy (cf.data, (UINT8 *) ev + 0, 8);
    CanWrite (&cf);

    cf.id++;
    udelay (5);

    (void) memcpy (cf.data, (UINT8 *) ev + 8, 8);
    CanWrite (&cf);
}

static void EventRec (CanbusFrame *cf)
{
    static volatile UINT16  i, k, l;    
    DBFlashIndexRec rec_flash, tmp_flash;
    DBEvent ev;
    static volatile UINT8 start_y, start_m, end_y, end_m;
    static volatile UINT8 start_y1, start_m1, end_y1, end_m1;
    static volatile UINT32 flash_len;
    static volatile fm_readNum;
    time_t Fm_tmpTim, tmp_timeFm;
    struct tm t1;
    struct tm *time_Fm;
    
    UINT8 tmp1[256] = {0};

    if (cf->len == 5 && cf->data[0] == 'R')
    {
        /*判断日期是否正确*/
        start_y = cf->data[1];
        start_m = cf->data[2];
        end_y = cf->data[3];
        end_m = cf->data[4];
        if (start_y > end_y)
            return;
        else if (start_y == end_y && start_m > end_m) 
            return; 
        
        start_y1 = start_y;
        start_m1 = start_m;
        end_y1 = end_y;
        end_m1 = end_m;      
        
        /*转换内部时间格式*/
        while ((start_y < end_y) || (start_y == end_y && start_m <= end_m))
        {                            
            struct tm   t2;
            (void) memset (&t2, 0, sizeof(t2));

            t2.tm_year = start_y + 2000 - 1900;
            t2.tm_mon  = start_m - 1;
            rec_flash.mon = mktime(&t2);
    
            //读出Flash中该月报警记录
            for (i = 0; i < FLA_INDEX_MAX; i++)
            {
                Fm_read (FLA_INDEX_START + i*FLA_INDEX_SIZE, &tmp_flash, sizeof(DBFlashIndexRec));
                
                if (tmp_flash.magic == 'y')
                {
                    if (tmp_flash.mon == rec_flash.mon)
                    {                    
                        while (tmp_flash.flash != tmp_flash.end)
                        {
                            (void) memset (&tmp1, 0, sizeof(tmp1));
                            if (!Flash_busy ())
                            {
                                if (tmp_flash.end > tmp_flash.flash)
                                    flash_len = tmp_flash.end - tmp_flash.flash;
                                else
                                    flash_len = (FLASH_ALARM_END-tmp_flash.flash)+(tmp_flash.end-0x1000000);
                                    //flash_len = (FLASH_ALARM_END-tmp_flash.flash)+(tmp_flash.end-FLASH_ALARM_START);    
                                if (flash_len < 256)
                                {
                                    Flash_read (tmp_flash.flash, tmp1, (UINT16)flash_len);
                                    for (k = 0; k < flash_len;)
                                    {
                                        (void) memcpy (&ev, &tmp1[k], 16);
                                        k += 16;
                                        if (ev.time != 0 && ev.crc == CRC8 ((UINT8 *) &ev, offsetof(DBEvent, crc)))
                                            sendev (&ev);
                                    }        
                                        
                                    tmp_flash.flash += flash_len;
                                    if (tmp_flash.flash >= FLASH_ALARM_END)
                                        tmp_flash.flash = 0x1000000;
                                        //tmp_flash.flash = FLASH_ALARM_START;
                                }
                                else                                                    
                                {
                                    Flash_read (tmp_flash.flash, tmp1, 256);
                                    for (l = 0; l < 256;)
                                    {
                                        (void) memcpy (&ev, &tmp1[l], 16);
                                        l += 16;
                                        if (ev.time != 0 && ev.crc == CRC8 ((UINT8 *) &ev, offsetof(DBEvent, crc)))
                                            sendev (&ev);
                                    }        
                                        
                                    tmp_flash.flash += 16*16;
                                    if (tmp_flash.flash >= FLASH_ALARM_END)
                                        tmp_flash.flash = 0x1000000;
                                        //tmp_flash.flash = FLASH_ALARM_START;
                                }            
                            }   
                        }
                    }   
                }
            }
            
            start_m++;
            if (start_m > 12)
            {
                start_m = 1;
                start_y++;
            }
        }    
        
        while ((start_y1 < end_y1) || (start_y1 == end_y1 && start_m1 <= end_m1))
        {                            
            struct tm   t3;
            (void) memset (&t3, 0, sizeof(t3));

            t3.tm_year = start_y1 + 2000 - 1900;
            t3.tm_mon  = start_m1 - 1;
            Fm_tmpTim = mktime(&t3);
            
            //读出铁电中该月报警记录
            fm_readNum = cache_rptr;
            while (fm_readNum != cache_wptr)
            {
                Fm_read (ALARM_REC_START + fm_readNum*16, &ev, sizeof(DBEvent));
                (void) memset (&t1, 0, sizeof(t1));
                time_Fm = localtime ((time_t *)&ev.time);
                t1.tm_year = time_Fm->tm_year;
                t1.tm_mon = time_Fm->tm_mon;
                tmp_timeFm = mktime(&t1);
                if (tmp_timeFm == Fm_tmpTim)
                    if (ev.time != 0 && ev.crc == CRC8 ((UINT8 *) &ev, offsetof(DBEvent, crc)))
                        sendev (&ev);
                fm_readNum = (fm_readNum+1)%ALARM_REC_NUM;        
            }
            start_m1++;
            if (start_m1 > 12)
            {
                start_m1 = 1;
                start_y1++;
            }
        }        
   
        
        //传输完毕
        cf->id = 0xe30000;
        cf->pri = 0;
        cf->len = 0;
        CanWrite (cf);
    }
    if (cf->len == 2 && cf->data[0] == 'R') //读采集模块报警记录
    {
        cf->id = 0xe40000;      // 模块不存在
        cf->len = 1;
        cf->pri = 0;
        cf->data[0] = cf->data[1];
        CanWrite (cf);
    }
    else if (cf->len == 1 && cf->data[0] == 'E')  //删除所有记录
    {
        EV_Reset ();
    }
}

/*****************************************************************************/
/* PC后台通讯协议 - 读数据FLASH                                              */
/*****************************************************************************/
static void ReadFlash (CanbusFrame *cf)
{
    UINT32  faddr;
    UINT16  i;
    UINT16   bytes = ((UINT16)cf->data[4] << 8) | cf->data[5];

    (void) memcpy (&faddr, cf->data, 4);

    while (bytes != 0)
    {
      UINT16 cb = bytes;
      if (cb > 512)
        cb = 512;

      Flash_read (faddr, zbuf, cb);
      for (i = 0; i < (cb + 7)/8; i++)
      {
          (void) memcpy (cf->data, zbuf + i*8, 8);

          cf->len = 8;
          CanWrite (cf);
      }

      bytes -= cb;
      faddr += cb;
    }
}

/*****************************************************************************/
/* PC后台通讯协议 - 读OEM参数                                                */
/*****************************************************************************/
static void ReadOEMPara (void)
{
    CanbusFrame cf;
    UINT16  i;

    cf.id   = 0x10D82992;
    cf.len  = 0;
    cf.pri  = 0;
    CanWrite (&cf);
    
    cf.id = 0x10000000;
    cf.pri = 0;

    for (i = 0; i < sizeof(OEM_Para) - 2; i += 8)
    {
        UINT8 cb = 8;

        if (i + cb > sizeof(OEM_Para) - 2)
          cb = (UINT8)((sizeof(OEM_Para) - 2) - i);

        (void) memcpy (cf.data, (UINT8 *) &OEM_Para + i, cb);

        /**/
        cf.len = cb;

        CanWrite (&cf);

        cf.id++;

        udelay (100);
    }
}

/*****************************************************************************/
/* PC后台通讯协议 - 写OEM参数                                                */
/*****************************************************************************/
static void WriteOEMPara (void)
{
    UINT32  loops = 800;
    UINT8   j = 0;
    CanbusFrame cf, *r;
    UINT8 buf[sizeof(OEM_Para) + 8];

    /*删除当前帧*/
    CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);

    while (loops)
    {
        mdelay (1);

        loops--;
        if (CAN0_RxRPtr == CAN0_RxWPtr)
          continue;

        r = &CAN0_RxBuf[CAN0_RxRPtr];
        if (r->id == j)
        {
          (void) memcpy (buf + j, r->data, 8);
          j += r->len;

          if (j >= sizeof(OEM_Para) - 2)
            goto ok;
        }

        /*删除当前帧*/
        CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);
    }
    return;

ok: (void) memcpy (&OEM_Para, buf, sizeof (OEM_Para) - 2);

    Sys_Para.lecu_num = OEM_Para.lecu_num;
    Sys_Para.hallpara = OEM_Para.hallpara;

    Sys_Para.crc = Modbus_crc ((UINT8 *)&Sys_Para, sizeof(Sys_Para) - 2);
    Fm_write (SYSPARA_START, &Sys_Para, sizeof(Sys_Para));

    OEM_Para.crc = Modbus_crc ((UINT8 *)&OEM_Para, sizeof(OEM_Para) - 2);
    Fm_write (OEMPARA_START, &OEM_Para, sizeof(OEM_Para));

    /*确认*/
    cf.id   = 0x10D82992;
    cf.len  = 2;
    cf.pri  = 0;
    cf.data[0] = Sys_Para.crc >> 8;
    cf.data[1] = Sys_Para.crc & 0xff;

    CanWrite (&cf);
}

/*****************************************************************************/
/* PC后台通讯协议 - 读系统参数                                               */
/*****************************************************************************/
static void ReadSysPara (void)
{
    CanbusFrame cf;
    UINT16  i;

    cf.id   = 0x10D81995;
    cf.len  = 0;
    cf.pri  = 0;
    CanWrite (&cf);
    cf.id = 0x10000000;
    cf.pri = 0;

    for (i = 0; i < sizeof(Sys_Para) - 2; i += 8)
    {
        UINT8 cb = 8;

        if (i + cb > sizeof(Sys_Para) - 2)
          cb = (UINT8)((sizeof(Sys_Para) - 2) - i);

        (void) memcpy (cf.data, (UINT8 *) &Sys_Para + i, cb);

        /**/
        cf.len = cb;

        CanWrite (&cf);

        cf.id++;

        udelay (100);
    }
}

/*****************************************************************************/
/* PC后台通讯协议 - 写系统参数                                               */
/*****************************************************************************/
static void WriteSysPara (void)
{
    UINT32  loops = 800;
    UINT8   i = 0, j = 0;
    CanbusFrame cf, *r;
    UINT8 buf[sizeof(DBSysPara)+8];

    /*删除当前帧*/
    CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);

    while (loops)
    {
        mdelay (1);

        loops--;
        if (CAN0_RxRPtr == CAN0_RxWPtr)
          continue;

        r = &CAN0_RxBuf[CAN0_RxRPtr];
        if (r->id == j)
        {
          (void) memcpy (buf + j, r->data, 8);
          j += r->len;

          if (j >= sizeof(DBSysPara) - 2)
            goto ok;
        }

        /*删除当前帧*/
        CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);
    }
    return;

ok: (void) memcpy (&Sys_Para, buf, sizeof (DBSysPara) - 2);

    if (Sys_Para.hist_prd == 0)
        Sys_Para.hist_prd = 2;    // 2秒
    if (Sys_Para.shutcur == 0)
        Sys_Para.shutcur = 4;     // 4安培

    /*这些参数不允许在这里修改*/
    Sys_Para.lecu_num = OEM_Para.lecu_num;
    Sys_Para.hallpara = OEM_Para.hallpara;

    Sys_Para.crc = Modbus_crc ((UINT8 *)&Sys_Para, sizeof(Sys_Para) - 2);
    Fm_write (SYSPARA_START, &Sys_Para, sizeof(Sys_Para));

    /*确认*/
    cf.id   = 0x10D81995;
    cf.len  = 2;
    cf.pri  = 0;
    cf.data[0] = Sys_Para.crc >> 8;
    cf.data[1] = Sys_Para.crc & 0xff;

    CanWrite (&cf);

    /*通知给采集模块*/
    cf.len  = 8;
    cf.pri  = 0;

    for (i = 0; i < DEV_MAX; i++)
    {
        (void)memcpy(&LECU_Para[i].coltd, &Sys_Para.coltd, offsetof(DBDevPara,amp_mav) - offsetof(DBDevPara,coltd));

        LECU_Para[i].crc = Modbus_crc ((UINT8 *)&LECU_Para[i], sizeof(DBDevPara) - 2);
        Fm_write (LECU_PARA_START+i*sizeof(DBDevPara), &LECU_Para[i], sizeof(DBDevPara));

        for (j = 0; j < 6; j++)
        {
            cf.id   = 0x311f6+(UINT32)i*0x1000+(UINT16)j*0x100;

            (void) memcpy (cf.data, (UINT8*)&LECU_Para[i].coltd + j*8, 8);  //lint !e662 !e670

            CAN4_write (&cf);
            while (CAN4TFLG != 0x7) {}

            udelay (100);
        }
    }
}


/*****************************************************************************/
/* PC后台通讯协议 - 读电池参数                                               */
/*****************************************************************************/
static void ReadBattPara (void)
{
    CanbusFrame cf;
    UINT16  i;
    
    cf.id   = 0x10D81994;
    cf.len  = 0;
    cf.pri  = 0;
    CanWrite (&cf);

    cf.id = 0x10000000;
    cf.pri = 0;

    for (i = 0; i < sizeof(Batt_Para) - 2; i += 8)
    {
        UINT8 cb = 8;

        if (i + cb > sizeof(Batt_Para) - 2)
          cb = (UINT8)((sizeof(Batt_Para) - 2) - i);

        (void) memcpy (cf.data, (UINT8 *) &Batt_Para + i, cb);

        
        cf.len = cb;

        CanWrite (&cf);

        cf.id++;

        udelay (100);
    }
}


/*****************************************************************************/
/* PC后台通讯协议 - 写电池参数                                                */
/*****************************************************************************/
static void WriteBattPara (void)
{
    UINT32  loops = 800;
    UINT8   j = 0;
    CanbusFrame cf, *r;
    UINT8 buf[sizeof(Batt_Para) + 8];

    //删除当前帧
    CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);

    while (loops)
    {
        mdelay (1);

        loops--;
        if (CAN0_RxRPtr == CAN0_RxWPtr)
          continue;

        r = &CAN0_RxBuf[CAN0_RxRPtr];
        if (r->id == j)
        {
          (void) memcpy (buf + j, r->data, 8);
          j += r->len;

          if (j >= sizeof(Batt_Para) - 2)
            goto ok;
        }

        //删除当前帧
        CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);
    }
    return;

ok: (void) memcpy (&Batt_Para, buf, sizeof (Batt_Para) - 2);

    Batt_Para.crc = Modbus_crc ((UINT8 *)&Batt_Para, sizeof(Batt_Para) - 2);
    Fm_write (BATTPARA_START, &Batt_Para, sizeof(Batt_Para));

    //确认
    cf.id   = 0x10D81993;
    cf.len  = 2;
    cf.pri  = 0;
    cf.data[0] = Batt_Para.crc >> 8;
    cf.data[1] = Batt_Para.crc & 0xff;

    CanWrite (&cf);
}


/*****************************************************************************/
/* PC后台通讯协议 - 读报警参数                                               */
/*****************************************************************************/
static void ReadAlarmPara (CanbusFrame *rf)
{
    CanbusFrame cf;
    UINT16  i;

    if (rf->data[1] == 'C' && rf->data[2] == 'O')   // 参数类型表
    {
        cf.id = 0x10000;
        cf.pri = 0;

        for (i = 0; i < para_tbl_size; i += 8)
        {
            UINT8 cb = 8;
    
            if (i + cb > para_tbl_size)
              cb = (UINT8)(para_tbl_size - i);
    
            (void) memcpy (cf.data, (UINT8 *) &para_tbl + i, cb);
    
            /**/
            cf.len = cb;
    
            CanWrite (&cf);
            cf.id++;
            
            udelay (100);
        }
    }
    else if (rf->data[1] == 'P' && rf->data[2] == 'A')// 参数表
    {
        cf.id = 0x20000;
        cf.pri = 0;




        //geshu =  sizeof(Sys_Alarm_Para) + 22;
        for (i = 0; i < sizeof(Sys_Alarm_Para) ; i += 8)
        {
            UINT8 cb = 8;
    
            if (i + cb > sizeof(Sys_Alarm_Para))
              cb = (UINT8)(sizeof(Sys_Alarm_Para) - i);
    
            (void) memcpy (cf.data, (UINT8 *) &Sys_Alarm_Para + i, cb);
    
            /**/
            cf.len = cb;
    
            CanWrite (&cf);
            cf.id++;
            
            udelay (100);
        }
    }
}

/*****************************************************************************/
/* PC后台通讯协议 - 写报警参数                                               */
/*****************************************************************************/
static void WriteAlarmPara (void)
{
    UINT32  loops = 800;
    UINT8   i = 0, j = 0;
    CanbusFrame cf, *r;
    UINT8 buf[sizeof(Sys_Alarm_Para)+8];

    /*删除当前帧*/
    CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);

    while (loops)
    {
        mdelay (1);

        loops--;
        if (CAN0_RxRPtr == CAN0_RxWPtr)
          continue;   //结束本次循环

        r = &CAN0_RxBuf[CAN0_RxRPtr];
        if (r->id == j)
        {
          (void) memcpy (buf + j, r->data, 8);
          j += r->len;

          if (j >= sizeof(Sys_Alarm_Para))
            goto ok;
        }

        /*删除当前帧*/
        CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);
    }
    return;

ok: (void) memcpy (&Sys_Alarm_Para, buf, sizeof (Sys_Alarm_Para));

    Sys_Alarm_Para_Save ();

    /*确认*/
    cf.id   = 0x10D81991;
    cf.len  = 2;
    cf.pri  = 0;
    cf.data[0] = Sys_Alarm_Para.chksum >> 8;
    cf.data[1] = Sys_Alarm_Para.chksum & 0xff;

    CanWrite (&cf);
}

/*****************************************************************************/
/* PC后台通讯协议 - 读采集模块参数                                           */
/*****************************************************************************/
static void ReadDevPara (UINT8 dev)
{
    CanbusFrame cf;
    UINT16  i;

    PITINTE_PINTE3 = 0;         /*PIT中断关闭,停止采集板数据招测*/
    mdelay (30);

    /**/
    TempDevGot = 0;

/*    cf.id = 0x310f6UL + ((UINT32)dev << 12);
    cf.pri = 0;
    cf.len = 1;
    cf.data[0] = 0x2;
    CAN4_write (&cf);

    for (i = 0; i < 20 && TempDevGot != 0x3f; i++)
        mdelay (5);

    mdelay (10);
    PITINTE_PINTE3 = 1;

    if (TempDevGot != 0x3f)
        return;  */

    /**/
    TempDevPara.serial_num = LECU_Para[dev].serial_num;
    TempDevPara.RegCnt = LECU_Para[dev].RegCnt;

    cf.id = 0x12000000;
    cf.pri = 0;
    cf.len = 8;

    for (i = 0; i < sizeof(LECU_Para[0]) - 2; i += 8)
    {
        (void) memcpy (cf.data, (UINT8 *) &LECU_Para[dev] + i, 8);
        //(void) memcpy (cf.data, (UINT8 *) &TempDevPara + i, 8);

        CanWrite (&cf);

        cf.id++;

        udelay (100);
    }
}

/*****************************************************************************/
/* PC后台通讯协议 - 写采集模块参数                                           */
/*****************************************************************************/
static void WriteDevPara (UINT8 dev)
{
    UINT32  loops = 800;
    UINT8   j = 0;
    CanbusFrame cf, *r;
    DBDevPara *para = &LECU_Para[dev];
    UINT8 buf[sizeof(DBDevPara)+8];

    /*删除当前帧*/
    CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);

    while (loops)
    {
        mdelay (1);

        loops--;
        if (CAN0_RxRPtr == CAN0_RxWPtr)
          continue;

        r = &CAN0_RxBuf[CAN0_RxRPtr];
        if (r->id == j)
        {
          (void) memcpy (buf + j, r->data, 8);
          j += r->len;

          if (j >= sizeof(DBDevPara) - 10/*SN,RegCnt,crc*/)
            goto ok;
        }

        /*删除当前帧*/
        CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);
    }
    return;

ok: (void) memcpy (&para->coltd, buf, sizeof (DBDevPara) - 10/*SN,RegCnt,crc*/);    //lint !e419

    para->crc = Modbus_crc ((UINT8 *)para, sizeof(DBDevPara) - 2);
    Fm_write (LECU_PARA_START + dev*sizeof(DBDevPara), para, sizeof(DBDevPara));

    PITINTE_PINTE3 = 0;         /*PIT中断关闭,停止采集板数据招测*/
    mdelay (30);

    /*确认*/
    cf.id   = 0x10D81992;
    cf.len  = 2;
    cf.pri  = 0;
    cf.data[0] = para->crc >> 8;
    cf.data[1] = para->crc & 0xff;

    CanWrite (&cf);

    /*通知给采集模块*/
    cf.len  = 8;
    cf.pri  = 0;

    for (j = 0; j < 6; j++)
    {
        cf.id   = 0x311f6+(UINT32)dev*0x1000+(UINT16)j*0x100;

        (void) memcpy (cf.data, (UINT8*)&para->coltd + j*8, 8); //lint !e662 !e670

        CAN4_write (&cf);
        while (CAN4TFLG != 0x7) {}

        udelay (100);
    }

    mdelay (10);
    PITINTE_PINTE3 = 1;
}

/*****************************************************************************/
/* PC后台通讯协议 - 读实时数据                                               */
/*****************************************************************************/
static void ReadRt (CanbusFrame *cf)
{
    UINT16  *cv;
    UINT8   dev = cf->data[0];
    UINT8   i,j;
    UINT32  c = (UINT32)(SINT32)Sys_Rt.current;
    UINT32  temp_id = 0; //临时用的id




    cf->data[0] = (UINT8)Sys_Rt.soc;
    cf->data[1] = (UINT8)(Sys_Rt.to_vol >> 8);
    cf->data[2] = (UINT8)Sys_Rt.to_vol;
    cf->data[3] = (UINT8)(c >> 24);
    cf->data[4] = (UINT8)(c >> 16);
    cf->data[5] = (UINT8)(c >> 8);
    cf->data[6] = (UINT8)c;
    cf->data[7] = Sys_Rt.num_lecu;

    cf->id = 0x13000000;
    cf->len = 8;
    CanWrite (cf);


    cf->data[0] = (UINT8)(Sys_Rt.max_v >> 8);
    cf->data[1] = (UINT8)Sys_Rt.max_v;
    cf->data[2] = (UINT8)(Sys_Rt.min_v >> 8);
    cf->data[3] = (UINT8)Sys_Rt.min_v;
    cf->data[4] = (UINT8)(Sys_Rt.wh >> 24);
    cf->data[5] = (UINT8)(Sys_Rt.wh >> 16);
    cf->data[6] = (UINT8)(Sys_Rt.wh >> 8);
    cf->data[7] = (UINT8)Sys_Rt.wh;

    cf->id = 0x13000001;
    cf->len = 8;
    CanWrite (cf);

    cf->data[0] = (UINT8)Sys_Rt.max_t;
    cf->data[1] = (UINT8)Sys_Rt.min_t;
    
    cf->data[4] = cf->data[2] = (UINT8)(Sys_Rt.pos_res >> 8);
    cf->data[5] = cf->data[3] = (UINT8)(Sys_Rt.pos_res);
    
   
    cf->data[6] = (UINT8)((UINT16)Sys_Rt.total_vol_res >> 8);
    cf->data[7] = (UINT8)Sys_Rt.total_vol_res;

    cf->id = 0x13000002;
    cf->len = 8;
    CanWrite (cf);

    /*报警信息*/
    cf->data[0] = (UINT8)(/*Sys_Alarm.warning*/0 >> 24);
    cf->data[1] = (UINT8)(/*Sys_Alarm.warning*/0 >> 16);
    cf->data[2] = (UINT8)(/*Sys_Alarm.warning*/0 >> 8);
    cf->data[3] = (UINT8)/*Sys_Alarm.warning*/0;
    cf->data[4] = (UINT8)(gSysErrFlags >> 24);
    cf->data[5] = (UINT8)(gSysErrFlags >> 16);
    cf->data[6] = (UINT8)(gSysErrFlags >> 8);
    cf->data[7] = (UINT8)gSysErrFlags;


    cf->id = 0x13000003;
    cf->len = 8;
    CanWrite (cf);
    
    
    /*报警信息*/
    cf->data[0] = (UINT8)(Sys_Alarm.warning1 >> 24);
    cf->data[1] = (UINT8)(Sys_Alarm.warning1 >> 16);
    cf->data[2] = (UINT8)(Sys_Alarm.warning1 >> 8);
    cf->data[3] = (UINT8)Sys_Alarm.warning1;
    cf->data[4] = (UINT8)(Sys_Alarm.warning2 >> 24);
    cf->data[5] = (UINT8)(Sys_Alarm.warning2 >> 16);
    cf->data[6] = (UINT8)(Sys_Alarm.warning2 >> 8);
    cf->data[7] = (UINT8)Sys_Alarm.warning2;
    cf->id = 0x13000004;
    cf->len = 8;
    CanWrite (cf);

#ifdef HAVE_L3_PARAMS
    cf->data[0] = (UINT8)(Sys_Alarm.warning3 >> 24);
    cf->data[1] = (UINT8)(Sys_Alarm.warning3 >> 16);
    cf->data[2] = (UINT8)(Sys_Alarm.warning3 >> 8);
    cf->data[3] = (UINT8)Sys_Alarm.warning3;
    cf->data[4] = (UINT8)(Sys_Alarm.warning4 >> 24);
    cf->data[5] = (UINT8)(Sys_Alarm.warning4 >> 16);
    cf->data[6] = (UINT8)(Sys_Alarm.warning4 >> 8);
    cf->data[7] = (UINT8)Sys_Alarm.warning4;
    cf->id = 0x13000005;
    cf->len = 8;
    CanWrite (cf);
#endif

    cf->data[0] = Sys_Rt.maxv_lecu;
    cf->data[1] = Sys_Rt.maxv_batt;
    cf->data[2] = Sys_Rt.minv_lecu;
    cf->data[3] = Sys_Rt.minv_batt;
    cf->data[4] = Sys_Rt.maxt_lecu;
    cf->data[5] = Sys_Rt.maxt_batt;
    cf->data[6] = Sys_Rt.mint_lecu;
    cf->data[7] = Sys_Rt.mint_batt;
    
    cf->id = 0x13000006;
    cf->len = 8;
    CanWrite (cf);
    
    
    cf->data[0] = (UINT8)(Sys_Rt.avg_v>>8);
    cf->data[1] = (UINT8)(Sys_Rt.avg_v);
    cf->data[2] = (UINT8)(Sys_Rt.relayPRE>>8);
    cf->data[3] = (UINT8)(Sys_Rt.relayPRE);
    cf->data[4] = (UINT8)(Sys_Rt.bodys>>8); 
    cf->data[5] = (UINT8)(Sys_Rt.bodys); 
    cf->data[6] = (UINT8)(Sys_Rt.avg_t); //平均温度，添加于20150407@yyzz
    cf->data[7] = 0;

    cf->id = 0x13000007;
    cf->len = 8;
    CanWrite (cf);
    
    cf->data[0] = (UINT8)(Sys_Rt.relayFAST>>8);
    cf->data[1] = (UINT8)(Sys_Rt.relayFAST);
    cf->data[2] = (UINT8)(Sys_Rt.relaySLOW>>8);
    cf->data[3] = (UINT8)(Sys_Rt.relaySLOW);
    cf->data[4] = (UINT8)(Sys_Rt.NCNC>>8); 
    cf->data[5] = (UINT8)(Sys_Rt.NCNC);
    cf->data[6] = 0;
    cf->data[7] = 0;
    
    cf->id = 0x13000008;
    cf->len = 8;
    CanWrite (cf);

    if (dev == 0)
      return;
   // LECU_Rt[dev - 1].bodys  = (LECU_Para[dev - 1].cell_num+LECU_Para[dev - 1].cell2_num+LECU_Para[dev - 1].cell3_num)|0x8000;
    for (i = 0; i < offsetof(DBDevRt, balsts); i += 8)
    {
        if (dev > DEV_MAX)
          (void) memset (cf->data, 0, 8);
        else
          (void) memcpy (cf->data, (UINT8 *)&LECU_Rt[dev - 1] + i, 8);  //lint !e662 !e670

        cf->id = 0x13000010 + i/8;
        temp_id = cf->id;
        cf->len = 8;
        CanWrite (cf);
    }

    {//补充模块温度统计信息报文
    
        (void) memcpy (cf->data, (UINT8 *)&LECU_Rt[dev - 1] + 24, 8);  //lint !e662 !e670

        cf->id = 0x13000010 + 0x100;//
        cf->len = 8;
        CanWrite (cf);
    
   }
    

       cf->id  = temp_id;

    //cv = RT_Volts (dev - 1);
    if (!cv)
        return;

    for (i = 0; i < (LECU_Rt[dev - 1].bodys & 0x7f); i += 4)
    {
       // (void) memcpy (cf->data, (UINT8 *)cv + i*2, 8);  //lint !e662 !e670
        
        cf->data[0] = Cell_Volts[Cell_Indexs[dev - 1]+i]>>8;
        cf->data[1] = Cell_Volts[Cell_Indexs[dev - 1]+i];
        cf->data[2] = Cell_Volts[Cell_Indexs[dev - 1]+i+1]>>8;
        cf->data[3] = Cell_Volts[Cell_Indexs[dev - 1]+i+1];
        cf->data[4] = Cell_Volts[Cell_Indexs[dev - 1]+i+2]>>8;
        cf->data[5] = Cell_Volts[Cell_Indexs[dev - 1]+i+2];
        cf->data[6] = Cell_Volts[Cell_Indexs[dev - 1]+i+3]>>8;
        cf->data[7] = Cell_Volts[Cell_Indexs[dev - 1]+i+3];
        
        cf->id++;
        cf->len = 8;
        CanWrite (cf);
    }
    
  
    
    
    
    
    
    
    
    
}

/*****************************************************************************/
/* PC后台通讯协议 - 手工均衡及其他控制                                       */
/*****************************************************************************/
static void BalEtc ( CanbusFrame *cf)
{
       UINT8  COMMOND = 0,send = 0,device = 0;
       UINT32    in = 0,out = 0;;
   
   
   
  device =  cf->data[4]; //模块地址   0Xff为主控本机
  //if(device!=0xff) device -= 1;//上位机已经减了
  COMMOND = cf->data[5]; //命令类型
   
   switch (COMMOND)
    {
      case 1://read in
      
             if(device==0xff)
             {
              
                 in = HVIL<<5|ACC_ON<<4|CC2<<3|IO_IN2<<2|IO_IN1<<1|IO_IN0;
                 in |= 0xffffffC0;
                 
                
                 
                 
                  *(UINT32 *) &cf->data[0]   =  in ;  //内部大小端模式与gb can协议不同
             } 
             else //模块读取
             {
                *(UINT32 *) &cf->data[0]   =  LECU_Rt[device].in_stauts ;
                
             }
             
             
              send =1;  
              Modify_flg = TRUE; 
            break;
   
      case 2://write in
            
              if(device==0xff)
             {
              
                
             } 
             else //模块读取
             {
             
                
             }
             
            
            
            
            Modify_flg = TRUE; 
            break;
      case 3: //read out
      
            if(device==0xff)
             {
          
                out =(IO_OUT5<<5) |(IO_OUT4<<4)|(IO_OUT3<<3)|(IO_OUT2<<2)|(IO_OUT1<<1)|IO_OUT0; 
                out |= 0xffffffC0 ;
               
                  *(UINT32 *) &cf->data[0]   =  out ;  //内部大小端模式与gb can协议不同
              
              
              
              } 
             else //模块读取
             {
                *(UINT32 *) &cf->data[0]   =  LECU_Rt[device].out_stauts ;  //内部大小端模式与gb can协议不同
                
             }
             
              Modify_flg = TRUE;
              send =1;   
            
            
            
            
            break;
   
   
      case 4: //write out
      
      
          if(device==0xff)
             {
               
                out =  	*(UINT32*)&cf->data[0];
              
      
                if(!(out&1))IO_OUT0 = 0;
                else  IO_OUT0 = 1;
                out >>= 1 ;
                if(!(out&1))IO_OUT1 = 0;
                else  IO_OUT1 = 1; 
                 out >>= 1 ;
                if(!(out&1))IO_OUT2 = 0; 
                else  IO_OUT2 = 1;
                out >>= 1 ;
                if(!(out&1))IO_OUT3 = 0; 
                else  IO_OUT3 = 1;
                out >>= 1 ;
                if(!(out&1))IO_OUT4 = 0; 
                else  IO_OUT4 = 1;
                out >>= 1 ;
                
                if(!(out&1))IO_OUT5 = 0; 
                else  IO_OUT5 = 1;
                out >>= 1 ;
                
            
             } 
             else //模块写
             {
                 
               out =  *(UINT32 *) &cf->data[0] ;
               
              
             
             
         if( PTL_LECU) 
         {
         
                    LECU_Rt[device].cmd |= 0x04;
                     if((out & 0x02))
                     {
                         LECU_Rt[device].cmd |= 0x01;
                     
                     }
                     
                     else   LECU_Rt[device].cmd &= ~0x01;   //关闭
                    if((out & 0x01))
                     {
                     
                        LECU_Rt[device].cmd |= 0x02;
                     
                     }
                     
                     else   LECU_Rt[device].cmd &= ~0x02;   //关闭   
         
         
              
         }
             
         if( PTL_QDWX) 
         {
          
                     if((out & 0x02))
                     {
                        LECU_Rt[device].cmd &= ~0x01;   //关闭  
                     
                     }
                     
                     else    LECU_Rt[device].cmd |= 0x01;
                    if((out & 0x01))
                     {
                     
                           LECU_Rt[device].cmd &= ~0x02;   //关?
                     
                     }
                     
                     else  LECU_Rt[device].cmd |= 0x02;  
                
         }
                
                
                                   
                
             }
             
            
             
             
            Modify_flg = TRUE;
            break;
            
      case 7://read  balence
             
              if(device==0xff)
             {
              
                
             } 
             else //模块读取
             {
                 *(UINT32 *) &cf->data[0] = LECU_Rt[device].balsts; //32-1
                 cf->data[4] = LECU_Rt[device].balsts2; //36-33    
             }
             Modify_flg = TRUE;
            break;
      case 8:
            
              if(device==0xff)
             {
              
                
             } 
             else //模块读取
             {
                LECU_Rt[device].balsts =  *(UINT32 *) &cf->data[0];
                LECU_Rt[device].balsts2 = cf->data[6];
                LECU_Rt[device].cmd |= COMMOND;
                
             }
            
           Modify_flg = TRUE;
          break;  
          default:
          break;
    }
   

    if(send) CanWrite ( cf);
    
    
    

}
/*****************************************************************************/
/* PC后台通讯协议 - 读版本信息                                               */
/*****************************************************************************/
static void ReadVerInfo (CanbusFrame *cf)
{
    UINT16  i;

    /*主控*/
    (void) memcpy (&cf->data[0], (void *) 0xFFA0, 4);
    (void) memcpy (&cf->data[4], (void *) 0xDC00, 2);
    (void) memcpy (&cf->data[6], (void *) &OEMVer, 2);

    cf->id = 0x13000;
    cf->len = 8;
    cf->data[7]=Project_Num;
    CanWrite (cf);
    udelay (100);

    (void) memcpy (&cf->data[0], (void *) &HWVer, 2);
    cf->id = 0x13001;
    cf->len = 2;
    CanWrite (cf);
    udelay (100);

    /*从控*/
    for (i = 0; i < DEV_MAX; i++)
    {
        (void) memcpy (&cf->data[0], &LECU_Rt[i].swver, 8); //lint !e420

        cf->id = 0x13002;
        cf->len = 8;
        CanWrite (cf);
        udelay (100);

        cf->data[0] = LECU_Rt[i].bodys & 0x7f;
        cf->data[1] = LECU_Rt[i].tmpnum & 0x7f;
        cf->data[2] = i + 1;
        cf->data[3] = (UINT8)(LECU_Rt[i].soh >> 8);
        cf->data[4] = (UINT8)(LECU_Rt[i].soh & 0xff);
        cf->id = 0x13003;
        cf->len = 8;
        CanWrite (cf);
        udelay (100);
    }
    
    /*绝缘*/
    (void) memcpy (&cf->data[0], &ISO_message.iso_sn, 4);
    cf->id = 0x13004;
    cf->len = 4;
    CanWrite (cf);
    udelay (100);
    
    (void) memcpy (&cf->data[0], &ISO_message.iso_oem, 2);
    cf->data[2] = ISO_message.iso_core;
    cf->data[3] = ISO_message.iso_swer;
    (void) memcpy (&cf->data[4], &ISO_message.iso_hwer, 2);
    cf->id = 0x13005;
    cf->len = 6;
    CanWrite (cf);
    udelay (100);
    
}

/*****************************************************************************/
/* PC后台通讯协议 - 设置SOC等等                                              */
/*****************************************************************************/
static void SetXXX (const CanbusFrame *cf)
{
    UINT8   what = cf->data[0];
    UINT8   j,i,N,id;
    UINT32  sn;
    static BOOL Modified = FALSE;

    switch (what)
    {
    case 0:
        RT_SetSOC (cf->data[1]);
        break;
        
    case 100:
        if((Sys_Shared.C0_Raw <= 20) && (Sys_Shared.C0_Raw >= -20))
        {
            OEM_Para.c0 = Sys_Shared.C0_Raw;
            OEM_Para.crc = Modbus_crc ((UINT8 *)&OEM_Para, sizeof(OEM_Para) - 2);
            Fm_write (OEMPARA_START, &OEM_Para, sizeof(OEM_Para));
        }
        break;
    

    case 200:
        N = cf->data[1];
        i = cf->data[2];
        sn = ((UINT32)cf->data[3] << 24) | ((UINT32)cf->data[4] << 16) | ((UINT32)cf->data[5] << 8)| cf->data[6];
        id = cf->data[7] - 1;

        /*if (id == 1)
            Modified = FALSE;*/
        if (cf->data[7] == 0)
            Modified = FALSE;    

        if (id >= DEV_MAX || i > N)
            break;

        for (j = 0; j < DEV_MAX; j++)
        {
            UINT8 *a;
            UINT8 *b;
            UINT8 k;
            UINT8 m;

            if( LECU_Para[j].serial_num != sn )  //找到SN对应的第J个参数表
                continue;

            if (j == id)
                break;

            Modified = TRUE;

            a = (UINT8 *)&LECU_Para[id];
            b = (UINT8 *)&LECU_Para[j];
            for (m = 0; m < DEV_MAX; m++)           //清除与被操作对象SN相同的id
            {
                if (m == j)
                    continue;
                if (LECU_Para[m].serial_num == sn)
                    LECU_Para[m].serial_num = 0;   
            }
            for (k = 0; k < sizeof(DBDevPara); k++)//采集板参数交换
            {
                UINT8 tmp = a[k];
                a[k] = b[k];
                b[k] = tmp;
            }
            break;
        }

        if (i == N && Modified)
        {
            /*所有参数一起保存*/
            Fm_write (LECU_PARA_START, &LECU_Para[0] , sizeof(LECU_Para));

            Modified = FALSE;
#if PTL_LECU
            LECU_ReInit ();
#endif
        }
        break;
    }
}


/*****************************************************************************/
/* PC后台通讯协议 - 电流校准                                                 */
/*****************************************************************************/
static void CurAdj (CanbusFrame *cf)
{
    UINT8   i = cf->data[0];
    UINT16  ref = (cf->data[1] << 8) | cf->data[2];
    SINT16  raw = Sys_Shared.CurrentRaw;
    UINT32  tmp = 0;
    
    if (i >= 2)
      return;

    if (raw < 0) raw = -raw;

    /*运放放大倍数约为19*/
   // if (raw / ref < 15)
    //    return;
    
    OEM_Para.Raw = raw;
    OEM_Para.Ref = (SINT16)ref;  //ref实测电流值 
    
    
    
    

    OEM_Para.crc = Modbus_crc ((UINT8 *)&OEM_Para, sizeof(OEM_Para) - 2);
    Fm_write (OEMPARA_START, &OEM_Para, sizeof(OEM_Para));
}

/*****************************************************************************/
/* PC维护协议入口函数                                                        */
/*****************************************************************************/
void PC_main (void)
{
    CanbusFrame *cf;
    UINT8 i = 0;

    if (CAN0_RxRPtr == CAN0_RxWPtr)
      return;

    cf = &CAN0_RxBuf[CAN0_RxRPtr];

    switch (cf->id)
    {
    case 0x10D806BD:  // 按日读历史数据
        //Read_flag = 1;     //wjm20241120 
        if (Flash_busy())                         /*如果忙，则推迟读操作*/
        {
          cf->pri = 0;
          return;
        }

        if (cf->len == 0)
            ListRec (cf);
        else if (cf->len == 4 || cf->len == 7)
            ReadHst (cf);
        else if (cf->len == 8 &&
                 cf->data[0] == 0xee && cf->data[1] == 0x55 && cf->data[2] == 0xaa && cf->data[3] == 0x88 &&
                 cf->data[4] == 0x00 && cf->data[5] == 0x00 && cf->data[6] == 0x00 && cf->data[7] == 0x00)
            HST_reset ();
        break;

    case 0x10D806BF:  //
        if (Flash_busy())                         /*如果忙，则推迟读操作*/
          return;

        ReadFlash (cf);
        break;

    case 0x10D80721:  // 读报警数据
        break;
        
    case 0x10D81991: //报警参数
        if (cf->len == 8 && cf->data[0] == 'R')
            ReadAlarmPara (cf);
        else if (cf->len == 8 && cf->data[0] == 'W')
        {
            WriteAlarmPara ();
            return;
        }
        break;    

    case 0x10D81995: //系统参数
        if (cf->len == 0)
          ReadSysPara ();
        else if (cf->len == 2 && cf->data[0] == 0 && cf->data[1] == sizeof(Sys_Para) - 2)
        {
            WriteSysPara ();
            return;
        }
        break;

    case 0x10D82992: //OEM参数
        if (cf->len == 0)
          ReadOEMPara ();
        else if (cf->len == 2 && cf->data[0] == 0 && cf->data[1] == sizeof(OEM_Para) - 2)
        {
            WriteOEMPara ();
            return;
        }
        else if (cf->len == 3 && cf->data[2] == 1)
        {
            OEM_Para.Raw = 1000;                                
            OEM_Para.Ref = 1000; // 0.1mv   缺省霍尔系数:4
            OEM_Para.c0 = 0;    
 
            OEM_Para.crc = Modbus_crc ((UINT8 *)&OEM_Para, sizeof(OEM_Para) - 2);
        
            Fm_write (OEMPARA_START, &OEM_Para, sizeof(OEM_Para));
        }
        break;

    case 0x10D81992: //设备参数
        if (cf->len == 1)
          ReadDevPara (cf->data[0]);
        else if (cf->len == 2 && cf->data[1] == sizeof(DBDevPara) - 10/*SN,RegCnt,crc*/)
        {
            WriteDevPara (cf->data[0]);
            return;
        }
        break;
        
    case 0x10D81994: // 电池及车辆部分参数
        if (cf->len == 0)
            ReadBattPara ();
        else if (cf->len == 2 && cf->data[0] == 0 && cf->data[1] == sizeof (Batt_Para) - 2)
        {
             WriteBattPara ();
             return;
        }
        break;

    case 0x10D84455:  //时间处理



        if (cf->data[0] == 0xAA) service = FALSE;

        else
       {
        
         service = TRUE;//如果进入系统维护状态

         TimeExt (cf);
       }
        break;

    case 0x10D84459:  //读BMS出厂编号和地址

        cf->len = 7;
        cf->data[0] = *(UINT8 *)0xFFA0;
        cf->data[1] = *(UINT8 *)0xFFA1;
        cf->data[2] = *(UINT8 *)0xFFA2;
        cf->data[3] = *(UINT8 *)0xFFA3;
        cf->data[4] = OEM_Para.bms_addr;
        cf->data[5] = SWVer >> 8;
        cf->data[6] = SWVer & 0xff;

        CanWrite (cf);
        break;

    case 0x10D8445A:  //版本信息
        if (cf->len == 0)
          ReadVerInfo (cf);
        break;
        
    case 0x10D8445B:  //维护状态读取
    
        cf->len = 7;
        cf->data[0] = 1;
        cf->data[1] = 2;
        cf->data[2] = 3;
        cf->data[3] = 4;
        cf->data[4] = 5;
        cf->data[5] = service;//1 维护模式，，0正常模式
        cf->data[6] = 0;

        CanWrite (cf);
        
        
        break;    

    case 0x10D8445E:  //读报警信息
        EventRec (cf);
        break;

    case 0x10D81997:  //实时数据
        //Read_flag = 0;
        ReadRt (cf);
        break;

    case 0x10D81998:  //均衡控制及其他控制
        if (cf->data[7] == 0xAA)
        {
            service = FALSE;
            if (Modify_flg)
            {
            for(i=0;i<20;i++) 
            {
              
              
               
               if( PTL_LECU) 
               {
         
                  // LECU_Rt[i].cmd &= ~0x04;
                   LECU_Rt[i].cmd |= 0x03;
              
             }
             
              if( PTL_QDWX) 
               {
               
                  LECU_Rt[i].cmd &= ~0x03;
               }
            }
            for (i = 0; i < DEV_MAX; i++)
                {
                    LECU_Rt[i].balsts = 0;
                    LECU_Rt[i].balsts2 = 0;
                    LECU_Rt[i].cmd &= ~0x8;
                }
                IO_OUT0 = 1;
                IO_OUT1 = 1;
                IO_OUT2 = 1;
                IO_OUT3 = 1;
                IO_OUT4 = 1;
                /*IO_OUT5 = 1;
                IO_OUT6 = 1;
                IO_OUT7 = 1;
                IO_OUT8 = 1;
                IO_OUT9 = 1;*/
                Modify_flg = FALSE;}
        }
        else
        {
            service = TRUE;
           BalEtc (cf);
        }       
        break;
        
    case 0x10D81999:  //
        CurAdj (cf);
        break;

    case 0x10D8199b:  //
        SetXXX (cf);
        break;

    case 0x10D82100:
    case 0x10D82020:
    case 0x10D82021:
        PITINTE_PINTE3 = 0;         /*PIT中断关闭,停止采集板数据招测*/
        mdelay (30);

        cf->id &= 0x1FF;
        CAN4_write_std (cf);

        mdelay (10);
        PITINTE_PINTE3 = 1;
        break;

    default:
        /*交给其他规约模块处理*/
        return;
    }
    
    

    /*删除当前帧*/
    CAN0_RxRPtr = (CAN0_RxRPtr + 1) & (CAN0_RXBSZ - 1);
}





























