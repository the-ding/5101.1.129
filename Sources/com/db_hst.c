#include <string.h>
#include <stddef.h>
#include "sys.h"
#include "db.h"
#include "ptl.h"
#include "drv_fm.h"
#include "drv_flash.h"
#include "deflate.h"
#include "zlib.h"

#define BIG         'W' // 以前是 'A'
#define SMALL       'w' // 以前是 'a'

/*临时历史数据在铁电中的起始位置*/
#define TMPHST_START    (UINT32)(DAY_INDEX_START + DAY_INDEX_MAX*DAY_INDEX_SIZE)


/*FLASH当前写位置信息*/
typedef struct
{
    UINT32    addr;     /* FLASH当前写地址 */
    UINT32    copy1;    /* addr的复制      */
    UINT32    copy2;    /* addr的复制      */
} FlashWritePosRec;


static FlashWritePosRec   flash_winfo;          /*FLASH当前写地址*/
static DBDayIndexRec      today;                /*今日历史数据摘要信息*/
static UINT16             today_fm;             /*today在铁电中的存储位置*/

static time_t             lastHistTime;         /*上次历史数据保存时间*/
static BOOL               bigRec;               /*上电、整点、跨日、电压突变：保存大记录*/
static BOOL               saveRec;              /*强制生成一条记录*/
static BOOL               firstRec;             /*上电第一条记录*/
//static UINT16             shotcut[SYS_CELL_MAX];/*每个采集板的实时数据*/
static SINT16             lastCurrent;          /*上一次生成记录时的电流*/

static UINT32             fm_faddr;             /*铁电中的历史数据在FLASH中的起始地址*/
static UINT16             fm_wpos;              /*临时历史数据在铁电中的保存位置*/
static UINT16             fm_rpos;
static UINT8              fm_state;             /*转存工作状态：0(空闲)，1(擦除)，2(可编程)*/


/*压缩库使用的变量，谨慎修改!*/
static z_stream       strm;
static deflate_state  deflate_memory;

#pragma DATA_SEG SHARED_DATA
UINT8                 zbuf[512];                /*压缩输出比特流(临时数组)*/
#pragma DATA_SEG DEFAULT

#pragma DATA_SEG __GPAGE_SEG PAGED_RAM
static uInt           window_memory[1 << MAX_WBITS];
static Pos            prev_memory[1 << MAX_WBITS];
static Pos            head_memory[1 << (MAX_MEM_LEVEL + 7)];
static Byte           overlay_memory[(1 << (MAX_MEM_LEVEL + 6)) * (sizeof(ush)+2)];
#pragma DATA_SEG DEFAULT

/*OUT变化的原因*/
volatile UINT16 OUTchg_reson = 0x0;       // OUT变化原因
volatile UINT16 IN_pre = 0xffff;          // IN上一次状态
volatile UINT16 OUT_pre = 0xffff;         // OUT上一次状态
static BOOL IO_change = FALSE;            // IO变化标志位
volatile BOOL OUT_chgnum[16] = {FALSE};   // OUT变化的序号,0-4,闭合OUT为FALSE,断开OUT为TRUE
static UINT32 LastSec = 0;
/*****************************************************************************/
/* 实时数据模块初始化                                                        */
/*****************************************************************************/
void HST_Init (void)
{
    UINT16  i, oldest = 0;
    time_t  victim = 0xffffffff;
    DBDayIndexRec tmp;

    /*尝试恢复FLASH当期写位置信息*/
    Fm_read (FLASH_WINFO_START, &flash_winfo, sizeof(flash_winfo));

    if (flash_winfo.addr != flash_winfo.copy1)
    {
        if (flash_winfo.copy1 == flash_winfo.copy2)
            flash_winfo.addr = flash_winfo.copy1;
    }

    /*循环存储*/
    /*ZYY HIST_RSVD=2000保留2000字节，防止FLASH中最后一条记录空间跨段
          防止将历史记录写到报警记录的Flash中*/
    if (flash_winfo.addr + HIST_RSVD >= FLASH_HIST_END)
    {
        flash_winfo.addr = FLASH_HIST_START;
    }

    /*尝试调出今日历史数据摘要信息*/
    /*ZYY gSysSec是将日期转化为以秒为单位的时间,每天有86400秒,
          today.cursor则是今天将小时分钟和秒换算成秒值,
          today.time则是出去今天时分秒的日期值换算成的秒值*/
    today.cursor = gSysSec % 86400UL;
    today.time = gSysSec - today.cursor;
    today.magic = 0;

    for (i = 0; i < DAY_INDEX_MAX; i++)
    {
        /*ZYY today_fm为today在铁电中的存储位置,从历史数据索引在铁电的存储起始位置
              逐次向下查找与本次上电日期相同的索引*/
        today_fm = DAY_INDEX_START + i*DAY_INDEX_SIZE;
        Fm_read (today_fm, &tmp, sizeof(today));

        if (tmp.magic == 'y')
        {
            if (tmp.time == today.time)
            {
                (void) memcpy (&today, &tmp, sizeof(today));
                break;
            }
        }
        else
        {
            tmp.time = 0;
        }

        /*定位最早一日的摘要信息*/
        if (tmp.time < victim)
        {
            victim = tmp.time;
            oldest = i;
        }
    }

    /*没有找到今日的摘要信息，重新生成记录*/
    /*ZYY 若没有找到今日摘要信息,则将今日摘要信息覆盖最早一日的摘要信息(索引区域已经填满)
          flash每次写完,地址会根据所写内容的大小递增,所以初始化时只有今日日期时间,
          无实质内容,索引中Flash起始结束地址相同*/
    if (today.magic != 'y')
    {
        today_fm = DAY_INDEX_START + oldest*DAY_INDEX_SIZE;
        today.magic = 'y';
        today.flash = flash_winfo.addr;
        today.end   = flash_winfo.addr;

        Fm_write (today_fm, &today, sizeof(today));
    }

    /*上一次保存历史数据时间*/
    /*将本次上电日期记为上次上电日期,用于判断是否跨日*/
    lastHistTime = today.time;

    /*强制生成一条记录*/
    bigRec = TRUE;/*ZYY bigRec上电作为保存大记录*/
    saveRec = TRUE;/*ZYY saveRec强制生成一条记录*/
    firstRec = TRUE;/*ZYY firstRec上电第一条记录*/

    /*临时历史数据在铁电中的保存位置指针*/
    fm_faddr = flash_winfo.addr;
    fm_wpos = fm_rpos = TMPHST_START;
    fm_state = 0;
    LastSec = gSysSec;
}

/*****************************************************************************/
/* 根据历史数据头判断记录是否有效                                            */
/*****************************************************************************/
BOOL HST_IsValidHdr (DBHistRecHdr *hdr)
{
    UINT8 typ = hdr->type | 0x20;

    return typ == 'a' || typ == 'w';
}

/*****************************************************************************/
/* 判断历史数据记录头尺寸                                                    */
/*****************************************************************************/
UINT16 HST_HdrSize (DBHistRecHdr *hdr)
{
    UINT8 typ = hdr->type | 0x20;
    
    // 以前生成的记录
    if (typ == 'a')
        return sizeof(*hdr) - 4;

    return sizeof(*hdr);
}

/*****************************************************************************/
/* 根据时间查找历史数据                                                      */
/*****************************************************************************/
BOOL HST_find (time_t from, DBDayIndexRec *rec, DBHistRecHdr *hdr, UINT32 *faddr)
{
    UINT16  i;
    time_t  day = from - (from % 86400UL);
    UINT32  addr;
    DBDayIndexRec tmp;

    for (i = 0; i < DAY_INDEX_MAX; i++)
    {
        UINT16  fm = DAY_INDEX_START + i*DAY_INDEX_SIZE;
        Fm_read (fm, &tmp, sizeof(tmp));
        /*ZYY 判断是否能找到指定日期索引*/
        if (tmp.magic == 'y' && tmp.time == day)
          break;
    }

    if (i == DAY_INDEX_MAX)                               /*没有指定日数据*/
      return FALSE;

    (void) memcpy (rec, &tmp, sizeof(tmp));
    addr = tmp.flash;/*ZYY 该索引在Flash的起始地址*/
    for (;;)
    {
        if (tmp.flash < tmp.end)                          /*日历史数据边界检查*/
        {
            if (addr >= tmp.end || addr < tmp.flash)
              return FALSE;
        }
        else
        {
            if (addr >= tmp.end && addr < tmp.flash)
              return FALSE;
        }
        /*ZYY 读出Flash中该指定日的历史数据*/
        Flash_read (addr, hdr, sizeof(*hdr));             /*先检查历史数据头*/
        /*ZYY 根据历史数据头判断记录是否有效*/
        if (!HST_IsValidHdr(hdr) || addr + hdr->length >= FLASH_HIST_END) /*检查记录有效性*/
          return FALSE;

        if (faddr != NULL)
          *faddr = addr + HST_HdrSize(hdr);/*ZYY 判断历史数据记录头尺寸 faddr为该记录的结束地址*/

        if (hdr->time < from)                             /*本记录早于指定时间*/
        {
            addr += (UINT16)(HST_HdrSize(hdr) + hdr->length);    /* 下一条记录地址 */

            if (addr + HIST_RSVD >= FLASH_HIST_END)
              addr = FLASH_HIST_START;

            continue;
        }

        return TRUE;
    }
}

/*****************************************************************************/
/* 读历史数据                                                                */
/*****************************************************************************/
void HST_read (UINT32 faddr, void *buf, UINT16 bytes)
{
    Flash_read (faddr, buf, bytes);
}

/*****************************************************************************/
/* 历史数据清零                                                              */
/*****************************************************************************/
void HST_reset (void)
{
    UINT16  i;
    DBDayIndexRec tmp;

    flash_winfo.addr = flash_winfo.copy1 = flash_winfo.copy2 = 0;
    lastHistTime = gSysSec - 60;

    /*临时历史数据在铁电中的保存位置指针*/
    fm_wpos = fm_rpos = TMPHST_START;
    fm_faddr = 0;
    fm_state = 1;                               /*等待擦除完成*/
    Flash_erase (fm_faddr);

    (void) memset (&tmp, 0, sizeof(tmp));
    for (i = 1; i < DAY_INDEX_MAX; i++)
    {
        UINT16 fm = DAY_INDEX_START + i*DAY_INDEX_SIZE;
        Fm_write (fm, &tmp, sizeof(tmp));
    }

    today_fm = DAY_INDEX_START;
    today.cursor = gSysSec % 86400UL;
    today.time = gSysSec - today.cursor;
    today.magic = 'y';
    today.flash = flash_winfo.addr;
    today.end = flash_winfo.addr;

    Fm_write (today_fm, &today, sizeof(today));

    /*强制生成一条记录*/
    bigRec = TRUE;
    saveRec = TRUE;
    LastSec = gSysSec;
}

/*****************************************************************************/
/* 压缩并写入铁电                                                            */
/*****************************************************************************/
static int ZipWrite (const void *in, UINT16 sz)
{
    int rc;

    strm.next_in = in;
    strm.avail_in = sz;
    
    rc = deflate (&strm, Z_NO_FLUSH);
    
    while (rc == Z_OK && strm.avail_out == 0)
    {
        Fm_write (fm_wpos, zbuf, strm.total_out);
        fm_wpos += strm.total_out;
        
        strm.next_out = zbuf;
        strm.avail_out = sizeof(zbuf);
        strm.total_out = 0;
        
        rc = deflate (&strm, Z_NO_FLUSH);
    }
    return rc;
}

/*****************************************************************************/
/* 生成一个历史数据记录                                                      */
/*****************************************************************************/
static void MkHst (time_t now)
{
    int         rc;
    UINT16      i, j;
    UINT16      fm_hdr;
    DBHistRecHdr  hdr;
    SINT8       volts[DEV_CELL_MAX];

    if (flash_winfo.addr + HIST_RSVD >= FLASH_HIST_END)  /*FLASH空间循环存储*/
    {
        flash_winfo.addr = FLASH_HIST_START;
        
        /*强制生成一条记录*/
        saveRec = TRUE;
        bigRec = TRUE;
    }

    if (fm_wpos == fm_rpos)
    {
        fm_wpos = fm_rpos = TMPHST_START;
        fm_faddr = flash_winfo.addr;
    }
    else if (fm_wpos + HIST_RSVD >= FG_CHG_START)
    {
        /*FLASH可能写失败*/
        return;
    }
    
    /*补一条停送电记录*/
    if (firstRec)
    {
        firstRec = FALSE;

        hdr.type = SMALL;
        hdr.time = now;                             /*现在时刻*/
        hdr.length = 0;
        hdr.lecu = 0;
        hdr.current = (int)today.cursor;            /*停电时刻*/
        hdr.soc = 0;
        hdr.soh = 0;
        //hdr.com_qua   = 0;                           /*通信质量,取代soh位置*/
        
        hdr.isovolt   = 0;
        hdr.posres    = 0;
        hdr.relayN1   = 0;
        hdr.vol_pre   = 0;
        
        //hdr.warning   = 0;
        hdr.relayP1 = 0;
        hdr.relayP2 = 0;
        
        hdr.maxt      = 0;
        hdr.mint      = 0;
        hdr.in_sts    = 0;
        hdr.out_sts   = 0;
        hdr.out_reson = 0;

        /*记录头*/
        fm_hdr = fm_wpos;
        fm_wpos += sizeof(hdr);
        
        /*写历史数据头*/
        Fm_write (fm_hdr, &hdr, sizeof(hdr));

        /*更新FLASH写位置信息*/
        flash_winfo.addr += sizeof(hdr);
    }

    /*本日最后一次存储历史数据时间*/
    today.cursor = now % 86400UL;

    if (lastHistTime/86400UL != now/86400UL)          /*处理跨日情况*/
    {
        today_fm += DAY_INDEX_SIZE;                   /*自动覆盖下一条记录*/

        if (today_fm > DAY_INDEX_START + DAY_INDEX_MAX*DAY_INDEX_SIZE)
          today_fm = DAY_INDEX_START;

        today.time  = now - today.cursor;
        today.magic = 'y';
        today.flash = flash_winfo.addr;
        today.end   = flash_winfo.addr;
        
        /*强制生成一条记录*/
        saveRec = TRUE;
        bigRec = TRUE;
    }

    if (lastHistTime/3600UL != now/3600UL)            /*整点时刻*/
    {
        /*强制生成一条记录*/
        saveRec = TRUE;
        bigRec = TRUE;
    }
    
    if (IO_change)
    {
        /*强制生成一条记录*/
        saveRec = TRUE;
        bigRec = TRUE;
    }

    /*考虑不生成历史记录*/
    if (!saveRec)
    {
        //if (OEM_Para.curr_delta < 20) OEM_Para.curr_delta = 20;   /*2A*/
        //if (OEM_Para.volt_delta < 3)  OEM_Para.volt_delta = 3;    /*3mV*/

        /*首先判断总电流是否出现突变*/
        rc = Sys_Rt.current - lastCurrent;
        if (rc < 0)
          rc = -rc;

        /*ZYY OEM_Para.curr_delta总电流突变超过该值时自动存储一条记录,3A*/
        if (rc > OEM_Para.curr_delta)
          goto MkDot;

        /*判断单体电压是否出现突变，如果出现则需要生成一个大记录*/
        for (i = 0; i < DEV_MAX; i++)
        {
     /*       UINT16 *cv = RT_Volts ((UINT8) i);
            //ZYY shotcut[Cell_Indexs]每个采集板的实时数据
            UINT16 *sv = &shotcut[Cell_Indexs[i]];
            UINT8 bodys = LECU_Rt[i].bodys & 0x7f;
            
            if (!bodys)
              continue;

            for (j = 0; j < bodys; j++)
            {
//                rc = (int)sv[j] - (int)cv[j];
                if (rc < 0)
                  rc = -rc;

                if (rc >= 127)
                {
                    bigRec = TRUE;
                    goto MkDot;
                }
                //ZYY OEM_Para.volt_delta电池电压突变超过该值时自动存储一条记录5mv
                if (rc >= OEM_Para.volt_delta)
                  goto MkDot;
            }  */
        }

        /*不需要生成历史记录，只要修改本日最后存储时间*/
      //  Fm_write (today_fm, &today, sizeof(today));
      //  return;
    }

MkDot:
    saveRec = FALSE;
    
    (void) memset (&strm, 0, sizeof(strm));
    
    strm.state = (struct internal_state *)&deflate_memory;  //lint !e740
    deflate_memory.window = (UINT8 *far)window_memory;
    deflate_memory.prev   = prev_memory;
    deflate_memory.head   = head_memory;
    deflate_memory.pending_buf = overlay_memory;
    (void)deflateInit(&strm, Z_BEST_COMPRESSION);
    
    /**/
    strm.next_out = zbuf;
    strm.avail_out = sizeof(zbuf);
    strm.total_out = 0;


    /*整车信息*/
    lastCurrent = Sys_Rt.current;
    hdr.current = lastCurrent;
    hdr.isovolt = (UINT16) Sys_Rt.total_vol_res;
    hdr.posres = (UINT16) Sys_Rt.pos_res;
    //hdr.negres = (UINT16) Sys_Rt.neg_res;
    hdr.relayN1 = Sys_Rt.NCNC;          /*继电器负1电压,取代总负绝缘电阻*/
    hdr.vol_pre = Sys_Rt.relayPRE;
    hdr.soh = 100;
    //hdr.com_qua   = 0;
    hdr.soc = (UINT8)Sys_Rt.soc;
    //hdr.warning = Sys_Alarm.warning1;
    hdr.relayP1 = Sys_Rt.relayFAST;          /*继电器正1电压,取代warning位置*/
    hdr.relayP2 = Sys_Rt.relaySLOW;          /*继电器正2电压,取代warning位置*/
    hdr.maxt = (UINT8)Sys_Rt.max_t;
    hdr.mint = (UINT8)Sys_Rt.min_t;
    hdr.in_sts = IN_pre;
    hdr.out_sts = OUT_pre;
    hdr.out_reson = OUTchg_reson;
    OUTchg_reson = 0;

    hdr.time = now;
    hdr.length = 0;
    hdr.lecu = 0;

    /*记录头*/
    fm_hdr = fm_wpos;
    fm_wpos += sizeof(hdr);

    /*首先压缩保存普通信息(soc,soh,sn)，然后是温度数据，最后是电池电压数据*/
    strm.total_in = 0;

    /*至少保留2字节*/
    j = 16;
    
    for (i = 0; i < DEV_MAX; i++)
    {
        UINT8   tmps = LECU_Rt[i].tmpnum & 0x7f;
        UINT8   bodys = LECU_Rt[i].bodys & 0x7f;

        if (!bodys && !tmps)
          continue;



        zbuf[j++] = LECU_Rt[i].soc & 0xff;
        zbuf[j++] = LECU_Rt[i].soh & 0xff;
        zbuf[j++] = bodys;
        zbuf[j++] = tmps;

#if OEM == OEM_DEBUG || PTL_LECU
        zbuf[j++] = (LECU_Para[i].serial_num >> 24) & 0xff;
        zbuf[j++] = (LECU_Para[i].serial_num >> 16) & 0xff;
        zbuf[j++] = (LECU_Para[i].serial_num >> 8) & 0xff;
        zbuf[j++] = (LECU_Para[i].serial_num & 0xff);
#endif

        hdr.lecu++;
    }
    
    rc = ZipWrite (zbuf + 16, j - 16);
    if (rc != Z_OK)
    {
err:    fm_wpos = fm_hdr;
        return;
    }

    /*温度数据*/
    for (i = 0; i < DEV_MAX; i++)
    {
        UINT8   tmps = LECU_Rt[i].tmpnum & 0x7f;

        if (!tmps)
          continue;

        rc = ZipWrite (LECU_Rt[i].tmps, tmps);

        if (rc != Z_OK)
          goto err;
    }
     bigRec= TRUE;
    /*电池电压数据*/
//    if (bigRec)
    {
        bigRec = FALSE;
        hdr.type = BIG;

        for (i = 0; i < DEV_MAX; i++)
        {
            UINT16 *sv = &Cell_Volts[Cell_Indexs[i]];
            UINT16 *cv = RT_Volts ((UINT8) i);
            UINT8 bodys = LECU_Rt[i].bodys & 0x7F;
            
            if (!bodys)
              continue;

    //        for (j = 0; j < bodys; j++)
    //            sv[j] = cv[j];

            rc = ZipWrite (sv, bodys*2);
            if (rc != Z_OK)
              goto err;
        }
    }
/*    else
    {
        hdr.type = SMALL;

        for (i = 0; i < DEV_MAX; i++)
        {
            UINT16 *cv = RT_Volts ((UINT8) i);
            UINT16 *sv = &Cell_Volts[Cell_Indexs[i]];
            UINT8 bodys = LECU_Rt[i].bodys & 0x7F;

            if (!bodys)
              continue;

            for (j = 0; j < bodys; j++)
            {
                UINT16 vv = cv[j];
                
                rc = (int)sv[j] - (int)vv;

                if (rc <= -127 || rc >= 127)
                {
                    bigRec = TRUE;
                    fm_wpos = fm_hdr;
                    goto MkDot;
                }
                
                if (rc <= -5 || rc >= 5)
                {
                    sv[j] = vv;
                    volts[j] = (SINT8) rc;
                }
                else
                {
                    volts[j] = 0;
                } 
            }

            rc = ZipWrite (volts, bodys);
            if (rc != Z_OK)
              goto err;
        }
    }*/

    strm.avail_in = 0;

    rc = deflate (&strm, Z_FINISH);
    if (rc != Z_OK && rc != Z_STREAM_END)
      goto err;
    
    while (strm.total_out != 0)
    {
        Fm_write (fm_wpos, zbuf, strm.total_out);
        fm_wpos += strm.total_out;
        
        strm.next_out = zbuf;
        strm.avail_out = sizeof(zbuf);
        strm.total_out = 0;

        (void) deflate (&strm, Z_FINISH);
    }
    
    /*压缩数据流长度*/
    hdr.length = (UINT16)(fm_wpos - fm_hdr) - sizeof(hdr);

    /*写历史数据头*/
    Fm_write (fm_hdr, &hdr, sizeof(hdr));


    /*更新FLASH写位置信息*/
    flash_winfo.addr += (UINT16)(sizeof(hdr) + hdr.length);
    flash_winfo.copy1 = flash_winfo.copy2 = flash_winfo.addr;

    Fm_write (FLASH_WINFO_START, &flash_winfo, sizeof(flash_winfo));

    /*更新日历史数据摘要信息*/
    today.end = flash_winfo.addr;
    Fm_write (today_fm, &today, sizeof(today));
}

/*****************************************************************************/
/* 将铁电中暂存的历史数据存入FLASH                                           */
/*****************************************************************************/
static void DelHistIn (UINT32 faddr)
{
    UINT16          i;
    DBDayIndexRec   tmp;

    /*ZYY 临时历史数据在Flash中的起始地址已经在Flash擦除边界,在历史记录索引里逐次搜索有效索引,
          若有效索引中保存的对应的Flash的地址在擦除边界,则将铁电中该索引清空*/
    for (i = 0; i < DAY_INDEX_MAX; i++)
    {
        UINT16  fm = DAY_INDEX_START + i*DAY_INDEX_SIZE;
        Fm_read (fm, &tmp, sizeof(tmp));

        if (tmp.magic != 'y')
          continue;

        if (tmp.end > tmp.flash)
        {
            if (faddr >= tmp.end || faddr + FLASH_ERASE_SIZE < tmp.flash)
              continue;
        }
        else
        {
            if (faddr >= tmp.end && faddr + FLASH_ERASE_SIZE < tmp.flash)
              continue;
        }

        /*删除该日历史数据<头>*/
        (void) memset (&tmp, 0, sizeof(tmp));
        
        Fm_write (fm, &tmp, sizeof(tmp));
    }
}

/*****************************************************************************/
/* 检测IO变化情况                                                            */
/*****************************************************************************/
static void Check_IO (UINT16 in, UINT16 out)
{
    if (in==IN_pre && out==OUT_pre)
    {
        IO_change = FALSE;
        return;
    }
    
    IO_change = TRUE;
    
    if (in != IN_pre)
    {
         // IN0
        if ((in&0x1) != (IN_pre&0x1))
        {
            if (IN_pre&0x1)
                IN_pre &= 0xfffe;
            else
                IN_pre |= 0x1; 
        }
        // IN1
        if ((in&0x2) != (IN_pre&0x2))
        {
            if (IN_pre&0x2)
                IN_pre &= 0xfffd;
            else
                IN_pre |= 0x2; 
        }
        // IN2
        if ((in&0x4) != (IN_pre&0x4))
        {
            if (IN_pre&0x4)
                IN_pre &= 0xfffb;
            else
                IN_pre |= 0x4; 
        }
        // CC2
        if ((in&0x8) != (IN_pre&0x8))
        {
            if (IN_pre&0x8)
                IN_pre &= 0xfff7;
            else
                IN_pre |= 0x8; 
        }
        // ACC_ON
        if ((in&0x10) != (IN_pre&0x10))
        {
            if (IN_pre&0x10)
                IN_pre &= 0xffEF;
            else
                IN_pre |= 0x10; 
        }
        // HVIL
        if ((in&0x20) != (IN_pre&0x20))
        {
            if (IN_pre&0x20)
                IN_pre &= 0xffDF;
            else
                IN_pre |= 0x20; 
        }
    }    
    
    if (out != OUT_pre)
    {
        // OUT0
        if ((out&0x1) != (OUT_pre&0x1))
        {
            OUT_chgnum[OUT0] = ~OUT_chgnum[OUT0];
            if (OUT_pre&0x1)
                OUT_pre &= 0xfffe;
            else
                OUT_pre |= 0x1; 
        }
        // OUT1
        if ((out&0x2) != (OUT_pre&0x2))
        {
            OUT_chgnum[OUT1] = ~OUT_chgnum[OUT1];
            if (OUT_pre&0x2)
                OUT_pre &= 0xfffd;
            else
                OUT_pre |= 0x2; 
        }
        // OUT2
        if ((out&0x4) != (OUT_pre&0x4))
        {
            OUT_chgnum[OUT2] = ~OUT_chgnum[OUT2];
            if (OUT_pre&0x4)
                OUT_pre &= 0xfffb;
            else
                OUT_pre |= 0x4; 
        }
        // OUT3
        if ((out&0x8) != (OUT_pre&0x8))
        {
            OUT_chgnum[OUT3] = ~OUT_chgnum[OUT3];
            if (OUT_pre&0x8)
                OUT_pre &= 0xfff7;
            else
                OUT_pre |= 0x8; 
        }
        // OUT4
        if ((out&0x10) != (OUT_pre&0x10))
        {
            OUT_chgnum[OUT4] = ~OUT_chgnum[OUT4];
            if (OUT_pre&0x10)
                OUT_pre &= 0xffef;
            else
                OUT_pre |= 0x10; 
        } 
        // OUT5
        if ((out&0x20) != (OUT_pre&0x20))
        {
            OUT_chgnum[OUT5] = ~OUT_chgnum[OUT5];
            if (OUT_pre&0x20)
                OUT_pre &= 0xffDf;
            else
                OUT_pre |= 0x20; 
        }   
    }
}


/*****************************************************************************/
/* 删除FLASH中位于该块的历史数据                                             */
/*****************************************************************************/
static void Dump (void)
{
    UINT32 cb;

    switch (fm_state)
    {
    default:
        fm_state = 0;
        return;

    case 0:/*空闲*/
        if (fm_rpos == fm_wpos)                         /*没有数据待转存*/
          return;

        if ((fm_faddr & (FLASH_ERASE_SIZE - 1)) != 0x0) /*不在FLASH擦除边界*/
        {
            fm_state = 2;                               /*可编程*/
            break;
        }

        if (!Flash_busy ())                             /*WIP == 0*/
        {
            /*ZYY 首先删除铁电中该历史记录索引*/
            DelHistIn (fm_faddr);                       /*删除FLASH中位于该块的历史数据*/

            Flash_erase (fm_faddr);                     /*下发擦除命令*/

            fm_state = 1;                               /*等待擦除完成*/
        }
        break;

    case 1:/*等待擦除完成*/
        if (!Flash_busy ())                             /*WIP == 0*/
        {
            fm_state = 2;                               /*等待擦除完成*/
        }
        break;

    case 2:/*可编程*/
        if (Flash_busy ())
          break;

        cb = FLASH_ERASE_SIZE - (fm_faddr & (FLASH_ERASE_SIZE - 1));
        if (cb > fm_wpos - fm_rpos)
          cb = fm_wpos - fm_rpos;

        if (cb == 0)
        {
            fm_state = 0;
            break;
        }

        if (cb > 256 - (fm_faddr & 255))                  /*PageWrite不能跨256字节边界*/
          cb = 256 - (fm_faddr & 255);

        if (cb > sizeof(zbuf))
          cb = sizeof(zbuf);

        Fm_read (fm_rpos, zbuf, (UINT16)cb);
        Flash_write (fm_faddr, zbuf, (UINT16)cb);

        fm_faddr += (UINT16)cb;
        fm_rpos += (UINT16)cb;

        if ((fm_faddr & (FLASH_ERASE_SIZE - 1)) == 0x0)   /*在FLASH擦除边界*/
          fm_state = 0;
        break;
    }
}

/*****************************************************************************/
/* 历史数据模块入口                                                          */
/*****************************************************************************/
void HST_main (void)
{
    time_t now = gSysSec;
    UINT16 IN_now = 0;
    UINT16 OUT_now = 0;

    IN_now = BMS_InSts ();
    OUT_now = BMS_OutSts ();
    Check_IO (IN_now, OUT_now);
    
    /*if (gSysSec % 60 == 0 && gSysSec != LastSec)            // 1分钟重新计算通信质量
    {  */
    /*启动后5秒钟内不存储历史数据*/
    if (gSysTicks < 500)/*ZYY gSysTicks每10ms增加1*/
      return;

    Dump ();

    if (now == lastHistTime && !IO_change)
        return; 

    if ((now % Sys_Para.hist_prd) == 0 || IO_change)
    {
        MkHst (now);

        /**/
        lastHistTime = now;
    }
}
