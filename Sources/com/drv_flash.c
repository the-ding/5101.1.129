#include "sys.h"
#include "drv_spi.h"
#include "drv_flash.h"

/*SPI收发函数*/
#define spi     SPI1_sndrcv   /* 当前硬件配置：SPI1 */


/*M25P128命令列表*/
#define WREN    0x06    /* 写使能       */
#define WRDI    0x04    /* 写禁止       */
#define RDID    0x9F    /* 读芯片ID     */
#define RDSR    0x05    /* 读状态寄存器 */
#define READ    0x03    /* 读           */
#define PP      0x02    /* 写           */
#define SE      0xD8    /* 擦除         */


/*****************************************************************************/
/* 禁止FLASH芯片片选有效                                                     */
/*****************************************************************************/
static void deselect (void)
{
    PORTA_PA2 = 1;
    PORTA_PA3 = 1;
}

/*****************************************************************************/
/* 根据地址使能FLASH芯片片选                                                 */
/*****************************************************************************/
static void select (UINT32 addr)
{
    if (addr < 16*1024*1024UL)
    {
        PORTA_PA2 = 0;
        PORTA_PA3 = 1;
    }
    else
    {
        PORTA_PA2 = 1;
        PORTA_PA3 = 0;
    }
}

/*****************************************************************************/
/* 读FLASH状态寄存器                                                         */
/*****************************************************************************/
static UINT8 rdsr (UINT32 addr)
{
    UINT8 sts;

    select (addr);
    (void) spi (RDSR);
    sts = spi (0xff);
    deselect ();

    return sts;
}

/*****************************************************************************/
/* 查询FLASH是否忙                                                           */
/*****************************************************************************/
BOOL Flash_busy (void)
{
    if (rdsr (0x0UL) & 0x3)
      return TRUE;
    
    if (rdsr (16*1024*1024UL) & 0x3)
      return TRUE;
    
    return FALSE;
}

/*****************************************************************************/
/* 从FLASH任意地址读取指定长度数据                                           */
/*****************************************************************************/
void Flash_read_in_sector (UINT32 addr, void *buf, UINT16 bytes)
{
    UINT16  i;
    
    if (!bytes)
      return;

    select (addr);
    (void) spi (READ);
    (void) spi ((UINT8)(addr >> 16));
    (void) spi ((UINT8)(addr >> 8));
    (void) spi ((UINT8)(addr));

    for (i = 0; i < bytes; i++)
      *((UINT8 *)buf + i) = spi (0xff);
    deselect ();
}

void Flash_read (UINT32 addr, void *buf, UINT16 bytes)
{
    if (bytes == 0 || addr >= FLASH_SIZE)
      return;

    if ((addr >> 24) != ((addr + bytes) >> 24))
    {
        UINT16  cb = (UINT16)(16*1024*1024UL - addr);
        
        Flash_read_in_sector (addr, buf, cb);
        
        bytes -= cb;
        addr += cb;
        buf = (char *)buf + cb;
        
        Flash_read_in_sector (addr, buf, bytes);
    }
    else
    {
        Flash_read_in_sector (addr, buf, bytes);
    }
}

/*****************************************************************************/
/* 往FLASH任意地址写指定长度数据 (*不检查是否已经擦除*)                      */
/*****************************************************************************/
void Flash_write (UINT32 addr, const void *buf, UINT16 bytes)
{
    UINT16  i;
    
    if (bytes == 0)
      return;

    /* 第一阶段：下发 “写使能” 命令 */
    select (addr);
    (void) spi (WREN);
    deselect ();


    /* 第二阶段：等待WEL状态位 */
    while (!(rdsr (addr) & 0x02))
      _FEED_COP();


    /* 第三阶段：写数据 */
    select (addr);
    (void) spi (PP);
    (void) spi ((UINT8)(addr >> 16));
    (void) spi ((UINT8)(addr >> 8));
    (void) spi ((UINT8)(addr));

    for (i = 0; i < bytes; i++)
      (void) spi (*((UINT8 *)buf + i));
    deselect ();

    ///* 第四阶段：下发 “写禁止” 命令 */
    //select (addr);
    //(void) spi (WRDI);
    //deselect ();


    /* 时间太长，不等待擦除完成 */

    /* 第五阶段：通过轮询WIP状态，等待写操作完成 */
    //while (rdsr (addr) & 0x01)
    //  _FEED_COP();
}

/*****************************************************************************/
/* 启动FLASH块擦除操作 (*注意：这里仅仅是启动了擦除，没有等待完成就返回了*)  */
/*****************************************************************************/
void Flash_erase (UINT32 addr)
{
    /* 第一阶段：下发 “写使能” 命令 */
    select (addr);
    (void) spi (WREN);
    deselect ();


    /* 第二阶段：等待WEL状态位 */
    while (!(rdsr (addr) & 0x02))
      _FEED_COP();


    /* 第三阶段：下发 “擦除” 命令 */
    select (addr);
    (void) spi (SE);
    (void) spi ((UINT8)(addr >> 16));
    (void) spi ((UINT8)(addr >> 8));
    (void) spi ((UINT8)(addr & 0xff));
    deselect ();

    /* 时间太长，不等待擦除完成 */
}

/*****************************************************************************/
/* FLASH模块初始化                                                           */
/*****************************************************************************/
void  Flash_Init (void)
{
    deselect ();
}
