#include "sys.h"
#include "drv_spi.h"

/*****************************************************************************/
/* SPI控制器初始化                                                           */
/*****************************************************************************/
void SPI_Init (void)
{
    /* SPI2 */
    MODRR_MODRR6 = 0;

    DDRP_DDRP6 = 1;     //SS
    PERP_PERP6 = 1;
    PPSP_PPSP6 = 0;     //配置为上拉
    PTP_PTP6 = 1;
    PTP_PTP3 = 1;

    SPI2CR1 = 0x54;
    SPI2CR2 = 0X00;     //
    SPI2BR  = 0X5;      //500k


    /* SPI1 */
    MODRR_MODRR5 = 1;   //ph  spi1

    DDRA_DDRA2 = 1;     //CS1
    DDRA_DDRA3 = 1;     //CS2

    PORTA_PA2 = 1;
    PORTA_PA3 = 1;

    SPI1CR1 = 0x50;
    SPI1CR2 = 0x00;     //
    SPI1BR  = 0x3;      //波特率     6M
}

/*****************************************************************************/
/* SPI2启动一次传输                                                          */
/*****************************************************************************/
UINT8 SPI2_sndrcv (UINT8 data)
{
    UINT8 temp;

    while (!SPI2SR_SPTEF) {}
    SPI2DR = data;
    while (!SPI2SR_SPIF) {}
    temp = SPI2DR;
    return temp;
}

/*****************************************************************************/
/* SPI1启动一次传输                                                          */
/*****************************************************************************/
UINT8 SPI1_sndrcv (UINT8 datax)
{
    UINT8 temp;

    while (!SPI1SR_SPTEF) {}
    SPI1DR = datax;
    while (!SPI1SR_SPIF) {}
    temp = SPI1DR;
    return temp;
}
