/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2013 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/*******************************************************************************
* File Name    : sd_dev_low.c
* $Rev: $
* $Date::                           $
* Device(s)    : RZ/A1H
* Tool-Chain   : DS-5 Ver 5.8
*              : ARM Complier
* OS           : 
* H/W Platform : RZ/A1H CPU Board
* Description  : RZ/A1H SD Driver Sample Program
* Operation    : 
* Limitations  : 
*******************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/

/* If the target has no SPI support then SDCard is not supported */
#if DEVICE_SDHI

#include <stdio.h>
#include <string.h>
#include "r_typedefs.h"
#include "iodefine.h"
#include "rza_io_regrw.h"
/*#include "devdrv_intc.h"*/
#include "sdif.h"
#include "sd_cfg.h"
/*#include "sd_dev_dmacdrv.h"*/
#include "us_ticker_api.h"
#include "cmsis_os2.h"
#include "mbed_assert.h"
#include "pinmap.h"


/******************************************************************************
Typedef definitions
******************************************************************************/


/******************************************************************************
Macro definitions
******************************************************************************/
//#define MTU_TIMER_CNT      32    /* P-phy = 32MHz         */
#define INT_LEVEL_SDHI     10    /* SDHI interrupt level  */
#define SDHI_PINS_COMMON    2
#define SDHI_PINS_SERIAL    3
#define SDHI_PINS_PARALLEL  6

#if defined(SDCFG_SDMODE_1BIT)
# define SDHI_PORT_MODE SD_PORT_SERIAL
#else
# define SDHI_PORT_MODE SD_PORT_PARALLEL
#endif

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/


/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/


/******************************************************************************
Private global variables and functions
******************************************************************************/
#if 0
static uint8_t g_sdhi_priority_backup;
#endif



static const PinName SDHIpin_Common[SDHI_COUNT][SDHI_PINS_COMMON] = { /* WP & CD */
#if defined(TARGET_VK_RZ_A1H)
   {P4_8, P4_9},
   {P3_8, P3_9}
#elif defined(TARGET_VK_RZ_A1LU)
   {P3_6, P3_7},
   {P7_1, P7_0}
#elif defined(TARGET_VK_RZ_A1R3)
   {P3_6, P3_7},
   {  NC, P7_0}
#else
 #error RZ_SDHI driver does not support this TARGET
#endif
};

static const PinName SDHIpin_serial[SDHI_COUNT][SDHI_PINS_SERIAL] = { /* CLK CMD D0 */
#if defined(TARGET_VK_RZ_A1H)
    {P4_11, P4_12, P4_13},
    {P3_11, P3_12, P3_13}
#elif (defined(TARGET_VK_RZ_A1LU) || defined(TARGET_VK_RZ_A1R3))
    {P3_3, P3_2, P3_4},
    {P7_4, P7_5, P7_3}
#else
#error RZ_SDHI driver does not support this TARGET
#endif
};

static const PinName SDHIpin_parallel[SDHI_COUNT][SDHI_PINS_PARALLEL] = { /* CLK CMD D0-D3 */
#if defined(TARGET_VK_RZ_A1H)
    {P4_10, P4_11, P4_12, P4_13, P4_14, P4_15},
    {P3_10, P3_11, P3_12, P3_13, P3_14, P3_15}
#elif (defined(TARGET_VK_RZ_A1LU) || defined(TARGET_VK_RZ_A1R3))
    {P3_3, P3_2, P3_4, P3_5, P3_0, P3_1},
    {P7_4, P7_5, P7_3, P7_2, P7_7, P7_6}
#else
#error RZ_SDHI driver does not support this TARGET
#endif
};


static const PinMap PinMap_SDHI_PIN[] = {
    /* pin | periph| func */
#if defined(TARGET_VK_RZ_A1H)
    {P4_8  , SDHI_0, 3}, /* SD_CD_0  */
    {P4_9  , SDHI_0, 3}, /* SD_WP_0  */
    {P4_10 , SDHI_0, 3}, /* SD_D1_0  */
    {P4_11 , SDHI_0, 3}, /* SD_D0_0  */
    {P4_12 , SDHI_0, 3}, /* SD_CLK_0 */
    {P4_13 , SDHI_0, 3}, /* SD_CMD_0 */
    {P4_14 , SDHI_0, 3}, /* SD_D3_0  */
    {P4_15 , SDHI_0, 3}, /* SD_D2_0  */
    /*----------------*/
    {P3_8  , SDHI_1, 7}, /* SD_CD_1  */
    {P3_9  , SDHI_1, 7}, /* SD_WP_1  */
    {P3_10 , SDHI_1, 7}, /* SD_D1_1  */
    {P3_11 , SDHI_1, 7}, /* SD_D0_1  */
    {P3_12 , SDHI_1, 7}, /* SD_CLK_1 */
    {P3_13 , SDHI_1, 7}, /* SD_CMD_1 */
    {P3_14 , SDHI_1, 7}, /* SD_D3_1  */
    {P3_15 , SDHI_1, 7}, /* SD_D2_1  */
#elif defined(TARGET_VK_RZ_A1LU)
    //{P3_7  , SDHI_0, 2}, /* SD_CD_0  can be used if SDRAM is not soldered */
    //{P3_6  , SDHI_0, 2}, /* SD_WP_0  can be used if SDRAM is not soldered */
    //{P3_5  , SDHI_0, 2}, /* SD_D1_0  can be used if SDRAM is not soldered */
    //{P3_4  , SDHI_0, 2}, /* SD_D0_0  can be used if SDRAM is not soldered */
    //{P3_3  , SDHI_0, 2}, /* SD_CLK_0 can be used if SDRAM is not soldered */
    //{P3_2  , SDHI_0, 2}, /* SD_CMD_0 can be used if SDRAM is not soldered */
    //{P3_1  , SDHI_0, 2}, /* SD_D3_0  can be used if SDRAM is not soldered */
    //{P3_0  , SDHI_0, 2}, /* SD_D2_0  can be used if SDRAM is not soldered */
    /*----------------*/
    {P7_0  , SDHI_1, 3}, /* SD_CD_1  */
    {P7_1  , SDHI_1, 3}, /* SD_WP_1  */
    {P7_2  , SDHI_1, 3}, /* SD_D1_1  */
    {P7_3  , SDHI_1, 3}, /* SD_D0_1  */
    {P7_4  , SDHI_1, 3}, /* SD_CLK_1 */
    {P7_5  , SDHI_1, 3}, /* SD_CMD_1 */
    {P7_6  , SDHI_1, 3}, /* SD_D3_1  */
    {P7_7  , SDHI_1, 3}, /* SD_D2_1  */
#elif defined(TARGET_VK_RZ_A1R3)
    //{P3_7  , SDHI_0, 2}, /* SD_CD_0  can be used if ETHERNET PHY is not soldered */
    //{P3_6  , SDHI_0, 2}, /* SD_WP_0  can be used if ETHERNET PHY is not soldered */
    //{P3_5  , SDHI_0, 2}, /* SD_D1_0  can be used if ETHERNET PHY is not soldered */
    //{P3_4  , SDHI_0, 2}, /* SD_D0_0  can be used if ETHERNET PHY is not soldered */
    //{P3_3  , SDHI_0, 2}, /* SD_CLK_0 can be used if ETHERNET PHY is not soldered */
    //{P3_2  , SDHI_0, 2}, /* SD_CMD_0 can be used if ETHERNET PHY is not soldered */
    //{P3_1  , SDHI_0, 2}, /* SD_D3_0  can be used if ETHERNET PHY is not soldered */
    //{P3_0  , SDHI_0, 2}, /* SD_D2_0  can be used if ETHERNET PHY is not soldered */
    /*----------------*/
    {P7_0  , SDHI_1, 3}, /* SD_CD_1  */
    {P7_2  , SDHI_1, 3}, /* SD_D1_1  */
    {P7_3  , SDHI_1, 3}, /* SD_D0_1  */
    {P7_4  , SDHI_1, 3}, /* SD_CLK_1 */
    {P7_5  , SDHI_1, 3}, /* SD_CMD_1 */
    {P7_6  , SDHI_1, 3}, /* SD_D3_1  */
    {P7_7  , SDHI_1, 3}, /* SD_D2_1  */	
#else
#error RZ_SDHI driver does not support this TARGET
#endif
    {NC    , NC    , 0}
};



static unsigned long _ulStart = 0;
static unsigned long _ulDelta = 0;
static const ticker_data_t *_ticker;

//static int sddev_init_0(void);
//static int sddev_init_1(void);
//static int sddev_set_port_0(int mode);
//static int sddev_set_port_1(int mode);

static int sddev_init_dma_0(unsigned long buff,unsigned long reg,long cnt,int dir);
static int sddev_init_dma_1(unsigned long buff,unsigned long reg,long cnt,int dir);

static int sddev_wait_dma_end_0(long cnt);
static int sddev_wait_dma_end_1(long cnt);

static int sddev_disable_dma_0(void);
static int sddev_disable_dma_1(void);

static void sddev_sd_int_handler_0(uint32_t int_sense);
static void sddev_sd_int_handler_1(uint32_t int_sense);
static void sddev_sdio_int_handler_0(uint32_t int_sense);
static void sddev_sdio_int_handler_1(uint32_t int_sense);
static void sddev_start_timer(int msec);
static void sddev_end_timer(void);
static int  sddev_check_timer(void);

/******************************************************************************
* Function Name: int sddev_cmd0_sdio_mount(int sd_port);
* Description  : Select to issue CMD0 before SDIO Mount
* Arguments    : none
* Return Value : SD_OK  : issue CMD0
*              : SD_ERR : not issue CMD0
******************************************************************************/
int sddev_cmd0_sdio_mount(int sd_port)
{
#ifdef SDCFG_IO
    return SD_ERR;
#else
    return SD_ERR;
#endif
}

/******************************************************************************
* Function Name: int sddev_cmd8_sdio_mount(int sd_port);
* Description  : Select to issue CMD8 before SDIO Mount
* Arguments    : none
* Return Value : SD_OK  : issue CMD8
*              : SD_ERR : not issue CMD8
******************************************************************************/
int sddev_cmd8_sdio_mount(int sd_port)
{
#ifdef SDCFG_IO
    return SD_OK;
#else
    return SD_ERR;
#endif
}



/******************************************************************************
* Function Name: int sddev_init(void);
* Description  : Initialize H/W to use SDHI
* Arguments    : none
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int sddev_init(int sd_port)
{
    if ( sd_port >= SDHI_COUNT )
        return SD_ERR;


    volatile uint8_t dummy_buf;

    CPG.STBCR12 = 0xF0u;        /* [1], [1], [1], [1], SDHI00, SDHI01, SDHI10, SDHI11           */
    dummy_buf   = CPG.STBCR12;  /* (Dummy read)                                                 */


    for( uint32_t no=0; no < SDHI_PINS_COMMON; no++ )
    {
        if ( pinmap_peripheral(SDHIpin_Common[sd_port][no], PinMap_SDHI_PIN ) != sd_port)
        {
            if(SDHIpin_Common[sd_port][no] != NC)
                return SD_ERR;
        }
        pinmap_pinout(SDHIpin_Common[sd_port][no], PinMap_SDHI_PIN);
    }

    sddev_set_port(sd_port, SDHI_PORT_MODE);

#ifdef    SDCFG_HWINT
    if ( sd_port == (uint32_t)SDHI_0 )
    {
        InterruptHandlerRegister(SDHI0_0_IRQn, sddev_sd_int_handler_0);
        GIC_SetPriority(SDHI0_0_IRQn, INT_LEVEL_SDHI);
        GIC_EnableIRQ(SDHI0_0_IRQn);

        InterruptHandlerRegister(SDHI0_3_IRQn, sddev_sd_int_handler_0);
        GIC_SetPriority(SDHI0_3_IRQn, INT_LEVEL_SDHI);
        GIC_EnableIRQ(SDHI0_3_IRQn);

        InterruptHandlerRegister(SDHI0_1_IRQn, sddev_sdio_int_handler_0);
        GIC_SetPriority(SDHI0_1_IRQn, INT_LEVEL_SDHI);
        GIC_EnableIRQ(SDHI0_1_IRQn);
    }
    else if ( sd_port == (uint32_t)SDHI_1 )
    {
        InterruptHandlerRegister(SDHI1_0_IRQn, sddev_sd_int_handler_1);
        GIC_SetPriority(SDHI1_0_IRQn, INT_LEVEL_SDHI);
        GIC_EnableIRQ(SDHI1_0_IRQn);

        InterruptHandlerRegister(SDHI1_3_IRQn, sddev_sd_int_handler_1);
        GIC_SetPriority(SDHI1_3_IRQn, INT_LEVEL_SDHI);
        GIC_EnableIRQ(SDHI1_3_IRQn);

        InterruptHandlerRegister(SDHI1_1_IRQn, sddev_sdio_int_handler_1);
        GIC_SetPriority(SDHI1_1_IRQn, INT_LEVEL_SDHI);
        GIC_EnableIRQ(SDHI1_1_IRQn);
    }
#endif

    /* ---- wait card detect ---- */
    osDelay(1000);    /* wait 1000ms */

    return SD_OK;
}


/******************************************************************************
* Function Name: int sddev_power_on(int sd_port);
* Description  : Power-on H/W to use SDHI
* Arguments    : none
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int sddev_power_on(int sd_port)
{
    /* ---Power On SD ---- */

    /* ---- Wait for  SD Wake up ---- */
    osDelay(100);            /* wait 100ms */

    return SD_OK;
}

/******************************************************************************
* Function Name: int sddev_power_off(int sd_port);
* Description  : Power-off H/W to use SDHI
* Arguments    : none
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int sddev_power_off(int sd_port)
{
    return SD_OK;
}

/******************************************************************************
* Function Name: int sddev_read_data(int sd_port, unsigned char *buff,unsigned long reg_addr,long num);
* Description  : read from SDHI buffer FIFO
* Arguments    : unsigned char *buff    : buffer addrees to store reading datas
*              : unsigned long reg_addr : SDIP FIFO address
*              : long num               : counts to read(unit:byte)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
#ifdef __CC_ARM
 #pragma push
 #pragma Ospace
#elif defined(__ICCARM__)
 #pragma optimize=size
#elif (defined( __GNUC__ ) && !defined( __CC_ARM ))
 #pragma GCC push_options
 #pragma GCC optimize ("Os")
#endif
int sddev_read_data(int sd_port, unsigned char *buff,unsigned long reg_addr,long num)
{
    long i;
    long cnt;
    unsigned long *reg;
    unsigned long *ptr_l;
    unsigned char *ptr_c;
    unsigned long tmp;

    reg = (unsigned long *)(reg_addr);

    cnt = (num / 4);
    if(((unsigned long)buff & 0x3) != 0)
    {
        ptr_c = (unsigned char *)buff;
        for(i = cnt; i > 0 ; i--)
        {
            tmp = *reg;
            *ptr_c++ = (unsigned char)(tmp);
            *ptr_c++ = (unsigned char)(tmp >> 8);
            *ptr_c++ = (unsigned char)(tmp >> 16);
            *ptr_c++ = (unsigned char)(tmp >> 24);
        }

        cnt = (num % 4);
        if( cnt != 0 )
        {
            tmp = *reg;
            for(i = cnt; i > 0 ; i--)
            {
                *ptr_c++ = (unsigned char)(tmp);
                tmp >>= 8;
            }
        }
    }
    else
    {
        ptr_l = (unsigned long *)buff;
        for(i = cnt; i > 0 ; i--)
        {
            *ptr_l++ = *reg;
        }

        cnt = (num % 4);
        if( cnt != 0 )
        {
            ptr_c = (unsigned char *)ptr_l;
            tmp = *reg;
            for(i = cnt; i > 0 ; i--)
            {
                *ptr_c++ = (unsigned char)(tmp);
                tmp >>= 8;
            }
        }
    }

    return SD_OK;
}
#ifdef __CC_ARM
 #pragma pop
#elif (defined( __GNUC__ ) && !defined( __CC_ARM ))
 #pragma GCC pop_options
#endif

/******************************************************************************
* Function Name: int sddev_write_data(int sd_port, unsigned char *buff,unsigned long reg_addr,long num);
* Description  : write to SDHI buffer FIFO
* Arguments    : unsigned char *buff    : buffer addrees to store writting datas
*              : unsigned long reg_addr : SDIP FIFO address
*              : long num               : counts to write(unit:byte)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int sddev_write_data(int sd_port, unsigned char *buff,unsigned long reg_addr,long num)
{
    long i;
    unsigned long *reg = (unsigned long *)(reg_addr);
    unsigned long *ptr = (unsigned long *)buff;
    unsigned long tmp;

    /* dont care non 4byte allignment data */
    num += 3;
    num /= 4;
    if(((unsigned long)buff & 0x3) != 0)
    {
        for(i = num; i > 0 ; i--)
        {
            tmp  = *buff++ ;
            tmp |= *buff++ << 8;
            tmp |= *buff++ << 16;
            tmp |= *buff++ << 24;
            *reg = tmp;
        }
    }
    else
    {
        for(i = num; i > 0 ; i--)
        {
            *reg = *ptr++;
        }
    }

    return SD_OK;
}

/******************************************************************************
* Function Name: unsigned int sddev_get_clockdiv(int sd_port, int clock);
* Description  : write to SDHI buffer FIFO
* Arguments    : int clock : request clock frequency
*              :   SD_CLK_50MHz
*              :   SD_CLK_25MHz
*              :   SD_CLK_20MHz
*              :   SD_CLK_10MHz
*              :   SD_CLK_5MHz
*              :   SD_CLK_1MHz
*              :   SD_CLK_400kHz
* Return Value : clock div value
*              :   SD_DIV_2 : 1/2   clock
*              :   SD_DIV_2 : 1/4   clock
*              :   SD_DIV_2 : 1/8   clock
*              :   SD_DIV_2 : 1/16  clock
*              :   SD_DIV_2 : 1/128 clock
*              :   SD_DIV_2 : 1/256 clock
******************************************************************************/
unsigned int sddev_get_clockdiv(int sd_port, int clock)
{
    unsigned int div;

    switch(clock)
    {
    case SD_CLK_50MHz:
        div = SD_DIV_2;        /* 64MHz/2 = 32MHz */
        break;
    case SD_CLK_25MHz:
        div = SD_DIV_4;        /* 64MHz/4 = 16MHz */
        break;
    case SD_CLK_20MHz:
        div = SD_DIV_4;        /* 64MHz/4 = 16MHz */
        break;
    case SD_CLK_10MHz:
        div = SD_DIV_8;        /* 64MHz/8 = 8MHz */
        break;
    case SD_CLK_5MHz:
        div = SD_DIV_16;       /* 64MHz/16 = 4MHz */
        break;
    case SD_CLK_1MHz:
        div = SD_DIV_128;      /* 64MHz/128 = 512kHz */
        break;
    case SD_CLK_400kHz:
        div = SD_DIV_256;      /* 64MHz/256 = 256kHz */
        break;
    default:
        div = SD_DIV_256;
        break;
    }

    return div;
}

/******************************************************************************
* Function Name: int sddev_set_port(int sd_port, int mode);
* Description  : setting ports to use MMCHI
* Arguments    : int mode : SD_PORT_PARALLEL : 4bit mode
*                         : SD_PORT_SERIAL   : 1bit mode
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int sddev_set_port(int sd_port, int mode)
{
    if ( sd_port >= SDHI_COUNT)
        return SD_ERR;

    if(mode == SD_PORT_SERIAL)
    {
        for( uint32_t no=0; no < SDHI_PINS_SERIAL; no++ )
        {
            if ( pinmap_peripheral(SDHIpin_serial[sd_port][no], PinMap_SDHI_PIN ) != sd_port)
            {
                return SD_ERR;
            }
            pinmap_pinout(SDHIpin_serial[sd_port][no], PinMap_SDHI_PIN);
        }
    }
    else if( mode == SD_PORT_PARALLEL )
    {
        for( uint32_t no=0; no < SDHI_PINS_PARALLEL; no++ )
        {
            if ( pinmap_peripheral(SDHIpin_parallel[sd_port][no], PinMap_SDHI_PIN ) != sd_port)
            {
                return SD_ERR;
            }
            pinmap_pinout(SDHIpin_parallel[sd_port][no], PinMap_SDHI_PIN);
        }
    }
    else
    {
        return SD_ERR;
    }

    return SD_OK;
}


/******************************************************************************
* Function Name: int sddev_int_wait(int sd_port, int time);
* Description  : Waitting for SDHI Interrupt
* Arguments    : int time : time out value to wait interrupt
* Return Value : get interrupt : SD_OK
*              : time out      : SD_ERR
******************************************************************************/
int sddev_int_wait(int sd_port, int time)
{
    sddev_start_timer(time);
    while( sddev_check_timer() == SD_OK )
    {
        /* interrupt generated? */
        if(sd_check_int(sd_port) == SD_OK)
        {
            sddev_end_timer();
            return SD_OK;
        }
    }

    sddev_end_timer();

    return SD_ERR;
}

/******************************************************************************
* Function Name: int sddev_init_dma(unsigned long buff,unsigned long reg,long cnt,int dir);
* Description  : Initialize DMAC to transfer data from SDHI FIFO
* Arguments    : unsigned long buff : buffer addrees to transfer datas
*              : unsigned long reg  : SDIP FIFO address
*              : long cnt           : counts to transfer(unit:byte)
*              : int dir            : direction to transfer
*              :                    :   0 : FIFO -> buffer
*              :                    :   1 : buffer -> FIFO
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int sddev_init_dma(int sd_port, unsigned long buff,unsigned long reg,long cnt,int dir)
{
    int ret;

    if( sd_port == 0 )
    {
        ret = sddev_init_dma_0(buff, reg, cnt, dir);
    }
    else if( sd_port == 1 )
    {
        ret = sddev_init_dma_1(buff, reg, cnt, dir);
    }

    return ret;
}

/******************************************************************************
* Function Name: static int sddev_init_dma_0(unsigned long buff,unsigned long reg,long cnt,int dir);
* Description  : Initialize DMAC to transfer data from SDHI FIFO
* Arguments    : unsigned long buff : buffer addrees to transfer datas
*              : unsigned long reg  : SDIP FIFO address
*              : long cnt           : counts to transfer(unit:byte)
*              : int dir            : direction to transfer
*              :                    :   0 : FIFO -> buffer
*              :                    :   1 : buffer -> FIFO
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
static int sddev_init_dma_0(unsigned long buff,unsigned long reg,long cnt,int dir)
{
#ifdef    SDCFG_TRNS_DMA
    dmac_transinfo_t    trans_info;
    uint32_t            request_factor;
    int32_t             ret;

    trans_info.count     = (uint32_t)cnt;
    #ifdef SDCFG_TRANS_DMA_64
    if( (cnt % 64) != 0 )
    {
        trans_info.src_size  = DMAC_TRANS_SIZE_32;
        trans_info.dst_size  = DMAC_TRANS_SIZE_32;
        if( reg & 0x0000003f )
        {
            trans_info.src_size  = DMAC_TRANS_SIZE_32;
            trans_info.dst_size  = DMAC_TRANS_SIZE_32;
        }
    }
    else
    {
        trans_info.src_size  = DMAC_TRANS_SIZE_512;
        trans_info.dst_size  = DMAC_TRANS_SIZE_512;
    }
    #else
    trans_info.src_size  = DMAC_TRANS_SIZE_32;
    trans_info.dst_size  = DMAC_TRANS_SIZE_32;
    #endif

    if( dir == 0 )
    {
        request_factor       = DMAC_REQ_SDHI_0_RX;
        trans_info.src_addr  = (uint32_t)reg;
        trans_info.dst_addr  = (uint32_t)buff;
        trans_info.saddr_dir = DMAC_TRANS_ADR_NO_INC;
        trans_info.daddr_dir = DMAC_TRANS_ADR_INC;
    }
    else if( dir == 1 )
    {
        request_factor       = DMAC_REQ_SDHI_0_TX;
        trans_info.src_addr  = (uint32_t)buff;
        trans_info.dst_addr  = (uint32_t)reg;
        trans_info.saddr_dir = DMAC_TRANS_ADR_INC;
        trans_info.daddr_dir = DMAC_TRANS_ADR_NO_INC;
    }

    sd_DMAC1_PeriReqInit(    (const dmac_transinfo_t *)&trans_info,
                            DMAC_MODE_REGISTER,
                            DMAC_SAMPLE_SINGLE,
                            request_factor,
                            0    );        /* Dont care DMAC_REQ_REQD is setting in usb0_host_DMAC1_PeriReqInit() */

    ret = sd_DMAC1_Open(DMAC_REQ_MODE_PERI);
    if( ret != 0 )
    {
        printf("DMAC1 Open error!!\n");
        return SD_ERR;
    }
#endif

    return SD_OK;
}

/******************************************************************************
* Function Name: static int sddev_init_dma_1(unsigned long buff,unsigned long reg,long cnt,int dir);
* Description  : Initialize DMAC to transfer data from SDHI FIFO
* Arguments    : unsigned long buff : buffer address to transfer datas
*              : unsigned long reg  : SDIP FIFO address
*              : long cnt           : counts to transfer(unit:byte)
*              : int dir            : direction to transfer
*              :                    :   0 : FIFO -> buffer
*              :                    :   1 : buffer -> FIFO
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
static int sddev_init_dma_1(unsigned long buff,unsigned long reg,long cnt,int dir)
{
#ifdef    SDCFG_TRNS_DMA
    dmac_transinfo_t    trans_info;
    uint32_t            request_factor;
    int32_t             ret;

    trans_info.count     = (uint32_t)cnt;
    #ifdef SDCFG_TRANS_DMA_64
    if( (cnt % 64) != 0 )
    {
        trans_info.src_size  = DMAC_TRANS_SIZE_32;
        trans_info.dst_size  = DMAC_TRANS_SIZE_32;
        if( reg & 0x0000003f )
        {
            trans_info.src_size  = DMAC_TRANS_SIZE_32;
            trans_info.dst_size  = DMAC_TRANS_SIZE_32;
        }
    }
    else
    {
        trans_info.src_size  = DMAC_TRANS_SIZE_512;
        trans_info.dst_size  = DMAC_TRANS_SIZE_512;
    }
    #else
    trans_info.src_size  = DMAC_TRANS_SIZE_32;
    trans_info.dst_size  = DMAC_TRANS_SIZE_32;
    #endif

    if( dir == 0 )
    {
        request_factor       = DMAC_REQ_SDHI_1_RX;
        trans_info.src_addr  = (uint32_t)reg;
        trans_info.dst_addr  = (uint32_t)buff;
        trans_info.saddr_dir = DMAC_TRANS_ADR_NO_INC;
        trans_info.daddr_dir = DMAC_TRANS_ADR_INC;
    }
    else if( dir == 1 )
    {
        request_factor       = DMAC_REQ_SDHI_1_TX;
        trans_info.src_addr  = (uint32_t)buff;
        trans_info.dst_addr  = (uint32_t)reg;
        trans_info.saddr_dir = DMAC_TRANS_ADR_INC;
        trans_info.daddr_dir = DMAC_TRANS_ADR_NO_INC;
    }

    sd_DMAC2_PeriReqInit(    (const dmac_transinfo_t *)&trans_info,
                            DMAC_MODE_REGISTER,
                            DMAC_SAMPLE_SINGLE,
                            request_factor,
                            0    );        /* Dont care DMAC_REQ_REQD is setting in usb0_host_DMAC1_PeriReqInit() */

    ret = sd_DMAC2_Open(DMAC_REQ_MODE_PERI);
    if( ret != 0 )
    {
        printf("DMAC1 Open error!!\n");
        return SD_ERR;
    }
#endif

    return SD_OK;
}

/******************************************************************************
* Function Name: int sddev_wait_dma_end(int sd_port, long cnt);
* Description  : Wait to complete DMAC transfer
* Arguments    : long cnt           : counts to transfer(unit:byte)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int sddev_wait_dma_end(int sd_port, long cnt)
{
    int ret;

    if( sd_port == 0 )
    {
        ret = sddev_wait_dma_end_0(cnt);
    }
    else if( sd_port == 1 )
    {
        ret = sddev_wait_dma_end_1(cnt);
    }

    return ret;
}

/******************************************************************************
* Function Name: static int sddev_wait_dma_end_0(long cnt);
* Description  : Wait to complete DMAC transfer
* Arguments    : long cnt           : counts to transfer(unit:byte)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
static int sddev_wait_dma_end_0(long cnt)
{
#ifdef    SDCFG_TRNS_DMA
    int loop;
    int time;

    time = (cnt / 512);
    time = ((time * 1000) / 1024);
    if(time < 1000)
    {
        time = 1000;
    }

    if(time > (0x0000ffff / MTU_TIMER_CNT))
    {
        /* @1000ms */
        loop = (time / 1000);
        if( (time % 1000) != 0 )
        {
            loop++;
        }
        time = 1000;
    }
    else
    {
        loop = 1;
    }

    do{
        sddev_start_timer(time);

        while(1)
        {
            /* get end flag? */
            if( sd_DMAC1_Get_Endflag() == 1 )
            {
                sddev_end_timer();
                return SD_OK;
            }
            /* detect timeout? */
            if(sddev_check_timer() == SD_ERR)
            {
                break;
            }
        }

        loop--;
        if( loop <= 0 )
        {
            break;
        }

    } while(1);

    sddev_end_timer();

    return SD_ERR;
#else
    return SD_OK;

#endif
}

/******************************************************************************
* Function Name: static int sddev_wait_dma_end_1(long cnt);
* Description  : Wait to complete DMAC transfer
* Arguments    : long cnt           : counts to transfer(unit:byte)
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
static int sddev_wait_dma_end_1(long cnt)
{
#ifdef    SDCFG_TRNS_DMA
    int loop;
    int time;

    time = (cnt / 512);
    time = ((time * 1000) / 1024);
    if(time < 1000)
    {
        time = 1000;
    }

    if(time > (0x0000ffff / MTU_TIMER_CNT))
    {
        /* @1000ms */
        loop = (time / 1000);
        if( (time % 1000) != 0 )
        {
            loop++;
        }
        time = 1000;
    }
    else
    {
        loop = 1;
    }

    do{
        sddev_start_timer(time);

        while(1)
        {
            /* get end flag? */
            if( sd_DMAC2_Get_Endflag() == 1 )
            {
                sddev_end_timer();
                return SD_OK;
            }
            /* detect timeout? */
            if(sddev_check_timer() == SD_ERR)
            {
                break;
            }
        }

        loop--;
        if( loop <= 0 )
        {
            break;
        }

    } while(1);

    sddev_end_timer();

    return SD_ERR;
#else
    return SD_OK;

#endif
}

/******************************************************************************
* Function Name: int sddev_disable_dma(int sd_port);
* Description  : Disable DMAC transfer
* Arguments    : none
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int sddev_disable_dma(int sd_port)
{
    int ret;

    if( sd_port == 0 )
    {
        ret = sddev_disable_dma_0();
    }
    else
    {
        ret = sddev_disable_dma_1();
    }
    return ret;
}

/******************************************************************************
* Function Name: static int sddev_disable_dma_0(void);
* Description  : Disable DMAC transfer
* Arguments    : none
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
static int sddev_disable_dma_0(void)
{
#ifdef    SDCFG_TRNS_DMA
    uint32_t    remain;

    sd_DMAC1_Close(&remain);
#endif
    return SD_OK;
}

/******************************************************************************
* Function Name: staticint sddev_disable_dma_1(void);
* Description  : Disable DMAC transfer
* Arguments    : none
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
static int sddev_disable_dma_1(void)
{
#ifdef    SDCFG_TRNS_DMA
    uint32_t    remain;

    sd_DMAC2_Close(&remain);
#endif
    return SD_OK;
}

/******************************************************************************
* Function Name: int sddev_loc_cpu(int sd_port);
* Description  : lock cpu to disable interrupt
* Arguments    : none
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int sddev_loc_cpu(int sd_port)
{
#if 0
    R_INTC_GetMaskLevel(&g_sdhi_priority_backup);
    R_INTC_SetMaskLevel(0);
    core_util_critical_section_enter();
#endif

    return SD_OK;
}

/******************************************************************************
* Function Name: int sddev_unl_cpu(int sd_port);
* Description  : unlock cpu to enable interrupt
* Arguments    : none
* Return Value : success : SD_OK
*              : fail    : SD_ERR
******************************************************************************/
int sddev_unl_cpu(int sd_port)
{
#if 0
    R_INTC_SetMaskLevel(g_sdhi_priority_backup);
    core_util_critical_section_exit();
#endif
    return SD_OK;
}

/******************************************************************************
* Function Name: int sddev_finalize(int sd_port);
* Description  : finalize SDHI
* Arguments    : none
* Return Value : none
******************************************************************************/
int sddev_finalize(int sd_port)
{
    return SD_OK;
}

/******************************************************************************
* Function Name: static void sddev_sd_int_handler_0(uint32_t int_sense);
* Description  : Setting Interrupt function for SDHI(INTC_ID_SDHI0_0,INTC_ID_SDHI0_3)
* Arguments    : Interrupt mode
* Return Value : none
******************************************************************************/
static void sddev_sd_int_handler_0(uint32_t int_sense)
{
    sd_int_handler(0);
}

/******************************************************************************
* Function Name: static void sddev_sd_int_handler_1(uint32_t int_sense);
* Description  : Setting Interrupt function for SDHI(INTC_ID_SDHI0_0,INTC_ID_SDHI0_3)
* Arguments    : Interrupt mode
* Return Value : none
******************************************************************************/
static void sddev_sd_int_handler_1(uint32_t int_sense)
{
    sd_int_handler(1);
}

/******************************************************************************
* Function Name: static void sddev_sdio_int_handler_0(uint32_t int_sense);
* Description  : Setting Interrupt function for SDHI(INTC_ID_SDHI0_1)
* Arguments    : Interrupt mode
* Return Value : none
******************************************************************************/
static void sddev_sdio_int_handler_0(uint32_t int_sense)
{
    sdio_int_handler(0);
}

/******************************************************************************
* Function Name: static void sddev_sdio_int_handler_1(uint32_t int_sense);
* Description  : Setting Interrupt function for SDHI(INTC_ID_SDHI1_1)
* Arguments    : Interrupt mode
* Return Value : none
******************************************************************************/
static void sddev_sdio_int_handler_1(uint32_t int_sense)
{
    sdio_int_handler(1);
}

/******************************************************************************
* Function Name: static void sddev_start_timer(int msec);
* Description  : start timer
* Arguments    : 
* Return Value : none
******************************************************************************/
static void sddev_start_timer(int msec)
{
    _ticker = get_us_ticker_data();
    _ulStart = ticker_read(_ticker);
    _ulDelta = msec*1000ul;
}

/******************************************************************************
* Function Name: static void sddev_end_timer(void);
* Description  : end timer
* Arguments    : 
* Return Value : none
******************************************************************************/
static void sddev_end_timer(void)
{
    _ulStart = 0ul;
    _ulDelta = 0ul;
}

/******************************************************************************
* Function Name: static int sddev_check_timer(void);
* Description  : check
* Arguments    : 
* Return Value : t
******************************************************************************/
static int sddev_check_timer(void)
{
    if ( _ulStart && _ulDelta )
    {
        return ((ticker_read(_ticker)-_ulStart) < _ulDelta) ? SD_OK : SD_ERR;
    }
    else
    {
        return SD_ERR;
    }
}

#endif /* DEVICE_SDHI */

/* End of File */
