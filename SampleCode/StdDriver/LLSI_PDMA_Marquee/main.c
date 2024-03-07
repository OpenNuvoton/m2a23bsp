/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is a LLSI demo for marquee display in PDMA mode.
 *           It needs to be used with WS2812 LED strip.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT 5

volatile uint32_t g_au32RED_Marquee0[TEST_COUNT] = {0x000000FF, 0x00000000, 0x00000000, 0x00000000, 0x00000000};
volatile uint32_t g_au32RED_Marquee1[TEST_COUNT] = {0xFF000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};
volatile uint32_t g_au32RED_Marquee2[TEST_COUNT] = {0x00000000, 0x00FF0000, 0x00000000, 0x00000000, 0x00000000};
volatile uint32_t g_au32RED_Marquee3[TEST_COUNT] = {0x00000000, 0x00000000, 0x0000FF00, 0x00000000, 0x00000000};
volatile uint32_t g_au32RED_Marquee4[TEST_COUNT] = {0x00000000, 0x00000000, 0x00000000, 0x000000FF, 0x00000000};
volatile uint32_t g_au32RED_Marquee5[TEST_COUNT] = {0x00000000, 0x00000000, 0x00000000, 0xFF000000, 0x00000000};
volatile uint32_t g_au32RED_Marquee6[TEST_COUNT] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};
volatile uint32_t g_u32PatternToggle = 0;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPIOACKEN_Msk | CLK_AHBCLK_GPIOBCKEN_Msk | CLK_AHBCLK_GPIOCCKEN_Msk |
                   CLK_AHBCLK_GPIODCKEN_Msk | CLK_AHBCLK_GPIOFCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable LLSI0 module clock */
    CLK_EnableModuleClock(LLSI0_MODULE);

    /* Enable PDMA peripheral clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PB multi-function pin for LLSI0 */
    SET_LLSI0_OUT_PB15();;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void LLSI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LLSI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as PDMA mode, GRB output format, 6 pixels in a frame and idle output low */
    /* Set clock divider. LLSI clock rate = 72MHz */
    /* Set data transfer period. T_Period = 1250ns */
    /* Set duty period. T_T0H = 400ns; T_T1H = 850ns */
    /* Set reset command period. T_ResetPeriod = 50000ns */
    LLSI_Open(LLSI0, LLSI_MODE_PDMA, LLSI_FORMAT_GRB, 72000000, 1250, 400, 850, 50000, 6, LLSI_IDLE_LOW);

    /* Enable reset command function */
    LLSI_ENABLE_RESET_COMMAND(LLSI0);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|      LLSI Marquee Sample Code (PDMA Mode)      |\n");
    printf("+------------------------------------------------+\n");
    printf("The first to sixth LEDs will flash red in sequence.\n\n");

    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    /* Open Channel 0 */
    PDMA_Open(PDMA0, 1 << 0);
    /* Transfer count is TEST_COUNT, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_32, TEST_COUNT);
    /* Transfer type is single transfer */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, 0);
    /* Set source address is g_au32RED_Marquee0, destination address is &LLSI0->DATA */
    PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee0, PDMA_SAR_INC, (uint32_t)&LLSI0->DATA, PDMA_DAR_FIX);
    /* Request source is LLSI0 */
    PDMA_SetTransferMode(PDMA0, 0, PDMA_LLSI0, FALSE, 0);

    /* Init LLSI */
    LLSI_Init();

    while(g_u32PatternToggle < 7)
    {
        CLK_SysTickDelay(100000);

        g_u32PatternToggle++;

        /* Reset PDMA module */
        SYS_ResetModule(PDMA_RST);

        /* Open Channel 0 */
        PDMA_Open(PDMA0, 1 << 0);
        /* Transfer count is TEST_COUNT, transfer width is 32 bits(one word) */
        PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_32, TEST_COUNT);
        /* Transfer type is single transfer */
        PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, 0);

        if(g_u32PatternToggle == 1)
        {
            /* Set source address is g_au32RED_Marquee1, destination address is &LLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee1, PDMA_SAR_INC, (uint32_t)&LLSI0->DATA, PDMA_DAR_FIX);
        }
        else if(g_u32PatternToggle == 2)
        {
            /* Set source address is g_au32RED_Marquee2, destination address is &LLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee2, PDMA_SAR_INC, (uint32_t)&LLSI0->DATA, PDMA_DAR_FIX);
        }
        else if(g_u32PatternToggle == 3)
        {
            /* Set source address is g_au32RED_Marquee3, destination address is &LLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee3, PDMA_SAR_INC, (uint32_t)&LLSI0->DATA, PDMA_DAR_FIX);
        }
        else if(g_u32PatternToggle == 4)
        {
            /* Set source address is g_au32RED_Marquee4, destination address is &LLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee4, PDMA_SAR_INC, (uint32_t)&LLSI0->DATA, PDMA_DAR_FIX);
        }
        else if(g_u32PatternToggle == 5)
        {
            /* Set source address is g_au32RED_Marquee5, destination address is &LLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee5, PDMA_SAR_INC, (uint32_t)&LLSI0->DATA, PDMA_DAR_FIX);
        }
        else if(g_u32PatternToggle == 6)
        {
            /* Set source address is g_au32RED_Marquee6, destination address is &LLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee6, PDMA_SAR_INC, (uint32_t)&LLSI0->DATA, PDMA_DAR_FIX);
        }

        /* Request source is LLSI0 */
        PDMA_SetTransferMode(PDMA0, 0, PDMA_LLSI0, FALSE, 0);
    }

    /* Close LLSI0 */
    LLSI_Close(LLSI0);

    printf("Exit LLSI sample code.\n");

    while(1);
}
