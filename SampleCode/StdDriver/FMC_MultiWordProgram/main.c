/****************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This sample run on SRAM to show FMC multi word program function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


static uint32_t  g_auPageBuff[FMC_FLASH_PAGE_SIZE / 4];


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set core clock to 72MHz */
    //CLK_SetCoreClock(72000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPIOACKEN_Msk | CLK_AHBCLK_GPIOBCKEN_Msk | CLK_AHBCLK_GPIOCCKEN_Msk |
                   CLK_AHBCLK_GPIODCKEN_Msk | CLK_AHBCLK_GPIOFCKEN_Msk ;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int32_t main(void)
{
    uint32_t  i, u32Addr, u32Maddr, done = 0; /* temporary variables */

    SYS_UnlockReg();                    /* Unlock protected registers */

    SYS_Init();                         /* Init System, IP clock and multi-function I/O */

    UART0_Init();                       /* Initialize UART0 */

    printf("\n\n");
    printf("+-------------------------------------+\n");
    printf("|       Multi-word Program Sample     |\n");
    printf("+-------------------------------------+\n");

    FMC_Open();                        /* Enable FMC ISP function */

    FMC_ENABLE_AP_UPDATE();            /* Enable APROM erase/program */

    for(u32Addr = 0x20000; u32Addr < 0x22000; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("Multiword program APROM page 0x%x =>\n", u32Addr);

        if(FMC_Erase(u32Addr) < 0)
        {
            printf("    Erase failed!!\n");
            goto err_out;
        }

        printf("    Program...\n");

        for(u32Maddr = u32Addr; u32Maddr < u32Addr + FMC_FLASH_PAGE_SIZE; u32Maddr += FMC_MULTI_WORD_PROG_LEN)
        {
            /* Prepare test pattern */
            for(i = 0; i < FMC_MULTI_WORD_PROG_LEN; i += 4)
                g_auPageBuff[i / 4] = u32Maddr + i;

            i = (uint32_t)FMC_WriteMultiple(u32Maddr, g_auPageBuff, FMC_MULTI_WORD_PROG_LEN);
            if(i <= 0)
            {
                printf("FMC_WriteMultiple failed: %d\n", i);
                goto err_out;
            }
            printf("programmed length = %d\n", i);
        }
        printf("    [OK]\n");

        printf("    Verify...");

        for(i = 0; i < FMC_FLASH_PAGE_SIZE; i += 4)
            g_auPageBuff[i / 4] = u32Addr + i;

        for(i = 0; i < FMC_FLASH_PAGE_SIZE; i += 4)
        {
            if(FMC_Read(u32Addr + i) != g_auPageBuff[i / 4])
            {
                printf("\n[FAILED] Data mismatch at address 0x%x, expect: 0x%x, read: 0x%x!\n", u32Addr + i, g_auPageBuff[i / 4], FMC_Read(u32Addr + i));
                goto err_out;
            }
            if(g_FMC_i32ErrCode != 0)
            {
                printf("FMC_Read address 0x%x failed!\n", u32Addr + i);
                goto err_out;
            }
        }
        printf("[OK]\n");
    }

    done = 1;

err_out:

    if(done)
    {
        printf("\n\nMulti-word program demo done.\n");
    }
    else
    {
        printf("\n\nERROR!\n");
    }
    while(1);

}
