/****************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Implement a code and execute in SRAM to program embedded Flash.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

extern int32_t FlashAccess_OnSRAM(void);

#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)

#define TOTAL_VECTORS   (58)                                /* Total vector numbers */
__ALIGNED(512) uint32_t g_au32Vector[TOTAL_VECTORS] = {0};  /* Vector space in SRAM */
volatile uint32_t g_u32Ticks = 0;
void SysTick_Handler()
{
    g_u32Ticks++;

    if((g_u32Ticks % 1000) == 0)
    {
        printf("Time elapse: %d\n", g_u32Ticks / 1000);
    }

}

#endif

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

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

int32_t main(void)
{
#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)

    int32_t i;
    uint32_t *au32Vectors = (uint32_t *)0x0;

#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      FMC Write/Read code execute in SRAM Sample Code      |\n");
    printf("+-----------------------------------------------------------+\n");

#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)

    /* Init Vector Table to SRAM */
    for(i = 0; i < TOTAL_VECTORS; i++)
    {
        g_au32Vector[i] = au32Vectors[i];
    }
    SCB->VTOR = (uint32_t)&g_au32Vector[0];

    /* SysTick used for test interrupts in SRAM */
    SysTick_Config(SystemCoreClock / 1000);

#endif
    /*
       This sample code demonstrates how to make a sub-routine code executed in SRAM.
    */

    printf("Will branch to address: 0x%x\n", (uint32_t)FlashAccess_OnSRAM);

    if(FlashAccess_OnSRAM())
    {
        printf("Flash access return error.\n");
    }
    else
    {
        printf("Flash access return ok.\n");
    }

    printf("\nFMC Sample Code Completed.\n");

    while(1);
}
