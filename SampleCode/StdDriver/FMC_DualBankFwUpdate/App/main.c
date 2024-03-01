/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Bank Remap sample code(Bank0 App).
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"
#include "xmodem.h"



/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t  s_u32DbLength;             /* dual bank program remaining length       */
static volatile uint32_t  s_u32DbAddr;               /* dual bank program current flash address  */
static volatile uint32_t  s_u32TickCnt;              /* timer ticks - 100 ticks per second       */

/*---------------------------------------------------------------------------------------------------------*/
/* Global Functions Declaration                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART_Init(UART_T* uart, uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32BaudRate);
void SysTick_Handler(void);
void EnableSysTick(int i8TicksPerSecond);
void SYS_Init(void);
uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len);

/*---------------------------------------------------------------------------------------------------------*/
/* Global Functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len)
{
    uint32_t  u32Idx, u32Data = 0UL;

    /* WDTAT_RVS, CHECKSUM_RVS, CHECKSUM_COM */
    for(u32Idx = 0; u32Idx < u32Len; u32Idx += 4)
    {
        u32Data += *(uint32_t *)(u32Start + u32Idx);
    }
    u32Data = 0xFFFFFFFF - u32Data + 1UL;

    return u32Data;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupt Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    /* Increase timer tick */
    s_u32TickCnt++;

    /* Calculate CRC32 value, just to consume CPU time  */
    FuncCrc32(0x10000, 0x100);
}

void EnableSysTick(int i8TicksPerSecond)
{
    s_u32TickCnt = 0;

    if(SysTick_Config(SystemCoreClock / (uint32_t)i8TicksPerSecond))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
    }
}


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


int main()
{
    uint32_t u32ch;
    int32_t  i32Err;
    /* CPU executing in which Bank */
    uint32_t  u32ExecBank = 0;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);

#ifdef NewApp
        printf("\n BANK%d APP processing (New Firmware!!!)\n", u32ExecBank);
#else
        printf("\n BANK%d APP processing \n", u32ExecBank);
#endif

        printf("\n Download new FW?[y/n]\n");
        u32ch = (uint32_t)getchar();
        if(u32ch == 'y')
        {
            printf("\n Bank%d processing, download data to Bank%d.\n", u32ExecBank, u32ExecBank ^ 1);

            /* Dual bank background program address */
            s_u32DbAddr   = FMC_BANK_SIZE * (u32ExecBank ^ 1) + APP_BASE;
            /* Dual bank background length */
            s_u32DbLength = APP_SIZE;

            EnableSysTick(1000);

            i32Err = Xmodem(s_u32DbAddr);
            if(i32Err < 0)
            {
                printf("Xmodem transfer fail!\n");
                while(1);
            }
            else
            {
                printf("Xomdem transfer done!\n");
                printf("Total trnasfer size is %d\n", i32Err);
            }

            printf("\n Firmware download completed!!\n");
        }
        else
        {
            printf("\n Reset from BANK%d Loader \n", u32ExecBank);
            /* Remap to Loader */
            FMC_SetVectorPageAddr(0x0);
            SYS_ResetCPU();
        }

    }
    while(1);

}
