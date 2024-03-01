/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This sample shows how to make an application running on APROM
 *           but with a sub-routine on SPROM, which can be secured.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void sprom_routine(void);


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

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t  ret;
    char     chr;                      /* user input character                            */

    SYS_UnlockReg();                   /* Unlock protected registers                      */

    SYS_Init();                        /* Init System, peripheral clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0                                */

    FMC_Open();                        /* Enable FMC ISP function                         */

    while (1)
    {
        printf("\n\n\n");
        printf("+--------------------------------------+\n");
        printf("| FMC SPROM Sample Demo                |\n");
        printf("+--------------------------------------+\n");
        printf("| [1] Lock SPROM                       |\n");
        printf("| [2] Erase SPROM                      |\n");
        printf("| [3] Branch to SPROM                  |\n");
        printf("+--------------------------------------+\n");

        if (FMC->ISPSTS & FMC_ISPSTS_SCODE_Msk)
            printf("ISPSTS=0x%x, SPROM is secured.\n", FMC->ISPSTS);
        else
            printf("ISPSTS=0x%x, SPROM is not secured.\n", FMC->ISPSTS);

        chr = getchar();

        switch (chr)
        {
        case '1':
            printf("Once SPROM is locked, it will become unreadable and can only "
                   "be unlocked by erase SPROM page.\n"
                   "Are you sure to lock SPROM? (y/n)");
            chr = getchar();
            if ((chr == 'y') || (chr == 'Y'))
            {
                FMC_ENABLE_SP_UPDATE();      /* enable SPROM update                   */

                /*
                 * Program a non-0xFF to the last byte of SPROM can make SPROM
                 * enter secure mode. Note that SPROM secure does not become
                 * effective until next chip boot. Because FMC check SPROM secure
                 * byte only when chip booting.
                 */
                if (FMC_Write(FMC_SPROM_END-4, 0x33333333) != 0)
                {
                    printf("FMC_Write address FMC_SPROM_END-4 failed!\n");
                    while (1);
                }

                /*
                 * Issued a chip reset to make SPROM secure mode take effects.
                 */
                SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
            }
            break;

        case '2':
            printf("Please note that this sample have a sub-routine running on SPROM.\n");
            printf("If SPROM was erased, branch to SPROM will cause a program fault.\n");
            printf("Are you sure to erase SPROM? (y/n)");
            chr = getchar();
            if ((chr == 'y') || (chr == 'Y'))
            {
                FMC_ENABLE_SP_UPDATE();      /* enable SPROM update                   */
                ret = FMC_Erase_SPROM();     /* erase SPROM                           */
                FMC_DISABLE_SP_UPDATE();     /* disable SPROM update                  */
                if (ret != 0)
                {
                    printf("FMC_Erase_SPROM failed!\n");
                    while (1);
                }
                printf("\n\nSPROM is erased.\n");
            }
            break;

        case '3':
            sprom_routine();
            break;
        }
    }

}
