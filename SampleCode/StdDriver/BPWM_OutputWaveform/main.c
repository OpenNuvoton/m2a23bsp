/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use BPWM counter output waveform.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

void SYS_Init(void);
void UART0_Init(void);

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

    /* Enable BPWM module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /* Select BPWM module clock source */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL3_BPWM0SEL_PCLK0, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pin for BPWM */
    SET_BPWM0_CH0_PA0();
    SET_BPWM0_CH1_PA1();
    SET_BPWM0_CH2_PA2();
    SET_BPWM0_CH3_PA3();
    SET_BPWM0_CH4_PA4();
    SET_BPWM0_CH5_PA5();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    uint8_t u8Option;
 
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with BPWM0 channel 0~5.\n");
    printf("  Choose target generator frequency: 1. 60000Hz 2. 180000Hz\n");
    u8Option = getchar();

    printf("  I/O configuration:\n");
    if (u8Option == '1')
    {
        printf("  BPWM0 channel 0: 180000 Hz, duty 90%%.\n");
        printf("  BPWM0 channel 1: 180000 Hz, duty 75%%.\n");
        printf("  BPWM0 channel 2: 180000 Hz, duty 50%%.\n");
        printf("  BPWM0 channel 3: 180000 Hz, duty 30%%.\n");
        printf("  BPWM0 channel 4: 180000 Hz, duty 25%%.\n");
        printf("  BPWM0 channel 5: 180000 Hz, duty 10%%.\n");

        /* BPWM0 and BPWM1 channel 0~5 frequency and duty configuration are as follows */
        /* Because of BPWM0 channel 0~5 share one period, so the period value of all channels need set the same. */
        BPWM_ConfigOutputChannel(BPWM0, 0, 180000, 90);
        BPWM_ConfigOutputChannel(BPWM0, 1, 180000, 75);
        BPWM_ConfigOutputChannel(BPWM0, 2, 180000, 50);
        BPWM_ConfigOutputChannel(BPWM0, 3, 180000, 30);
        BPWM_ConfigOutputChannel(BPWM0, 4, 180000, 25);
        BPWM_ConfigOutputChannel(BPWM0, 5, 180000, 10);
    }
    else if (u8Option == '2')
    {
        printf("  BPWM0 channel 0: 60000 Hz, duty 90%%.\n");
        printf("  BPWM0 channel 1: 60000 Hz, duty 75%%.\n");
        printf("  BPWM0 channel 2: 60000 Hz, duty 50%%.\n");
        printf("  BPWM0 channel 3: 60000 Hz, duty 30%%.\n");
        printf("  BPWM0 channel 4: 60000 Hz, duty 25%%.\n");
        printf("  BPWM0 channel 5: 60000 Hz, duty 10%%.\n");

        /* BPWM0 and BPWM1 channel 0~5 frequency and duty configuration are as follows */
        /* Because of BPWM0 channel 0~5 share one period, so the period value of all channels need set the same. */
        BPWM_ConfigOutputChannel(BPWM0, 0, 60000, 90);
        BPWM_ConfigOutputChannel(BPWM0, 1, 60000, 75);
        BPWM_ConfigOutputChannel(BPWM0, 2, 60000, 50);
        BPWM_ConfigOutputChannel(BPWM0, 3, 60000, 30);
        BPWM_ConfigOutputChannel(BPWM0, 4, 60000, 25);
        BPWM_ConfigOutputChannel(BPWM0, 5, 60000, 10);
    }
    else
    {
        printf("  Invalid selection!\n");
        while(1);
    }
    printf("  waveform output pin: BPWM0_CH0(PA.0), BPWM0_CH1(PA.1), BPWM0_CH2(PA.2), BPWM0_CH3(PA.3), BPWM0_CH4(PA.4), BPWM0_CH5(PA.5)\n");

    printf("Press any key to stop.\n");

    /* Enable output of BPWM0 and BPWM1 channel 0~5 */
    BPWM_EnableOutput(BPWM0, 0x3F);

    /* Start BPWM0 counter */
    BPWM_Start(BPWM0, 0x3F);

    /* Wait for user press any key to stop */
    getchar();

    /* Stop BPWM0 counter */
    BPWM_ForceStop(BPWM0, 0x3F);

    printf("Done.");
    while(1);

}


/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
