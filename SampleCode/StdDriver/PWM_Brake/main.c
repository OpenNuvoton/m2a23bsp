/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use PWM brake function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BRAKE0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);


/**
 * @brief       PWM0 Brake0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWM0 Brake0 interrupt event
 */
void BRAKE0_IRQHandler(void)
{
    printf("\nFault brake!\n");
    printf("Press any key to unlock brake state. (PWM0 channel 0 output will toggle again)\n");
    getchar();

    /* Clear brake interrupt flag */
    PWM_ClearFaultBrakeIntFlag(PWM0, PWM_FB_EDGE);
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
                   CLK_AHBCLK_GPIODCKEN_Msk | CLK_AHBCLK_GPIOFCKEN_Msk;

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


    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL3_PWM0SEL_PCLK0, 0);

    /* Set multi-function pin for PWM */
    SET_PWM0_CH0_PB5();
    SET_PWM0_CH1_PB4();
    SET_PWM0_CH2_PB3();
    SET_PWM0_CH3_PB2();
    
    /* Set multi-function pin for PWM brake pin */
    SET_PWM0_BRAKE0_PB1();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
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
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with PWM0 channel 0~3.\n");
    printf("  I/O configuration:\n");
    printf("  PWM0 channel 0: 100 Hz, duty 30%%.\n");
    printf("  PWM0 channel 1: 100 Hz, duty 30%%.\n");
    printf("  PWM0 channel 2: 100 Hz, duty 30%%.\n");
    printf("  PWM0 channel 3: 100 Hz, duty 30%%.\n");
    printf("    waveform output pin: PWM0_CH0(PB.5), PWM0_CH1(PB.4), PWM0_CH2(PB.3), PWM0_CH3(PB.2)\n");

    printf("\nConnet PB.1 (PWM0 brake pin 0) to PD.3.\n");
    printf("It will generate brake interrupt and PWM0 channel 0 output stop toggling.\n");

    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);
    PD3 = 0;

    /* PWM0 frequency is 100Hz, duty 30%, */
    PWM_ConfigOutputChannel(PWM0, 0, 100, 30);

    /* Enable output of all PWM channels */
    PWM_EnableOutput(PWM0, 0xF);

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable brake and interrupt */
    PWM_EnableFaultBrake(PWM0, PWM_CH_0_MASK, 1, PWM_FB_EDGE_BKP0);
    PWM_EnableFaultBrakeInt(PWM0, 0);
    /* Enable brake noise filter : brake pin 0, filter count=7, filter clock=HCLK/128 */
    PWM_EnableBrakeNoiseFilter(PWM0, 0, 7, PWM_NF_CLK_DIV_128);
    /* Clear brake interrupt flag */
    PWM_ClearFaultBrakeIntFlag(PWM0, PWM_FB_EDGE);

    NVIC_EnableIRQ(BRAKE0_IRQn);

    /* Start */
    PWM_Start(PWM0, 1);

    printf("\nPress any key to generate a brake event\n");
    getchar();
    PD3 = 1;
		
    while(1) {}
}
