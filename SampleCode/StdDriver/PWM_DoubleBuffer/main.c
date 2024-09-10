/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Change duty cycle and period of output waveform by PWM Double Buffer function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PWM0P0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);


/**
 * @brief       PWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWM0 interrupt event
 */
void PWM0P0_IRQHandler(void)
{
    static int32_t i32Toggle = 0;

    /* Update PWM0 channel 0 period and duty */
    if(i32Toggle == 0)
    {
        PWM_SET_CNR(PWM0, 0, 99);
        PWM_SET_CMR(PWM0, 0, 40);
    }
    else
    {
        PWM_SET_CNR(PWM0, 0, 399);
        PWM_SET_CMR(PWM0, 0, 200);
    }

    i32Toggle ^= 1;

    /* Clear channel 0 period interrupt flag */
    PWM_ClearPeriodIntFlag(PWM0, 0);
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
    printf("  This sample code will use PWM0 channel 0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0 channel 0(PB.5)\n");
    printf("\nUse double buffer feature.\n");

    /*
        PWM0 channel 0 waveform of this sample shown below(up counter type):

        |<-        CNR + 1  clk     ->|  CNR + 1 = 399 + 1 CLKs
        |<-  CMR clk ->|                 CMR = 200 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                      |<-CMR->|           CMR = 40 CLKs

         ______________                _______          ____
        |      200     |_____200______|   40  |____60__|     PWM waveform

    */

    /* PWM0 channel 0 frequency is 160000Hz, duty 50%, */
    PWM_ConfigOutputChannel(PWM0, 0, 160000, 50);

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    /* Enable PWM0 channel 0 period interrupt, use channel 0 to measure time. */
    PWM_EnablePeriodInt(PWM0, 0, 0);
    NVIC_EnableIRQ(PWM0P0_IRQn);

    /* Start */
    PWM_Start(PWM0, PWM_CH_0_MASK);

    while(1) {}
}
