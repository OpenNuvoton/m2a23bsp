/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of GPIO EINT trigger PWM function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



void EINT1_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);



/**
 * @brief       External INT1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT1 default IRQ, declared in startup_m2a23.s.
 */
void EINT1_IRQHandler(void)
{
    /* To check if PA.7 external interrupt occurred */
    if(GPIO->INT_EDSTS & GPIO_INT_EDSTS_EDIF1_Msk)
    {
        GPIO->INT_EDSTS = GPIO_INT_EDSTS_EDIF1_Msk;
        printf("PA.7 EINT1 occurred.\n");
    }
}

void PWM0P1_IRQHandler(void)
{
    /* Check PWM0 channel 2 compare up count interrupt flag */
    if(PWM0->INTSTS0 & BIT18)
    {
        /* Disable PWM0 channel 2 counting */
        PWM0->CNTEN &= ~PWM_CNTEN_CNTEN2_Msk;

        /* Clear PWM0 channel 2 compare up count interrupt flag */
        PWM0->INTSTS0 = BIT18;
        printf("PWM0 channel 2 compare up count interrupt occurred.\n");
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

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Select PWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL3_PWM0SEL_PCLK0, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pin for EINT1(PA.7) */
    SET_INT1_PA7();
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
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    GPIO EINT Trigger PWM Sample Code    |\n");
    printf("+-----------------------------------------+\n\n");
    printf("Use EINT1(PA.7) falling edge to start PWM0 channel 2 counting.\n");

    /* Set PWM0 channel 2 comparator register */
    PWM_SET_CMR(PWM0, 2, 0xFFFF);

    /* Set PWM0 channel 2 period register */
    PWM_SET_CNR(PWM0, 2, 0xFFFF);

    /* Enable PWM0 channel 2 compare up count interrupt */
    PWM0->INTEN0 |= PWM_INTEN0_CMPUIEN2_Msk;
    NVIC_EnableIRQ(PWM0P1_IRQn);

    /* Start PWM0 channel 2 counting by external trigger EINT1 */
    PWM0->EXTETCTL2 = (0x1<<PWM_EXTETCTL2_EXTTRGS_Pos) | (0x1<<PWM_EXTETCTL0_CNTACTS_Pos) | PWM_EXTETCTL2_EXTETEN_Msk;

    /* Configure PA.7 as EINT1 pin and enable interrupt by falling edge trigger */
    GPIO_SetMode(PA, BIT7, GPIO_MODE_INPUT);
    GPIO->INT_EDETCTL = (GPIO->INT_EDETCTL & (~GPIO_INT_EDETCTL_EDETCTL1_Msk)) | (0x2<<GPIO_INT_EDETCTL_EDETCTL1_Pos);
    GPIO->INT_EDINTEN |= GPIO_INT_EDINTEN_EDIEN1_Msk;
    NVIC_EnableIRQ(EINT1_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PA, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT7);

    while(1);
}
