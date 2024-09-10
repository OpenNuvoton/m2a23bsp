/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Capture the PWM0 Channel 0 waveform by PWM0 Channel 2, and use PDMA to transfer captured data.
             Frequency of PWM Channel 0 is 1 MHz to test maximum input frequency for PWM Capture function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define Global variables                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define Transfer_Count 4
uint16_t g_au16Count[Transfer_Count];
volatile uint32_t g_u32IsTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_IRQHandler(void);
void CalPeriodTime(PWM_T *PWM, uint32_t u32Ch);
void SYS_Init(void);
void UART0_Init(void);


void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & 0x1)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA0) & 0x1)
            g_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(status & 0x2)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA0) & 0x1)
            g_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* g_au16Count[4] : Keep the internal counter value when input signal rising / falling  */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0xFFFF, and reload to 0xFFFF after      */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
void CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
{
    uint16_t u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCount;

    /* setup timeout */
    u32TimeOutCount = SystemCoreClock;

    /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
    while(g_u32IsTestOver == 0)
    {
        if(u32TimeOutCount == 0)
        {
            printf("\nSomething is wrong, please check if pin connection is correct. \n");
            while(1);
        }
        u32TimeOutCount--;
    }

    u16HighPeriod = g_au16Count[1] - g_au16Count[2] + 1;

    u16LowPeriod = 0xFFFF - g_au16Count[1];

    u16TotalPeriod = 0xFFFF - g_au16Count[2] + 1;

    printf("\nHigh Period = %d ns, Low Period = %d ns , Total Period = %d ns.\n",
           u16HighPeriod * 1000 / 48, u16LowPeriod * 1000 / 48, u16TotalPeriod * 1000 / 48);
    printf("Frequency = %d Hz, Duty = %d %%.\n\n",
           48000000 / u16TotalPeriod, u16HighPeriod * 100 / u16TotalPeriod);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

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
    SET_PWM0_CH2_PB3();

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);
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
    uint8_t u8Option;

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
    printf("PWM0 clock is from %s\n", (CLK->CLKSEL3 & CLK_CLKSEL3_PWM0SEL_Msk) ? "PCLK" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Capture Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM0 channel 2 to capture the signal from PWM0 channel 0.\n");
    printf("  And the captured data is transferred by PDMA channel 0.\n");
    printf("  PWM0 channel 0 is 1 MHz, duty is 50 %%.\n");
    printf("  PWM0 channel 2 counter period is about 27.8 (1/36,000,000) ns\n");
    printf("  I/O configuration:\n");
    printf("    PWM0 channel 2(PB.3) <--> PWM0 channel 0(PB.5)\n\n");
    printf("Use PWM0 Channel 2(PB.3) to capture the PWM0 Channel 0(PB.5) Waveform\n");

    /*--------------------------------------------------------------------------------------*/
    /* Set the PWM0 Channel 0 as PWM output function.                                       */
    /*--------------------------------------------------------------------------------------*/

    /* Assume PWM output frequency is 1 MHz and duty ratio is 50%, user can calculate PWM settings by follows.
         duty ratio = CMR+/(CNR+1)
         cycle time = CNR+1
         High level = CMR
         PWM clock source frequency from PLL/2 is 36,000,000
         (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
                         = 36,000,000/1/1,000,000 = 36
         (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
         CNR = 35
         duty ratio = 50% ==> CMR/(CNR+1) = 50%
         CMR = 18
         Prescale value is 0 : prescaler = 1
    */

    /* PWM0 channel 0 frequency prescaler to 1 */
    PWM_SET_PRESCALER(PWM0, 0, 1 - 1);

    /* PWM0 channel 0 frequency period to 35 */
    PWM_SET_CNR(PWM0, 0, 35);

    /* PWM0 channel 0 frequency comparator to 18 */
    PWM_SET_CMR(PWM0, 0, 18);

    /* PWM0 channel 0 is edge-aligned and down counter type */
    PWM_SET_ALIGNED_TYPE(PWM0, BIT0, PWM_EDGE_ALIGNED);

    /* ZeroLevel: Low, CmpUpLevel: nothing, PeriodLevel: nothing, CmpDownLevel: High */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_LOW, PWM_OUTPUT_HIGH, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH);

    /* Enable PWM Output path for PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    /* Enable Timer for PWM0 channel 0 */
    PWM_Start(PWM0, PWM_CH_0_MASK);

    /*--------------------------------------------------------------------------------------*/
    /* Configure PDMA peripheral mode form PWM to memory                                    */
    /*--------------------------------------------------------------------------------------*/
    /* Open Channel 0 */
    PDMA_Open(PDMA0, BIT0);

    /* Set source address as PWM capture channel PDMA register(no increment) and destination address as g_au16Count array(increment) */
    PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)&PWM0->PDMACAP2_3, PDMA_SAR_FIX, (uint32_t)&g_au16Count[0], PDMA_DAR_INC);

    /* Set PDMA as single request type for PWM */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, PDMA_BURST_4);

    PDMA_EnableInt(PDMA0, 0, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    /*--------------------------------------------------------------------------------------*/
    /* Set the PWM0 channel 2 for capture function                                          */
    /*--------------------------------------------------------------------------------------*/
    /* (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
         CNR = 0xFFFF
         (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)

         Capture unit time = 1/Capture clock source frequency/prescaler
         27.8ns = 1/36,000,000/1
    */

    /* Set PWM0 channel 2 capture configuration */
    PWM_ConfigCaptureChannel(PWM0, 2, 27, 0);

    /* Enable falling capture reload */
    PWM0->CAPCTL |= PWM_CAPCTL_FCRLDEN2_Msk;

    while(1)
    {
        g_u32IsTestOver = 0;

        printf("[0] Trigger PWM0 Channel 2 capture function\n");
        printf("[Other] Exit \n");
        printf("Please input key:");
        u8Option = getchar();

        if(u8Option != '0')
        {
            printf("Exit\n");
            break;

        }

        /*--------------------------------------------------------------------------------------*/
        /* Configure PDMA transfer count and enable PDMA function                               */
        /*--------------------------------------------------------------------------------------*/
        /* Select PDMA request source as PWM RX(PWM0 channel 2 should be PWM0 pair 2) */
        PDMA_SetTransferMode(PDMA0, 0, PDMA_PWM0_P2_RX, FALSE, 0);

        /* Transfer width is half word(16 bit) and transfer count is 4 */
        PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_16, Transfer_Count);

        /* Enable Capture Function for PWM0 channel 2 */
        PWM_EnableCapture(PWM0, PWM_CH_2_MASK);

        /* Enable Timer for PWM0 channel 2 */
        PWM_Start(PWM0, PWM_CH_2_MASK);

        /* Enable PDMA for PWM0 channel 2 capture function, and set capture order as falling first, */
        /* And select capture mode as both rising and falling to do PDMA transfer. */
        PWM_EnablePDMA(PWM0, 2, FALSE, PWM_CAPTURE_PDMA_RISING_FALLING_LATCH);

        /* Capture the Input Waveform Data */
        CalPeriodTime(PWM0, 2);

        /* Set loaded value as 0 for PWM0 channel 2 */
        PWM_Stop(PWM0, PWM_CH_2_MASK);

        /* Clear Capture Interrupt flag for PWM0 channel 2 */
        PWM_ClearCaptureIntFlag(PWM0, 2, PWM_CAPTURE_INT_FALLING_LATCH);

        /* Disable PDMA for PWM0 channel 2 capture function */
        PWM_DisablePDMA(PWM0, 2);
    }

    /*---------------------------------------------------------------------------------------------------------*/
    /* Stop PWM0 channel 2 (Recommended procedure method 1)                                                    */
    /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set loaded value as 0 for PWM0 channel 2 */
    PWM_Stop(PWM0, PWM_CH_2_MASK);

    /* Disable Timer for PWM0 channel 2 */
    PWM_ForceStop(PWM0, PWM_CH_2_MASK);

    /* Stop PWM counter */
    PWM_Stop(PWM0, PWM_CH_0_MASK);
    /* Disable output of PWM0 channel 0 */
    PWM_DisableOutput(PWM0, PWM_CH_0_MASK);

    PDMA_Close(PDMA0);

    while(1) {}
}
