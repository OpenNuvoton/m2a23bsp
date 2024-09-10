/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use the timer2 capture function to capture timer2 counter value.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void TMR2_IRQHandler(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[4] = {0};


void TMR2_IRQHandler(void)
{
    if(TIMER_GetCaptureIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 capture trigger interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER2);

        g_au32TMRINTCount[2]++;
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
    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);

    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HIRC, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HXT, 0);

    /* Set multi-function pins for Timer0/Timer3 toggle-output pin and Timer2 event counter pin */
    SET_TM0_PB5();
    SET_TM2_PB3();
    SET_TM3_PB2();

    /* Set multi-function pin for Timer2 external capture pin */
    SET_TM2_EXT_PB13();
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
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32InitCount;
    uint32_t au32CAPValue[12], u32CAPDiff;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    Timer2 Capture Counter Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 1000 Hz\n");
    printf("    - Toggle-output mode and frequency is 500 Hz\n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Toggle-output mode and frequency is 1 Hz\n");
    printf("# Timer2 Settings:\n");
    printf("    - Clock source is HIRC              \n");
    printf("    - Continuous counting mode          \n");
    printf("    - Interrupt enable                  \n");
    printf("    - Compared value is 0xFFFFFF        \n");
    printf("    - Event counter mode enable         \n");
    printf("    - External capture mode enable      \n");
    printf("    - Capture trigger interrupt enable  \n");
    printf("# Connect TM0(PB.5) toggle-output pin to TM2(PB.3) event counter pin.\n");
    printf("# Connect TM3(PB.2) toggle-output pin to TM2_EXT(PB.12) external capture pin.\n\n");

    /* Enable Timer2 NVIC */
    NVIC_EnableIRQ(TMR2_IRQn);

    /* Open Timer0 in toggle-output mode and toggle-output frequency is 500 Hz*/
    TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, 1000);

    /* Open Timer3 in toggle-output mode and toggle-output frequency is 1 Hz */
    TIMER_Open(TIMER3, TIMER_TOGGLE_MODE, 2);

    /* Enable Timer2 event counter input and external capture function */
    TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_SET_CMP_VALUE(TIMER2, 0xFFFFFF);
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_EVENT_FALLING);
    TIMER_EnableCapture(TIMER2, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_FALLING);
    TIMER_EnableInt(TIMER2);
    TIMER_EnableCaptureInt(TIMER2);


    /* case 1. */
    printf("# Period between two falling edge captured event should be 500 counts.\n");

    /* Clear Timer2 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount[2] = 0;

    /* Start Timer0, Timer3 and Timer2 counting */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER3);
    TIMER_Start(TIMER2);

    /* Check Timer2 capture trigger interrupt counts */
    while(g_au32TMRINTCount[2] < 10)
    {
        if(g_au32TMRINTCount[2] != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER_GetCaptureData(TIMER2);
            if(u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);
                if(au32CAPValue[u32InitCount] != 0)   // First capture event will reset counter value
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }
            else if(u32InitCount ==  1)
            {
                printf("    [%2d]: %4d. (2nd captured value) \n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);
                if(au32CAPValue[u32InitCount] != 500)   // Second event gets two capture event duration counts directly
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount], u32CAPDiff);
                if(u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }
            u32InitCount = g_au32TMRINTCount[2];
        }
    }
    printf("*** PASS ***\n\n");


    /* case 2. */
    TIMER_StopCapture(TIMER2);
    TIMER_Stop(TIMER2);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(TIMER_IS_ACTIVE(TIMER2))
        if(--u32TimeOutCnt == 0) break;
    TIMER_ClearIntFlag(TIMER2);
    TIMER_ClearCaptureIntFlag(TIMER2);
    /* Enable Timer2 event counter input and external capture function */
    TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_SET_CMP_VALUE(TIMER2, 0xFFFFFF);
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_EVENT_FALLING);
    TIMER_EnableCapture(TIMER2, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_GET_LOW_PERIOD);
    TIMER_EnableInt(TIMER2);
    TIMER_EnableCaptureInt(TIMER2);
    TIMER_Start(TIMER2);

    printf("# Get first low duration should be 250 counts.\n");
    printf("# And follows duration between two rising edge captured event should be 500 counts.\n");

    /* Clear Timer2 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount[2] = 0;

    /* Enable Timer2 event counter input and external capture function */
    TIMER2->CMP = 0xFFFFFF;
    TIMER2->CTL = TIMER_CTL_CNTEN_Msk | TIMER_CTL_INTEN_Msk | TIMER_CTL_EXTCNTEN_Msk | TIMER_CONTINUOUS_MODE;
    TIMER2->EXTCTL = TIMER_EXTCTL_CAPEN_Msk | TIMER_CAPTURE_FREE_COUNTING_MODE | TIMER_CAPTURE_EVENT_GET_LOW_PERIOD | TIMER_EXTCTL_CAPIEN_Msk;

    /* Check Timer2 capture trigger interrupt counts */
    while(g_au32TMRINTCount[2] <= 10)
    {
        if(g_au32TMRINTCount[2] != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER_GetCaptureData(TIMER2);
            if(u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);
                if(au32CAPValue[u32InitCount] != 0)   // First capture event will reset counter value
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }
            else if(u32InitCount ==  1)
            {
                printf("    [%2d]: %4d. (2nd captured value)\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);
                if(au32CAPValue[u32InitCount] != 250)   // Get low duration counts directly
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount], u32CAPDiff);
                if(u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }
            u32InitCount = g_au32TMRINTCount[2];
        }
    }

    printf("*** PASS ***\n");

lexit:

    /* Stop Timer0, Timer2 and Timer3 counting */
    TIMER_Stop(TIMER0);
    TIMER_Stop(TIMER2);
    TIMER_Stop(TIMER3);

    while(1) {}
}
