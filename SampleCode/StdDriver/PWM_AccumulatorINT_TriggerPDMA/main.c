/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate PWM accumulator interrupt trigger PDMA.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static uint16_t g_au16Period[2] = {31999, 15999};
static volatile uint32_t g_u32IsTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);


/**
 * @brief       PDMA IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PDMA interrupt event
 */
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF0_Msk)
            g_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(u32Status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF0_Msk)
            g_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt !!\n");
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
    uint32_t u32NewCNR = 0;
    uint32_t u32TimeOutCnt = 0;

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
    printf("|                         PWM Driver Sample Code                         |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code demonstrate PWM0 channel 0 accumulator interrupt trigger PDMA.\n");
    printf("  When accumulator interrupt happens, PWM0 channel 0 period register will be updated \n");
    printf("  to g_u32Count array content, 31999(0x7CFF), by PDMA.\n");

    printf("\n\nPress any key to start PWM0 channel 0.\n");
    getchar();

    /*--------------------------------------------------------------------------------------*/
    /* Set the PWM0 Channel 0 as PWM output function.                                     */
    /*--------------------------------------------------------------------------------------*/

    /* Set PWM0 channel 0 output configuration */
    PWM_ConfigOutputChannel(PWM0, 0, 300, 30);

    /* Enable PWM Output path for PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    /* Enable PWM0 channel 0 accumulator, interrupt count 50, accumulator source select to zero point */
    PWM_EnableAcc(PWM0, 0, 50, PWM_IFA_ZERO_POINT);

    /* Enable PWM0 channel 0 accumulator interrupt */
    PWM_EnableAccInt(PWM0, 0);

    /* Enable PWM0 channel 0 accumulator interrupt trigger PDMA */
    PWM_EnableAccPDMA(PWM0, 0);

    /* Enable Timer for PWM0 channel 0 */
    PWM_Start(PWM0, PWM_CH_0_MASK);

    /*--------------------------------------------------------------------------------------*/
    /* Configure PDMA peripheral mode form memory to PWM                                   */
    /*--------------------------------------------------------------------------------------*/
    /* Open Channel 0 */
    PDMA_Open(PDMA0, BIT0);

    /* Transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_16, 1);

    /* Set source address as g_u32Count array(increment) and destination address as PWM0 channel 0 period register(no increment) */
    PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)&g_au16Period[0], PDMA_SAR_INC, (uint32_t) & (PWM0->PERIOD[0]), PDMA_DAR_FIX);

    /* Select PDMA request source as PWM0_CH0_TX(PWM0 channel 0 accumulator interrupt) */
    PDMA_SetTransferMode(PDMA0, 0, PDMA_PWM0_CH0_TX, FALSE, 0);

    /* Set PDMA as single request type for PWM */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, PDMA_BURST_1);

    PDMA_EnableInt(PDMA0, 0, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    g_u32IsTestOver = 0;

    /* Wait for PDMA transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(g_u32IsTestOver != 1)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            goto lexit;
        }
    }

    u32NewCNR = PWM_GET_CNR(PWM0, 0);
    printf("\n\nPWM0 channel0 period register is updated to %d(0x%x)\n", u32NewCNR, u32NewCNR);
    printf("Press any key to stop PWM0 channel 0.\n");
    getchar();

    /* Set PWM0 channel 0 loaded value as 0 */
    PWM_Stop(PWM0, PWM_CH_0_MASK);

    /* Wait until PWM0 channel 0 Timer Stop */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((PWM0->CNT[0] & PWM_CNT_CNT_Msk) != 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PWM stop time-out!\n");
            break;
        }
    }

lexit:

    /* Disable Timer for PWM0 channel 0 */
    PWM_ForceStop(PWM0, PWM_CH_0_MASK);

    /* Disable PWM Output path for PWM0 channel 0 */
    PWM_DisableOutput(PWM0, PWM_CH_0_MASK);

    /* Disable PDMA NVIC */
    NVIC_DisableIRQ(PDMA0_IRQn);

    /* Close PDMA */
    PDMA_Close(PDMA0);

    while(1) {}
}
