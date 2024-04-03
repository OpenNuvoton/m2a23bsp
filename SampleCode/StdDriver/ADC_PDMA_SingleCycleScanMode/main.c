/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * @brief    Perform A/D Conversion with ADC single cycle scan mode and transfer result by PDMA.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define PDMA_CH         2
#define ADC_TEST_COUNT  32


/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void AdcSingleCycleScanModePDMATest(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef __ICCARM__
#pragma data_alignment=4
uint32_t g_au32RxPDMADestination[ADC_TEST_COUNT];
#else
__attribute__((aligned(4))) uint32_t g_au32RxPDMADestination[ADC_TEST_COUNT];
#endif
uint32_t au32AdcData[ADC_TEST_COUNT];

volatile uint32_t g_u32PdmaTDoneInt;
volatile uint32_t g_u32PdmaTAbortInt;


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

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


    /* ADC clock source is HCLK = 72MHz, set divider to 2, ADC clock is 72/2 MHz */
    CLK_SetModuleClock(ADC0_MODULE, CLK_CLKSEL3_ADC0SEL_HCLK, CLK_CLKDIV0_ADC0(2));

    /* Enable LPADC module clock */
    CLK_EnableModuleClock(ADC0_MODULE);

    /* Set multi-function pins for ADC channels */
    SET_ADC0_CH0_PB0(); 
    SET_ADC0_CH1_PB1();
    SET_ADC0_CH2_PB2();
    SET_ADC0_CH3_PB3();
    /* Disable digital input path of ADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (BIT0|BIT1|BIT2|BIT3));
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_PORT, 115200);
}

void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);
    uint32_t u32TDStatus = PDMA_GET_TD_STS(PDMA0);

    if(u32Status & PDMA_INTSTS_TDIF_Msk)
    {
        if(u32TDStatus & PDMA_TDSTS_TDIF0_Msk) /* CH0 */
        {
            g_u32PdmaTDoneInt = 1;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk);
        }
        else if(u32TDStatus & PDMA_TDSTS_TDIF1_Msk) /* CH1 */
        {
            g_u32PdmaTDoneInt = 2;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF1_Msk);
        }
        else if(u32TDStatus & PDMA_TDSTS_TDIF2_Msk) /* CH2 */
        {
            g_u32PdmaTDoneInt = 3;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
        }
        else if(u32TDStatus & PDMA_TDSTS_TDIF3_Msk) /* CH3 */
        {
            g_u32PdmaTDoneInt = 4;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF3_Msk);
        }
        else if(u32TDStatus & PDMA_TDSTS_TDIF4_Msk) /* CH4 */
        {
            g_u32PdmaTDoneInt = 5;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF4_Msk);
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: PDMA_Init                                                                                     */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Configure PDMA channel for ADC continuous scan mode test.                                             */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_Init()
{
    /* Enable PDMA module clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Open Channel 2 */
    PDMA->CHCTL |= (1 << PDMA_CH);

    /* Transfer configuration of Channel 2 */
    PDMA->DSCT[PDMA_CH].CTL = \
                              ((ADC_TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is ADC_TEST_COUNT */ \
                              PDMA_WIDTH_32 |  /* Transfer width is 32 bits(one word) */ \
                              PDMA_SAR_FIX |   /* Source increment size is no increment(fixed) */ \
                              PDMA_DAR_INC |   /* Destination increment size is 32 bits(one word) */ \
                              PDMA_REQ_SINGLE | /* Transfer type is burst transfer type */ \
                              PDMA_BURST_1 |   /* Burst size is 1. No effect in single transfer type */ \
                              PDMA_OP_BASIC;   /* Operation mode is basic mode */

    /* Configure source address */
    PDMA->DSCT[PDMA_CH].SA = (uint32_t)&ADC0->ADPDMA;

    /* Configure destination address */
    PDMA->DSCT[PDMA_CH].DA = (uint32_t)&g_au32RxPDMADestination;

    /* Configure PDMA channel 2 as memory to memory transfer */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC0_Pos) | (PDMA_ADC0_RX << PDMA_REQSEL0_3_REQSRC2_Pos);

    /* Enable transfer done interrupt */
    PDMA->INTEN |= (1 << PDMA_CH);

    /* Enable PDMA IRQ */
    NVIC_EnableIRQ(PDMA0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: PDMA_ConfigReload                                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Reload transfer count and operation mode of PDMA channel for ADC continuous scan mode test.           */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_ConfigReload()
{
    /* Transfer configuration of Channel 2 */
    PDMA->DSCT[PDMA_CH].CTL |= \
                               ((ADC_TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is ADC_TEST_COUNT */ \
                               PDMA_OP_BASIC;   /* Operation mode is basic mode */
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: AdcSingleCycleScanModePDMATest                                                                */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC single cycle scan mode with PDMA test.                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void AdcSingleCycleScanModePDMATest()
{
    uint8_t  u8Option;
    uint32_t u32DataCount;
    uint32_t u32ErrorCount;
    uint32_t u32TimeOutCnt;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|            ADC single cycle scan mode with PDMA sample code          |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Init PDMA channel to transfer ADC conversion results */
    PDMA_Init();

    while(1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (only input channel pair 0)\n");
        printf("  Other keys: exit single cycle scan mode with PDMA test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Reload transfer count and operation mode of PDMA channel for ADC continuous scan mode test. */
            PDMA_ConfigReload();

            /* Power on ADC module */
            ADC_POWER_ON(ADC0);

            /* Set the ADC operation mode as single-cycle, input mode as single-end and
                 enable the analog input channel 0, 1, 2 and 3 */
            ADC_Open(ADC0, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, 0xF);

            /* Enable PDMA transfer */
            ADC_ENABLE_PDMA(ADC0);

            /* Clear destination buffer */
            for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
                g_au32RxPDMADestination[u32DataCount] = 0;

            u32DataCount = 0;
            u32ErrorCount = 0;
            g_u32PdmaTDoneInt = 0;

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

            /* Start A/D conversion */
            ADC_START_CONV(ADC0);

            while(1)
            {
                uint32_t u32Ch;
                if(ADC_GET_INT_FLAG(ADC0, ADC_ADF_INT))
                {
                    /* Clear the ADC interrupt flag */
                    ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

                    for(u32Ch = 0; u32Ch < 4; u32Ch++)
                    {
                        au32AdcData[u32DataCount++] = ADC_GET_CONVERSION_DATA(ADC0, u32Ch);
                        if(u32DataCount >= ADC_TEST_COUNT)
                            break;
                    }
                    if(u32DataCount < ADC_TEST_COUNT)
                        ADC_START_CONV(ADC0); /* Start A/D conversion */
                    else
                        break;
                }
            }

            /* Wait for PDMA transfer done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(g_u32PdmaTDoneInt == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for PDMA transfer done time-out!\n");
                    return;
                }
            }

            /* Compare the log of ADC conversion data register with the content of PDMA target buffer */
            for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
            {
                if(au32AdcData[u32DataCount] != (g_au32RxPDMADestination[u32DataCount] & 0xFFF))
                {
                    printf("*** Count %d, conversion result: 0x%X, PDMA result: 0x%X.\n",
                           u32DataCount, au32AdcData[u32DataCount], g_au32RxPDMADestination[u32DataCount]);
                    u32ErrorCount++;
                }
            }

            if(u32ErrorCount == 0)
                printf("\n    PASS!\n");
            else
                printf("\n    FAIL!\n");
        }
        else if(u8Option == '2')
        {
            /* Reload transfer count and operation mode of PDMA channel for ADC continuous scan mode test. */
            PDMA_ConfigReload();

            /* Power on ADC module */
            ADC_POWER_ON(ADC0);

			/* Set the ADC operation mode as single-cycle, input mode as differential and
               enable analog input channel 0 */
            ADC_Open(ADC0, ADC_ADCR_DIFFEN_DIFFERENTIAL, ADC_ADCR_ADMD_SINGLE_CYCLE, BIT0);

            /* Enable PDMA transfer */
            ADC_ENABLE_PDMA(ADC0);

            /* Clear destination buffer */
            for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
                g_au32RxPDMADestination[u32DataCount] = 0;

            u32DataCount = 0;
            u32ErrorCount = 0;
            g_u32PdmaTDoneInt = 0;

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

            /* Start A/D conversion */
            ADC_START_CONV(ADC0);

            while(1)
            {
                uint32_t u32Ch;
                if(ADC_GET_INT_FLAG(ADC0, ADC_ADF_INT))
                {
                    /* Clear the ADC interrupt flag */
                    ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

                    for(u32Ch = 0; u32Ch < 2; u32Ch += 2)
                    {
                        au32AdcData[u32DataCount++] = ADC_GET_CONVERSION_DATA(ADC0, u32Ch);
                        if(u32DataCount >= ADC_TEST_COUNT)
                            break;
                    }
                    if(u32DataCount < ADC_TEST_COUNT)
                        ADC_START_CONV(ADC0); /* Start A/D conversion */
                    else
                        break;
                }
            }

            /* Wait for PDMA transfer done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(g_u32PdmaTDoneInt == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for PDMA transfer done time-out!\n");
                    return;
                }
            }

            /* Compare the log of ADC conversion data register with the content of PDMA target buffer */
            for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
            {
                if(au32AdcData[u32DataCount] != (g_au32RxPDMADestination[u32DataCount] & 0xFFF))
                {
                    printf("*** Count %d, conversion result: 0x%X, PDMA result: 0x%X.\n",
                           u32DataCount, au32AdcData[u32DataCount], g_au32RxPDMADestination[u32DataCount]);
                    u32ErrorCount++;
                }
            }

            if(u32ErrorCount == 0)
                printf("\n    PASS!\n");
            else
                printf("\n    FAIL!\n");
        }
        else
            return ;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* Single cycle scan mode test */
    AdcSingleCycleScanModePDMATest();

    /* Disable ADC module */
    ADC_Close(ADC0);

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC0_IRQn);

    printf("\nExit ADC sample code\n");

    while(1) {}
}
