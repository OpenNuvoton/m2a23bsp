/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * @brief    Perform A/D Conversion with ADC continuous scan mode.
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
void AdcContScanModeTest(void);


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


    /* ADC clock source is HCLK = 72MHz, set divider to 12, ADC clock is 72/12 MHz */
    CLK_SetModuleClock(ADC0_MODULE, CLK_CLKSEL3_ADC0SEL_HCLK, CLK_CLKDIV0_ADC0(12));

    /* Enable LPADC module clock */
    CLK_EnableModuleClock(ADC0_MODULE);

    /* Set multi-function pins for ADC channels */
    SET_ADC0_CH0_PB0();
    SET_ADC0_CH1_PB1();
    SET_ADC0_CH2_PB2();
    SET_ADC0_CH3_PB3();
    /* Disable digital input path of ADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (BIT0 | BIT1 | BIT2 | BIT3));
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

/*---------------------------------------------------------------------------------------------------------*/
/* Function: ADC_GetConversionRate                                                                         */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   Return the A/D conversion rate (sample/second)                                                        */
/*                                                                                                         */
/* Description:                                                                                            */
/*   The conversion rate depends on the clock source of ADC clock.                                         */
/*   It only needs 18 ADC clocks to complete an A/D conversion.                                            */
/*---------------------------------------------------------------------------------------------------------*/
static __INLINE uint32_t ADC_GetConversionRate()
{
    uint32_t u32AdcClkSrcSel;
    uint32_t u32ClkTbl[4] = {0, 0, 0, __HIRC};
    uint32_t u32ADCCLKCnt = 18;

    /* Set the PLL clock frequency */
    u32ClkTbl[1] = PllClock;

    /* Set the system core clock frequency */
    u32ClkTbl[2] = SystemCoreClock;

    /* Get the clock source setting */
    u32AdcClkSrcSel = (((CLK->CLKSEL3 & CLK_CLKSEL3_ADCSEL_Msk) >> CLK_CLKSEL3_ADCSEL_Pos));

    /* Return the ADC conversion rate */
    return ((u32ClkTbl[u32AdcClkSrcSel]) / (((CLK->CLKDIV0 & CLK_CLKDIV0_ADCDIV_Msk) >> CLK_CLKDIV0_ADCDIV_Pos) + 1) / u32ADCCLKCnt);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: AdcContScanModeTest                                                                           */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC continuous scan mode test.                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AdcContScanModeTest(void)
{
    uint8_t  u8Option;
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;
    uint32_t u32TimeOutCnt;

    printf("\n\nConversion rate: %d samples/second\n", ADC_GetConversionRate());
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC continuous scan mode sample code                 |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 2 cycles of conversion result from the specified channels.\n");

    while(1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (only input channel pair 0)\n");
        printf("  Other keys: exit continuous scan mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Power on ADC module */
            ADC_POWER_ON(ADC0);

            /* Set the ADC operation mode as continuous scan, input mode as single-end and
                 enable the analog input channel 0, 1, 2 and 3 */
            ADC_Open(ADC0, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, 0xF);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

            /* Start A/D conversion */
            ADC_START_CONV(ADC0);

            /* Wait conversion done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(!ADC_GET_INT_FLAG(ADC0, ADC_ADF_INT))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for ADC conversion done time-out!\n");
                    return;
                }
            }

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC0, u32ChannelCount);
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Wait conversion done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(!ADC_GET_INT_FLAG(ADC0, ADC_ADF_INT))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for ADC conversion done time-out!\n");
                    return;
                }
            }

            /* Stop A/D conversion */
            ADC_STOP_CONV(ADC0);

            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC0, u32ChannelCount);
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

        }
        else if(u8Option == '2')
        {
            /* Power on ADC module */
            ADC_POWER_ON(ADC0);

            /* Set the ADC operation mode as continuous scan, input mode as differential and
               enable analog input channel 0 */
            ADC_Open(ADC0, ADC_ADCR_DIFFEN_DIFFERENTIAL, ADC_ADCR_ADMD_CONTINUOUS, BIT0);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

            /* Start A/D conversion */
            ADC_START_CONV(ADC0);

            /* Wait conversion done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(!ADC_GET_INT_FLAG(ADC0, ADC_ADF_INT))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for ADC conversion done time-out!\n");
                    return;
                }
            }

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

            for(u32ChannelCount = 0; u32ChannelCount < 1; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC0, u32ChannelCount * 2);
                printf("Conversion result of differential input pair %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Wait conversion done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(!ADC_GET_INT_FLAG(ADC0, ADC_ADF_INT))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for ADC conversion done time-out!\n");
                    return;
                }
            }

            /* Stop A/D conversion */
            ADC_STOP_CONV(ADC0);

            for(u32ChannelCount = 0; u32ChannelCount < 1; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC0, u32ChannelCount * 2);
                printf("Conversion result of differential input pair %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

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

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* Continuous scan mode test */
    AdcContScanModeTest();

    /* Disable ADC module */
    ADC_Close(ADC0);

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC0_IRQn);

    printf("\nExit ADC sample code\n");

    while(1) {}
}
