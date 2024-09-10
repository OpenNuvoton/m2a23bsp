/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * @brief    Perform A/D Conversion with ADC burst mode.
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
void ADC_FunctionTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define CONV_TOTAL_COUNT    20


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

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC0_MODULE);

    /* Set multi-function pins for ADC channels */
    SET_ADC0_CH2_PB2();
    SET_ADC0_CH3_PB3();
    /* Disable digital input path of ADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (BIT2 | BIT3));
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

void ADC_FunctionTest()
{
    uint8_t  u8Option;
    uint32_t u32ConvCount;
    int32_t  i32ConversionData[CONV_TOTAL_COUNT];

    printf("\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("|                   ADC burst mode sample code                       |\n");
    printf("+--------------------------------------------------------------------+\n");

    for(u32ConvCount = 0; u32ConvCount < CONV_TOTAL_COUNT; u32ConvCount++)
    {
        i32ConversionData[u32ConvCount] = 0;
    }

    /* Enable ADC converter */
    ADC_POWER_ON(ADC0);

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  Other keys: exit burst mode test\n");
        u8Option = getchar();

        if(u8Option == '1')
        {
            /* Set input mode as single-end, burst mode, and select channel 2 */
            ADC_Open(ADC0, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_BURST, BIT2);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

            /* Reset the ADC interrupt indicator and trigger sample module to start A/D conversion */
            u32ConvCount = 0;
            ADC_START_CONV(ADC0);

            while(1)
            {
                /* Wait ADC conversion completed */
                while(ADC_GET_INT_FLAG(ADC0, ADC_ADF_INT) == 0);
                ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT); /* clear ADF interrupt flag */

                /* Get the conversion result until VALIDF turns to 0 */
                while(ADC0->ADSR0 & ADC_ADSR0_VALIDF_Msk)
                {
                    /* Get the conversion result from ADC channel 0 always */
                    i32ConversionData[u32ConvCount++] = ADC_GET_CONVERSION_DATA(ADC0, 0);
                    if(u32ConvCount == CONV_TOTAL_COUNT)
                        break;
                }

                if(u32ConvCount == CONV_TOTAL_COUNT)
                    break;
            }

            /* Stop A/D conversion */
            ADC_STOP_CONV(ADC0);

            /* Show the conversion result */
            for(u32ConvCount = 0; u32ConvCount < CONV_TOTAL_COUNT; u32ConvCount++)
            {
                printf("Conversion result of channel 2 [#%d]: 0x%X (%d)\n", u32ConvCount + 1, i32ConversionData[u32ConvCount], i32ConversionData[u32ConvCount]);
            }

            /* Clear remaining data in FIFO that got before stop ADC */
            while(ADC_IS_DATA_VALID(ADC0, 0))
            {
                i32ConversionData[0] = ADC_GET_CONVERSION_DATA(ADC0, 0);
            }
        }
        else
            return;

        printf("\n");
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

    /* PADC function test */
    ADC_FunctionTest();

    /* Disable ADC module */
    ADC_Close(ADC0);

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC0_MODULE);

    printf("\nExit ADC sample code\n");

    while(1) {}
}
