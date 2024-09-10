/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * @brief    Perform A/D Conversion with ADC single mode.
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
void AdcSingleModeTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;


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
    /* Disable digital input path of ADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (BIT0 | BIT1 | BIT2));
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
/* Function: AdcSingleModeTest                                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC single mode test.                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void AdcSingleModeTest()
{
    uint8_t  u8Option;
    int32_t  i32ConversionData;
    uint32_t u32TimeOutCnt;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADC single mode sample code                     |\n");
    printf("+----------------------------------------------------------------------+\n");

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (only channel pair 0)\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Power on ADC module */
            ADC_POWER_ON(ADC0);

            /* Set the ADC operation mode as single, input mode as single-end and enable the analog input channel 2 */
            ADC_Open(ADC0, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT2);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

            /* Enable the ADC interrupt */
            ADC_EnableInt(ADC0, ADC_ADF_INT);
            NVIC_EnableIRQ(ADC0_IRQn);

            /* Reset the ADC interrupt indicator and Start A/D conversion */
            g_u32AdcIntFlag = 0;

            ADC_START_CONV(ADC0);

            /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function)*/
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(g_u32AdcIntFlag == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for ADC interrupt time-out!\n");
                    return;
                }
            }

            /* Disable the ADC interrupt */
            ADC_DisableInt(ADC0, ADC_ADF_INT);

            /* Get the conversion result of the ADC channel 2 */
            i32ConversionData = ADC_GET_CONVERSION_DATA(ADC0, 2);
            printf("Conversion result of channel 2: 0x%X (%d)\n\n", i32ConversionData, i32ConversionData);
        }
        else if(u8Option == '2')
        {
            /* Power on ADC module */
            ADC_POWER_ON(ADC0);

            /* Set the ADC operation mode as single, input mode as differential and
               enable analog input channel 0 for differential input channel pair 0 */
            ADC_Open(ADC0, ADC_ADCR_DIFFEN_DIFFERENTIAL, ADC_ADCR_ADMD_SINGLE, BIT0);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

            /* Enable the ADC interrupt */
            ADC_EnableInt(ADC0, ADC_ADF_INT);
            NVIC_EnableIRQ(ADC0_IRQn);

            /* Reset the ADC interrupt indicator and Start A/D conversion */
            g_u32AdcIntFlag = 0;
            ADC_START_CONV(ADC0);

            /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function)*/
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(g_u32AdcIntFlag == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for ADC interrupt time-out!\n");
                    return;
                }
            }

            /* Disable the ADC interrupt */
            ADC_DisableInt(ADC0, ADC_ADF_INT);

            /* Get the conversion result of the specified ADC channel */
            i32ConversionData = ADC_GET_CONVERSION_DATA(ADC0, 0);
            printf("Conversion result of channel pair 0: 0x%X (%d)\n\n", i32ConversionData, i32ConversionData);
        }
        else
            return ;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ADC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void ADC0_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT); /* clear the A/D conversion flag */
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

    /* Single Mode test */
    AdcSingleModeTest();

    /* Disable ADC module */
    ADC_Close(ADC0);

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC0_IRQn);

    printf("\nExit ADC sample code\n");

    while(1) {}
}
