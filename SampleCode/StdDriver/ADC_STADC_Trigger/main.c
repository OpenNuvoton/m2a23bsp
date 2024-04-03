/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * @brief    Show how to trigger ADC by STADC pin.
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
void UART0_Init(void);
void ADC0_IRQHandler(void);
void ADC_FunctionTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32adcIntFlag, g_u32COVNUMFlag = 0;


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
    SET_ADC0_CH0_PB0();
    /* Disable digital input path of ADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0);
    
    /* Configure the PC.1 ADC trigger pin */
    SET_ADC0_ST_PC1();
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
/* ADC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void ADC0_IRQHandler(void)
{
    ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT); /* Clear the A/D interrupt flag */
    g_u32adcIntFlag = 1;
    g_u32COVNUMFlag++;
    printf("[#%d] ADC conversion done.\n", g_u32COVNUMFlag);
}

void ADC_FunctionTest()
{
    uint8_t  u8Option;
    int32_t  i32ConversionData[6] = {0};

    printf("\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("|                 ADC trigger by STADC pin test                      |\n");
    printf("+--------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel\n");
    printf("   that triggered by STADC pin (PC.1).\n");

    /* Enable ADC converter */
    ADC_POWER_ON(ADC0);

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 0 only)\n");
        printf("  [2] Differential input (channel pair 0 only)\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set input mode as single-end, Single mode, and select channel 0 */
            ADC_Open(ADC0, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT0);

            /* Configure the sample module and enable STADC pin trigger source */
            ADC_EnableHWTrigger(ADC0, ADC_ADCR_TRGS_STADC, ADC_ADCR_TRGCOND_FALLING_EDGE);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

            /* Enable the sample module interrupt */
            ADC_EnableInt(ADC0, ADC_ADF_INT);
            NVIC_EnableIRQ(ADC0_IRQn);

            printf("Wait falling edge from STADC pin (PC.1) ...\n");

            /* Reset the ADC indicator and wait falling edge on STADC pin */
            g_u32adcIntFlag = 0;
            g_u32COVNUMFlag = 0;

            while(1)
            {
                /* Wait ADC interrupt (g_u32adcIntFlag will be set at IRQ_Handler function) */
                while(g_u32adcIntFlag == 0);

                /* Reset the ADC interrupt indicator */
                g_u32adcIntFlag = 0;

                /* Get the conversion result of ADC channel 0 */
                i32ConversionData[g_u32COVNUMFlag - 1] = ADC_GET_CONVERSION_DATA(ADC0, 0);

                if(g_u32COVNUMFlag >= 6)
                    break;
            }

            /* Disable the sample module interrupt */
            ADC_DisableInt(ADC0, ADC_ADF_INT);

            printf("Conversion result of channel 0:\n");
            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);
        }
        else if(u8Option == '2')
        {
            /* Set input mode as differential, Single mode, and select channel 0 */
            ADC_Open(ADC0, ADC_ADCR_DIFFEN_DIFFERENTIAL, ADC_ADCR_ADMD_SINGLE, BIT0);

            /* Configure the sample module and enable STADC pin trigger source */
            ADC_EnableHWTrigger(ADC0, ADC_ADCR_TRGS_STADC, ADC_ADCR_TRGCOND_FALLING_EDGE);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC0, ADC_ADF_INT);

            /* Enable the sample module interrupt */
            ADC_EnableInt(ADC0, ADC_ADF_INT);
            NVIC_EnableIRQ(ADC0_IRQn);

            printf("Wait falling edge from STADC pin (PC.1) ...\n");

            /* Reset the ADC indicator and wait falling edge on STADC pin */
            g_u32adcIntFlag = 0;
            g_u32COVNUMFlag = 0;

            while(1)
            {
                /* Wait ADC interrupt (g_u32adcIntFlag will be set at IRQ_Handler function) */
                while(g_u32adcIntFlag == 0);

                /* Reset the ADC interrupt indicator */
                g_u32adcIntFlag = 0;

                /* Get the conversion result of the sample module 0 */
                i32ConversionData[g_u32COVNUMFlag - 1] = ADC_GET_CONVERSION_DATA(ADC0, 0);

                if(g_u32COVNUMFlag >= 6)
                    break;
            }

            /* Disable the sample module interrupt */
            ADC_DisableInt(ADC0, ADC_ADF_INT);

            printf("Conversion result of channel pair 0:\n");
            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);
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
    
    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC module */
    ADC_Close(ADC0);

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC0_IRQn);

    printf("\nExit ADC sample code\n");

    while(1) {}
}
