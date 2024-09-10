/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of GPIO EINT trigger PDMA function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



void EINT1_IRQHandler(void);
void PDMA0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);


uint32_t PDMA_TEST_LENGTH = 64;
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t au8SrcArray[256];
uint8_t au8DestArray[256];
#else
__attribute__((aligned(4))) uint8_t au8SrcArray[256];
__attribute__((aligned(4))) uint8_t au8DestArray[256];
#endif
uint32_t volatile g_u32IsTestOver = 0;


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

/**
 * @brief       PDMA0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PDMA0 default IRQ, declared in startup_m2a23.s.
 */
void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA0);

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        /* Check if channel 2 has abort error */
        if(PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF2_Msk)
            g_u32IsTestOver = 2;
        /* Clear abort flag of channel 2 */
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        /* Check transmission of channel 2 has been transfer done */
        if(PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF2_Msk)
            g_u32IsTestOver = 1;
        /* Clear transfer done flag of channel 2 */
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
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
                   CLK_AHBCLK_GPIODCKEN_Msk | CLK_AHBCLK_GPIOFCKEN_Msk ;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

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
    printf("+------------------------------------------+\n");
    printf("|    GPIO EINT Trigger PDMA Sample Code    |\n");
    printf("+------------------------------------------+\n\n");
    printf("Use EINT1(PA.7) falling edge to start PDMA transfer.\n");

    /*---------------------------------------------------------------------------------------------------------*/
    /* PDMA Configuration                                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Open Channel 2 */
    PDMA_Open(PDMA0, BIT2);
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, destination address is au8DestArray, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA0, 2, (uint32_t)au8SrcArray, PDMA_SAR_INC, (uint32_t)au8DestArray, PDMA_DAR_INC);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(PDMA0, 2, PDMA_EINT1, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA0, 2, PDMA_REQ_BURST, PDMA_BURST_4);

    /* Enable interrupt */
    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA0_IRQn);
    g_u32IsTestOver = 0;

    /*---------------------------------------------------------------------------------------------------------*/
    /* GPIO Configuration                                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure PA.7 as EINT1 pin and enable interrupt by falling edge trigger */
    GPIO_SetMode(PA, BIT7, GPIO_MODE_INPUT);
    GPIO->INT_EDETCTL = (GPIO->INT_EDETCTL & (~GPIO_INT_EDETCTL_EDETCTL1_Msk)) | (0x2 << GPIO_INT_EDETCTL_EDETCTL1_Pos);
    GPIO->INT_EDINTEN |= GPIO_INT_EDINTEN_EDIEN1_Msk;
    NVIC_EnableIRQ(EINT1_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PA, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT7);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Waiting for PDMA transfer done                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Waiting for transfer done */
    while(g_u32IsTestOver == 0);

    /* Check transfer result */
    if(g_u32IsTestOver == 1)
        printf("test done...\n");
    else if(g_u32IsTestOver == 2)
        printf("target abort...\n");

    /* Close PDMA0 */
    PDMA_Close(PDMA0);

    while(1);
}
