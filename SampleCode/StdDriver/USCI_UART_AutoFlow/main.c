/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data using auto flow control.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile int32_t g_i32Pointer = 0;
static uint8_t g_au8RecData[RXBUFSIZE] = {0};

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionTest(void);
void USCI_AutoFlow_FunctionTxTest(void);
void USCI_AutoFlow_FunctionRxTest(void);
void SYS_Init(void);
void UART0_Init(void);
void USCI0_Init(void);
void USCI0_IRQHandler(void);



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

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable USCI0 module clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Select UART module clock source and UART module clock divider */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PA multi-function pins for USCI0_DAT0(PA.10), USCI0_DAT1(PA.9), USCI0_CTL0(PB.0) and USCI0_CTL1(PA.8) */
    SET_USCI0_DAT0_PA10();
    SET_USCI0_DAT1_PA9();
    SET_USCI0_CTL0_PB0();
    SET_USCI0_CTL1_PA8();
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

void USCI0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS_ResetModule(USCI0_RST);

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
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

    /* Init USCI0 */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART auto flow sample function */
    USCI_AutoFlow_FunctionTest();

    printf("\nUSCI UART Sample Program End\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionTest(void)
{
    uint8_t u8Item;

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     Pin Configure                                           |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  ______                                              _____  |\n");
    printf("| |      |                                            |     | |\n");
    printf("| |Master|                                            |Slave| |\n");
    printf("| |    TX|--USCI0_DAT1(PA.9)  <==> USCI0_DAT0(PA.10)--|RX   | |\n");
    printf("| |  nCTS|--USCI0_CTL0(PB.0)  <==> USCI0_CTL1(PA.8)---|nRTS | |\n");
    printf("| |______|                                            |_____| |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n");

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test                                |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Description :                                              |\n");
    printf("|    The sample code needs two boards. One is Master and      |\n");
    printf("|    the other is slave. Master will send 1k bytes data       |\n");
    printf("|    to slave. Slave will check if received data is correct   |\n");
    printf("|  Please select Master or Slave test                         |\n");
    printf("|  [0] Master    [1] Slave                                    |\n");
    printf("+-------------------------------------------------------------+\n");
    u8Item = (uint8_t)getchar();

    if(u8Item == '0')
        USCI_AutoFlow_FunctionTxTest();
    else
        USCI_AutoFlow_FunctionRxTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Master)                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionTxTest(void)
{
    uint32_t u32Idx;

    /* Enable CTS and RTS autoflow control */
    UUART_EnableFlowCtrl(UUART0);

    /* Send 1k bytes data */
    for(u32Idx = 0; u32Idx < RXBUFSIZE; u32Idx++)
    {
        /* Send 1 byte data */
        UUART_WRITE(UUART0, (u32Idx & 0xFF));

        /* Wait if Tx FIFO is full */
        while(UUART_GET_TX_FULL(UUART0));
    }

    printf("\n Transmit Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Slave)                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionRxTest(void)
{
    uint32_t u32Idx, u32Err = 0;

    /* Enable CTS and RTS autoflow control */
    UUART_EnableFlowCtrl(UUART0);

    /* Enable USCI receive end and receive buffer over-run error Interrupt */
    UUART_EnableInt(UUART0, UUART_RXEND_INT_MASK | UUART_BUF_RXOV_INT_MASK);
    NVIC_EnableIRQ(USCI0_IRQn);

    printf("\n Starting to receive data...\n");

    /* Wait for receive 1k bytes data */
    while(g_i32Pointer < RXBUFSIZE);

    /* Compare Data */
    for(u32Idx = 0; u32Idx < RXBUFSIZE; u32Idx++)
    {
        if(g_au8RecData[u32Idx] != (u32Idx & 0xFF))
        {
            u32Err = 1;
            break;
        }
    }

    if( u32Err )
        printf("Compare Data Failed\n");
    else
        printf("\n Receive OK & Check OK\n");

    /* Disable USCI interrupt */
    UUART_DisableInt(UUART0, UUART_RXEND_INT_MASK | UUART_BUF_RXOV_INT_MASK);
    NVIC_DisableIRQ(USCI0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI interrupt event                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{

    volatile uint32_t u32ProtSts = UUART_GET_PROT_STATUS(UUART0);
    volatile uint32_t u32BufSts = UUART_GET_BUF_STATUS(UUART0);
    uint8_t u8InChar = 0xFF;

    if(u32ProtSts & UUART_PROTSTS_RXENDIF_Msk)      /* Receive end interrupt */
    {
        /* Handle received data */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);
        u8InChar = (uint8_t)UUART_READ(UUART0);
        g_au8RecData[g_i32Pointer++] = u8InChar;
    }
    else if(u32BufSts & UUART_BUFSTS_RXOVIF_Msk)      /* Receive buffer over-run error interrupt */
    {
        UUART_CLR_BUF_INT_FLAG(UUART0, UUART_BUFSTS_RXOVIF_Msk);
        printf("\nRx buffer is over-run.");
    }
}
