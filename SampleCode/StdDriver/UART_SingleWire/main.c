/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data by UART Single-Wire mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define BUFSIZE     128


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t g_u8RecData[BUFSIZE] = {0};
static uint8_t g_u8TxData [BUFSIZE] = {0};
static volatile uint32_t g_u32RecLen  =  0;
static volatile int32_t  g_i32RecOK  = FALSE;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART1_IRQHandler(void);
void UART1_TEST_HANDLE(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void Build_Src_Pattern(uint32_t u32Addr, uint8_t type, uint32_t u32Length);
uint8_t Check_Pattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length);
void SingleWireFunction_Test(void);
void SingleWireFunction_TxTest(void);
void SingleWireFunction_RxTest(void);



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
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART module clock source and UART module clock divider */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL2_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PA multi-function pins for UART1 RXD(PA.2) */
    SET_UART1_RXD_PA2();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
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

    /* Init UART1 */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART single wire sample function */
    SingleWireFunction_Test();

    printf("\nUART Sample Program End\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                       ISR to handle UART Channel 1 interrupt event                                      */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART1_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                  UART1 Callback function                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE(void)
{
    uint32_t u32temp;

    /* Handle Receive Data Available interrupt */
    if(UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAIF_Msk))
    {
        /* Get all the input characters */
        while(UART_IS_RX_READY(UART1))
        {
            /* Get the character from UART Buffer */
            u32temp = UART_READ(UART1);
            g_u8RecData[g_u32RecLen] = (uint8_t)u32temp;

            if(g_u32RecLen == BUFSIZE - 1)
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    /* Handle Single-wire Bit Error Detection interrupt */
    if(UART_GET_INT_FLAG(UART1, UART_INTSTS_SWBEIF_Msk))
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART1, UART_INTSTS_SWBEINT_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*                              Build Source Pattern function                                              */
/*---------------------------------------------------------------------------------------------------------*/
void Build_Src_Pattern(uint32_t u32Addr, uint8_t type, uint32_t u32Length)
{
    uint32_t i = 0, pattern = 0;
    uint8_t *pAddr;
    pAddr = (uint8_t *)u32Addr;

    if(type == 0)      pattern = 0x1f;
    else if(type == 1) pattern = 0x3f;
    else if(type == 2) pattern = 0x7f;
    else if(type == 3) pattern = 0xff;
    else  pattern = 0xff;

    for(i = 0; i < u32Length ; i++)
        pAddr[i] = (i & (uint8_t)pattern);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                    Verify that the received data is correct                                             */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t Check_Pattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length)
{
    uint32_t i = 0;
    uint8_t result = 1;
    uint8_t *pAddr0;
    uint8_t *pAddr1;
    pAddr0 = (uint8_t *)u32Addr0;
    pAddr1 = (uint8_t *)u32Addr1;

    for(i = 0; i < u32Length ; i++)
    {
        if(pAddr0[i] != pAddr1[i])
        {
            printf("Data Error Index=%d,tx =%d,rx=%d\n", i, pAddr0[i], pAddr1[i]) ;
            result = 0;
        }
    }

    return result;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Single Wire Function Test                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SingleWireFunction_Test(void)
{
    uint8_t u8Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_RXD(PA.2)  <==>  UART1_RXD(PA.2)--|Slave| |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Single Wire Function Test                             |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave. Master will send data. Slave will  |\n");
    printf("|    check if it receive correct data.                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    u8Item = (uint32_t)getchar();

    if(u8Item == '0')
        SingleWireFunction_TxTest();
    else
        SingleWireFunction_RxTest();
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Single Wire Function Tx Test                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void SingleWireFunction_TxTest(void)
{
    uint32_t u32TimeOutCnt;

    /* Set UART single wire function */
    UART_SelectSingleWireMode(UART1);

    /* Prepare data */
    Build_Src_Pattern((uint32_t)g_u8TxData, UART_WORD_LEN_8, BUFSIZE);

    /* Check the Rx status is Idle */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!UART_RX_IDLE(UART1))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for UART Rx idle time-out!\n");
            return;
        }
    }

    /* Send data */
    UART_Write(UART1, g_u8TxData, BUFSIZE);

    printf("\n Transmit Done\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Single Wire Function Rx Test                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void SingleWireFunction_RxTest(void)
{
    /* Set UART single wire function */
    UART_SelectSingleWireMode(UART1);

    /* Enable UART1 RDA/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_SWBEIEN_Msk));

    printf("\n Starting to receive data...\n");

    /* Wait for receive data */
    while(g_i32RecOK != TRUE);

    /* Check received data */
    Check_Pattern((uint32_t)g_u8TxData, (uint32_t)g_u8RecData, BUFSIZE) ? printf(" Pass\n") : printf(" Fail\n");

    /* Disable UART1 RDA/Single-wire Bit Error Detection interrupt */
    UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    NVIC_DisableIRQ(UART1_IRQn);
}
