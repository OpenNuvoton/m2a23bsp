/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    LIN Power-down/Wake-up and header and response.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define LIN_ID      0x30

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile int32_t g_i32RxCounter;
static uint8_t g_u8ReceiveData[9];

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest(void);
int32_t LIN_Send_Wakeup_FunctionTest(void);
void LIN_PowerDown_Wakeup_FunctionTest(void);
void UART0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt);
void TestItem(void);
uint8_t GetParityValue(uint32_t u32id);


void UART0_IRQHandler(void)
{
    volatile uint32_t u32IntSts = UART0->INTSTS;
    uint32_t u32Data;

    if(u32IntSts & UART_INTSTS_WKINT_Msk) /* UART wake-up interrupt flag */
    {
        g_i32RxCounter = 0;
        UART_ClearIntFlag(UART0, UART_INTSTS_WKINT_Msk);
    }

    if(u32IntSts & UART_INTSTS_LININT_Msk) /* UART LIN interrupt flag */
    {
        if(UART0->LINSTS & UART_LINSTS_SLVHDETF_Msk)
        {
            /* Clear LIN slave header detection flag */
            UART0->LINSTS = UART_LINSTS_SLVHDETF_Msk;
            printf("\n LIN Slave Header detected ");
        }

        if(UART0->LINSTS & (UART_LINSTS_SLVHEF_Msk | UART_LINSTS_SLVIDPEF_Msk | UART_LINSTS_BITEF_Msk))
        {
            /* Clear LIN error flag */
            UART0->LINSTS = (UART_LINSTS_SLVHEF_Msk | UART_LINSTS_SLVIDPEF_Msk | UART_LINSTS_BITEF_Msk);
            printf("\n LIN error detected ");
        }
    }

    if(u32IntSts & UART_INTSTS_RDAINT_Msk) /* UART receive data available interrupt flag */
    {
        u32Data = UART0->DAT;
        g_u8ReceiveData[g_i32RxCounter] = (uint8_t)u32Data;
        g_i32RxCounter++;
    }

}

void SYS_Init(void)
{
    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

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

    /* Set PA multi-function pins for UART0 RXD(PA.6) and TXD(PA.7) */
    SET_UART0_RXD_PA6();
    SET_UART0_TXD_PA7();

    /*
       Set PA multi-function pins for UART1_RXD(PA.8)(debug port UART_RXD)
                                      UART1_TXD(PA.9)(debug port UART_TXD)
    */
    SET_UART1_RXD_PA8();
    SET_UART1_TXD_PA9();

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

    /* Init UART1. This sample code uses UART1 as debug port to demonstrate UART0 LIN function. */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART sample LIN wake-up function */
    UART_FunctionTest();

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute Parity Value                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t GetParityValue(uint32_t u32id)
{
    uint32_t u32Res = 0, ID[6], p_Bit[2], mask = 0;

    for(mask = 0; mask < 6; mask++)
        ID[mask] = (u32id & (1 << mask)) >> mask;

    p_Bit[0] = (ID[0] + ID[1] + ID[2] + ID[4]) % 2;
    p_Bit[1] = (!((ID[1] + ID[3] + ID[4] + ID[5]) % 2));

    u32Res = u32id + (p_Bit[0] << 6) + (p_Bit[1] << 7);
    return (uint8_t)u32Res;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt)
{
    uint32_t i, CheckSum = 0;

    for(i = 0 ; i < u32ByteCnt; i++)
    {
        CheckSum += pu8Buf[i];
        if(CheckSum >= 256)
            CheckSum -= 255;
    }
    return (uint8_t)(255 - CheckSum);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void TestItem(void)
{
    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  UART LIN Demo Function Select                                                 |\n");
    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  1 : Demo Power-Down/Wake-up and RX Function                                   |\n");
    printf("|  2 : Demo Send Wake-up signal and TX Function                                  |\n");
    printf("+--------------------------------------------------------------------------------+\n");
    printf("| Quit                                                                   - [ESC] |\n");
    printf("+--------------------------------------------------------------------------------+\n\n");
    printf("Please Select key (1~2): ");

    /* Forces a write of all user-space buffered data for the given output */
    fflush(stdout);
}

void UART_FunctionTest(void)
{
    uint32_t u32Item;

    do
    {
        TestItem();
        u32Item = (uint32_t)getchar();
        printf("%c\n", u32Item);

        switch(u32Item)
        {
            case '1':
                LIN_PowerDown_Wakeup_FunctionTest();
                break;

            case '2':
                LIN_Send_Wakeup_FunctionTest();
                break;

            default:
                break;
        }
    }
    while(u32Item != 27);

    printf("\nUART Demo Program End\n");

}

int32_t LIN_Send_Wakeup_FunctionTest(void)
{
    uint32_t u32TimeOutCnt;

    /*
        The sample code will send LIN wake-up signal, LIN header with ID is 0x30 and response field.
        The response field with 8 data bytes and checksum without including ID.
    */

    uint32_t u32WakeupPulse;
    uint8_t au8TestPattern[9] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x0}; /* 8 data byte + 1 byte checksum */

    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  UART LIN Send Wake-up Signal Function Test                                    |\n");
    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  Description :                                                                 |\n");
    printf("|    The sample code will send a Wake-up Signal with 250us,                      |\n");
    printf("|    then send a LIN header with ID is 0x30 and response field                   |\n");
    printf("+--------------------------------------------------------------------------------+\n");

    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLineConfig(UART0, 9600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* Switch back to LIN Function */
    UART0->FUNCSEL = UART_FUNCSEL_LIN;

    /* Set LIN Wake-up Signal length with 250us at HIRC clock source */
    u32WakeupPulse = ((__HIRC / 1000000) * 250);
    UART0->LINWKCTL = (UART0->LINWKCTL & ~UART_LINWKCTL_LINWKC_Msk) | u32WakeupPulse;

    /* Send Wake-up Signal */
    UART0->LINWKCTL |= UART_LINWKCTL_SENDLINW_Msk;

    /* Wait LIN send wake-up signal transfer completed */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(UART0->LINWKCTL & UART_LINWKCTL_SENDLINW_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for UART LIN send wake-up signal transfer completed time-out!\n");
            return -1;
        }
    }

    /* Wait slve is wakeup and initialized in 100ms */
    CLK_SysTickDelay(100000);

    /* Send Header */
    UART0->LINCTL = UART_LINCTL_PID(LIN_ID) | UART_LINCTL_HSEL_BREAK_SYNC_ID |
                    UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12) | UART_LINCTL_IDPEN_Msk;
    /* LIN TX Send Header Enable */
    UART0->LINCTL |= UART_LINCTL_SENDH_Msk;
    /* Wait until break field, sync field and ID field transfer completed */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((UART0->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for UART LIN header transfer completed time-out!\n");
            return -1;
        }
    }

    /* Compute checksum without ID and fill checksum value to au8TestPattern[8] */
    au8TestPattern[8] = ComputeChksumValue(&au8TestPattern[0], 8);
    UART_Write(UART0, &au8TestPattern[0], 9);

    printf("\n UART LIN Send Wake-up Signal Function Demo End !!\n");

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)
    if(--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

void LIN_PowerDown_Wakeup_FunctionTest(void)
{
    /*
        The sample code will wake-up by wake-up signal, detect LIN header(break+sync+ID)
        and receive response field.
        The response field with 8 data bytes and checksum without including ID.
    */

    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  UART LIN Power-down/Wake-up Function Test                                     |\n");
    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  Description :                                                                 |\n");
    printf("|    The sample code will enter Power-down mode, wake-up by wake-up signal,      |\n");
    printf("|    receive LIN header(break+sync+ID) and response field data.                  |\n");
    printf("+--------------------------------------------------------------------------------+\n");

    g_i32RxCounter = 0;

    /* reset RX FIFO */
    UART0->FIFO |= UART_FIFO_RXRST_Msk;

    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLineConfig(UART0, 9600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* Switch back to LIN Function */
    UART0->FUNCSEL = UART_FUNCSEL_LIN;

    /* Enable UART wake-up interrupt */
    UART_ENABLE_INT(UART0, UART_INTEN_WKIEN_Msk);
    NVIC_EnableIRQ(UART0_IRQn);

    /* Select UART0 clock source as LXT before entering Power-down mode. */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_LXT, CLK_CLKDIV0_UART0(1));

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* LIN Wake-up Enable to wailt wake-up signal from master */
    UART0->LINWKCTL |= UART_LINWKCTL_LINWKEN_Msk;

    /* Enter to Power-down mode */
    printf("Power-down\n");
    PowerDownFunction();

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    /* Disable UART wake-up interrupt */
    UART_DISABLE_INT(UART0, UART_INTEN_WKIEN_Msk);

    /* Select UART0 clock source back to HIRC after wake-up. */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    printf("wakeup\n");

    /* Enable UART RDA and LIN interrupt */
    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_LINIEN_Msk));

    /* Receive Header: break+sync+ID */
    UART0->LINCTL = UART_LINCTL_PID(LIN_ID) | UART_LINCTL_HSEL_BREAK_SYNC_ID | UART_LINCTL_SLVHDEN_Msk |
                    UART_LINCTL_IDPEN_Msk | UART_LINCTL_MUTE_Msk | UART_LINCTL_SLVEN_Msk;

    /* Wait for receiving data */
    while(g_i32RxCounter < 9);

    printf("\n Receive Data:\n [ 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x 0x%x ] \n",
           g_u8ReceiveData[0], g_u8ReceiveData[1], g_u8ReceiveData[2], g_u8ReceiveData[3],
           g_u8ReceiveData[4], g_u8ReceiveData[5], g_u8ReceiveData[6], g_u8ReceiveData[7],
           g_u8ReceiveData[8]);

    NVIC_DisableIRQ(UART0_IRQn);

    printf("\n UART LIN Power-down/Wake-up Function Demo End !!\n");
}
