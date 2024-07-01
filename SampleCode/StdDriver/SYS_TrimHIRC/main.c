/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use LXT to trim HIRC.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"




/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void TrimHIRC(void);
void CLKDIRC_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/*--------------------------------------------------------------------------------------------------------*/
/*  IRCTrim IRQ Handler                                                                                   */
/*--------------------------------------------------------------------------------------------------------*/
void IRC_IRQHandler(void)
{
    if(SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_TFAILIF_Msk)   /* Get Trim Failure Interrupt */
    {
        /* Display HIRC trim status */
        printf("HIRC Trim Failure Interrupt\n");
        /* Clear Trim Failure Interrupt */
        SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_TFAILIF_Msk;
    }
    if(SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_CLKERIF_Msk)   /* Get LXT Clock Error Interrupt */
    {
        /* Display HIRC trim status */
        printf("LXT Clock Error Interrupt\n");
        /* Clear LXT Clock Error Interrupt */
        SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_CLKERIF_Msk;
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

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPIOACKEN_Msk | CLK_AHBCLK_GPIOBCKEN_Msk | CLK_AHBCLK_GPIOCCKEN_Msk |
                   CLK_AHBCLK_GPIODCKEN_Msk | CLK_AHBCLK_GPIOFCKEN_Msk ;

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

    /* Set multi-function pin for CLKO(PB.14) */
    SET_CLKO_PB14();
}


void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


void TrimHIRC(void)
{
    uint32_t u32TimeOutCnt;

    /* Enable IRC Trim, set HIRC clock and enable interrupt */
    SYS->HIRCTRIMIEN |= (SYS_HIRCTRIMIEN_CLKEIEN_Msk | SYS_HIRCTRIMIEN_TFALIEN_Msk);
    SYS->HIRCTRIMCTL = (SYS->HIRCTRIMCTL & (~SYS_HIRCTRIMCTL_FREQSEL_Msk)) | 0x1;

    CLK_SysTickDelay(2000); /* Waiting for HIRC Frequency Lock */

    /* Get HIRC Frequency Lock */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while( (SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_FREQLOCK_Msk) == 0 )
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("HIRC Trim failed\n");
            return;
        }
    }

    printf("HIRC Frequency Lock\n");

    /* Clear Trim Lock */
    SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_FREQLOCK_Msk;

    /* Enable CLKO and output frequency = HIRC/4 */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HIRC, 1, 0);
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

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    /* Enable Interrupt */
    NVIC_EnableIRQ(IRC_IRQn);

    /* Trim HIRC to 48MHz */
    TrimHIRC();

    /* Disable IRC Trim */
    SYS->HIRCTRIMCTL = 0;
    printf("Disable IRC Trim\n");

    while(1);

}

