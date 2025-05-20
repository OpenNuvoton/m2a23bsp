/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x32
 * @date     9, Nov, 2021
 *
 * @note
 * Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "targetdev.h"
#include "i2c_transfer.h"

#define PLL_CLOCK 200000000

uint32_t Pclk0;
uint32_t Pclk1;

void ProcessHardFault(void);
void SH_Return(void);
void SendChar_ToUART(void);
int32_t SYS_Init(void);

void ProcessHardFault(void) {}
void SH_Return(void) {}
void SendChar_ToUART(void) {}

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
        if(--u32TimeOutCnt == 0) return -1;

    /* Set HCLK clock source as HIRC first */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL clock as 144MHz from HIRC/2 */
    CLK->PLLCTL = CLK_PLLCTL_144MHz_HIRC_DIV2;

    /* Wait for PLL clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
    {
        if(--u32TimeOutCnt == 0)
            return -1;
    }

    /* Select HCLK clock source as PLL/2 and HCLK source divider as 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL_DIV2;

    /* Update System Core Clock */
    PllClock        = 144000000;
    SystemCoreClock = 144000000 / 2;
    CyclesPerUs     = SystemCoreClock / 1000000;  /* For CLK_SysTickDelay() */

    /* Enable UART0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Enable I2C0 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_I2C0CKEN_Msk;

    /* Select UART0 module clock source as HIRC */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_UART0SEL_Msk)) | CLK_CLKSEL2_UART0SEL_HIRC;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set I2C0 multi-function pins */
    SET_I2C0_SDA_PA4();
    SET_I2C0_SCL_PA5();

    /* I2C pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk;

    return 0;
}

int main(void)
{
    uint32_t cmd_buff[16];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    if( SYS_Init() < 0 )
        goto _APROM;

    /* Enable FMC ISP AP CFG function & clear ISPFF */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_CFGUEN_Msk | FMC_ISPCTL_ISPFF_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    I2C_Init();
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (1)
    {
        if (bI2cDataReady == 1)
        {
            goto _ISP;
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    while (1)
    {
        if (bI2cDataReady == 1)
        {
        	/* Disabled I2C IRQ until ParseCmd() is completed to prevent ISP from triggering an interrupt and fetching data prematurely. */
            NVIC_DisableIRQ(I2C0_IRQn);
            /* Get command from I2C receive buffer */
            memcpy(cmd_buff, i2c_rcvbuf, 64);
            bI2cDataReady = 0;
            /* Parse the current command */
            ParseCmd((unsigned char *)cmd_buff, 64);
			/* Restored I2C IRQ settings. */
            NVIC_EnableIRQ(I2C0_IRQn);
        }
    }

_APROM:
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}
