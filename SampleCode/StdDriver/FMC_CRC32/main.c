/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use FMC CRC32 ISP command to calculate the
 *           CRC32 checksum of APROM, LDROM, and SPROM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void SYS_Init(void);
void UART0_Init(void);


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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
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
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32Data, u32ChkSum;       /* temporary data */

    SYS_UnlockReg();                   /* Unlock protected registers */

    SYS_Init();                        /* Init System, peripheral clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    printf("+------------------------------------+\n");
    printf("|       FMC CRC32 Sample Demo        |\n");
    printf("+------------------------------------+\n");

    FMC_Open();                        /* Enable FMC ISP function */

    u32Data = FMC_ReadCID();           /* Read company ID. Should be 0x000000DA. */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }

    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }

    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+4) failed!\n");
        goto lexit;
    }

    /* Read Data Flash base address */
    printf("  Data Flash Base Address ............... [0x%08x]\n", FMC_ReadDataFlashBaseAddr());

    printf("\nLDROM (0x100000 ~ 0x101000) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 calculation on LDROM.
     */
    u32ChkSum = FMC_GetChkSum(FMC_LDROM_BASE, FMC_LDROM_SIZE);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating LDROM CRC32 checksum!\n");
        goto lexit;                    /* failed */
    }
    printf("0x%x\n", u32ChkSum);       /* print out LDROM CRC32 check sum value */

    printf("\nSPROM (0x200000 ~ 0x200800) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 calculation on SPROM.
     */
    u32ChkSum = FMC_GetChkSum(FMC_SPROM_BASE, FMC_SPROM_SIZE);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating SPROM CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out SPROM CRC32 check sum value */
    
    printf("\nAPROM(256KB) bank0 (0x0 ~ 0x20000) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 calculation on APROM(256KB) bank 0.
     *  Note that FMC CRC32 checksum calculation area must not cross bank boundary.
     */
    u32ChkSum = FMC_GetChkSum(FMC_APROM_BASE, FMC_BANK_SIZE);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM bank0 CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */

    /*
     *  Request FMC hardware to run CRC32 calculation on APROM(256KB) bank 1.
     */
    printf("\nAPROM bank1 (0x20000 ~ 0x40000) CRC32 checksum =>  ");
    u32ChkSum = FMC_GetChkSum(FMC_APROM_BASE+FMC_BANK_SIZE, FMC_BANK_SIZE);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM(256KB) bank1 CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */

    printf("\nFMC CRC32 checksum test done.\n");

lexit:
    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while (1);
}
