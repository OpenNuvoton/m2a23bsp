/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Bank Remap sample code(Bank0 Loader).
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"


static int SetIAPBoot(void);
void SYS_Init(void);
int main(void);


static int SetIAPBoot(void)
{
    uint32_t  au32Config[2];
    uint32_t u32CBS;

    /* Read current boot mode */
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;
    if(u32CBS & 1)
    {
        /* Modify User Configuration when it is not in IAP mode */

        FMC_ReadConfig(au32Config, 2);
        if(au32Config[0] & 0x40)
        {
            FMC_ENABLE_CFG_UPDATE();
            au32Config[0] &= ~0x40;
            FMC_Erase(FMC_CONFIG_BASE);
            FMC_WriteConfig(au32Config, 2);

            /* Perform chip reset to make new User Config take effect */
            SYS_ResetChip();
        }
    }
    return 0;
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}



int main(void)
{
    uint8_t u8GetCh;
    uint32_t u32ExecBank, i;
    uint32_t u32Loader0ChkSum, u32Loader1ChkSum;
    uint32_t u32App0ChkSum, u32App1ChkSum;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    if(SetIAPBoot() < 0)
    {
        printf("Failed to set IAP boot mode!\n");
        goto lexit;
    }

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
        printf("\n BANK%d Loader processing \n\n", u32ExecBank);

        u32Loader0ChkSum = FMC_GetChkSum(FMC_APROM_BASE, LOADER_SIZE);
        u32Loader1ChkSum = FMC_GetChkSum(FMC_APROM_BANK0_END, LOADER_SIZE);
        printf(" Loader0 checksum: 0x%08x \n Loader1 checksum: 0x%08x\n", u32Loader0ChkSum, u32Loader1ChkSum);

        u32App0ChkSum = FMC_GetChkSum(FMC_APROM_BASE + APP_BASE, APP_SIZE);
        u32App1ChkSum = FMC_GetChkSum(FMC_APROM_BANK0_END + APP_BASE, APP_SIZE);
        printf(" App0 checksum: 0x%08x \n App1 checksum: 0x%08x\n", u32App0ChkSum, u32App1ChkSum);

        if((u32ExecBank == 0) && (u32Loader0ChkSum != u32Loader1ChkSum))
        {
            printf("\n Create BANK%d Loader... \n",  u32ExecBank ^ 1);

            /* Erase loader region */
            for(i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_BANK_SIZE * (u32ExecBank ^ 1) + i);
            }
            /* Create loader in the other bank */
            for(i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += 4)
            {
                FMC_Write(FMC_BANK_SIZE * (u32ExecBank ^ 1) + i, FMC_Read((FMC_BANK_SIZE * u32ExecBank) + i));
            }
            printf(" Create Bank%d Loader completed! \n", (u32ExecBank ^ 1));
        }

        if((u32ExecBank == 0) && ((FMC_CheckAllOne((FMC_APROM_BANK0_END + APP_BASE), APP_SIZE)) == READ_ALLONE_YES))
        {
            printf("\n Create BANK%d App... \n", u32ExecBank ^ 1);

            /* Erase app region */
            for(i = APP_BASE; i < (APP_BASE + APP_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_BANK_SIZE * (u32ExecBank ^ 1) + i);
            }
            /* Create app in the other bank(just for test)*/
            for(i = APP_BASE; i < (APP_BASE + APP_SIZE); i += 4)
            {
                FMC_Write(FMC_BANK_SIZE * (u32ExecBank ^ 1) + i, FMC_Read((FMC_BANK_SIZE * u32ExecBank) + i));
            }
            printf(" Create Bank%d App completed! \n", (u32ExecBank ^ 1));
        }

        printf("\n Execute BANK%d APP? [y/n] \n", u32ExecBank);
        u8GetCh = (uint8_t)getchar();

        if(u8GetCh == 'y')
        {
            /* Remap to App */
            FMC_SetVectorPageAddr(APP_BASE);
            SYS_ResetCPU();
        }
        else
        {
            printf("\n Remap to BANK%d Loader? [y/n] \n", u32ExecBank ^ 1);
            u8GetCh = (uint8_t)getchar();

            if(u8GetCh == 'y')
            {
                /* Remap Bank */
                printf("\n BANK%d Loader before remap \n", u32ExecBank);
                FMC_RemapBank(u32ExecBank ^ 1);
                printf("\n Remap completed!\n");
            }
            else
            {
                printf("\n Continue to execute BANK%d Loader? \n ", u32ExecBank);
            }
        }

    }
    while(1);

lexit:

    while(1);

}
