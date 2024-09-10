/**************************************************************************//**
 * @file     APROM_main.c
 * @version  V3.00
 * @brief    FMC APROM IAP sample program run on APROM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"

typedef void (FUNC_PTR)(void);

extern uint32_t  loaderImage1Base, loaderImage1Limit;   /* symbol of image start and end */


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
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/**
  * @brief    Check User Configuration CONFIG0 bit 6 IAP boot setting. If it's not boot with IAP
  *           mode, modify it and execute a chip reset to make it take effect.
  * @return   Is boot with IAP mode or not.
  * @retval   0   Success.
  * @retval   -1  Failed on reading or programming User Configuration.
  */
static int SetIAPBoot(void)
{
    uint32_t  au32Config[2];           /* User Configuration */
    uint32_t u32CBS;

    /* Read current boot mode */
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;
    if(u32CBS & 1)
    {
        /* Modify User Configuration when it is not in IAP mode */

        if(FMC_ReadConfig(au32Config, 2) < 0)       /* Read User Configuration CONFIG0 and CONFIG1. */
        {
            printf("\nRead User Config failed!\n");
            return -1;                              /* Failed on reading User Configuration */
        }

        if(au32Config[0] & 0x40)                    /* Check if it's boot from APROM/LDROM with IAP. */
        {
            FMC_ENABLE_CFG_UPDATE();                /* Enable User Configuration update. */
            au32Config[0] &= ~0x40;                 /* Select IAP boot mode. */
            FMC_Erase(FMC_CONFIG_BASE);
            if(FMC_WriteConfig(au32Config, 2) != 0)  /* Update User Configuration CONFIG0 and CONFIG1. */
            {
                printf("FMC_WriteConfig failed!\n");
                return -1;
            }

            /* Perform chip reset to make new User Config take effect */
            SYS_ResetChip();
        }
    }
    return 0;                                       /* success */
}

/**
  * @brief    Load an image to specified flash address. The flash area must have been enabled by
  *           caller. For example, if caller want to program an image to LDROM, FMC_ENABLE_LD_UPDATE()
  *           must be called prior to calling this function.
  * @return   Image is successfully programmed or not.
  * @retval   0   Success.
  * @retval   -1  Program/verify failed.
  */
static int  LoadImage(uint32_t u32ImageBase, uint32_t u32ImageLimit, uint32_t u32FlashAddr, uint32_t u32MaxSize)
{
    uint32_t   i, j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = u32MaxSize; /* Give the maximum size of programmable flash area. */

    printf("Program image to flash address 0x%x...", u32FlashAddr); /* information message */

    /*
     * program the whole image to specified flash area
     */
    pu32Loader = (uint32_t *)u32ImageBase;
    for(i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        FMC_Erase(u32FlashAddr + i);     /* erase a flash page */

        for(j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)                  /* program image to this flash page */
        {
            FMC_Write(u32FlashAddr + i + j, pu32Loader[(i + j) / 4]);
        }
    }
    printf("OK.\nVerify ...");

    /* Verify loader */
    for(i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        for(j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            u32Data = FMC_Read(u32FlashAddr + i + j);        /* read a word from flash memory */

            if(u32Data != pu32Loader[(i + j) / 4])         /* check if the word read from flash be matched with original image */
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", u32FlashAddr + i + j, u32Data, pu32Loader[(i + j) / 4]);
                return -1;             /* image program failed */
            }

            if(i + j >= u32ImageSize)  /* check if it reach the end of image */
                break;
        }
    }
    printf("OK.\n");
    return 0;                          /* success */
}


int32_t main(void)
{
    uint8_t     u8Item;                /* menu item */
    uint32_t    u32Data;               /* temporary data word */
    FUNC_PTR    *func;                 /* function pointer */
    char *acBootMode[] = {"LDROM+IAP", "LDROM", "APROM+IAP", "APROM"};
    uint32_t u32CBS;
    uint32_t    u32TimeOutCnt;         /* time-out counter */

    SYS_UnlockReg();                   /* Unlock register lock protect */

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|          FMC IAP Sample Code           |\n");
    printf("|             [APROM code]               |\n");
    printf("+----------------------------------------+\n");

    FMC_Open();                        /* Enable FMC ISP function */

    /*
     *  Check if User Configuration CBS is boot with IAP mode.
     *  If not, modify it.
     */
    if(SetIAPBoot() < 0)
    {
        printf("Failed to set IAP boot mode!\n");
        goto lexit; /* Failed to set IAP boot mode. Program aborted. */
    }

    /* Get boot mode */
    printf("  Boot Mode ............................. ");
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;
    printf("[%s]\n", acBootMode[u32CBS]);


    u32Data = FMC_ReadCID();           /* get company ID */
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE + 4));

    do
    {
        printf("\n\n\n");
        printf("+----------------------------------------+\n");
        printf("|               Select                   |\n");
        printf("+----------------------------------------+\n");
        printf("| [0] Load IAP code to LDROM             |\n");
        printf("| [1] Run IAP program (in LDROM)         |\n");
        printf("+----------------------------------------+\n");
        printf("Please select...");
        fflush(stdout);                /* Forces a write of all user-space buffered data for the given output */
        u8Item = getchar();            /* block waiting to receive any one character from UART0 */
        printf("%c\n", u8Item);        /* print out the selected item */

        switch(u8Item)
        {
            case '0':
                FMC_ENABLE_LD_UPDATE();    /* Enable LDROM update capability */
                /*
                 *  The binary image of LDROM code is embedded in this sample.
                 *  LoadImage() will program this LDROM code to LDROM.
                 */
                if(LoadImage((uint32_t)&loaderImage1Base, (uint32_t)&loaderImage1Limit,
                             FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
                {
                    printf("Load image to LDROM failed!\n");
                    goto lexit;            /* Load LDROM code failed. Program aborted. */
                }
                FMC_DISABLE_LD_UPDATE();   /* Disable LDROM update capability */
                break;

            case '1':
                printf("\n\nChange VECMAP and branch to LDROM...\n");
                printf("LDROM code SP = 0x%x\n", *(uint32_t *)(FMC_LDROM_BASE));
                printf("LDROM code ResetHandler = 0x%x\n", *(uint32_t *)(FMC_LDROM_BASE + 4));
                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
                UART_WAIT_TX_EMPTY(UART0)  /* To make sure all message has been print out */
                if(--u32TimeOutCnt == 0) break;
                /*  NOTE!
                 *     Before change VECMAP, user MUST disable all interrupts.
                 *     The following code CANNOT locate in address 0x0 ~ 0x200.
                 */
                FMC_SetVectorPageAddr(FMC_LDROM_BASE);

                /*
                 *  The reset handler address of an executable image is located at offset 0x4.
                 *  Thus, this sample get reset handler address of LDROM code from FMC_LDROM_BASE + 0x4.
                 */
                func = (FUNC_PTR *) * (uint32_t *)(FMC_LDROM_BASE + 4);
                /*
                 *  The stack base address of an executable image is located at offset 0x0.
                 *  Thus, this sample get stack base address of LDROM code from FMC_LDROM_BASE + 0x0.
                 */

                __set_MSP(M32(FMC_LDROM_BASE));

                /*
                 *  Branch to the LDROM code's reset handler in way of function call.
                 */
                func();
                break;

            default :
                continue;                  /* invalid selection */
        }
    }
    while(1);


lexit:                                 /* program exit */

    FMC_Close();                       /* Disable FMC ISP function */

    SYS_LockReg();                     /* Lock protected registers */

    printf("\nFMC Sample Code Completed.\n");

    while(1);
}
