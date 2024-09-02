/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Implement a multi-boot system to boot from different applications in APROM/LDROM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



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

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t ch;
    uint32_t u32Data;
    uint32_t u32Cfg;

    /* Unlock protected registers for ISP function */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0~0x1ff.
        NOTE: VECMAP only valid when CBS = 00'b or 10'b.

        To use this sample code, please:
        1. Build all targets and download to device individually.
           For Keil project, the targets are:
               FMC_MultiBoot, RO=0x0
               FMC_Boot0, RO=0x2000
               FMC_Boot1, RO=0x4000
               FMC_Boot2, RO=0x6000
               FMC_Boot3, RO=0x8000
               FMC_BootLD, RO=0x100000
           For IAR project, the targets are:
               FMC_MultiBoot, RO=0x0
               FMC_Boot0, RO=0x2000
               FMC_Boot1, RO=0x4000
               FMC_Boot2, RO=0x6000
               FMC_Boot3, RO=0x8000
           For GCC project, the targets are:
               FMC_MultiBoot, RO=0x0
               FMC_Boot1, RO=0x4000
               FMC_Boot3, RO=0x8000
        2. Reset MCU to execute FMC_MultiBoot.

    */

    printf("\n\n");
    printf("+----------------------------------------------+\n");
    printf("|     Multi-Boot Sample Code                   |\n");
    printf("+----------------------------------------------+\n");

    printf("\nCPU @ %dHz\n\n", SystemCoreClock);

    /* Enable FMC ISP function */
    FMC_ENABLE_ISP();

    printf("VECMAP = 0x%x\n", FMC_GetVECMAP());

    /* Check IAP mode */
    u32Cfg = FMC_Read(FMC_CONFIG_BASE);
    if((u32Cfg & 0xc0) != 0x80)
    {
        printf("Do you want to set to new IAP mode (APROM boot + LDROM)? (y/n)\n");
        if(getchar() == 'y')
        {
            FMC_ENABLE_CFG_UPDATE(); /* Enable user configuration update */

            /* Set CBS to b'10 */
            u32Cfg &= ~0xc0ul;
            u32Cfg |= 0x80;
            u32Data = FMC_Read(FMC_CONFIG_BASE + 0x4); /* Backup the data of config1 */
            FMC_Erase(FMC_CONFIG_BASE);
            FMC_Write(FMC_CONFIG_BASE, u32Cfg);
            FMC_Write(FMC_CONFIG_BASE + 0x4, u32Data);

            printf("Press any key to reset system to enable new IAP mode ...\n");
            getchar();
            SYS->IPRST0 = 0x1; /* Reset MCU */
            while(1);
        }
        else
        {
            printf("VECMAP only valid in new IAP mode. CBS = 10'b or 00'b\n");
            goto lexit;
        }
    }

    printf("Select one boot image: \n");
#if defined(__ARMCC_VERSION)
    printf("[0] Boot 0, base = 0x2000\n");
    printf("[1] Boot 1, base = 0x4000\n");
    printf("[2] Boot 2, base = 0x6000\n");
    printf("[3] Boot 3, base = 0x8000\n");
    printf("[4] Boot 4, base = 0x100000\n");
#elif defined(__ICCARM__)
    printf("[0] Boot 0, base = 0x2000\n");
    printf("[1] Boot 1, base = 0x4000\n");
    printf("[2] Boot 2, base = 0x6000\n");
    printf("[3] Boot 3, base = 0x8000\n");
#else
    printf("[1] Boot 1, base = 0x8000\n");
    printf("[3] Boot 3, base = 0x10000\n");
#endif
    printf("[Others] Boot, base = 0x0\n");

    ch = getchar();
    switch(ch)
    {
#if defined(__ARMCC_VERSION)
        case '0':
            FMC_SetVectorPageAddr(0x2000);
            break;
        case '1':
            FMC_SetVectorPageAddr(0x4000);
            break;
        case '2':
            FMC_SetVectorPageAddr(0x6000);
            break;
        case '3':
            FMC_SetVectorPageAddr(0x8000);
            break;
        case '4':
            FMC_SetVectorPageAddr(0x100000);
            break;
#elif defined(__ICCARM__)
        case '0':
            FMC_SetVectorPageAddr(0x2000);
            break;
        case '1':
            FMC_SetVectorPageAddr(0x4000);
            break;
        case '2':
            FMC_SetVectorPageAddr(0x6000);
            break;
        case '3':
            FMC_SetVectorPageAddr(0x8000);
            break;
#else
        case '1':
            FMC_SetVectorPageAddr(0x4000);
            break;
        case '3':
            FMC_SetVectorPageAddr(0x8000);
            break;
#endif
        default:
            FMC_SetVectorPageAddr(0x0);
            break;
    }

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_SetVectorPageAddr failed!\n");
        goto lexit;
    }

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();

    /* Reset System to reset to new vector page. */
    //NVIC_SystemReset();

lexit:

    while(1);
}
