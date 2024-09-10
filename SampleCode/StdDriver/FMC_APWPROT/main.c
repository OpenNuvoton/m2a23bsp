/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use FMC APROM Protect function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


extern char GetChar(void);
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
    uint32_t i, j, u32ISPReadData, u32CPUReadData, u32Start, u32End, u32ISPFF;
    int32_t i32ISPSTS = 0, i32GetInt = 0;
    uint8_t u8GetCh = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+----------------------------------------+\n");
    printf("|     FMC APROM Protect Sample Demo      |\n");
    printf("+----------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC_Open();

    /* Enable FMC APROM update function */
    FMC_ENABLE_AP_UPDATE();

    /* Set APROM testing range */
    u32Start = FMC_APROM_BASE + FMC_APPROT_BLOCK_SIZE * 4;
    u32End   = FMC_APROM_BASE + FMC_APPROT_BLOCK_SIZE * 8;

    /* Clear all APROM protect settings */
    for(i = 0; i < 8; i++)
    {
        FMC_DISABLE_APPROT(i);
    }

    /* Erase testing APROM range */
    printf("\nErase APROM....\n");

    for(i = u32Start; i < u32End; i += FMC_FLASH_PAGE_SIZE)
    {
        printf("Erase page: 0x%08x \r", i);
        i32ISPSTS = FMC_Erase(i);

        if(i32ISPSTS)
        {
            printf("[Erase FAIL at addr 0x%x \n", i);
            goto lexit;
        }
    }


    /* Verify testing APROM range after erase done */
    for(i = u32Start; i < u32End; i += 4)
    {
        u32ISPReadData = FMC_Read(i);

        u32ISPFF = FMC_GET_FAIL_FLAG();
        if(u32ISPFF)
        {
            printf("line(%d)[FAIL]ISP Fail Flag[0x%x] \n",  __LINE__, u32ISPFF);
            goto lexit;
        }

        u32CPUReadData = M32(i);

        /* Verify testing APROM range, ISP read should be 0xFFFFFFFF */
        if(u32ISPReadData != 0xFFFFFFFF)
        {
            printf("line[%d][ISP verify 0xFFFFFFFF FAIL]addr[0x%x] data[0x%x]\n", __LINE__, i, u32ISPReadData);
            goto lexit;
        }

        /* Verify testing APROM range, CPU read should be 0xFFFFFFFF */
        if(u32CPUReadData != 0xFFFFFFFF)
        {
            printf("line[%d][CPU verify 0xFFFFFFFF FAIL]addr[0x%x] data[0x%x]\n",  __LINE__, i, u32CPUReadData);
            goto lexit;
        }
    }

    /* Sequential program on testing APROM block */
    printf("\nProgram APROM....\n");

    for(i = u32Start; i < u32End; i += 4)
    {
        if(i % FMC_FLASH_PAGE_SIZE == 0)
            printf("Program page: 0x%08x \r", i);

        /* Program testing APROM range with its address */
        FMC_Write(i, i);
        u32ISPReadData = FMC_Read(i);

        u32ISPFF = FMC_GET_FAIL_FLAG();

        if(u32ISPFF)
        {
            printf("line(%d)[FAIL]ISP Fail Flag[0x%x] \n",  __LINE__, u32ISPFF);
            goto lexit;
        }

        u32CPUReadData = M32(i);

        /* Verify testing APROM range, ISP read should be its address */
        if(u32ISPReadData != i)
        {
            printf("line[%d][ISP verify FAIL]addr[0x%x] data[0x%x]\n", __LINE__, i, u32ISPReadData);
            goto lexit;
        }

        /* Verify testing APROM range, CPU read should be its address */
        if(u32CPUReadData != i)
        {
            printf("line[%d][CPU verify FAIL]addr[0x%x] data[0x%x]\n", __LINE__,  i, u32CPUReadData);
            goto lexit;
        }

    }
    printf("\nProgram done.\n");

    /* Set APROM Protect block */
    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|            APROM Protect Test          |\n");
    printf("+----------------------------------------+\n");

    printf("+----------------------------------------+\n");
    printf("|       Dump APROM Protect Settings      |\n");
    printf("+----------------------------------------+\n");
    for(i = 0; i < 8; i++)
    {
        printf("APPROEN%02d  [%s]  address is 0x%08x ~ 0x%08x\n", i, (FMC->APWPROT0 & (1 << i)) ? "ENABLE " : "DISABLE", (uint32_t)(i*FMC_APPROT_BLOCK_SIZE), (uint32_t)((i + 1)*FMC_APPROT_BLOCK_SIZE - 1));
    }

    printf("+----------------------------------------+\n");
    printf("| Select protected APROM Block           |\n");
    printf("+----------------------------------------+\n");

    while(1)
    {

        /* User select the protect range: block 4 ~ 7 */
        printf("\nPlease input number 4 ~ 7 and press enter:\n");
        i32GetInt = GetChar() - '0';
        //scanf("%d", &i32GetInt);
        printf("\nSelect APPROEN%d\n\n", i32GetInt);

        /* Set the protect address by the selected block */
        u32Start = i32GetInt*FMC_APPROT_BLOCK_SIZE;
        u32End   = (i32GetInt + 1) * FMC_APPROT_BLOCK_SIZE;

        if(i32GetInt < 4)
        {
            /* In this sample code, skip the block 0~3 testing because it's the code execution region */
            /* if user wants to test block 0 and 1, please let code execute in SRAM */
            printf("\tAPROM 0x%08x ~ 0x%08x is code execution region!\n\n", u32Start, u32End);
        }
        else if(i32GetInt > 7)
        {
            /* Skip if the selected range exceeds APROM region */
            printf("\tAddress 0x%08x ~ 0x%08x exceeds test APROM range!\n\n", u32Start, u32End);
        }
        else
        {
            /* Enable APROM protected function for selected region */
            printf("\tAPROM protect ENABLE from APROM 0x%08x ~ 0x%08x\n\n", u32Start, u32End);
            FMC_ENABLE_APPROT(i32GetInt);
        }

        /* User can select to continue to set other APROM protect region, or end the setting and start the protect function testing */
        printf("Press 'e' to exit APROM protect setting, others to continue setting other range.\n");
        u8GetCh = GetChar();

        if(u8GetCh == 'e')
            break;

    }

    printf("+----------------------------------------+\n");
    printf("|       Dump APROM Protect Settings      |\n");
    printf("+----------------------------------------+\n");
    for(i = 4; i < 8; i++)
    {
        printf("APPROEN%02d  [%s]  address is 0x%08x ~ 0x%08x\n", i, (FMC->APWPROT0 & (1 << i)) ? " ENABLE" : "DISABLE", (uint32_t)(i*FMC_APPROT_BLOCK_SIZE), (uint32_t)((i + 1)*FMC_APPROT_BLOCK_SIZE - 1));
    }

    for(i = 4; i < 8; i++)
    {
        if(FMC->APWPROT0 & (1 << i))
        {

            /* Set APROM protected address by selected region */
            u32Start = i*FMC_APPROT_BLOCK_SIZE;
            u32End   = (i + 1) * FMC_APPROT_BLOCK_SIZE;

            printf("\n\nAPPROEN%02d Testing, address is 0x%08x ~ 0x%08x \n", i, (uint32_t)(i*FMC_APPROT_BLOCK_SIZE), (uint32_t)((i + 1)*FMC_APPROT_BLOCK_SIZE - 1));

            for(j = u32Start; j < u32End; j += 4)
            {
                u32ISPReadData = FMC_Read(j);
                u32ISPFF = FMC_GET_FAIL_FLAG();
                if(u32ISPFF)
                {
                    printf("line(%d)[FAIL]ISP Fail Flag[0x%x] \n",  __LINE__, u32ISPFF);
                    goto lexit;
                }

                u32CPUReadData = M32(j);

                /* Verify testing APROM range, ISP read should be its address */
                if(u32ISPReadData != j)
                {
                    printf("line[%d][ISP verify protect data FAIL]addr[0x%x] data[0x%x]\n", __LINE__, j, u32ISPReadData);
                    goto lexit;
                }

                /* Verify testing APROM range, CPU read should be its address */
                if(u32CPUReadData != j)
                {
                    printf("line[%d][CPU verify protect data FAIL]addr[0x%x] data[0x%x]\n",  __LINE__, j, u32CPUReadData);
                    goto lexit;
                }
            }

            /* After APROM protect enable, do erase testing */
            printf("\n\t   ENABLE APROM Protect, erase testing start(should be erase FAIL!).....\n");
            for(j = u32Start; j < u32End; j += FMC_FLASH_PAGE_SIZE)
            {
                i32ISPSTS = FMC_Erase(j);

                if(i32ISPSTS == 0)
                {
                    printf("line(%d)[FAIL]ISP erase should be fail, i32ISPSTS = [0x%x] \n", __LINE__, i32ISPSTS);
                    goto lexit;
                }
            }

            /* After APROM protect enable, verify APROM after erase testing */
            printf("\n\t   Verify after erase APROM when ENABLE APROM Protect(it should not be erased!).....\n");
            for(j = u32Start; j < u32End; j += 4)
            {
                u32ISPReadData = FMC_Read(j);
                u32ISPFF = FMC_GET_FAIL_FLAG();
                if(u32ISPFF)
                {
                    printf("line(%d)[FAIL]ISP Fail Flag[0x%x] \n",  __LINE__, u32ISPFF);
                    goto lexit;
                }

                u32CPUReadData = M32(j);

                /* Verify testing APROM range, ISP read should be its address */
                if(u32ISPReadData != j)
                {
                    printf("line[%d][ISP verify protect data FAIL]addr[0x%x] data[0x%x]\n", __LINE__, j, u32ISPReadData);
                    goto lexit;
                }

                /* Verify testing APROM range, CPU read should be its address */
                if(u32CPUReadData != j)
                {
                    printf("line[%d][CPU verify protect data FAIL]addr[0x%x] data[0x%x]\n",  __LINE__, j, u32CPUReadData);
                    goto lexit;
                }
            }
            printf("\n\t=> Erase check OK! All data is not erased\n");

            printf("\n\t   When ENABLE APROM Protect, program APROM test(it should not be programed!).....\n");

            for(j = u32Start; j < u32End; j += 4)
            {
                /* Program 0 to protected region */
                FMC_Write(j, 0);

                u32ISPFF = FMC_GET_FAIL_FLAG();
                if(u32ISPFF)
                {
                    FMC_CLR_FAIL_FLAG();
                }
                else
                {
                    printf("line(%d)[FAIL]ISP Fail Flag should be set \n", __LINE__);
                    goto lexit;
                }

                u32ISPReadData = FMC_Read(j);
                u32ISPFF = FMC_GET_FAIL_FLAG();
                if(u32ISPFF)
                {
                    printf("line(%d)[FAIL]ISP Fail Flag[0x%x] \n",  __LINE__, u32ISPFF);
                    goto lexit;
                }

                u32CPUReadData = M32(j);

                /* Verify testing APROM range, ISP read should be its address, not 0 */
                if(u32ISPReadData != j)
                {
                    printf("line[%d][ISP verify fail]addr[0x%x] data[0x%x]\n", __LINE__, j, u32ISPReadData);
                    goto lexit;
                }

                /* Verify testing APROM range, CPU read should be its address,  not 0 */
                if(u32CPUReadData != j)
                {
                    printf("line[%d][CPU verify fail]addr[0x%x] data[0x%x]\n", __LINE__,  j, u32CPUReadData);
                    goto lexit;
                }

            }
            printf("\n\t=> Program check OK! All data is not programed\n");
        }
    }


    printf("\nFMC APPROT test PASS.\n");

lexit:

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    while(1);

}
