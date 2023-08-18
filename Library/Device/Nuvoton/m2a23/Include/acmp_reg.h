/**************************************************************************//**
 * @file     acmp_reg.h
 * @version  V1.00
 * @brief    ACMP register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __ACMP_REG_H__
#define __ACMP_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


 /******************************************************************************/
 /*                Device Specific Peripheral registers structures             */
 /******************************************************************************/

 /** @addtogroup REGISTER Control Register

   @{

 */


 /*---------------------- Analog Comparator Controller -------------------------*/
 /**
     @addtogroup ACMP Analog Comparator Controller(ACMP)
     Memory Mapped Structure for ACMP Controller
 @{ */

typedef struct
{


    /**
     * @var ACMP_T::CTL0
     * Offset: 0x00  Analog Comparator 0 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPEN    |Comparator Enable Bit
     * |        |          |0 = Comparator 0/2 Disabled.
     * |        |          |1 = Comparator 0/2 Enabled.
     * |[1]     |ACMPIE    |Comparator Interrupt Enable Bit
     * |        |          |0 = Comparator 0/2 interrupt Disabled.
     * |        |          |1 = Comparator 0/2 interrupt Enabled
     * |        |          |If WKEN (ACMP_CTL0[16]) is set to 1, the wake-up interrupt function will be enabled as well.
     * |[3]     |ACMPOINV  |Comparator Output Inverse
     * |        |          |0 = Comparator 0/2 output inverse Disabled.
     * |        |          |1 = Comparator 0/2 output inverse Enabled.
     * |[5:4]   |NEGSEL    |Comparator Negative Input Selection
     * |        |          |00 = ACMP0/2_N pin.
     * |        |          |01 = Internal comparator reference voltage (CRV0/2).
     * |        |          |10 = Band-gap voltage.
     * |        |          |11 = DAC0/2 output.
     * |        |          |Note: NEGSEL must select 2u2019b01 in calibration mode.
     * |[7:6]   |POSSEL    |Comparator Positive Input Selection
     * |        |          |00 = Input from ACMP0/2_P0.
     * |        |          |01 = Input from ACMP0/2_P1.
     * |        |          |10 = Input from ACMP0/2_P2.
     * |        |          |11 = Input from ACMP0/2_P3.
     * |[9:8]   |INTPOL    |Interrupt Condition Polarity Selection
     * |        |          |ACMPIF0 will be set to 1 when comparator output edge condition is detected.
     * |        |          |00 = Rising edge or falling edge.
     * |        |          |01 = Rising edge.
     * |        |          |10 = Falling edge.
     * |        |          |11 = Reserved.
     * |[12]    |OUTSEL    |Comparator Output Select
     * |        |          |0 = Comparator 0/2 output to ACMP0/2_O pin is unfiltered comparator output.
     * |        |          |1 = Comparator 0/2 output to ACMP0/2_O pin is from filter output.
     * |[15:13] |FILTSEL   |Comparator Output Filter Count Selection
     * |        |          |000 = Filter function is Disabled.
     * |        |          |001 = ACMP0/2 output is sampled 1 consecutive PCLK.
     * |        |          |010 = ACMP0/2 output is sampled 2 consecutive PCLKs.
     * |        |          |011 = ACMP0/2 output is sampled 4 consecutive PCLKs.
     * |        |          |100 = ACMP0/2 output is sampled 8 consecutive PCLKs.
     * |        |          |101 = ACMP0/2 output is sampled 16 consecutive PCLKs.
     * |        |          |110 = ACMP0/2 output is sampled 32 consecutive PCLKs.
     * |        |          |111 = ACMP0/2 output is sampled 64 consecutive PCLKs.
     * |[16]    |WKEN      |Power-down Wake-up Enable Bit
     * |        |          |0 = Wake-up function Disabled.
     * |        |          |1 = Wake-up function Enabled.
     * |[17]    |WLATEN    |Window Latch Mode Enable Bit
     * |        |          |0 = Window Latch Mode Disabled.
     * |        |          |1 = Window Latch Mode Enabled.
     * |[18]    |WCMPSEL   |Window Compare Mode Selection
     * |        |          |0 = Window Compare Mode Disabled.
     * |        |          |1 = Window Compare Mode is Selected.
     * |[21:20] |FCLKDIV   |Comparator Output Filter Clock Divider
     * |        |          |00 = Comparator output filter clock = PCLK.
     * |        |          |01 = Comparator output filter clock = PCLK/2.
     * |        |          |10 = Comparator output filter clock = PCLK/4.
     * |        |          |11 = Reserved.
     * |        |          |Note: Use FCLKDIV under the condition of FILTSEL = 3u2019h7, then set FCLKDIV to get the effect of filtering 128,256 consecutive PCLKs.
     * |[26:24] |HYSSEL    |Hysteresis Mode Selection
     * |        |          |000 = Hysteresis is 0mV.
     * |        |          |010 = Hysteresis is 20mV.
     * |        |          |100 = Hysteresis is 40mV.
     * |        |          |Others = Reserved.
     * @var ACMP_T::CTL1
     * Offset: 0x04  Analog Comparator 1 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPEN    |Comparator Enable Bit
     * |        |          |0 = Comparator 1 Disabled.
     * |        |          |1 = Comparator 1 Enabled.
     * |[1]     |ACMPIE    |Comparator Interrupt Enable Bit
     * |        |          |0 = Comparator 1/3 interrupt Disabled.
     * |        |          |1 = Comparator 1/3 interrupt Enabled
     * |        |          |If WKEN (ACMP_CTL1[16]) is set to 1, the wake-up interrupt function will be enabled as well.
     * |[3]     |ACMPOINV  |Comparator Output Inverse Control
     * |        |          |0 = Comparator 1/3 output inverse Disabled.
     * |        |          |1 = Comparator 1/3 output inverse Enabled.
     * |[5:4]   |NEGSEL    |Comparator Negative Input Selection
     * |        |          |00 = ACMP1/3_N pin.
     * |        |          |01 = Internal comparator reference voltage (CRV1/3).
     * |        |          |10 = Band-gap voltage.
     * |        |          |11 = DAC1/3 output.
     * |        |          |Note: NEGSEL must select 2u2019b01 in calibration mode.
     * |[7:6]   |POSSEL    |Comparator Positive Input Selection
     * |        |          |00 = Input from ACMP1/3_P0.
     * |        |          |01 = Input from ACMP1/3_P1.
     * |        |          |10 = Input from ACMP1/3_P2.
     * |        |          |11 = Input from ACMP1/3_P3.
     * |[9:8]   |INTPOL    |Interrupt Condition Polarity Selection
     * |        |          |ACMPIF1 will be set to 1 when comparator output edge condition is detected.
     * |        |          |00 = Rising edge or falling edge.
     * |        |          |01 = Rising edge.
     * |        |          |10 = Falling edge.
     * |        |          |11 = Reserved.
     * |[12]    |OUTSEL    |Comparator Output Select
     * |        |          |0 = Comparator 1/3 output to ACMP1/3_O pin is unfiltered comparator output.
     * |        |          |1 = Comparator 1/3 output to ACMP1/3_O pin is from filter output.
     * |[15:13] |FILTSEL   |Comparator Output Filter Count Selection
     * |        |          |000 = Filter function is Disabled.
     * |        |          |001 = ACMP1/3 output is sampled 1 consecutive PCLK.
     * |        |          |010 = ACMP1/3 output is sampled 2 consecutive PCLKs.
     * |        |          |011 = ACMP1/3 output is sampled 4 consecutive PCLKs.
     * |        |          |100 = ACMP1/3 output is sampled 8 consecutive PCLKs.
     * |        |          |101 = ACMP1/3 output is sampled 16 consecutive PCLKs.
     * |        |          |110 = ACMP1/3 output is sampled 32 consecutive PCLKs.
     * |        |          |111 = ACMP1/3 output is sampled 64 consecutive PCLKs.
     * |[16]    |WKEN      |Power-down Wakeup Enable Bit
     * |        |          |0 = Wake-up function Disabled.
     * |        |          |1 = Wake-up function Enabled.
     * |[17]    |WLATEN    |Window Latch Mode Enable Bit
     * |        |          |0 = Window Latch Mode Disabled.
     * |        |          |1 = Window Latch Mode Enabled.
     * |[18]    |WCMPSEL   |Window Compare Mode Selection
     * |        |          |0 = Window Compare Mode Disabled.
     * |        |          |1 = Window Compare Mode is Selected.
     * |[21:20] |FCLKDIV   |Comparator Output Filter Clock Divider
     * |        |          |00 = comparator output filter clock = PCLK.
     * |        |          |01 = comparator output filter clock = PCLK/2.
     * |        |          |10 = comparator output filter clock = PCLK/4.
     * |        |          |11 = Reserved.
     * |[26:24] |HYSSEL    |Hysteresis Mode Selection
     * |        |          |000 = Hysteresis is 0mV.
     * |        |          |010 = Hysteresis is 20mV.
     * |        |          |100 = Hysteresis is 40mV.
     * |        |          |Others = Reserved.
     * @var ACMP_T::STATUS
     * Offset: 0x08  Analog Comparator Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPIF0   |Comparator 0 Interrupt Flag
     * |        |          |This bit is set by hardware when the edge condition defined by INTPOL (ACMP_CTL0[9:8]) is detected on comparator 0/2 output
     * |        |          |This will generate an interrupt if ACMPIE (ACMP_CTL0[1]) is set to 1.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[1]     |ACMPIF1   |Comparator 1 Interrupt Flag
     * |        |          |This bit is set by hardware when the edge condition defined by INTPOL (ACMP_CTL1[9:8]) is detected on comparator 1/3 output
     * |        |          |This will cause an interrupt if ACMPIE (ACMP_CTL1[1]) is set to 1.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[4]     |ACMPO0    |Comparator 0 Output
     * |        |          |Synchronized to the PCLK to allow reading by software
     * |        |          |Cleared when the comparator 0/2 is disabled, i.e
     * |        |          |ACMPEN (ACMP_CTL0[0]) is cleared to 0.
     * |[5]     |ACMPO1    |Comparator 1 Output
     * |        |          |Synchronized to the PCLK to allow reading by software
     * |        |          |Cleared when the comparator 1/3 is disabled, i.e
     * |        |          |ACMPEN (ACMP_CTL1[0]) is cleared to 0.
     * |[8]     |WKIF0     |Comparator 0 Power-down Wake-up Interrupt Flag
     * |        |          |This bit will be set to 1 when ACMP0/2 wake-up interrupt event occurs.
     * |        |          |0 = No power-down wake-up occurred.
     * |        |          |1 = Power-down wake-up occurred.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[9]     |WKIF1     |Comparator 1 Power-down Wake-up Interrupt Flag
     * |        |          |This bit will be set to 1 when ACMP1/3 wake-up interrupt event occurs.
     * |        |          |0 = No power-down wake-up occurred.
     * |        |          |1 = Power-down wake-up occurred.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[12]    |ACMPS0    |Comparator 0 Status
     * |        |          |Synchronized to the PCLK to allow reading by software
     * |        |          |Cleared when the comparator 0/2 is disabled, i.e
     * |        |          |ACMPEN (ACMP_CTL0[0]) is cleared to 0.
     * |[13]    |ACMPS1    |Comparator 1 Status
     * |        |          |Synchronized to the PCLK to allow reading by software
     * |        |          |Cleared when the comparator 1/3 is disabled, i.e
     * |        |          |ACMPEN (ACMP_CTL1[0]) is cleared to 0.
     * |[16]    |ACMPWO    |Comparator Window Output
     * |        |          |This bit shows the output status of window compare mode
     * |        |          |0 = The positive input voltage is outside the window.
     * |        |          |1 = The positive input voltage is in the window.
     * @var ACMP_T::VREF
     * Offset: 0x0C  Analog Comparator Reference Voltage Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |CRV0SEL   |Comparator0 Reference Voltage Setting
     * |        |          |CRV0/2 = CRV0/2 source voltage * (ACMP_VREF[5:0] )/ 63.
     * |[6]     |CRV0SSEL  |CRV0 Source Voltage Selection
     * |        |          |0 = AVDD is selected as CRV0/2 source voltage.
     * |        |          |1 = The reference voltage defined by SYS_VREFCTL register is selected as CRV0/2 source voltage.
     * |[8]     |CRV0EN    |CRV0 Enable Bit
     * |        |          |0 = CRV0/2 Disabled.
     * |        |          |1 = CRV0/2 Enabled.
     * |[21:16] |CRV1SEL   |Comparator1 Reference Voltage Setting
     * |        |          |CRV1/3 = CRV1/3 source voltage * (ACMP_VREF[21:16] )/ 63.
     * |[22]    |CRV1SSEL  |CRV1 Source Voltage Selection
     * |        |          |0 = AVDD is selected as CRV1/3 source voltage.
     * |        |          |1 = The reference voltage defined by SYS_VREFCTL register is selected as CRV1/3 source voltage.
     * |[24]    |CRV1EN    |CRV1 Enable Bit
     * |        |          |0 = CRV1/3 Disabled.
     * |        |          |1 = CRV1/3 Enabled.
     * @var ACMP_T::CALCTL
     * Offset: 0x10  Analog Comparator Calibration Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CALTRG0   |Comparator0 Calibration Trigger Bit
     * |        |          |0 = Calibration is stopped.
     * |        |          |1 = Calibration is triggered.
     * |        |          |Note 1: Before this bit is enabled,ACMPEN(ACMP_CTL0[0]) should be set and the internal high speed RC oscillator (HIRC) should be enabled in advance.
     * |        |          |Note 2: Hardware will auto clear this bit when the next calibration is triggered by software.
     * |        |          |Note 3: If user must trigger calibration twice or more times, the second trigger has to wait at least 300us after the previous calibration is done.
     * |[1]     |CALTRG1   |Comparator1 Calibration Trigger Bit
     * |        |          |0 = Calibration is stopped.
     * |        |          |1 = Calibration is triggered.
     * |        |          |Note 1: Before this bit is enabled, ACMPEN(ACMP_CTL1[0]) should be set and the internal high speed RC oscillator (HIRC) should be enabled in advance.
     * |        |          |Note 2: Hardware will auto clear this bit when the next calibration is triggered by software.
     * |        |          |Note 3: If user must trigger calibration twice or more times, the second trigger has to wait at least 300us after the previous calibration is done.
     * @var ACMP_T::CALSTS
     * Offset: 0x14  Analog Comparator Calibration Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DONE0     |Comparator0 Calibration Done Status
     * |        |          |0 = Calibrating.
     * |        |          |1 = Calibration done.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[4]     |DONE1     |Comparator1 Calibration Done Status
     * |        |          |0 = Calibrating.
     * |        |          |1 = Calibration done.
     * |        |          |Note: Write 1 to clear this bit to 0.
     */
    __IO uint32_t CTL[2];                /*!< [0x0000-0x0004] Analog Comparator 0 Control Register                      */
    __IO uint32_t STATUS;                /*!< [0x0008] Analog Comparator Status Register                                */
    __IO uint32_t VREF;                  /*!< [0x000c] Analog Comparator Reference Voltage Control Register             */
    __IO uint32_t CALCTL;                /*!< [0x0010] Analog Comparator Calibration Control Register                   */
    __IO uint32_t CALSTS;                /*!< [0x0014] Analog Comparator Calibration Status Register                    */

} ACMP_T;

/**
    @addtogroup ACMP_CONST ACMP Bit Field Definition
    Constant Definitions for ACMP Controller
@{ */

#define ACMP_CTL_ACMPEN_Pos              (0)                                               /*!< ACMP_T::CTL: ACMPEN Position           */
#define ACMP_CTL_ACMPEN_Msk              (0x1ul << ACMP_CTL_ACMPEN_Pos)                    /*!< ACMP_T::CTL: ACMPEN Mask               */

#define ACMP_CTL_ACMPIE_Pos              (1)                                               /*!< ACMP_T::CTL0: ACMPIE Position          */
#define ACMP_CTL_ACMPIE_Msk              (0x1ul << ACMP_CTL_ACMPIE_Pos)                    /*!< ACMP_T::CTL0: ACMPIE Mask              */
                                                                                    
#define ACMP_CTL_ACMPOINV_Pos            (3)                                               /*!< ACMP_T::CTL0: ACMPOINV Position        */
#define ACMP_CTL_ACMPOINV_Msk            (0x1ul << ACMP_CTL_ACMPOINV_Pos)                  /*!< ACMP_T::CTL0: ACMPOINV Mask            */
                                                                                    
#define ACMP_CTL_NEGSEL_Pos              (4)                                               /*!< ACMP_T::CTL0: NEGSEL Position          */
#define ACMP_CTL_NEGSEL_Msk              (0x3ul << ACMP_CTL_NEGSEL_Pos)                    /*!< ACMP_T::CTL0: NEGSEL Mask              */
                                                                                    
#define ACMP_CTL_POSSEL_Pos              (6)                                               /*!< ACMP_T::CTL0: POSSEL Position          */
#define ACMP_CTL_POSSEL_Msk              (0x3ul << ACMP_CTL_POSSEL_Pos)                    /*!< ACMP_T::CTL0: POSSEL Mask              */
                                                                                    
#define ACMP_CTL_INTPOL_Pos              (8)                                               /*!< ACMP_T::CTL0: INTPOL Position          */
#define ACMP_CTL_INTPOL_Msk              (0x3ul << ACMP_CTL_INTPOL_Pos)                    /*!< ACMP_T::CTL0: INTPOL Mask              */
                                                                                    
#define ACMP_CTL_OUTSEL_Pos              (12)                                              /*!< ACMP_T::CTL0: OUTSEL Position          */
#define ACMP_CTL_OUTSEL_Msk              (0x1ul << ACMP_CTL_OUTSEL_Pos)                    /*!< ACMP_T::CTL0: OUTSEL Mask              */
                                                                                    
#define ACMP_CTL_FILTSEL_Pos             (13)                                              /*!< ACMP_T::CTL0: FILTSEL Position         */
#define ACMP_CTL_FILTSEL_Msk             (0x7ul << ACMP_CTL_FILTSEL_Pos)                   /*!< ACMP_T::CTL0: FILTSEL Mask             */
                                                                                    
#define ACMP_CTL_WKEN_Pos                (16)                                              /*!< ACMP_T::CTL0: WKEN Position            */
#define ACMP_CTL_WKEN_Msk                (0x1ul << ACMP_CTL_WKEN_Pos)                      /*!< ACMP_T::CTL0: WKEN Mask                */
                                                                                    
#define ACMP_CTL_WLATEN_Pos              (17)                                              /*!< ACMP_T::CTL0: WLATEN Position          */
#define ACMP_CTL_WLATEN_Msk              (0x1ul << ACMP_CTL_WLATEN_Pos)                    /*!< ACMP_T::CTL0: WLATEN Mask              */
                                                                                    
#define ACMP_CTL_WCMPSEL_Pos             (18)                                              /*!< ACMP_T::CTL0: WCMPSEL Position         */
#define ACMP_CTL_WCMPSEL_Msk             (0x1ul << ACMP_CTL_WCMPSEL_Pos)                   /*!< ACMP_T::CTL0: WCMPSEL Mask             */
                                                                                    
#define ACMP_CTL_FCLKDIV_Pos             (20)                                              /*!< ACMP_T::CTL0: FCLKDIV Position         */
#define ACMP_CTL_FCLKDIV_Msk             (0x3ul << ACMP_CTL_FCLKDIV_Pos)                   /*!< ACMP_T::CTL0: FCLKDIV Mask             */
                                                                                    
#define ACMP_CTL_HYSSEL_Pos              (24)                                              /*!< ACMP_T::CTL0: HYSSEL Position          */
#define ACMP_CTL_HYSSEL_Msk              (0x7ul << ACMP_CTL_HYSSEL_Pos)                    /*!< ACMP_T::CTL0: HYSSEL Mask              */

#define ACMP_STATUS_ACMPIF0_Pos          (0)                                               /*!< ACMP_T::STATUS: ACMPIF0 Position       */
#define ACMP_STATUS_ACMPIF0_Msk          (0x1ul << ACMP_STATUS_ACMPIF0_Pos)                /*!< ACMP_T::STATUS: ACMPIF0 Mask           */

#define ACMP_STATUS_ACMPIF1_Pos          (1)                                               /*!< ACMP_T::STATUS: ACMPIF1 Position       */
#define ACMP_STATUS_ACMPIF1_Msk          (0x1ul << ACMP_STATUS_ACMPIF1_Pos)                /*!< ACMP_T::STATUS: ACMPIF1 Mask           */

#define ACMP_STATUS_ACMPO0_Pos           (4)                                               /*!< ACMP_T::STATUS: ACMPO0 Position        */
#define ACMP_STATUS_ACMPO0_Msk           (0x1ul << ACMP_STATUS_ACMPO0_Pos)                 /*!< ACMP_T::STATUS: ACMPO0 Mask            */

#define ACMP_STATUS_ACMPO1_Pos           (5)                                               /*!< ACMP_T::STATUS: ACMPO1 Position        */
#define ACMP_STATUS_ACMPO1_Msk           (0x1ul << ACMP_STATUS_ACMPO1_Pos)                 /*!< ACMP_T::STATUS: ACMPO1 Mask            */

#define ACMP_STATUS_WKIF0_Pos            (8)                                               /*!< ACMP_T::STATUS: WKIF0 Position         */
#define ACMP_STATUS_WKIF0_Msk            (0x1ul << ACMP_STATUS_WKIF0_Pos)                  /*!< ACMP_T::STATUS: WKIF0 Mask             */

#define ACMP_STATUS_WKIF1_Pos            (9)                                               /*!< ACMP_T::STATUS: WKIF1 Position         */
#define ACMP_STATUS_WKIF1_Msk            (0x1ul << ACMP_STATUS_WKIF1_Pos)                  /*!< ACMP_T::STATUS: WKIF1 Mask             */

#define ACMP_STATUS_ACMPS0_Pos           (12)                                              /*!< ACMP_T::STATUS: ACMPS0 Position        */
#define ACMP_STATUS_ACMPS0_Msk           (0x1ul << ACMP_STATUS_ACMPS0_Pos)                 /*!< ACMP_T::STATUS: ACMPS0 Mask            */

#define ACMP_STATUS_ACMPS1_Pos           (13)                                              /*!< ACMP_T::STATUS: ACMPS1 Position        */
#define ACMP_STATUS_ACMPS1_Msk           (0x1ul << ACMP_STATUS_ACMPS1_Pos)                 /*!< ACMP_T::STATUS: ACMPS1 Mask            */

#define ACMP_STATUS_ACMPWO_Pos           (16)                                              /*!< ACMP_T::STATUS: ACMPWO Position        */
#define ACMP_STATUS_ACMPWO_Msk           (0x1ul << ACMP_STATUS_ACMPWO_Pos)                 /*!< ACMP_T::STATUS: ACMPWO Mask            */

#define ACMP_VREF_CRV0SEL_Pos            (0)                                               /*!< ACMP_T::VREF: CRV0SEL Position         */
#define ACMP_VREF_CRV0SEL_Msk            (0x3ful << ACMP_VREF_CRV0SEL_Pos)                 /*!< ACMP_T::VREF: CRV0SEL Mask             */

#define ACMP_VREF_CRV0SSEL_Pos           (6)                                               /*!< ACMP_T::VREF: CRV0SSEL Position        */
#define ACMP_VREF_CRV0SSEL_Msk           (0x1ul << ACMP_VREF_CRV0SSEL_Pos)                 /*!< ACMP_T::VREF: CRV0SSEL Mask            */

#define ACMP_VREF_CRV0EN_Pos             (8)                                               /*!< ACMP_T::VREF: CRV0EN Position          */
#define ACMP_VREF_CRV0EN_Msk             (0x1ul << ACMP_VREF_CRV0EN_Pos)                   /*!< ACMP_T::VREF: CRV0EN Mask              */

#define ACMP_VREF_CRV1SEL_Pos            (16)                                              /*!< ACMP_T::VREF: CRV1SEL Position         */
#define ACMP_VREF_CRV1SEL_Msk            (0x3ful << ACMP_VREF_CRV1SEL_Pos)                 /*!< ACMP_T::VREF: CRV1SEL Mask             */

#define ACMP_VREF_CRV1SSEL_Pos           (22)                                              /*!< ACMP_T::VREF: CRV1SSEL Position        */
#define ACMP_VREF_CRV1SSEL_Msk           (0x1ul << ACMP_VREF_CRV1SSEL_Pos)                 /*!< ACMP_T::VREF: CRV1SSEL Mask            */

#define ACMP_VREF_CRV1EN_Pos             (24)                                              /*!< ACMP_T::VREF: CRV1EN Position          */
#define ACMP_VREF_CRV1EN_Msk             (0x1ul << ACMP_VREF_CRV1EN_Pos)                   /*!< ACMP_T::VREF: CRV1EN Mask              */

#define ACMP_CALCTL_CALTRG0_Pos          (0)                                               /*!< ACMP_T::CALCTL: CALTRG0 Position       */
#define ACMP_CALCTL_CALTRG0_Msk          (0x1ul << ACMP_CALCTL_CALTRG0_Pos)                /*!< ACMP_T::CALCTL: CALTRG0 Mask           */

#define ACMP_CALCTL_CALTRG1_Pos          (1)                                               /*!< ACMP_T::CALCTL: CALTRG1 Position       */
#define ACMP_CALCTL_CALTRG1_Msk          (0x1ul << ACMP_CALCTL_CALTRG1_Pos)                /*!< ACMP_T::CALCTL: CALTRG1 Mask           */

#define ACMP_CALSTS_DONE0_Pos            (0)                                               /*!< ACMP_T::CALSTS: DONE0 Position         */
#define ACMP_CALSTS_DONE0_Msk            (0x1ul << ACMP_CALSTS_DONE0_Pos)                  /*!< ACMP_T::CALSTS: DONE0 Mask             */

#define ACMP_CALSTS_DONE1_Pos            (4)                                               /*!< ACMP_T::CALSTS: DONE1 Position         */
#define ACMP_CALSTS_DONE1_Msk            (0x1ul << ACMP_CALSTS_DONE1_Pos)                  /*!< ACMP_T::CALSTS: DONE1 Mask             */

/**@}*/ /* ACMP_CONST */
/**@}*/ /* end of ACMP register group */


/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif /* __ACMP_REG_H__ */
