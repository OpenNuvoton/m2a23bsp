/**************************************************************************//**
 * @file     spdh_reg.h
 * @version  V3.00
 * @brief    SPDH register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SPDH_REG_H__
#define __SPDH_REG_H__

 /******************************************************************************/
 /*                Device Specific Peripheral registers structures             */
 /******************************************************************************/

 /** @addtogroup REGISTER Control Register

   @{

 */

/*---------------------- SPDH Controller -------------------------*/
/**
    @addtogroup SPD Hub Controller(SPDH)
    Memory Mapped Structure for SPDH Controller
    @{
*/

typedef struct
{
    __IO uint32_t CTL;                   /*!< [0x0000] SPD5 HUB Control Register                                   */
    __IO uint32_t HID;                   /*!< [0x0004] SPD5 HUB HID Register                                       */
    __IO uint32_t BUSRST;                /*!< [0x0008] SPD5 HUB Bus Reset Register                                 */
    __I  uint32_t STS;                   /*!< [0x000c] SPD5 HUB Status Register                                    */
    __IO uint32_t DCTL;                  /*!< [0x0010] SPD5 HUB Local Device Control Register                      */
    __I  uint32_t DSTS;                  /*!< [0x0014] SPD5 HUB Local Device Status Register                       */
    __IO uint32_t INTEN;                 /*!< [0x0018] SPD5 HUB Interrupt Enable Register                          */
    __IO uint32_t INTSTS;                /*!< [0x001C] SPD5 HUB Interrupt Status Register                          */
    __IO uint32_t HSDASW;                /*!< [0x0020] SPD5 HUB HSDA IO Switch Register                            */
    __IO uint32_t HCAP;                  /*!< [0x0024] SPD5 HUB DEVCAP Register                                    */
    __IO uint32_t DCAP;                  /*!< [0x0028] SPD5 HUB Local Device DEVCAP Register                       */
    __I  uint32_t HDEVCTRL0;             /*!< [0x002C] SPD5 HUB DEVCTRL DATA 0 Register                            */
    __I  uint32_t HDEVCTRL1;             /*!< [0x0030] SPD5 HUB DEVCTRL DATA 1 Register                            */
    __IO uint32_t PWRD;                  /*!< [0x0034] SPD5 HUB Power Down Register                                */

} SPDH_T;

/**
    @addtogroup TIMER_CONST TIMER Bit Field Definition
    Constant Definitions for TIMER Controller
    @{
*/

#define SPDH_CTL_HUBEN_Pos                (0)                                               /*!< SPDH_T::CTL: HUBEN Position           */
#define SPDH_CTL_HUBEN_Msk                (0x1ul << SPDH_CTL_HUBEN_Pos)                     /*!< SPDH_T::CTL: HUBEN Mask               */

#define SPDH_CTL_SIRENVAL_Pos             (2)                                               /*!< SPDH_T::CTL: SIRENVAL Position        */
#define SPDH_CTL_SIRENVAL_Msk             (0x1ul << SPDH_CTL_SIRENVAL_Pos)                  /*!< SPDH_T::CTL: SIRENVAL Mask            */

#define SPDH_CTL_PWRWUEN_Pos              (3)                                               /*!< SPDH_T::CTL: PWRWUEN Position         */
#define SPDH_CTL_PWRWUEN_Msk              (0x1ul << SPDH_CTL_PWRWUEN_Pos)                   /*!< SPDH_T::CTL: PWRWUEN Mask             */

#define SPDH_CTL_CRCEN_Pos                (4)                                               /*!< SPDH_T::CTL: CRCEN Position           */
#define SPDH_CTL_CRCEN_Msk                (0x1ul << SPDH_CTL_CRCEN_Pos)                     /*!< SPDH_T::CTL: CRCEN Mask               */

#define SPDH_CTL_BUSRSTEN_Pos             (5)                                               /*!< SPDH_T::CTL: BUSRSTEN Position        */
#define SPDH_CTL_BUSRSTEN_Msk             (0x1ul << SPDH_CTL_BUSRSTEN_Pos)                  /*!< SPDH_T::CTL: BUSRSTEN Mask            */

#define SPDH_CTL_HSDATOEN_Pos             (6)                                               /*!< SPDH_T::CTL: HSDATOEN Position        */
#define SPDH_CTL_HSDATOEN_Msk             (0x1ul << SPDH_CTL_HSDATOEN_Pos)                  /*!< SPDH_T::CTL: HSDATOEN Mask            */

#define SPDH_CTL_EVENTCLR_Pos             (7)                                               /*!< SPDH_T::CTL: EVENTCLR Position        */
#define SPDH_CTL_EVENTCLR_Msk             (0x1ul << SPDH_CTL_EVENTCLR_Pos)                  /*!< SPDH_T::CTL: EVENTCLR Mask            */

#define SPDH_CTL_LSCLPUS_Pos              (8)                                               /*!< SPDH_T::CTL: LSCLPUS Position         */
#define SPDH_CTL_LSCLPUS_Msk              (0x3ul << SPDH_CTL_LSCLPUS_Pos)                   /*!< SPDH_T::CTL: LSCLPUS Mask             */

#define SPDH_CTL_LSDAPUS_Pos              (12)                                              /*!< SPDH_T::CTL: LSDAPUS Position         */
#define SPDH_CTL_LSDAPUS_Msk              (0x3ul << SPDH_CTL_LSDAPUS_Pos)                   /*!< SPDH_T::CTL: LSDAPUS Mask             */

#define SPDH_CTL_PECCLR_Pos               (15)                                              /*!< SPDH_T::CTL: PECCLR Position          */
#define SPDH_CTL_PECCLR_Msk               (0x1ul << SPDH_CTL_PECCLR_Pos)                    /*!< SPDH_T::CTL: PECCLR Mask              */

#define SPDH_CTL_LPUSEL_Pos               (16)                                              /*!< SPDH_T::CTL: LPUSEL Position          */
#define SPDH_CTL_LPUSEL_Msk               (0x3ul << SPDH_CTL_LPUSEL_Pos)                    /*!< SPDH_T::CTL: LPUSEL Mask              */

#define SPDH_HID_HID_Pos                  (0)                                               /*!< SPDH_T::HID: HID Position             */
#define SPDH_HID_HID_Msk                  (0x7ul << SPDH_HID_HID_Pos)                       /*!< SPDH_T::HID: HID Mask                 */

#define SPDH_BUSRST_BUSTO_Pos             (0)                                               /*!< SPDH_T::BUSRST: BUSTO Position        */
#define SPDH_BUSRST_BUSTO_Msk             (0xfful << SPDH_BUSRST_BUSTO_Pos)                 /*!< SPDH_T::BUSRST: BUSTO Mask            */

#define SPDH_STS_PECSTS_Pos               (0)                                               /*!< SPDH_T::STS: PECSTS Position          */
#define SPDH_STS_PECSTS_Msk               (0x1ul << SPDH_STS_PECSTS_Pos)                    /*!< SPDH_T::STS: PECSTS Mask              */

#define SPDH_STS_PARDIS_Pos               (1)                                               /*!< SPDH_T::STS: PARDIS Position          */
#define SPDH_STS_PARDIS_Msk               (0x1ul << SPDH_STS_PARDIS_Pos)                    /*!< SPDH_T::STS: PARDIS Mask              */

#define SPDH_STS_IBICLR_Pos               (2)                                               /*!< SPDH_T::STS: IBICLR Position          */
#define SPDH_STS_IBICLR_Msk               (0x1ul << SPDH_STS_IBICLR_Pos)                    /*!< SPDH_T::STS: IBICLR Mask              */

#define SPDH_STS_PECCHK_Pos               (4)                                               /*!< SPDH_T::STS: PECCHK Position          */
#define SPDH_STS_PECCHK_Msk               (0x1ul << SPDH_STS_PECCHK_Pos)                    /*!< SPDH_T::STS: PECCHK Mask              */

#define SPDH_STS_MODE_Pos                 (5)                                               /*!< SPDH_T::STS: MODE Position            */
#define SPDH_STS_MODE_Msk                 (0x1ul << SPDH_STS_MODE_Pos)                      /*!< SPDH_T::STS: MODE Mask                */

#define SPDH_STS_IBIHEAD_Pos              (6)                                               /*!< SPDH_T::STS: IBIHEAD Position         */
#define SPDH_STS_IBIHEAD_Msk              (0x1ul << SPDH_STS_IBIHEAD_Pos)                   /*!< SPDH_T::STS: IBIHEAD Mask             */

#define SPDH_STS_PENDIBI_Pos              (8)                                               /*!< SPDH_T::STS: PENDIBI Position         */
#define SPDH_STS_PENDIBI_Msk              (0xful << SPDH_STS_PENDIBI_Pos)                   /*!< SPDH_T::STS: PENDIBI Mask             */

#define SPDH_STS_PROERR_Pos               (12)                                              /*!< SPDH_T::STS: PROERR Position          */
#define SPDH_STS_PROERR_Msk               (0x1ul << SPDH_STS_PROERR_Pos)                    /*!< SPDH_T::STS: PROERR Mask              */

#define SPDH_STS_PECERR_Pos               (13)                                              /*!< SPDH_T::STS: PECERR Position          */
#define SPDH_STS_PECERR_Msk               (0x1ul << SPDH_STS_PECERR_Pos)                    /*!< SPDH_T::STS: PECERR Mask              */

#define SPDH_DCTL_HID_Pos                 (0)                                               /*!< SPDH_T::DCTL: HID Position            */
#define SPDH_DCTL_HID_Msk                 (0x7ul << SPDH_DCTL_HID_Pos)                      /*!< SPDH_T::DCTL: HID Mask                */

#define SPDH_DCTL_LID_Pos                 (3)                                               /*!< SPDH_T::DCTL: LID Position            */
#define SPDH_DCTL_LID_Msk                 (0xful << SPDH_DCTL_LID_Pos)                      /*!< SPDH_T::DCTL: LID Mask                */

#define SPDH_DCTL_EVENTCLR_Pos            (7)                                               /*!< SPDH_T::DCTL: EVENTCLR Position       */
#define SPDH_DCTL_EVENTCLR_Msk            (0x1ul << SPDH_DCTL_EVENTCLR_Pos)                 /*!< SPDH_T::DCTL: EVENTCLR Mask           */

#define SPDH_DCTL_CRCEN_Pos               (8)                                               /*!< SPDH_T::DCTL: CRCEN Position          */
#define SPDH_DCTL_CRCEN_Msk               (0x1ul << SPDH_DCTL_CRCEN_Pos)                    /*!< SPDH_T::DCTL: CRCEN Mask              */

#define SPDH_DCTL_SIRENVAL_Pos            (9)                                               /*!< SPDH_T::DCTL: SIRENVAL Position       */
#define SPDH_DCTL_SIRENVAL_Msk            (0x1ul << SPDH_DCTL_SIRENVAL_Pos)                 /*!< SPDH_T::DCTL: SIRENVAL Mask           */

#define SPDH_DCTL_PECCLR_Pos              (15)                                              /*!< SPDH_T::DCTL: PECCLR Position         */
#define SPDH_DCTL_PECCLR_Msk              (0x1ul << SPDH_DCTL_PECCLR_Pos)                   /*!< SPDH_T::DCTL: PECCLR Mask             */

#define SPDH_DSTS_PECSTS_Pos              (0)                                               /*!< SPDH_T::DSTS: PECSTS Position         */
#define SPDH_DSTS_PECSTS_Msk              (0x1ul << SPDH_DSTS_PECSTS_Pos)                   /*!< SPDH_T::DSTS: PECSTS Mask             */

#define SPDH_DSTS_PARDIS_Pos              (1)                                               /*!< SPDH_T::DSTS: PARDIS Position         */
#define SPDH_DSTS_PARDIS_Msk              (0x1ul << SPDH_DSTS_PARDIS_Pos)                   /*!< SPDH_T::DSTS: PARDIS Mask             */

#define SPDH_DSTS_IBICLR_Pos              (2)                                               /*!< SPDH_T::DSTS: IBICLR Position         */
#define SPDH_DSTS_IBICLR_Msk              (0x1ul << SPDH_DSTS_IBICLR_Pos)                   /*!< SPDH_T::DSTS: IBICLR Mask             */

#define SPDH_DSTS_PECCHK_Pos              (4)                                               /*!< SPDH_T::DSTS: PECCHK Position         */
#define SPDH_DSTS_PECCHK_Msk              (0x1ul << SPDH_DSTS_PECCHK_Pos)                   /*!< SPDH_T::DSTS: PECCHK Mask             */

#define SPDH_DSTS_MODE_Pos                (5)                                               /*!< SPDH_T::DSTS: MODE Position           */
#define SPDH_DSTS_MODE_Msk                (0x1ul << SPDH_DSTS_MODE_Pos)                     /*!< SPDH_T::DSTS: MODE Mask               */

#define SPDH_DSTS_IBIHEAD_Pos             (6)                                               /*!< SPDH_T::DSTS: IBIHEAD Position        */
#define SPDH_DSTS_IBIHEAD_Msk             (0x1ul << SPDH_DSTS_IBIHEAD_Pos)                  /*!< SPDH_T::DSTS: IBIHEAD Mask            */

#define SPDH_DSTS_PENDIBI_Pos             (8)                                               /*!< SPDH_T::DSTS: PENDIBI Position        */
#define SPDH_DSTS_PENDIBI_Msk             (0xful << SPDH_DSTS_PENDIBI_Pos)                  /*!< SPDH_T::DSTS: PENDIBI Mask            */

#define SPDH_DSTS_PROERR_Pos              (12)                                              /*!< SPDH_T::DSTS: PROERR Position         */
#define SPDH_DSTS_PROERR_Msk              (0x1ul << SPDH_DSTS_PROERR_Pos)                   /*!< SPDH_T::DSTS: PROERR Mask             */

#define SPDH_DSTS_PECERR_Pos              (13)                                              /*!< SPDH_T::DSTS: PECERR Position         */
#define SPDH_DSTS_PECERR_Msk              (0x1ul << SPDH_DSTS_PECERR_Pos)                   /*!< SPDH_T::DSTS: PECERR Mask             */

#define SPDH_INTEN_BUSRTOEN_Pos           (0)                                               /*!< SPDH_T::INTEN: BUSRTOEN Position      */
#define SPDH_INTEN_BUSRTOEN_Msk           (0x1ul << SPDH_INTEN_BUSRTOEN_Pos)                /*!< SPDH_T::INTEN: BUSRTOEN Mask          */

#define SPDH_INTEN_HDEVCTLEN_Pos          (1)                                               /*!< SPDH_T::INTEN: HDEVCTLEN Position     */
#define SPDH_INTEN_HDEVCTLEN_Msk          (0x1ul << SPDH_INTEN_HDEVCTLEN_Pos)               /*!< SPDH_T::INTEN: HDEVCTLEN Mask         */

#define SPDH_INTEN_DDEVCTLEN_Pos          (2)                                               /*!< SPDH_T::INTEN: DDEVCTLEN Position     */
#define SPDH_INTEN_DDEVCTLEN_Msk          (0x1ul << SPDH_INTEN_DDEVCTLEN_Pos)               /*!< SPDH_T::INTEN: DDEVCTLEN Mask         */

#define SPDH_INTEN_HDEVCAPEN_Pos          (3)                                               /*!< SPDH_T::INTEN: HDEVCAPEN Position     */
#define SPDH_INTEN_HDEVCAPEN_Msk          (0x1ul << SPDH_INTEN_HDEVCAPEN_Pos)               /*!< SPDH_T::INTEN: HDEVCAPEN Mask         */

#define SPDH_INTEN_DDEVCAPEN_Pos          (4)                                               /*!< SPDH_T::INTEN: DDEVCAPEN Position     */
#define SPDH_INTEN_DDEVCAPEN_Msk          (0x1ul << SPDH_INTEN_DDEVCAPEN_Pos)               /*!< SPDH_T::INTEN: DDEVCAPEN Mask         */

#define SPDH_INTEN_DSETHIDEN_Pos          (5)                                               /*!< SPDH_T::INTEN: DSETHIDEN Position     */
#define SPDH_INTEN_DSETHIDEN_Msk          (0x1ul << SPDH_INTEN_DSETHIDEN_Pos)               /*!< SPDH_T::INTEN: DSETHIDEN Mask         */

#define SPDH_INTEN_HUBPCEN_Pos            (6)                                               /*!< SPDH_T::INTEN: HUBPCEN Position       */
#define SPDH_INTEN_HUBPCEN_Msk            (0x1ul << SPDH_INTEN_HUBPCEN_Pos)                 /*!< SPDH_T::INTEN: HUBPCEN Mask           */

#define SPDH_INTEN_DEVPCEN_Pos            (7)                                               /*!< SPDH_T::INTEN: DEVPCEN Position       */
#define SPDH_INTEN_DEVPCEN_Msk            (0x1ul << SPDH_INTEN_DEVPCEN_Pos)                 /*!< SPDH_T::INTEN: DEVPCEN Mask           */

#define SPDH_INTEN_HSDATOEN_Pos           (8)                                               /*!< SPDH_T::INTEN: HSDATOEN Position      */
#define SPDH_INTEN_HSDATOEN_Msk           (0x1ul << SPDH_INTEN_HSDATOEN_Pos)                /*!< SPDH_T::INTEN: HSDATOEN Mask          */

#define SPDH_INTEN_HUBIHDEN_Pos           (9)                                               /*!< SPDH_T::INTEN: HUBIHDEN Position      */
#define SPDH_INTEN_HUBIHDEN_Msk           (0x1ul << SPDH_INTEN_HUBIHDEN_Pos)                /*!< SPDH_T::INTEN: HUBIHDEN Mask          */

#define SPDH_INTEN_DEVIHDEN_Pos           (10)                                              /*!< SPDH_T::INTEN: DEVIHDEN Position      */
#define SPDH_INTEN_DEVIHDEN_Msk           (0x1ul << SPDH_INTEN_DEVIHDEN_Pos)                /*!< SPDH_T::INTEN: DEVIHDEN Mask          */

#define SPDH_INTEN_PWRDTOEN_Pos           (11)                                              /*!< SPDH_T::INTEN: PWRDTOEN Position      */
#define SPDH_INTEN_PWRDTOEN_Msk           (0x1ul << SPDH_INTEN_PWRDTOEN_Pos)                /*!< SPDH_T::INTEN: PWRDTOEN Mask          */

#define SPDH_INTEN_WKUPEN_Pos             (12)                                              /*!< SPDH_T::INTEN: WKUPEN Position        */
#define SPDH_INTEN_WKUPEN_Msk             (0x1ul << SPDH_INTEN_WKUPEN_Pos)                  /*!< SPDH_T::INTEN: WKUPEN Mask            */

#define SPDH_INTSTS_BUSRTOIF_Pos          (0)                                               /*!< SPDH_T::INTSTS: BUSRTOIF Position     */
#define SPDH_INTSTS_BUSRTOIF_Msk          (0x1ul << SPDH_INTSTS_BUSRTOIF_Pos)               /*!< SPDH_T::INTSTS: BUSRTOIF Mask         */

#define SPDH_INTSTS_HDEVCTLIF_Pos         (1)                                               /*!< SPDH_T::INTSTS: HDEVCTLIF Position    */
#define SPDH_INTSTS_HDEVCTLIF_Msk         (0x1ul << SPDH_INTSTS_HDEVCTLIF_Pos)              /*!< SPDH_T::INTSTS: HDEVCTLIF Mask        */

#define SPDH_INTSTS_DDEVCTLIF_Pos         (2)                                               /*!< SPDH_T::INTSTS: DDEVCTLIF Position    */
#define SPDH_INTSTS_DDEVCTLIF_Msk         (0x1ul << SPDH_INTSTS_DDEVCTLIF_Pos)              /*!< SPDH_T::INTSTS: DDEVCTLIF Mask        */

#define SPDH_INTSTS_HDEVCAPIF_Pos         (3)                                               /*!< SPDH_T::INTSTS: HDEVCAPIF Position    */
#define SPDH_INTSTS_HDEVCAPIF_Msk         (0x1ul << SPDH_INTSTS_HDEVCAPIF_Pos)              /*!< SPDH_T::INTSTS: HDEVCAPIF Mask        */

#define SPDH_INTSTS_DDEVCAPIF_Pos         (4)                                               /*!< SPDH_T::INTSTS: DDEVCAPIF Position    */
#define SPDH_INTSTS_DDEVCAPIF_Msk         (0x1ul << SPDH_INTSTS_DDEVCAPIF_Pos)              /*!< SPDH_T::INTSTS: DDEVCAPIF Mask        */

#define SPDH_INTSTS_DSETHIDIF_Pos         (5)                                               /*!< SPDH_T::INTSTS: DSETHIDIF Position    */
#define SPDH_INTSTS_DSETHIDIF_Msk         (0x1ul << SPDH_INTSTS_DSETHIDIF_Pos)              /*!< SPDH_T::INTSTS: DSETHIDIF Mask        */

#define SPDH_INTSTS_HUBPCIF_Pos           (6)                                               /*!< SPDH_T::INTSTS: HUBPCIF Position      */
#define SPDH_INTSTS_HUBPCIF_Msk           (0x1ul << SPDH_INTSTS_HUBPCIF_Pos)                /*!< SPDH_T::INTSTS: HUBPCIF Mask          */

#define SPDH_INTSTS_DEVPCIF_Pos           (7)                                               /*!< SPDH_T::INTSTS: DEVPCIF Position      */
#define SPDH_INTSTS_DEVPCIF_Msk           (0x1ul << SPDH_INTSTS_DEVPCIF_Pos)                /*!< SPDH_T::INTSTS: DEVPCIF Mask          */

#define SPDH_INTSTS_HSDATOIF_Pos          (8)                                               /*!< SPDH_T::INTSTS: HSDATOIF Position     */
#define SPDH_INTSTS_HSDATOIF_Msk          (0x1ul << SPDH_INTSTS_HSDATOIF_Pos)               /*!< SPDH_T::INTSTS: HSDATOIF Mask         */

#define SPDH_INTSTS_HUBIHDIF_Pos          (9)                                               /*!< SPDH_T::INTSTS: HUBIHDIF Position     */
#define SPDH_INTSTS_HUBIHDIF_Msk          (0x1ul << SPDH_INTSTS_HUBIHDIF_Pos)               /*!< SPDH_T::INTSTS: HUBIHDIF Mask         */

#define SPDH_INTSTS_DEVIHDIF_Pos          (10)                                              /*!< SPDH_T::INTSTS: DEVIHDIF Position     */
#define SPDH_INTSTS_DEVIHDIF_Msk          (0x1ul << SPDH_INTSTS_DEVIHDIF_Pos)               /*!< SPDH_T::INTSTS: DEVIHDIF Mask         */

#define SPDH_INTSTS_PWRDTOIF_Pos          (11)                                              /*!< SPDH_T::INTSTS: PWRDTOIF Position     */
#define SPDH_INTSTS_PWRDTOIF_Msk          (0x1ul << SPDH_INTSTS_PWRDTOIF_Pos)               /*!< SPDH_T::INTSTS: PWRDTOIF Mask         */

#define SPDH_INTSTS_WKUPIF_Pos            (12)                                              /*!< SPDH_T::INTSTS: WKUPIF Position       */
#define SPDH_INTSTS_WKUPIF_Msk            (0x1ul << SPDH_INTSTS_WKUPIF_Pos)                 /*!< SPDH_T::INTSTS: WKUPIF Mask           */

#define SPDH_HSDASW_IOSWTO_Pos            (0)                                               /*!< SPDH_T::HSDASW: IOSWTO Position       */
#define SPDH_HSDASW_IOSWTO_Msk            (0x3ful << SPDH_HSDASW_IOSWTO_Pos)                /*!< SPDH_T::HSDASW: IOSWTO Mask           */

#define SPDH_HCAP_LSB_Pos                 (0)                                               /*!< SPDH_T::HCAP: LSB Position            */
#define SPDH_HCAP_LSB_Msk                 (0xfful << SPDH_HCAP_LSB_Pos)                     /*!< SPDH_T::HCAP: LSB Mask                */

#define SPDH_HCAP_MSB_Pos                 (8)                                               /*!< SPDH_T::HCAP: MSB Position            */
#define SPDH_HCAP_MSB_Msk                 (0xfful << SPDH_HCAP_MSB_Pos)                     /*!< SPDH_T::HCAP: MSB Mask                */

#define SPDH_DCAP_LSB_Pos                 (0)                                               /*!< SPDH_T::DCAP: LSB Position            */
#define SPDH_DCAP_LSB_Msk                 (0xfful << SPDH_DCAP_LSB_Pos)                     /*!< SPDH_T::DCAP: LSB Mask                */

#define SPDH_DCAP_MSB_Pos                 (8)                                               /*!< SPDH_T::DCAP: MSB Position            */
#define SPDH_DCAP_MSB_Msk                 (0xfful << SPDH_DCAP_MSB_Pos)                     /*!< SPDH_T::DCAP: MSB Mask                */

#define SPDH_HDEVCTRL0_REGMOD_Pos         (0)                                               /*!< SPDH_T::HDEVCTRL0: REGMOD Position    */
#define SPDH_HDEVCTRL0_REGMOD_Msk         (0x1ul << SPDH_HDEVCTRL0_REGMOD_Pos)              /*!< SPDH_T::HDEVCTRL0: REGMOD Mask        */

#define SPDH_HDEVCTRL0_PECBL_Pos          (1)                                               /*!< SPDH_T::HDEVCTRL0: PECBL Position     */
#define SPDH_HDEVCTRL0_PECBL_Msk          (0x3ul << SPDH_HDEVCTRL0_PECBL_Pos)               /*!< SPDH_T::HDEVCTRL0: PECBL Mask         */

#define SPDH_HDEVCTRL0_STAOFSET_Pos       (3)                                               /*!< SPDH_T::HDEVCTRL0: STAOFSET Position  */
#define SPDH_HDEVCTRL0_STAOFSET_Msk       (0x3ul << SPDH_HDEVCTRL0_STAOFSET_Pos)            /*!< SPDH_T::HDEVCTRL0: STAOFSET Mask      */

#define SPDH_HDEVCTRL0_ADDRMSK_Pos        (5)                                               /*!< SPDH_T::HDEVCTRL0: ADDRMSK Position   */
#define SPDH_HDEVCTRL0_ADDRMSK_Msk        (0x7ul << SPDH_HDEVCTRL0_ADDRMSK_Pos)             /*!< SPDH_T::HDEVCTRL0: ADDRMSK Mask       */

#define SPDH_HDEVCTRL0_DEVID_Pos          (9)                                               /*!< SPDH_T::HDEVCTRL0: DEVID Position     */
#define SPDH_HDEVCTRL0_DEVID_Msk          (0x7ful << SPDH_HDEVCTRL0_DEVID_Pos)              /*!< SPDH_T::HDEVCTRL0: DEVID Mask         */

#define SPDH_HDEVCTRL1_PAYLOAD0_Pos       (0)                                               /*!< SPDH_T::HDEVCTRL1: PAYLOAD0 Position  */
#define SPDH_HDEVCTRL1_PAYLOAD0_Msk       (0xfful << SPDH_HDEVCTRL1_PAYLOAD0_Pos)           /*!< SPDH_T::HDEVCTRL1: PAYLOAD0 Mask      */

#define SPDH_HDEVCTRL1_PAYLOAD1_Pos       (8)                                               /*!< SPDH_T::HDEVCTRL1: PAYLOAD1 Position  */
#define SPDH_HDEVCTRL1_PAYLOAD1_Msk       (0xfful << SPDH_HDEVCTRL1_PAYLOAD1_Pos)           /*!< SPDH_T::HDEVCTRL1: PAYLOAD1 Mask      */

#define SPDH_HDEVCTRL1_PAYLOAD2_Pos       (16)                                              /*!< SPDH_T::HDEVCTRL1: PAYLOAD2 Position  */
#define SPDH_HDEVCTRL1_PAYLOAD2_Msk       (0xfful << SPDH_HDEVCTRL1_PAYLOAD2_Pos)           /*!< SPDH_T::HDEVCTRL1: PAYLOAD2 Mask      */

#define SPDH_HDEVCTRL1_PAYLOAD3_Pos       (24)                                              /*!< SPDH_T::HDEVCTRL1: PAYLOAD3 Position  */
#define SPDH_HDEVCTRL1_PAYLOAD3_Msk       (0xfful << SPDH_HDEVCTRL1_PAYLOAD3_Pos)           /*!< SPDH_T::HDEVCTRL1: PAYLOAD3 Mask      */

#define SPDH_PWRD_PWRDTO_Pos              (0)                                               /*!< SPDH_T::PWRD: PWRDTO Position         */
#define SPDH_PWRD_PWRDTO_Msk              (0xfful << SPDH_PWRD_PWRDTO_Pos)                  /*!< SPDH_T::PWRD: PWRDTO Mask             */


/**@}*/ /* SPDH_CONST */
/**@}*/ /* end of SPDH register group */
/**@}*/ /* end of REGISTER group */

#endif /* __SPDH_REG_H__ */
