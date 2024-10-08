;/**************************************************************************//**
; * @file     ap_image.S
; * @version  V1.00
; * @brief    Assembly code include AP image.
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
    .syntax unified
    .arch   armv6-m

    .globl  loaderImage1Base
    .globl  loaderImage1Limit

    .align  4

    .text

loaderImage1Base:
    .incbin   "LDROM_iap.bin"
loaderImage1Limit:
    .space   4

    .end
