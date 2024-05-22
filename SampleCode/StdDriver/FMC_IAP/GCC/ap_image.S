;/**************************************************************************//**
; * @file     ap_image.S
; * @version  V1.00
; * @brief    Assembly code include AP image.
; *
; *
; * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
    .syntax unified
    .arch   armv6-m

    .globl  loaderImage1Base
    .globl  loaderImage1Limit

    .align  4

    .text

loaderImage1Base:
# ifdef BIN_FILE
    .incbin   BIN_FILE
# else
    .incbin  "../../LDROM_iap/Release/LDROM_iap.bin"
# endif
loaderImage1Limit:
    .space   4

    .end
