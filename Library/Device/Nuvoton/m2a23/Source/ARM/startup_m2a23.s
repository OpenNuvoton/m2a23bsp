;/**************************************************************************//**
; * @file     startup_m2a23.s
; * @version  V3.00
; * @brief    M2A23 Series Startup Source File
; *
; * @copyright SPDX-License-Identifier: Apache-2.0
; * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
; ******************************************************************************/


;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

    IF :LNOT: :DEF: Stack_Size
Stack_Size      EQU     0x00000400
    ENDIF

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

    IF :LNOT: :DEF: Heap_Size
Heap_Size       EQU     0x00000000
    ENDIF

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts

                DCD     BOD_IRQHandler
                DCD     IRC_IRQHandler
                DCD     PWRWU_IRQHandler
                DCD     RAMPE_IRQHandler
                DCD     CKFAIL_IRQHandler
                DCD     ISP_IRQHandler
                DCD     DEFAULT_IRQHandler
                DCD     DEFAULT_IRQHandler
                DCD     WDT_IRQHandler
                DCD     WWDT_IRQHandler
                DCD     EINT0_IRQHandler
                DCD     EINT1_IRQHandler
                DCD     EINT2_IRQHandler
                DCD     EINT3_IRQHandler
                DCD     EINT4_IRQHandler
                DCD     EINT5_IRQHandler
                DCD     GPA_IRQHandler
                DCD     GPB_IRQHandler
                DCD     GPC_IRQHandler
                DCD     GPD_IRQHandler
                DCD     DEFAULT_IRQHandler
                DCD     GPF_IRQHandler
                DCD     DEFAULT_IRQHandler                    
                DCD     SPI0_IRQHandler
                DCD     BRAKE0_IRQHandler
                DCD     PWM0P0_IRQHandler
                DCD     PWM0P1_IRQHandler
                DCD     PWM0P2_IRQHandler
                DCD     DEFAULT_IRQHandler
                DCD     DEFAULT_IRQHandler
                DCD     DEFAULT_IRQHandler
                DCD     DEFAULT_IRQHandler
                DCD     TMR0_IRQHandler
                DCD     TMR1_IRQHandler
                DCD     TMR2_IRQHandler
                DCD     TMR3_IRQHandler
                DCD     UART0_IRQHandler
                DCD     UART1_IRQHandler
                DCD     I2C0_IRQHandler
                DCD     DEFAULT_IRQHandler
                DCD     PDMA0_IRQHandler
                DCD     DEFAULT_IRQHandler                    
                DCD     ADC0_IRQHandler
                DCD     DEFAULT_IRQHandler                    
                DCD     ACMP01_IRQHandler
                DCD     BPWM0_IRQHandler
                DCD     LLSI0_IRQHandler
                DCD     LLSI1_IRQHandler
                DCD     CANFD00_IRQHandler
                DCD     CANFD01_IRQHandler
                DCD     CANFD10_IRQHandler
                DCD     CANFD11_IRQHandler
                DCD     CANFD20_IRQHandler
                DCD     CANFD21_IRQHandler
                DCD     DEFAULT_IRQHandler
                DCD     DEFAULT_IRQHandler
                DCD     USCI0_IRQHandler
                DCD     USCI1_IRQHandler

__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
					

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                IMPORT  ProcessHardFault
                EXPORT  HardFault_Handler         [WEAK]
                MOV     R0, LR                 
                MRS     R1, MSP                
                MRS     R2, PSP                
                LDR     R3, =ProcessHardFault 
                BLX     R3                     
                BX      R0                     
                ENDP
ProcessHardFaultx\
                PROC
                EXPORT  ProcessHardFaultx          [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  IRC_IRQHandler            [WEAK]
                EXPORT  PWRWU_IRQHandler          [WEAK]
                EXPORT  RAMPE_IRQHandler          [WEAK]
                EXPORT  CKFAIL_IRQHandler         [WEAK]
                EXPORT  ISP_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  WWDT_IRQHandler           [WEAK]
                EXPORT  EINT0_IRQHandler          [WEAK]
                EXPORT  EINT1_IRQHandler          [WEAK]
                EXPORT  EINT2_IRQHandler          [WEAK]
                EXPORT  EINT3_IRQHandler          [WEAK]
                EXPORT  EINT4_IRQHandler          [WEAK]
                EXPORT  EINT5_IRQHandler          [WEAK]
                EXPORT  GPA_IRQHandler            [WEAK]
                EXPORT  GPB_IRQHandler            [WEAK]
                EXPORT  GPC_IRQHandler            [WEAK]
                EXPORT  GPD_IRQHandler            [WEAK]
                EXPORT  GPF_IRQHandler            [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  BRAKE0_IRQHandler         [WEAK]
                EXPORT  PWM0P0_IRQHandler         [WEAK]
                EXPORT  PWM0P1_IRQHandler         [WEAK]
                EXPORT  PWM0P2_IRQHandler         [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  TMR2_IRQHandler           [WEAK]
                EXPORT  TMR3_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  PDMA0_IRQHandler          [WEAK]
                EXPORT  ADC0_IRQHandler           [WEAK]
                EXPORT  ACMP01_IRQHandler         [WEAK]
                EXPORT  BPWM0_IRQHandler          [WEAK]
                EXPORT  LLSI0_IRQHandler          [WEAK]
                EXPORT  LLSI1_IRQHandler          [WEAK]
                EXPORT  CANFD00_IRQHandler        [WEAK]
                EXPORT  CANFD01_IRQHandler        [WEAK]
                EXPORT  CANFD10_IRQHandler        [WEAK]
                EXPORT  CANFD11_IRQHandler        [WEAK]
                EXPORT  CANFD20_IRQHandler        [WEAK]
                EXPORT  CANFD21_IRQHandler        [WEAK]
                EXPORT  USCI0_IRQHandler          [WEAK]
                EXPORT  USCI1_IRQHandler          [WEAK]
                EXPORT  DEFAULT_IRQHandler        [WEAK]

BOD_IRQHandler
IRC_IRQHandler
PWRWU_IRQHandler
RAMPE_IRQHandler
CKFAIL_IRQHandler
ISP_IRQHandler
WDT_IRQHandler
WWDT_IRQHandler
EINT0_IRQHandler
EINT1_IRQHandler
EINT2_IRQHandler
EINT3_IRQHandler
EINT4_IRQHandler
EINT5_IRQHandler
GPA_IRQHandler
GPB_IRQHandler
GPC_IRQHandler
GPD_IRQHandler
GPF_IRQHandler
SPI0_IRQHandler
BRAKE0_IRQHandler
PWM0P0_IRQHandler
PWM0P1_IRQHandler
PWM0P2_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
I2C0_IRQHandler
PDMA0_IRQHandler
ADC0_IRQHandler
ACMP01_IRQHandler
BPWM0_IRQHandler
LLSI0_IRQHandler
LLSI1_IRQHandler
CANFD00_IRQHandler
CANFD01_IRQHandler
CANFD10_IRQHandler
CANFD11_IRQHandler
CANFD20_IRQHandler
CANFD21_IRQHandler
USCI0_IRQHandler
USCI1_IRQHandler
DEFAULT_IRQHandler
                B       .
                ENDP

                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, = Heap_Mem
                LDR     R1, = (Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP


                ALIGN

                ENDIF

;int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
SH_DoCommand    PROC
    
                EXPORT      SH_DoCommand
                IMPORT      SH_Return
                    
                BKPT   0xAB                ; Wait ICE or HardFault
                LDR    R3, =SH_Return 
                MOV    R4, lr          
                BLX    R3                  ; Call SH_Return. The return value is in R0
                BX     R4                  ; Return value = R0
                
                ENDP

__PC            PROC
                EXPORT      __PC
                
                MOV     r0, lr
                BLX     lr
                ALIGN
                    
                ENDP
                    
                END
