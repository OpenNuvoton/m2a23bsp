/**************************************************************************//**
 * @file     sys.h
 * @version  V3.00
 * @brief    M2A23 series System Manager (SYS) driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __SYS_H__
#define __SYS_H__


#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SYS_Driver SYS Driver
  @{
*/

/** @addtogroup SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_RST    ((0x0<<24)|SYS_IPRST0_PDMARST_Pos)      /*!< PDMA reset is one of the SYS_ResetModule parameter */
#define CRC_RST     ((0x0<<24)|SYS_IPRST0_CRCRST_Pos)       /*!< CRC reset is one of the SYS_ResetModule parameter */

#define GPIO_RST    ((0x4<<24)|SYS_IPRST1_GPIORST_Pos)      /*!< GPIO reset is one of the SYS_ResetModule parameter */
#define TMR0_RST    ((0x4<<24)|SYS_IPRST1_TMR0RST_Pos)      /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST    ((0x4<<24)|SYS_IPRST1_TMR1RST_Pos)      /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR2_RST    ((0x4<<24)|SYS_IPRST1_TMR2RST_Pos)      /*!< TMR2 reset is one of the SYS_ResetModule parameter */
#define TMR3_RST    ((0x4<<24)|SYS_IPRST1_TMR3RST_Pos)      /*!< TMR3 reset is one of the SYS_ResetModule parameter */
#define I2C0_RST    ((0x4<<24)|SYS_IPRST1_I2C0RST_Pos)      /*!< I2C0 reset is one of the SYS_ResetModule parameter */
#define I2C1_RST    ((0x4<<24)|SYS_IPRST1_I2C1RST_Pos)      /*!< I2C1 reset is one of the SYS_ResetModule parameter */
#define I2C2_RST    ((0x4<<24)|SYS_IPRST1_I2C2RST_Pos)      /*!< I2C2 reset is one of the SYS_ResetModule parameter */
#define SPI0_RST    ((0x4<<24)|SYS_IPRST1_SPI0RST_Pos)      /*!< SPI0 reset is one of the SYS_ResetModule parameter */
#define SPI1_RST    ((0x4<<24)|SYS_IPRST1_SPI1RST_Pos)      /*!< SPI1 reset is one of the SYS_ResetModule parameter */
#define SPI2_RST    ((0x4<<24)|SYS_IPRST1_SPI2RST_Pos)      /*!< SPI2 reset is one of the SYS_ResetModule parameter */
#define UART0_RST   ((0x4<<24)|SYS_IPRST1_UART0RST_Pos)     /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST   ((0x4<<24)|SYS_IPRST1_UART1RST_Pos)     /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define UART2_RST   ((0x4<<24)|SYS_IPRST1_UART2RST_Pos)     /*!< UART2 reset is one of the SYS_ResetModule parameter */
#define BPWM0_RST   ((0x4<<24)|SYS_IPRST1_BPWM0RST_Pos)     /*!< BPWM0 reset is one of the SYS_ResetModule parameter */
#define BPWM1_RST   ((0x4<<24)|SYS_IPRST1_BPWM1RST_Pos)     /*!< BPWM1 reset is one of the SYS_ResetModule parameter */
#define BPWM2_RST   ((0x4<<24)|SYS_IPRST1_BPWM2RST_Pos)     /*!< BPWM2 reset is one of the SYS_ResetModule parameter */
#define BPWM3_RST   ((0x4<<24)|SYS_IPRST1_BPWM3RST_Pos)     /*!< BPWM3 reset is one of the SYS_ResetModule parameter */
#define USBD_RST    ((0x4<<24)|SYS_IPRST1_USBDRST_Pos)      /*!< USBD reset is one of the SYS_ResetModule parameter */
#define ADC_RST     ((0x4<<24)|SYS_IPRST1_ADCRST_Pos)       /*!< ADC reset is one of the SYS_ResetModule parameter */
#define DAC_RST     ((0x4<<24)|SYS_IPRST1_DACRST_Pos)       /*!< DAC reset is one of the SYS_ResetModule parameter */
#define ACMP01_RST  ((0x4<<24)|SYS_IPRST1_ACMP01RST_Pos)    /*!< ACMP01 reset is one of the SYS_ResetModule parameter */
#define ACMP23_RST  ((0x4<<24)|SYS_IPRST1_ACMP23RST_Pos)    /*!< ACMP23 reset is one of the SYS_ResetModule parameter */
#define I3CS0_RST   ((0x4<<24)|SYS_IPRST1_I3CS0RST_Pos)     /*!< I3CS0 reset is one of the SYS_ResetModule parameter */
#define I3CS1_RST   ((0x4<<24)|SYS_IPRST1_I3CS1RST_Pos)     /*!< I3CS1 reset is one of the SYS_ResetModule parameter */
#define SPDH_RST    ((0x4<<24)|SYS_IPRST1_SPDHRST_Pos)      /*!< SPDH reset is one of the SYS_ResetModule parameter */

#define LLSI0_RST   ((0x8<<24)|SYS_IPRST2_LLSI0RST_Pos)     /*!< LLSI0 reset is one of the SYS_ResetModule parameter */
#define LLSI1_RST   ((0x8<<24)|SYS_IPRST2_LLSI1RST_Pos)     /*!< LLSI1 reset is one of the SYS_ResetModule parameter */
#define LLSI2_RST   ((0x8<<24)|SYS_IPRST2_LLSI2RST_Pos)     /*!< LLSI2 reset is one of the SYS_ResetModule parameter */
#define LLSI3_RST   ((0x8<<24)|SYS_IPRST2_LLSI3RST_Pos)     /*!< LLSI3 reset is one of the SYS_ResetModule parameter */
#define LLSI4_RST   ((0x8<<24)|SYS_IPRST2_LLSI4RST_Pos)     /*!< LLSI4 reset is one of the SYS_ResetModule parameter */
#define LLSI5_RST   ((0x8<<24)|SYS_IPRST2_LLSI5RST_Pos)     /*!< LLSI5 reset is one of the SYS_ResetModule parameter */
#define TS_RST      ((0x8<<24)|SYS_IPRST2_TSRST_Pos)        /*!< TS reset is one of the SYS_ResetModule parameter */


/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BOD_RST_EN           (1UL<<SYS_BODCTL_BODRSTEN_Pos)    /*!< Brown-out Reset Enable */
#define SYS_BODCTL_BOD_INTERRUPT_EN     (0UL<<SYS_BODCTL_BODRSTEN_Pos)    /*!< Brown-out Interrupt Enable */
#define SYS_BODCTL_BODVL_4_5V           (3UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 4.5V */
#define SYS_BODCTL_BODVL_3_7V           (2UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 3.7V */
#define SYS_BODCTL_BODVL_2_7V           (1UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCTL_BODVL_2_2V           (0UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.2V */

#define SYS_BODCTL_LVRDGSEL_0CLK       (0x0UL<<SYS_BODCTL_LVRDGSEL_Pos)   /*! LVR Output De-glitch Time Without de-glitch function. */
#define SYS_BODCTL_LVRDGSEL_4CLK       (0x1UL<<SYS_BODCTL_LVRDGSEL_Pos)   /*! LVR Output De-glitch Time is selected 4 HCLK clock */
#define SYS_BODCTL_LVRDGSEL_8CLK       (0x2UL<<SYS_BODCTL_LVRDGSEL_Pos)   /*! LVR Output De-glitch Time is selected 8 HCLK clock */
#define SYS_BODCTL_LVRDGSEL_16CLK      (0x3UL<<SYS_BODCTL_LVRDGSEL_Pos)   /*! LVR Output De-glitch Time is selected 16 HCLK clock */
#define SYS_BODCTL_LVRDGSEL_32CLK      (0x4UL<<SYS_BODCTL_LVRDGSEL_Pos)   /*! LVR Output De-glitch Time is selected 32 HCLK clock */
#define SYS_BODCTL_LVRDGSEL_64CLK      (0x5UL<<SYS_BODCTL_LVRDGSEL_Pos)   /*! LVR Output De-glitch Time is selected 64 HCLK clock */
#define SYS_BODCTL_LVRDGSEL_128CLK     (0x6UL<<SYS_BODCTL_LVRDGSEL_Pos)   /*! LVR Output De-glitch Time is selected 128 HCLK clock */
#define SYS_BODCTL_LVRDGSEL_256CLK     (0x7UL<<SYS_BODCTL_LVRDGSEL_Pos)   /*! LVR Output De-glitch Time is selected 256 HCLK clock */

#define SYS_BODCTL_BODDGSEL_0CLK       (0x0UL<<SYS_BODCTL_BODDGSEL_Pos)   /*! BOD Output De-glitch Time is sampled by LIRC clock. */
#define SYS_BODCTL_BODDGSEL_4CLK       (0x1UL<<SYS_BODCTL_BODDGSEL_Pos)   /*! BOD Output De-glitch Time is selected 4 HCLK clock */
#define SYS_BODCTL_BODDGSEL_8CLK       (0x2UL<<SYS_BODCTL_BODDGSEL_Pos)   /*! BOD Output De-glitch Time is selected 8 HCLK clock */
#define SYS_BODCTL_BODDGSEL_16CLK      (0x3UL<<SYS_BODCTL_BODDGSEL_Pos)   /*! BOD Output De-glitch Time is selected 16 HCLK clock */
#define SYS_BODCTL_BODDGSEL_32CLK      (0x4UL<<SYS_BODCTL_BODDGSEL_Pos)   /*! BOD Output De-glitch Time is selected 32 HCLK clock */
#define SYS_BODCTL_BODDGSEL_64CLK      (0x5UL<<SYS_BODCTL_BODDGSEL_Pos)   /*! BOD Output De-glitch Time is selected 64 HCLK clock */
#define SYS_BODCTL_BODDGSEL_128CLK     (0x6UL<<SYS_BODCTL_BODDGSEL_Pos)   /*! BOD Output De-glitch Time is selected 128 HCLK clock */
#define SYS_BODCTL_BODDGSEL_256CLK     (0x7UL<<SYS_BODCTL_BODDGSEL_Pos)   /*! BOD Output De-glitch Time is selected 256 HCLK clock */

#define SYS_BODCTL_VDETPINSEL_PIN0     (0UL<<SYS_BODCTL_VDETPINSEL_Pos)   /*!< VDET input voltage is from VDET_P0 (PB.0) */
#define SYS_BODCTL_VDETPINSEL_PIN1     (1UL<<SYS_BODCTL_VDETPINSEL_Pos)   /*!< VDET input voltage is from VDET_P1 (PB.1) */

#define SYS_BODCTL_VDETDGSEL_0CLK      (0x0UL<<SYS_BODCTL_VDETDGSEL_Pos)  /*! VDET Output De-glitch Time is sampled by VDET clock. */
#define SYS_BODCTL_VDETDGSEL_16CLK     (0x1UL<<SYS_BODCTL_VDETDGSEL_Pos)  /*! VDET Output De-glitch Time is selected 16 HCLK clock */
#define SYS_BODCTL_VDETDGSEL_32CLK     (0x2UL<<SYS_BODCTL_VDETDGSEL_Pos)  /*! VDET Output De-glitch Time is selected 32 HCLK clock */
#define SYS_BODCTL_VDETDGSEL_64CLK     (0x3UL<<SYS_BODCTL_VDETDGSEL_Pos)  /*! VDET Output De-glitch Time is selected 64 HCLK clock */
#define SYS_BODCTL_VDETDGSEL_128CLK    (0x4UL<<SYS_BODCTL_VDETDGSEL_Pos)  /*! VDET Output De-glitch Time is selected 128 HCLK clock */
#define SYS_BODCTL_VDETDGSEL_256CLK    (0x5UL<<SYS_BODCTL_VDETDGSEL_Pos)  /*! VDET Output De-glitch Time is selected 256 HCLK clock */
#define SYS_BODCTL_VDETDGSEL_512CLK    (0x6UL<<SYS_BODCTL_VDETDGSEL_Pos)  /*! VDET Output De-glitch Time is selected 512 HCLK clock */
#define SYS_BODCTL_VDETDGSEL_1024CLK   (0x7UL<<SYS_BODCTL_VDETDGSEL_Pos)  /*! VDET Output De-glitch Time is selected 1024 HCLK clock */


/*---------------------------------------------------------------------------------------------------------*/
/*  VREFCTL constant definitions. (Write-Protection Register)                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_VREFCTL_VREF_PIN           (0x0UL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = Vref pin */
#define SYS_VREFCTL_VREF_2_048V        (0x2UL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 2.048V */
#define SYS_VREFCTL_VREF_2_56V         (0x6UL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 2.56V */
#define SYS_VREFCTL_VREF_3_072V        (0xAUL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 3.072V */
#define SYS_VREFCTL_VREF_4_096V        (0xEUL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 4.096V */


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

/* How to use below #define?

Example: If user want to set PA.0 as UART0_RXD and PA.1 as UART0_TXD in initial function,
         user can issue following command to achieve it.

         SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_UART0_RXD;
         SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA1MFP_Msk)) | SYS_GPA_MFPL_PA1MFP_UART0_TXD;
*/

/* PA.0 MFP */
#define SYS_GPA_MFPL_PA0MFP_GPIO         (0x00UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for GPIO        */
#define SYS_GPA_MFPL_PA0MFP_SPI_MOSI_MUX (0x02UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for SPI_MOSI_MUX*/
#define SYS_GPA_MFPL_PA0MFP_SPI0_MOSI    (0x04UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for SPI0_MOSI   */
#define SYS_GPA_MFPL_PA0MFP_SPI2_MOSI    (0x05UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for SPI2_MOSI   */
#define SYS_GPA_MFPL_PA0MFP_I3CS0_SDA    (0x06UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for I3CS0_SDA   */
#define SYS_GPA_MFPL_PA0MFP_UART0_RXD    (0x07UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for UART0_RXD   */
#define SYS_GPA_MFPL_PA0MFP_UART1_nRTS   (0x08UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for UART1_nRTS  */
#define SYS_GPA_MFPL_PA0MFP_I2C2_SDA     (0x09UL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for I2C2_SDA    */
#define SYS_GPA_MFPL_PA0MFP_BPWM0_CH0    (0x0cUL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for BPWM0_CH0   */
#define SYS_GPA_MFPL_PA0MFP_BPWM2_CH5    (0x0dUL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for BPWM2_CH5   */
#define SYS_GPA_MFPL_PA0MFP_ACMP3_O      (0x0eUL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for ACMP3_O     */
#define SYS_GPA_MFPL_PA0MFP_DAC0_ST      (0x0fUL<<SYS_GPA_MFPL_PA0MFP_Pos) /*!< GPA_MFPL PA0 setting for DAC0_ST     */

/* PA.1 MFP */
#define SYS_GPA_MFPL_PA1MFP_GPIO         (0x00UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for GPIO        */
#define SYS_GPA_MFPL_PA1MFP_SPI_MISO_MUX (0x02UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for SPI_MISO_MUX*/
#define SYS_GPA_MFPL_PA1MFP_SPI0_MISO    (0x04UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for SPI0_MISO   */
#define SYS_GPA_MFPL_PA1MFP_SPI2_MISO    (0x05UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for SPI2_MISO   */
#define SYS_GPA_MFPL_PA1MFP_I3CS0_SCL    (0x06UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for I3CS0_SCL   */
#define SYS_GPA_MFPL_PA1MFP_UART0_TXD    (0x07UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for UART0_TXD   */
#define SYS_GPA_MFPL_PA1MFP_UART1_nCTS   (0x08UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for UART1_nCTS  */
#define SYS_GPA_MFPL_PA1MFP_I2C2_SCL     (0x09UL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for I2C2_SCL    */
#define SYS_GPA_MFPL_PA1MFP_BPWM0_CH1    (0x0cUL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for BPWM0_CH1   */
#define SYS_GPA_MFPL_PA1MFP_BPWM2_CH4    (0x0dUL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for BPWM2_CH4   */
#define SYS_GPA_MFPL_PA1MFP_ACMP2_O      (0x0fUL<<SYS_GPA_MFPL_PA1MFP_Pos) /*!< GPA_MFPL PA1 setting for ACMP2_O     */

/* PA.2 MFP */
#define SYS_GPA_MFPL_PA2MFP_GPIO         (0x00UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for GPIO        */
#define SYS_GPA_MFPL_PA2MFP_SPI_CLK_MUX  (0x02UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for SPI_CLK_MUX */
#define SYS_GPA_MFPL_PA2MFP_SPI0_CLK     (0x04UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for SPI0_CLK    */
#define SYS_GPA_MFPL_PA2MFP_SPI2_CLK     (0x05UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for SPI2_CLK    */
#define SYS_GPA_MFPL_PA2MFP_I3CS1_SDA    (0x06UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for I3CS1_SDA   */
#define SYS_GPA_MFPL_PA2MFP_I2C0_SMBSUS  (0x07UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for I2C0_SMBSUS */
#define SYS_GPA_MFPL_PA2MFP_UART1_RXD    (0x08UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for UART1_RXD   */
#define SYS_GPA_MFPL_PA2MFP_I2C1_SDA     (0x09UL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for I2C1_SDA    */
#define SYS_GPA_MFPL_PA2MFP_LLSI5_OUT    (0x0aUL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for LLSI5_OUT   */
#define SYS_GPA_MFPL_PA2MFP_BPWM0_CH2    (0x0cUL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for BPWM0_CH2   */
#define SYS_GPA_MFPL_PA2MFP_BPWM2_CH3    (0x0dUL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for BPWM2_CH3   */
#define SYS_GPA_MFPL_PA2MFP_ACMP3_WLAT   (0x0fUL<<SYS_GPA_MFPL_PA2MFP_Pos) /*!< GPA_MFPL PA2 setting for ACMP3_WLAT  */

/* PA.3 MFP */
#define SYS_GPA_MFPL_PA3MFP_GPIO         (0x00UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for GPIO        */
#define SYS_GPA_MFPL_PA3MFP_SPI_SS_MUX   (0x02UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for SPI_SS_MUX  */
#define SYS_GPA_MFPL_PA3MFP_SPI0_SS      (0x04UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for SPI0_SS     */
#define SYS_GPA_MFPL_PA3MFP_SPI2_SS      (0x05UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for SPI2_SS     */
#define SYS_GPA_MFPL_PA3MFP_I3CS1_SCL    (0x06UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for I3CS1_SCL   */
#define SYS_GPA_MFPL_PA3MFP_I2C0_SMBAL   (0x07UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for I2C0_SMBAL  */
#define SYS_GPA_MFPL_PA3MFP_UART1_TXD    (0x08UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for UART1_TXD   */
#define SYS_GPA_MFPL_PA3MFP_I2C1_SCL     (0x09UL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for I2C1_SCL    */
#define SYS_GPA_MFPL_PA3MFP_LLSI4_OUT    (0x0aUL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for LLSI4_OUT   */
#define SYS_GPA_MFPL_PA3MFP_BPWM0_CH3    (0x0cUL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for BPWM0_CH3   */
#define SYS_GPA_MFPL_PA3MFP_BPWM2_CH2    (0x0dUL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for BPWM2_CH2   */
#define SYS_GPA_MFPL_PA3MFP_CLKO         (0x0eUL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for CLKO        */
#define SYS_GPA_MFPL_PA3MFP_ACMP2_WLAT   (0x0fUL<<SYS_GPA_MFPL_PA3MFP_Pos) /*!< GPA_MFPL PA3 setting for ACMP2_WLAT  */

/* PA.4 MFP */
#define SYS_GPA_MFPL_PA4MFP_GPIO         (0x00UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for GPIO        */
#define SYS_GPA_MFPL_PA4MFP_SPI0_I2SMCLK (0x04UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for SPI0_I2SMCLK*/
#define SYS_GPA_MFPL_PA4MFP_I3CS0_SDA    (0x06UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for I3CS0_SDA   */
#define SYS_GPA_MFPL_PA4MFP_UART0_nRTS   (0x07UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for UART0_nRTS  */
#define SYS_GPA_MFPL_PA4MFP_UART0_RXD    (0x08UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for UART0_RXD   */
#define SYS_GPA_MFPL_PA4MFP_I2C0_SDA     (0x09UL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for I2C0_SDA    */
#define SYS_GPA_MFPL_PA4MFP_BPWM0_CH4    (0x0cUL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for BPWM0_CH4   */
#define SYS_GPA_MFPL_PA4MFP_BPWM2_CH1    (0x0dUL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for BPWM2_CH1   */
#define SYS_GPA_MFPL_PA4MFP_DAC3_ST      (0x0fUL<<SYS_GPA_MFPL_PA4MFP_Pos) /*!< GPA_MFPL PA4 setting for DAC3_ST     */

/* PA.5 MFP */
#define SYS_GPA_MFPL_PA5MFP_GPIO         (0x00UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for GPIO        */
#define SYS_GPA_MFPL_PA5MFP_SPI1_I2SMCLK (0x02UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for SPI1_I2SMCLK*/
#define SYS_GPA_MFPL_PA5MFP_SPI0_I2SMCLK (0x04UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for SPI0_I2SMCLK*/
#define SYS_GPA_MFPL_PA5MFP_SPI2_I2SMCLK (0x05UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for SPI2_I2SMCLK*/
#define SYS_GPA_MFPL_PA5MFP_I3CS0_SCL    (0x06UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for I3CS0_SCL   */
#define SYS_GPA_MFPL_PA5MFP_UART0_nCTS   (0x07UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for UART0_nCTS  */
#define SYS_GPA_MFPL_PA5MFP_UART0_TXD    (0x08UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for UART0_TXD   */
#define SYS_GPA_MFPL_PA5MFP_I2C0_SCL     (0x09UL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for I2C0_SCL    */
#define SYS_GPA_MFPL_PA5MFP_BPWM0_CH5    (0x0cUL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for BPWM0_CH5   */
#define SYS_GPA_MFPL_PA5MFP_BPWM2_CH0    (0x0dUL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for BPWM2_CH0   */
#define SYS_GPA_MFPL_PA5MFP_DAC2_ST      (0x0fUL<<SYS_GPA_MFPL_PA5MFP_Pos) /*!< GPA_MFPL PA5 setting for DAC2_ST     */

/* PA.6 MFP */
#define SYS_GPA_MFPL_PA6MFP_GPIO         (0x00UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for GPIO        */
#define SYS_GPA_MFPL_PA6MFP_SPI1_SS      (0x02UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for SPI1_SS     */
#define SYS_GPA_MFPL_PA6MFP_UART0_RXD    (0x07UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for UART0_RXD   */
#define SYS_GPA_MFPL_PA6MFP_I2C1_SDA     (0x08UL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for I2C1_SDA    */
#define SYS_GPA_MFPL_PA6MFP_BPWM3_CH5    (0x0bUL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for BPWM3_CH5   */
#define SYS_GPA_MFPL_PA6MFP_BPWM1_CH3    (0x0cUL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for BPWM1_CH3   */
#define SYS_GPA_MFPL_PA6MFP_ACMP1_WLAT   (0x0dUL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for ACMP1_WLAT  */
#define SYS_GPA_MFPL_PA6MFP_TM3          (0x0eUL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for TM3         */
#define SYS_GPA_MFPL_PA6MFP_INT0         (0x0fUL<<SYS_GPA_MFPL_PA6MFP_Pos) /*!< GPA_MFPL PA6 setting for INT0        */

/* PA.7 MFP */
#define SYS_GPA_MFPL_PA7MFP_GPIO         (0x00UL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for GPIO        */
#define SYS_GPA_MFPL_PA7MFP_SPI1_CLK     (0x02UL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for SPI1_CLK    */
#define SYS_GPA_MFPL_PA7MFP_UART0_TXD    (0x07UL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for UART0_TXD   */
#define SYS_GPA_MFPL_PA7MFP_I2C1_SCL     (0x08UL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for I2C1_SCL    */
#define SYS_GPA_MFPL_PA7MFP_BPWM3_CH4    (0x0bUL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for BPWM3_CH4   */
#define SYS_GPA_MFPL_PA7MFP_BPWM1_CH2    (0x0cUL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for BPWM1_CH2   */
#define SYS_GPA_MFPL_PA7MFP_ACMP0_WLAT   (0x0dUL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for ACMP0_WLAT  */
#define SYS_GPA_MFPL_PA7MFP_TM2          (0x0eUL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for TM2         */
#define SYS_GPA_MFPL_PA7MFP_INT1         (0x0fUL<<SYS_GPA_MFPL_PA7MFP_Pos) /*!< GPA_MFPL PA7 setting for INT1        */

/* PA.8 MFP */
#define SYS_GPA_MFPH_PA8MFP_GPIO         (0x00UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for GPIO        */
#define SYS_GPA_MFPH_PA8MFP_SPI1_SS      (0x03UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for SPI1_SS     */
#define SYS_GPA_MFPH_PA8MFP_SPI2_MOSI    (0x04UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for SPI2_MOSI   */
#define SYS_GPA_MFPH_PA8MFP_UART1_RXD    (0x07UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for UART1_RXD   */
#define SYS_GPA_MFPH_PA8MFP_BPWM0_CH3    (0x09UL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for BPWM0_CH3   */
#define SYS_GPA_MFPH_PA8MFP_I2C0_SCL     (0x0aUL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for I2C0_SCL    */
#define SYS_GPA_MFPH_PA8MFP_TM3_EXT      (0x0dUL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for TM3_EXT     */
#define SYS_GPA_MFPH_PA8MFP_DAC3_ST      (0x0eUL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for DAC3_ST     */
#define SYS_GPA_MFPH_PA8MFP_INT4         (0x0fUL<<SYS_GPA_MFPH_PA8MFP_Pos) /*!< GPA_MFPH PA8 setting for INT4        */

/* PA.9 MFP */
#define SYS_GPA_MFPH_PA9MFP_GPIO         (0x00UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for GPIO        */
#define SYS_GPA_MFPH_PA9MFP_SPI1_CLK     (0x03UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for SPI1_CLK    */
#define SYS_GPA_MFPH_PA9MFP_SPI2_MISO    (0x04UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for SPI2_MISO   */
#define SYS_GPA_MFPH_PA9MFP_UART1_TXD    (0x07UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for UART1_TXD   */
#define SYS_GPA_MFPH_PA9MFP_BPWM0_CH2    (0x09UL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for BPWM0_CH2   */
#define SYS_GPA_MFPH_PA9MFP_I2C1_SDA     (0x0aUL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for I2C1_SDA    */
#define SYS_GPA_MFPH_PA9MFP_TM2_EXT      (0x0dUL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for TM2_EXT     */
#define SYS_GPA_MFPH_PA9MFP_DAC2_ST      (0x0eUL<<SYS_GPA_MFPH_PA9MFP_Pos) /*!< GPA_MFPH PA9 setting for DAC2_ST     */

/* PA.10 MFP */
#define SYS_GPA_MFPH_PA10MFP_GPIO        (0x00UL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for GPIO       */
#define SYS_GPA_MFPH_PA10MFP_SPI1_MISO   (0x03UL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for SPI1_MISO  */
#define SYS_GPA_MFPH_PA10MFP_SPI2_CLK    (0x04UL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for SPI2_CLK   */
#define SYS_GPA_MFPH_PA10MFP_I2C2_SDA    (0x07UL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for I2C2_SDA   */
#define SYS_GPA_MFPH_PA10MFP_BPWM0_CH1   (0x09UL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for BPWM0_CH1  */
#define SYS_GPA_MFPH_PA10MFP_I2C1_SCL    (0x0aUL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for I2C1_SCL   */
#define SYS_GPA_MFPH_PA10MFP_TM1_EXT     (0x0dUL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for TM1_EXT    */
#define SYS_GPA_MFPH_PA10MFP_DAC0_ST     (0x0eUL<<SYS_GPA_MFPH_PA10MFP_Pos)/*!< GPA_MFPH PA10 setting for DAC0_ST    */

/* PA.11 MFP */
#define SYS_GPA_MFPH_PA11MFP_GPIO        (0x00UL<<SYS_GPA_MFPH_PA11MFP_Pos)/*!< GPA_MFPH PA11 setting for GPIO       */
#define SYS_GPA_MFPH_PA11MFP_SPI1_MOSI   (0x03UL<<SYS_GPA_MFPH_PA11MFP_Pos)/*!< GPA_MFPH PA11 setting for SPI1_MOSI  */
#define SYS_GPA_MFPH_PA11MFP_SPI2_SS     (0x04UL<<SYS_GPA_MFPH_PA11MFP_Pos)/*!< GPA_MFPH PA11 setting for SPI2_SS    */
#define SYS_GPA_MFPH_PA11MFP_I2C2_SCL    (0x07UL<<SYS_GPA_MFPH_PA11MFP_Pos)/*!< GPA_MFPH PA11 setting for I2C2_SCL   */
#define SYS_GPA_MFPH_PA11MFP_BPWM0_CH0   (0x09UL<<SYS_GPA_MFPH_PA11MFP_Pos)/*!< GPA_MFPH PA11 setting for BPWM0_CH0  */
#define SYS_GPA_MFPH_PA11MFP_TM0_EXT     (0x0dUL<<SYS_GPA_MFPH_PA11MFP_Pos)/*!< GPA_MFPH PA11 setting for TM0_EXT    */
#define SYS_GPA_MFPH_PA11MFP_DAC1_ST     (0x0eUL<<SYS_GPA_MFPH_PA11MFP_Pos)/*!< GPA_MFPH PA11 setting for DAC1_ST    */

/* PB.0 MFP */
#define SYS_GPB_MFPL_PB0MFP_GPIO         (0x00UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for GPIO        */
#define SYS_GPB_MFPL_PB0MFP_ADC0_CH0     (0x01UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for ADC0_CH0    */
#define SYS_GPB_MFPL_PB0MFP_VDET_P0      (0x01UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for VDET_P0     */
#define SYS_GPB_MFPL_PB0MFP_SPI0_SS      (0x03UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for SPI0_SS     */
#define SYS_GPB_MFPL_PB0MFP_SPI2_I2SMCLK (0x04UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for SPI2_I2SMCLK*/
#define SYS_GPB_MFPL_PB0MFP_UART2_RXD    (0x07UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for UART2_RXD   */
#define SYS_GPB_MFPL_PB0MFP_SPI0_I2SMCLK (0x08UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for SPI0_I2SMCLK*/
#define SYS_GPB_MFPL_PB0MFP_I2C1_SDA     (0x09UL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for I2C1_SDA    */
#define SYS_GPB_MFPL_PB0MFP_BPWM2_CH5    (0x0bUL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for BPWM2_CH5   */
#define SYS_GPB_MFPL_PB0MFP_BPWM3_CH5    (0x0cUL<<SYS_GPB_MFPL_PB0MFP_Pos) /*!< GPB_MFPL PB0 setting for BPWM3_CH5   */

/* PB.1 MFP */
#define SYS_GPB_MFPL_PB1MFP_GPIO         (0x00UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for GPIO        */
#define SYS_GPB_MFPL_PB1MFP_ADC0_CH1     (0x01UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for ADC0_CH1    */
#define SYS_GPB_MFPL_PB1MFP_VDET_P1      (0x01UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for VDET_P1     */
#define SYS_GPB_MFPL_PB1MFP_SPDH_HSA     (0x01UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for SPDH_HSA    */
#define SYS_GPB_MFPL_PB1MFP_SPI1_I2SMCLK (0x02UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for SPI1_I2SMCLK*/
#define SYS_GPB_MFPL_PB1MFP_UART2_TXD    (0x07UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for UART2_TXD   */
#define SYS_GPB_MFPL_PB1MFP_I2C1_SCL     (0x09UL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for I2C1_SCL    */
#define SYS_GPB_MFPL_PB1MFP_BPWM2_CH4    (0x0bUL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for BPWM2_CH4   */
#define SYS_GPB_MFPL_PB1MFP_BPWM3_CH4    (0x0cUL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for BPWM3_CH4   */
#define SYS_GPB_MFPL_PB1MFP_TM3_EXT      (0x0dUL<<SYS_GPB_MFPL_PB1MFP_Pos) /*!< GPB_MFPL PB1 setting for TM3_EXT     */

/* PB.2 MFP */
#define SYS_GPB_MFPL_PB2MFP_GPIO         (0x00UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for GPIO        */
#define SYS_GPB_MFPL_PB2MFP_ADC0_CH2     (0x01UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for ADC0_CH2    */
#define SYS_GPB_MFPL_PB2MFP_ACMP0_P0     (0x01UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for ACMP0_P0    */
#define SYS_GPB_MFPL_PB2MFP_SPI1_SS      (0x02UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for SPI1_SS     */
#define SYS_GPB_MFPL_PB2MFP_I2C1_SDA     (0x04UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for I2C1_SDA    */
#define SYS_GPB_MFPL_PB2MFP_UART1_RXD    (0x06UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for UART1_RXD   */
#define SYS_GPB_MFPL_PB2MFP_I2C2_SDA     (0x07UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for I2C2_SDA    */
#define SYS_GPB_MFPL_PB2MFP_SPI0_I2SMCLK (0x08UL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for SPI0_I2SMCLK*/
#define SYS_GPB_MFPL_PB2MFP_BPWM2_CH3    (0x0bUL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for BPWM2_CH3   */
#define SYS_GPB_MFPL_PB2MFP_TM3          (0x0eUL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for TM3         */
#define SYS_GPB_MFPL_PB2MFP_INT3         (0x0fUL<<SYS_GPB_MFPL_PB2MFP_Pos) /*!< GPB_MFPL PB2 setting for INT3        */

/* PB.3 MFP */
#define SYS_GPB_MFPL_PB3MFP_GPIO         (0x00UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for GPIO        */
#define SYS_GPB_MFPL_PB3MFP_ADC0_CH3     (0x01UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for ADC0_CH3    */
#define SYS_GPB_MFPL_PB3MFP_ACMP0_P1     (0x01UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for ACMP0_P1    */
#define SYS_GPB_MFPL_PB3MFP_ACMP1_P0     (0x01UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for ACMP1_P0    */
#define SYS_GPB_MFPL_PB3MFP_SPI1_CLK     (0x02UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for SPI1_CLK    */
#define SYS_GPB_MFPL_PB3MFP_I2C1_SCL     (0x04UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for I2C1_SCL    */
#define SYS_GPB_MFPL_PB3MFP_UART1_TXD    (0x06UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for UART1_TXD   */
#define SYS_GPB_MFPL_PB3MFP_I2C2_SCL     (0x07UL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for I2C2_SCL    */
#define SYS_GPB_MFPL_PB3MFP_BPWM2_CH2    (0x0bUL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for BPWM2_CH2   */
#define SYS_GPB_MFPL_PB3MFP_TM2          (0x0eUL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for TM2         */
#define SYS_GPB_MFPL_PB3MFP_INT2         (0x0fUL<<SYS_GPB_MFPL_PB3MFP_Pos) /*!< GPB_MFPL PB3 setting for INT2        */

/* PB.4 MFP */
#define SYS_GPB_MFPL_PB4MFP_GPIO         (0x00UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for GPIO        */
#define SYS_GPB_MFPL_PB4MFP_ADC0_CH4     (0x01UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for ADC0_CH4    */
#define SYS_GPB_MFPL_PB4MFP_ACMP0_N      (0x01UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for ACMP0_N     */
#define SYS_GPB_MFPL_PB4MFP_ACMP1_P1     (0x01UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for ACMP1_P1    */
#define SYS_GPB_MFPL_PB4MFP_ACMP2_P0     (0x01UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for ACMP2_P0    */
#define SYS_GPB_MFPL_PB4MFP_SPI1_MOSI    (0x02UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for SPI1_MOSI   */
#define SYS_GPB_MFPL_PB4MFP_I2C0_SDA     (0x06UL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for I2C0_SDA    */
#define SYS_GPB_MFPL_PB4MFP_BPWM2_CH1    (0x0bUL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for BPWM2_CH1   */
#define SYS_GPB_MFPL_PB4MFP_LLSI5_OUT    (0x0cUL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for LLSI5_OUT   */
#define SYS_GPB_MFPL_PB4MFP_UART2_RXD    (0x0dUL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for UART2_RXD   */
#define SYS_GPB_MFPL_PB4MFP_TM1          (0x0eUL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for TM1         */
#define SYS_GPB_MFPL_PB4MFP_INT1         (0x0fUL<<SYS_GPB_MFPL_PB4MFP_Pos) /*!< GPB_MFPL PB4 setting for INT1        */

/* PB.5 MFP */
#define SYS_GPB_MFPL_PB5MFP_GPIO         (0x00UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for GPIO        */
#define SYS_GPB_MFPL_PB5MFP_ADC0_CH5     (0x01UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for ADC0_CH5    */
#define SYS_GPB_MFPL_PB5MFP_ACMP1_N      (0x01UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for ACMP1_N     */
#define SYS_GPB_MFPL_PB5MFP_ACMP2_P1     (0x01UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for ACMP2_P1    */
#define SYS_GPB_MFPL_PB5MFP_ACMP3_P0     (0x01UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for ACMP3_P0    */
#define SYS_GPB_MFPL_PB5MFP_SPI1_MISO    (0x02UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for SPI1_MISO   */
#define SYS_GPB_MFPL_PB5MFP_I2C0_SCL     (0x06UL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for I2C0_SCL    */
#define SYS_GPB_MFPL_PB5MFP_BPWM2_CH0    (0x0bUL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for BPWM2_CH0   */
#define SYS_GPB_MFPL_PB5MFP_LLSI4_OUT    (0x0cUL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for LLSI4_OUT   */
#define SYS_GPB_MFPL_PB5MFP_UART2_TXD    (0x0dUL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for UART2_TXD   */
#define SYS_GPB_MFPL_PB5MFP_TM0          (0x0eUL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for TM0         */
#define SYS_GPB_MFPL_PB5MFP_INT0         (0x0fUL<<SYS_GPB_MFPL_PB5MFP_Pos) /*!< GPB_MFPL PB5 setting for INT0        */

/* PB.6 MFP */
#define SYS_GPB_MFPL_PB6MFP_GPIO         (0x00UL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for GPIO        */
#define SYS_GPB_MFPL_PB6MFP_ADC0_CH6     (0x01UL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for ADC0_CH6    */
#define SYS_GPB_MFPL_PB6MFP_ACMP2_N      (0x01UL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for ACMP2_N     */
#define SYS_GPB_MFPL_PB6MFP_ACMP3_P1     (0x01UL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for ACMP3_P1    */
#define SYS_GPB_MFPL_PB6MFP_UART1_RXD    (0x06UL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for UART1_RXD   */
#define SYS_GPB_MFPL_PB6MFP_BPWM1_CH5    (0x0aUL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for BPWM1_CH5   */
#define SYS_GPB_MFPL_PB6MFP_BPWM3_CH5    (0x0cUL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for BPWM3_CH5   */
#define SYS_GPB_MFPL_PB6MFP_INT4         (0x0dUL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for INT4        */
#define SYS_GPB_MFPL_PB6MFP_ACMP1_O      (0x0fUL<<SYS_GPB_MFPL_PB6MFP_Pos) /*!< GPB_MFPL PB6 setting for ACMP1_O     */

/* PB.7 MFP */
#define SYS_GPB_MFPL_PB7MFP_GPIO         (0x00UL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for GPIO        */
#define SYS_GPB_MFPL_PB7MFP_ADC0_CH7     (0x01UL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for ADC0_CH7    */
#define SYS_GPB_MFPL_PB7MFP_ACMP3_N      (0x01UL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for ACMP3_N     */
#define SYS_GPB_MFPL_PB7MFP_UART1_TXD    (0x06UL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for UART1_TXD   */
#define SYS_GPB_MFPL_PB7MFP_BPWM1_CH4    (0x0aUL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for BPWM1_CH4   */
#define SYS_GPB_MFPL_PB7MFP_BPWM3_CH4    (0x0cUL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for BPWM3_CH4   */
#define SYS_GPB_MFPL_PB7MFP_INT5         (0x0dUL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for INT5        */
#define SYS_GPB_MFPL_PB7MFP_ACMP0_O      (0x0fUL<<SYS_GPB_MFPL_PB7MFP_Pos) /*!< GPB_MFPL PB7 setting for ACMP0_O     */

/* PB.8 MFP */
#define SYS_GPB_MFPH_PB8MFP_GPIO         (0x00UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< GPB_MFPH PB8 setting for GPIO        */
#define SYS_GPB_MFPH_PB8MFP_ADC0_CH8     (0x01UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< GPB_MFPH PB8 setting for ADC0_CH8    */
#define SYS_GPB_MFPH_PB8MFP_UART0_RXD    (0x05UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< GPB_MFPH PB8 setting for UART0_RXD   */
#define SYS_GPB_MFPH_PB8MFP_UART1_nRTS   (0x06UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< GPB_MFPH PB8 setting for UART1_nRTS  */
#define SYS_GPB_MFPH_PB8MFP_I2C1_SMBSUS  (0x07UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< GPB_MFPH PB8 setting for I2C1_SMBSUS */
#define SYS_GPB_MFPH_PB8MFP_I2C0_SDA     (0x09UL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< GPB_MFPH PB8 setting for I2C0_SDA    */
#define SYS_GPB_MFPH_PB8MFP_BPWM1_CH3    (0x0aUL<<SYS_GPB_MFPH_PB8MFP_Pos) /*!< GPB_MFPH PB8 setting for BPWM1_CH3   */

/* PB.9 MFP */
#define SYS_GPB_MFPH_PB9MFP_GPIO         (0x00UL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< GPB_MFPH PB9 setting for GPIO        */
#define SYS_GPB_MFPH_PB9MFP_ADC0_CH9     (0x01UL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< GPB_MFPH PB9 setting for ADC0_CH9    */
#define SYS_GPB_MFPH_PB9MFP_UART0_TXD    (0x05UL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< GPB_MFPH PB9 setting for UART0_TXD   */
#define SYS_GPB_MFPH_PB9MFP_UART1_nCTS   (0x06UL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< GPB_MFPH PB9 setting for UART1_nCTS  */
#define SYS_GPB_MFPH_PB9MFP_I2C1_SMBAL   (0x07UL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< GPB_MFPH PB9 setting for I2C1_SMBAL  */
#define SYS_GPB_MFPH_PB9MFP_I2C0_SCL     (0x09UL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< GPB_MFPH PB9 setting for I2C0_SCL    */
#define SYS_GPB_MFPH_PB9MFP_BPWM1_CH2    (0x0aUL<<SYS_GPB_MFPH_PB9MFP_Pos) /*!< GPB_MFPH PB9 setting for BPWM1_CH2   */

/* PB.10 MFP */
#define SYS_GPB_MFPH_PB10MFP_GPIO        (0x00UL<<SYS_GPB_MFPH_PB10MFP_Pos)/*!< GPB_MFPH PB10 setting for GPIO       */
#define SYS_GPB_MFPH_PB10MFP_ADC0_CH10   (0x01UL<<SYS_GPB_MFPH_PB10MFP_Pos)/*!< GPB_MFPH PB10 setting for ADC0_CH10  */
#define SYS_GPB_MFPH_PB10MFP_UART0_nRTS  (0x05UL<<SYS_GPB_MFPH_PB10MFP_Pos)/*!< GPB_MFPH PB10 setting for UART0_nRTS */
#define SYS_GPB_MFPH_PB10MFP_I2C1_SDA    (0x07UL<<SYS_GPB_MFPH_PB10MFP_Pos)/*!< GPB_MFPH PB10 setting for I2C1_SDA   */
#define SYS_GPB_MFPH_PB10MFP_BPWM1_CH1   (0x0aUL<<SYS_GPB_MFPH_PB10MFP_Pos)/*!< GPB_MFPH PB10 setting for BPWM1_CH1  */

/* PB.11 MFP */
#define SYS_GPB_MFPH_PB11MFP_GPIO        (0x00UL<<SYS_GPB_MFPH_PB11MFP_Pos)/*!< GPB_MFPH PB11 setting for GPIO       */
#define SYS_GPB_MFPH_PB11MFP_ADC0_CH11   (0x01UL<<SYS_GPB_MFPH_PB11MFP_Pos)/*!< GPB_MFPH PB11 setting for ADC0_CH11  */
#define SYS_GPB_MFPH_PB11MFP_UART0_nCTS  (0x05UL<<SYS_GPB_MFPH_PB11MFP_Pos)/*!< GPB_MFPH PB11 setting for UART0_nCTS */
#define SYS_GPB_MFPH_PB11MFP_I2C1_SCL    (0x07UL<<SYS_GPB_MFPH_PB11MFP_Pos)/*!< GPB_MFPH PB11 setting for I2C1_SCL   */
#define SYS_GPB_MFPH_PB11MFP_SPI0_I2SMCLK (0x09UL<<SYS_GPB_MFPH_PB11MFP_Pos)/*!< GPB_MFPH PB11 setting for SPI0_I2SMCLK*/
#define SYS_GPB_MFPH_PB11MFP_BPWM1_CH0   (0x0aUL<<SYS_GPB_MFPH_PB11MFP_Pos)/*!< GPB_MFPH PB11 setting for BPWM1_CH0  */

/* PB.12 MFP */
#define SYS_GPB_MFPH_PB12MFP_GPIO        (0x00UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for GPIO       */
#define SYS_GPB_MFPH_PB12MFP_ADC0_CH12   (0x01UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for ADC0_CH12  */
#define SYS_GPB_MFPH_PB12MFP_DAC0_OUT    (0x01UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for DAC0_OUT   */
#define SYS_GPB_MFPH_PB12MFP_ACMP0_P2    (0x01UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for ACMP0_P2   */
#define SYS_GPB_MFPH_PB12MFP_ACMP1_P2    (0x01UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for ACMP1_P2   */
#define SYS_GPB_MFPH_PB12MFP_SPI0_MOSI   (0x04UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for SPI0_MOSI  */
#define SYS_GPB_MFPH_PB12MFP_UART0_RXD   (0x06UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for UART0_RXD  */
#define SYS_GPB_MFPH_PB12MFP_I2C2_SDA    (0x08UL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for I2C2_SDA   */
#define SYS_GPB_MFPH_PB12MFP_LLSI3_OUT   (0x0aUL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for LLSI3_OUT  */
#define SYS_GPB_MFPH_PB12MFP_BPWM3_CH3   (0x0bUL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for BPWM3_CH3  */
#define SYS_GPB_MFPH_PB12MFP_TM3_EXT     (0x0dUL<<SYS_GPB_MFPH_PB12MFP_Pos)/*!< GPB_MFPH PB12 setting for TM3_EXT    */

/* PB.13 MFP */
#define SYS_GPB_MFPH_PB13MFP_GPIO        (0x00UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for GPIO       */
#define SYS_GPB_MFPH_PB13MFP_ADC0_CH13   (0x01UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for ADC0_CH13  */
#define SYS_GPB_MFPH_PB13MFP_DAC1_OUT    (0x01UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for DAC1_OUT   */
#define SYS_GPB_MFPH_PB13MFP_ACMP0_P3    (0x01UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for ACMP0_P3   */
#define SYS_GPB_MFPH_PB13MFP_ACMP1_P3    (0x01UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for ACMP1_P3   */
#define SYS_GPB_MFPH_PB13MFP_SPI0_MISO   (0x04UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for SPI0_MISO  */
#define SYS_GPB_MFPH_PB13MFP_UART0_TXD   (0x06UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for UART0_TXD  */
#define SYS_GPB_MFPH_PB13MFP_I2C2_SCL    (0x08UL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for I2C2_SCL   */
#define SYS_GPB_MFPH_PB13MFP_LLSI2_OUT   (0x0aUL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for LLSI2_OUT  */
#define SYS_GPB_MFPH_PB13MFP_BPWM3_CH2   (0x0bUL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for BPWM3_CH2  */
#define SYS_GPB_MFPH_PB13MFP_TM2_EXT     (0x0dUL<<SYS_GPB_MFPH_PB13MFP_Pos)/*!< GPB_MFPH PB13 setting for TM2_EXT    */

/* PB.14 MFP */
#define SYS_GPB_MFPH_PB14MFP_GPIO        (0x00UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for GPIO       */
#define SYS_GPB_MFPH_PB14MFP_ADC0_CH14   (0x01UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for ADC0_CH14  */
#define SYS_GPB_MFPH_PB14MFP_DAC2_OUT    (0x01UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for DAC2_OUT   */
#define SYS_GPB_MFPH_PB14MFP_ACMP2_P2    (0x01UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for ACMP2_P2   */
#define SYS_GPB_MFPH_PB14MFP_ACMP3_P2    (0x01UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for ACMP3_P2   */
#define SYS_GPB_MFPH_PB14MFP_SPI0_CLK    (0x04UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for SPI0_CLK   */
#define SYS_GPB_MFPH_PB14MFP_UART0_nRTS  (0x06UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for UART0_nRTS */
#define SYS_GPB_MFPH_PB14MFP_I2C2_SMBSUS (0x08UL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for I2C2_SMBSUS*/
#define SYS_GPB_MFPH_PB14MFP_LLSI1_OUT   (0x0aUL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for LLSI1_OUT  */
#define SYS_GPB_MFPH_PB14MFP_BPWM3_CH1   (0x0bUL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for BPWM3_CH1  */
#define SYS_GPB_MFPH_PB14MFP_TM1_EXT     (0x0dUL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for TM1_EXT    */
#define SYS_GPB_MFPH_PB14MFP_CLKO        (0x0eUL<<SYS_GPB_MFPH_PB14MFP_Pos)/*!< GPB_MFPH PB14 setting for CLKO       */

/* PB.15 MFP */
#define SYS_GPB_MFPH_PB15MFP_GPIO        (0x00UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for GPIO       */
#define SYS_GPB_MFPH_PB15MFP_ADC0_CH15   (0x01UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for ADC0_CH15  */
#define SYS_GPB_MFPH_PB15MFP_DAC3_OUT    (0x01UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for DAC3_OUT   */
#define SYS_GPB_MFPH_PB15MFP_ACMP2_P3    (0x01UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for ACMP2_P3   */
#define SYS_GPB_MFPH_PB15MFP_ACMP3_P3    (0x01UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for ACMP3_P3   */
#define SYS_GPB_MFPH_PB15MFP_SPI0_SS     (0x04UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for SPI0_SS    */
#define SYS_GPB_MFPH_PB15MFP_UART0_nCTS  (0x06UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for UART0_nCTS */
#define SYS_GPB_MFPH_PB15MFP_I2C2_SMBAL  (0x08UL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for I2C2_SMBAL */
#define SYS_GPB_MFPH_PB15MFP_LLSI0_OUT   (0x0aUL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for LLSI0_OUT  */
#define SYS_GPB_MFPH_PB15MFP_BPWM3_CH0   (0x0bUL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for BPWM3_CH0  */
#define SYS_GPB_MFPH_PB15MFP_TM0_EXT     (0x0dUL<<SYS_GPB_MFPH_PB15MFP_Pos)/*!< GPB_MFPH PB15 setting for TM0_EXT    */

/* PC.0 MFP */
#define SYS_GPC_MFPL_PC0MFP_GPIO         (0x00UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< GPC_MFPL PC0 setting for GPIO        */
#define SYS_GPC_MFPL_PC0MFP_SPI1_SS      (0x02UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< GPC_MFPL PC0 setting for SPI1_SS     */
#define SYS_GPC_MFPL_PC0MFP_I2C1_SDA     (0x07UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< GPC_MFPL PC0 setting for I2C1_SDA    */
#define SYS_GPC_MFPL_PC0MFP_UART2_RXD    (0x08UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< GPC_MFPL PC0 setting for UART2_RXD   */
#define SYS_GPC_MFPL_PC0MFP_I2C0_SDA     (0x09UL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< GPC_MFPL PC0 setting for I2C0_SDA    */
#define SYS_GPC_MFPL_PC0MFP_BPWM3_CH5    (0x0cUL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< GPC_MFPL PC0 setting for BPWM3_CH5   */
#define SYS_GPC_MFPL_PC0MFP_BPWM3_CH3    (0x0dUL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< GPC_MFPL PC0 setting for BPWM3_CH3   */
#define SYS_GPC_MFPL_PC0MFP_ACMP1_O      (0x0eUL<<SYS_GPC_MFPL_PC0MFP_Pos) /*!< GPC_MFPL PC0 setting for ACMP1_O     */

/* PC.1 MFP */
#define SYS_GPC_MFPL_PC1MFP_GPIO         (0x00UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for GPIO        */
#define SYS_GPC_MFPL_PC1MFP_SPI1_CLK     (0x02UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for SPI1_CLK    */
#define SYS_GPC_MFPL_PC1MFP_I2C1_SCL     (0x07UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for I2C1_SCL    */
#define SYS_GPC_MFPL_PC1MFP_UART2_TXD    (0x08UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for UART2_TXD   */
#define SYS_GPC_MFPL_PC1MFP_I2C0_SCL     (0x09UL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for I2C0_SCL    */
#define SYS_GPC_MFPL_PC1MFP_ADC0_ST      (0x0bUL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for ADC0_ST     */
#define SYS_GPC_MFPL_PC1MFP_BPWM3_CH4    (0x0cUL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for BPWM3_CH4   */
#define SYS_GPC_MFPL_PC1MFP_BPWM2_CH1    (0x0dUL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for BPWM2_CH1   */
#define SYS_GPC_MFPL_PC1MFP_ACMP0_O      (0x0eUL<<SYS_GPC_MFPL_PC1MFP_Pos) /*!< GPC_MFPL PC1 setting for ACMP0_O     */

/* PC.2 MFP */
#define SYS_GPC_MFPL_PC2MFP_GPIO         (0x00UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< GPC_MFPL PC2 setting for GPIO        */
#define SYS_GPC_MFPL_PC2MFP_SPI1_MOSI    (0x02UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< GPC_MFPL PC2 setting for SPI1_MOSI   */
#define SYS_GPC_MFPL_PC2MFP_I2C2_SMBSUS  (0x07UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< GPC_MFPL PC2 setting for I2C2_SMBSUS */
#define SYS_GPC_MFPL_PC2MFP_UART2_nCTS   (0x08UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< GPC_MFPL PC2 setting for UART2_nCTS  */
#define SYS_GPC_MFPL_PC2MFP_I2C0_SMBSUS  (0x09UL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< GPC_MFPL PC2 setting for I2C0_SMBSUS */
#define SYS_GPC_MFPL_PC2MFP_I2C0_SDA     (0x0aUL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< GPC_MFPL PC2 setting for I2C0_SDA    */
#define SYS_GPC_MFPL_PC2MFP_BPWM3_CH3    (0x0cUL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< GPC_MFPL PC2 setting for BPWM3_CH3   */
#define SYS_GPC_MFPL_PC2MFP_LLSI3_OUT    (0x0fUL<<SYS_GPC_MFPL_PC2MFP_Pos) /*!< GPC_MFPL PC2 setting for LLSI3_OUT   */

/* PC.3 MFP */
#define SYS_GPC_MFPL_PC3MFP_GPIO         (0x00UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< GPC_MFPL PC3 setting for GPIO        */
#define SYS_GPC_MFPL_PC3MFP_SPI1_MISO    (0x02UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< GPC_MFPL PC3 setting for SPI1_MISO   */
#define SYS_GPC_MFPL_PC3MFP_I2C2_SMBAL   (0x07UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< GPC_MFPL PC3 setting for I2C2_SMBAL  */
#define SYS_GPC_MFPL_PC3MFP_UART2_nRTS   (0x08UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< GPC_MFPL PC3 setting for UART2_nRTS  */
#define SYS_GPC_MFPL_PC3MFP_I2C0_SMBAL   (0x09UL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< GPC_MFPL PC3 setting for I2C0_SMBAL  */
#define SYS_GPC_MFPL_PC3MFP_I2C0_SCL     (0x0aUL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< GPC_MFPL PC3 setting for I2C0_SCL    */
#define SYS_GPC_MFPL_PC3MFP_BPWM3_CH2    (0x0cUL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< GPC_MFPL PC3 setting for BPWM3_CH2   */
#define SYS_GPC_MFPL_PC3MFP_LLSI2_OUT    (0x0fUL<<SYS_GPC_MFPL_PC3MFP_Pos) /*!< GPC_MFPL PC3 setting for LLSI2_OUT   */

/* PC.4 MFP */
#define SYS_GPC_MFPL_PC4MFP_GPIO         (0x00UL<<SYS_GPC_MFPL_PC4MFP_Pos) /*!< GPC_MFPL PC4 setting for GPIO        */
#define SYS_GPC_MFPL_PC4MFP_SPI1_I2SMCLK (0x02UL<<SYS_GPC_MFPL_PC4MFP_Pos) /*!< GPC_MFPL PC4 setting for SPI1_I2SMCLK*/
#define SYS_GPC_MFPL_PC4MFP_UART2_RXD    (0x08UL<<SYS_GPC_MFPL_PC4MFP_Pos) /*!< GPC_MFPL PC4 setting for UART2_RXD   */
#define SYS_GPC_MFPL_PC4MFP_I2C1_SDA     (0x09UL<<SYS_GPC_MFPL_PC4MFP_Pos) /*!< GPC_MFPL PC4 setting for I2C1_SDA    */
#define SYS_GPC_MFPL_PC4MFP_BPWM3_CH1    (0x0cUL<<SYS_GPC_MFPL_PC4MFP_Pos) /*!< GPC_MFPL PC4 setting for BPWM3_CH1   */
#define SYS_GPC_MFPL_PC4MFP_LLSI1_OUT    (0x0fUL<<SYS_GPC_MFPL_PC4MFP_Pos) /*!< GPC_MFPL PC4 setting for LLSI1_OUT   */

/* PC.5 MFP */
#define SYS_GPC_MFPL_PC5MFP_GPIO         (0x00UL<<SYS_GPC_MFPL_PC5MFP_Pos) /*!< GPC_MFPL PC5 setting for GPIO        */
#define SYS_GPC_MFPL_PC5MFP_UART2_TXD    (0x08UL<<SYS_GPC_MFPL_PC5MFP_Pos) /*!< GPC_MFPL PC5 setting for UART2_TXD   */
#define SYS_GPC_MFPL_PC5MFP_I2C1_SCL     (0x09UL<<SYS_GPC_MFPL_PC5MFP_Pos) /*!< GPC_MFPL PC5 setting for I2C1_SCL    */
#define SYS_GPC_MFPL_PC5MFP_BPWM3_CH0    (0x0cUL<<SYS_GPC_MFPL_PC5MFP_Pos) /*!< GPC_MFPL PC5 setting for BPWM3_CH0   */
#define SYS_GPC_MFPL_PC5MFP_LLSI0_OUT    (0x0fUL<<SYS_GPC_MFPL_PC5MFP_Pos) /*!< GPC_MFPL PC5 setting for LLSI0_OUT   */

/* PC.6 MFP */
#define SYS_GPC_MFPL_PC6MFP_GPIO         (0x00UL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< GPC_MFPL PC6 setting for GPIO        */
#define SYS_GPC_MFPL_PC6MFP_SPI1_MOSI    (0x04UL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< GPC_MFPL PC6 setting for SPI1_MOSI   */
#define SYS_GPC_MFPL_PC6MFP_UART0_nRTS   (0x07UL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< GPC_MFPL PC6 setting for UART0_nRTS  */
#define SYS_GPC_MFPL_PC6MFP_I2C1_SMBSUS  (0x08UL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< GPC_MFPL PC6 setting for I2C1_SMBSUS */
#define SYS_GPC_MFPL_PC6MFP_BPWM3_CH3    (0x0bUL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< GPC_MFPL PC6 setting for BPWM3_CH3   */
#define SYS_GPC_MFPL_PC6MFP_BPWM1_CH1    (0x0cUL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< GPC_MFPL PC6 setting for BPWM1_CH1   */
#define SYS_GPC_MFPL_PC6MFP_TM1          (0x0eUL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< GPC_MFPL PC6 setting for TM1         */
#define SYS_GPC_MFPL_PC6MFP_INT2         (0x0fUL<<SYS_GPC_MFPL_PC6MFP_Pos) /*!< GPC_MFPL PC6 setting for INT2        */

/* PC.7 MFP */
#define SYS_GPC_MFPL_PC7MFP_GPIO         (0x00UL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< GPC_MFPL PC7 setting for GPIO        */
#define SYS_GPC_MFPL_PC7MFP_SPI1_MISO    (0x04UL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< GPC_MFPL PC7 setting for SPI1_MISO   */
#define SYS_GPC_MFPL_PC7MFP_UART0_nCTS   (0x07UL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< GPC_MFPL PC7 setting for UART0_nCTS  */
#define SYS_GPC_MFPL_PC7MFP_I2C1_SMBAL   (0x08UL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< GPC_MFPL PC7 setting for I2C1_SMBAL  */
#define SYS_GPC_MFPL_PC7MFP_BPWM3_CH2    (0x0bUL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< GPC_MFPL PC7 setting for BPWM3_CH2   */
#define SYS_GPC_MFPL_PC7MFP_BPWM1_CH0    (0x0cUL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< GPC_MFPL PC7 setting for BPWM1_CH0   */
#define SYS_GPC_MFPL_PC7MFP_TM0          (0x0eUL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< GPC_MFPL PC7 setting for TM0         */
#define SYS_GPC_MFPL_PC7MFP_INT3         (0x0fUL<<SYS_GPC_MFPL_PC7MFP_Pos) /*!< GPC_MFPL PC7 setting for INT3        */

/* PC.14 MFP */
#define SYS_GPC_MFPH_PC14MFP_GPIO        (0x00UL<<SYS_GPC_MFPH_PC14MFP_Pos)/*!< GPC_MFPH PC14 setting for GPIO       */
#define SYS_GPC_MFPH_PC14MFP_SPI1_MOSI   (0x02UL<<SYS_GPC_MFPH_PC14MFP_Pos)/*!< GPC_MFPH PC14 setting for SPI1_MOSI  */
#define SYS_GPC_MFPH_PC14MFP_SPI0_I2SMCLK (0x04UL<<SYS_GPC_MFPH_PC14MFP_Pos)/*!< GPC_MFPH PC14 setting for SPI0_I2SMCLK*/
#define SYS_GPC_MFPH_PC14MFP_TM1         (0x0dUL<<SYS_GPC_MFPH_PC14MFP_Pos)/*!< GPC_MFPH PC14 setting for TM1        */
#define SYS_GPC_MFPH_PC14MFP_DAC1_ST     (0x0fUL<<SYS_GPC_MFPH_PC14MFP_Pos)/*!< GPC_MFPH PC14 setting for DAC1_ST    */

/* PD.0 MFP */
#define SYS_GPD_MFPL_PD0MFP_GPIO         (0x00UL<<SYS_GPD_MFPL_PD0MFP_Pos) /*!< GPD_MFPL PD0 setting for GPIO        */
#define SYS_GPD_MFPL_PD0MFP_SPI0_MOSI    (0x04UL<<SYS_GPD_MFPL_PD0MFP_Pos) /*!< GPD_MFPL PD0 setting for SPI0_MOSI   */
#define SYS_GPD_MFPL_PD0MFP_TM2          (0x0eUL<<SYS_GPD_MFPL_PD0MFP_Pos) /*!< GPD_MFPL PD0 setting for TM2         */

/* PD.1 MFP */
#define SYS_GPD_MFPL_PD1MFP_GPIO         (0x00UL<<SYS_GPD_MFPL_PD1MFP_Pos) /*!< GPD_MFPL PD1 setting for GPIO        */
#define SYS_GPD_MFPL_PD1MFP_SPI0_MISO    (0x04UL<<SYS_GPD_MFPL_PD1MFP_Pos) /*!< GPD_MFPL PD1 setting for SPI0_MISO   */

/* PD.2 MFP */
#define SYS_GPD_MFPL_PD2MFP_GPIO         (0x00UL<<SYS_GPD_MFPL_PD2MFP_Pos) /*!< GPD_MFPL PD2 setting for GPIO        */
#define SYS_GPD_MFPL_PD2MFP_SPI0_CLK     (0x04UL<<SYS_GPD_MFPL_PD2MFP_Pos) /*!< GPD_MFPL PD2 setting for SPI0_CLK    */
#define SYS_GPD_MFPL_PD2MFP_UART0_RXD    (0x09UL<<SYS_GPD_MFPL_PD2MFP_Pos) /*!< GPD_MFPL PD2 setting for UART0_RXD   */

/* PD.3 MFP */
#define SYS_GPD_MFPL_PD3MFP_GPIO         (0x00UL<<SYS_GPD_MFPL_PD3MFP_Pos) /*!< GPD_MFPL PD3 setting for GPIO        */
#define SYS_GPD_MFPL_PD3MFP_SPI0_SS      (0x04UL<<SYS_GPD_MFPL_PD3MFP_Pos) /*!< GPD_MFPL PD3 setting for SPI0_SS     */
#define SYS_GPD_MFPL_PD3MFP_UART0_TXD    (0x09UL<<SYS_GPD_MFPL_PD3MFP_Pos) /*!< GPD_MFPL PD3 setting for UART0_TXD   */

/* PD.15 MFP */
#define SYS_GPD_MFPH_PD15MFP_GPIO        (0x00UL<<SYS_GPD_MFPH_PD15MFP_Pos)/*!< GPD_MFPH PD15 setting for GPIO       */
#define SYS_GPD_MFPH_PD15MFP_BPWM2_CH5   (0x0cUL<<SYS_GPD_MFPH_PD15MFP_Pos)/*!< GPD_MFPH PD15 setting for BPWM2_CH5  */
#define SYS_GPD_MFPH_PD15MFP_TM3         (0x0eUL<<SYS_GPD_MFPH_PD15MFP_Pos)/*!< GPD_MFPH PD15 setting for TM3        */
#define SYS_GPD_MFPH_PD15MFP_INT1        (0x0fUL<<SYS_GPD_MFPH_PD15MFP_Pos)/*!< GPD_MFPH PD15 setting for INT1       */

/* PF.0 MFP */
#define SYS_GPF_MFPL_PF0MFP_GPIO         (0x00UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for GPIO        */
#define SYS_GPF_MFPL_PF0MFP_UART1_TXD    (0x02UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for UART1_TXD   */
#define SYS_GPF_MFPL_PF0MFP_I2C1_SCL     (0x03UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for I2C1_SCL    */
#define SYS_GPF_MFPL_PF0MFP_UART0_TXD    (0x04UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for UART0_TXD   */
#define SYS_GPF_MFPL_PF0MFP_I2C2_SCL     (0x07UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for I2C2_SCL    */
#define SYS_GPF_MFPL_PF0MFP_UART2_TXD    (0x08UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for UART2_TXD   */
#define SYS_GPF_MFPL_PF0MFP_I2C1_SMBAL   (0x09UL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for I2C1_SMBAL  */
#define SYS_GPF_MFPL_PF0MFP_BPWM1_CH0    (0x0cUL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for BPWM1_CH0   */
#define SYS_GPF_MFPL_PF0MFP_ACMP0_O      (0x0dUL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for ACMP0_O     */
#define SYS_GPF_MFPL_PF0MFP_ICE_DAT      (0x0eUL<<SYS_GPF_MFPL_PF0MFP_Pos) /*!< GPF_MFPL PF0 setting for ICE_DAT     */

/* PF.1 MFP */
#define SYS_GPF_MFPL_PF1MFP_GPIO         (0x00UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for GPIO        */
#define SYS_GPF_MFPL_PF1MFP_UART1_RXD    (0x02UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for UART1_RXD   */
#define SYS_GPF_MFPL_PF1MFP_I2C1_SDA     (0x03UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for I2C1_SDA    */
#define SYS_GPF_MFPL_PF1MFP_UART0_RXD    (0x04UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for UART0_RXD   */
#define SYS_GPF_MFPL_PF1MFP_I2C2_SDA     (0x07UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for I2C2_SDA    */
#define SYS_GPF_MFPL_PF1MFP_UART2_RXD    (0x08UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for UART2_RXD   */
#define SYS_GPF_MFPL_PF1MFP_I2C1_SMBSUS  (0x09UL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for I2C1_SMBSUS */
#define SYS_GPF_MFPL_PF1MFP_BPWM1_CH1    (0x0cUL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for BPWM1_CH1   */
#define SYS_GPF_MFPL_PF1MFP_ACMP1_O      (0x0dUL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for ACMP1_O     */
#define SYS_GPF_MFPL_PF1MFP_ICE_CLK      (0x0eUL<<SYS_GPF_MFPL_PF1MFP_Pos) /*!< GPF_MFPL PF1 setting for ICE_CLK     */

/* PF.2 MFP */
#define SYS_GPF_MFPL_PF2MFP_GPIO         (0x00UL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for GPIO        */
#define SYS_GPF_MFPL_PF2MFP_UART0_RXD    (0x03UL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for UART0_RXD   */
#define SYS_GPF_MFPL_PF2MFP_I2C0_SDA     (0x04UL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for I2C0_SDA    */
#define SYS_GPF_MFPL_PF2MFP_UART1_RXD    (0x07UL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for UART1_RXD   */
#define SYS_GPF_MFPL_PF2MFP_XT1_OUT      (0x0aUL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for XT1_OUT     */
#define SYS_GPF_MFPL_PF2MFP_BPWM1_CH1    (0x0bUL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for BPWM1_CH1   */
#define SYS_GPF_MFPL_PF2MFP_ACMP3_O      (0x0dUL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for ACMP3_O     */
#define SYS_GPF_MFPL_PF2MFP_INT4         (0x0fUL<<SYS_GPF_MFPL_PF2MFP_Pos) /*!< GPF_MFPL PF2 setting for INT4        */

/* PF.3 MFP */
#define SYS_GPF_MFPL_PF3MFP_GPIO         (0x00UL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for GPIO        */
#define SYS_GPF_MFPL_PF3MFP_UART0_TXD    (0x03UL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for UART0_TXD   */
#define SYS_GPF_MFPL_PF3MFP_I2C0_SCL     (0x04UL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for I2C0_SCL    */
#define SYS_GPF_MFPL_PF3MFP_UART2_RXD    (0x07UL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for UART2_RXD   */
#define SYS_GPF_MFPL_PF3MFP_BPWM0_CH3    (0x09UL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for BPWM0_CH3   */
#define SYS_GPF_MFPL_PF3MFP_XT1_IN       (0x0aUL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for XT1_IN      */
#define SYS_GPF_MFPL_PF3MFP_BPWM1_CH0    (0x0bUL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for BPWM1_CH0   */
#define SYS_GPF_MFPL_PF3MFP_ACMP2_O      (0x0dUL<<SYS_GPF_MFPL_PF3MFP_Pos) /*!< GPF_MFPL PF3 setting for ACMP2_O     */

/* PF.4 MFP */
#define SYS_GPF_MFPL_PF4MFP_GPIO         (0x00UL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for GPIO        */
#define SYS_GPF_MFPL_PF4MFP_UART2_TXD    (0x02UL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for UART2_TXD   */
#define SYS_GPF_MFPL_PF4MFP_UART2_nRTS   (0x04UL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for UART2_nRTS  */
#define SYS_GPF_MFPL_PF4MFP_SPI0_MISO    (0x05UL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for SPI0_MISO   */
#define SYS_GPF_MFPL_PF4MFP_BPWM2_CH1    (0x07UL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for BPWM2_CH1   */
#define SYS_GPF_MFPL_PF4MFP_BPWM0_CH5    (0x08UL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for BPWM0_CH5   */
#define SYS_GPF_MFPL_PF4MFP_I2C0_SDA     (0x09UL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for I2C0_SDA    */
#define SYS_GPF_MFPL_PF4MFP_X32_OUT      (0x0aUL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for X32_OUT     */
#define SYS_GPF_MFPL_PF4MFP_BPWM2_CH5    (0x0bUL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for BPWM2_CH5   */
#define SYS_GPF_MFPL_PF4MFP_BPWM3_CH5    (0x0cUL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for BPWM3_CH5   */
#define SYS_GPF_MFPL_PF4MFP_ACMP3_WLAT   (0x0dUL<<SYS_GPF_MFPL_PF4MFP_Pos) /*!< GPF_MFPL PF4 setting for ACMP3_WLAT  */

/* PF.5 MFP */
#define SYS_GPF_MFPL_PF5MFP_GPIO         (0x00UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for GPIO        */
#define SYS_GPF_MFPL_PF5MFP_UART2_RXD    (0x02UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for UART2_RXD   */
#define SYS_GPF_MFPL_PF5MFP_SPI1_SS      (0x03UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for SPI1_SS     */
#define SYS_GPF_MFPL_PF5MFP_UART2_nCTS   (0x04UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for UART2_nCTS  */
#define SYS_GPF_MFPL_PF5MFP_SPI0_CLK     (0x05UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for SPI0_CLK    */
#define SYS_GPF_MFPL_PF5MFP_BPWM2_CH0    (0x07UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for BPWM2_CH0   */
#define SYS_GPF_MFPL_PF5MFP_BPWM0_CH4    (0x08UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for BPWM0_CH4   */
#define SYS_GPF_MFPL_PF5MFP_I2C0_SCL     (0x09UL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for I2C0_SCL    */
#define SYS_GPF_MFPL_PF5MFP_X32_IN       (0x0aUL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for X32_IN      */
#define SYS_GPF_MFPL_PF5MFP_ADC0_ST      (0x0bUL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for ADC0_ST     */
#define SYS_GPF_MFPL_PF5MFP_ACMP2_WLAT   (0x0dUL<<SYS_GPF_MFPL_PF5MFP_Pos) /*!< GPF_MFPL PF5 setting for ACMP2_WLAT  */

/* PF.6 MFP */
#define SYS_GPF_MFPL_PF6MFP_GPIO         (0x00UL<<SYS_GPF_MFPL_PF6MFP_Pos) /*!< GPF_MFPL PF6 setting for GPIO        */
#define SYS_GPF_MFPL_PF6MFP_SPI0_MOSI    (0x05UL<<SYS_GPF_MFPL_PF6MFP_Pos) /*!< GPF_MFPL PF6 setting for SPI0_MOSI   */
#define SYS_GPF_MFPL_PF6MFP_I2C0_SDA     (0x0aUL<<SYS_GPF_MFPL_PF6MFP_Pos) /*!< GPF_MFPL PF6 setting for I2C0_SDA    */
#define SYS_GPF_MFPL_PF6MFP_LLSI3_OUT    (0x0bUL<<SYS_GPF_MFPL_PF6MFP_Pos) /*!< GPF_MFPL PF6 setting for LLSI3_OUT   */
#define SYS_GPF_MFPL_PF6MFP_BPWM2_CH4    (0x0cUL<<SYS_GPF_MFPL_PF6MFP_Pos) /*!< GPF_MFPL PF6 setting for BPWM2_CH4   */
#define SYS_GPF_MFPL_PF6MFP_CLKO         (0x0dUL<<SYS_GPF_MFPL_PF6MFP_Pos) /*!< GPF_MFPL PF6 setting for CLKO        */
#define SYS_GPF_MFPL_PF6MFP_TM3          (0x0eUL<<SYS_GPF_MFPL_PF6MFP_Pos) /*!< GPF_MFPL PF6 setting for TM3         */
#define SYS_GPF_MFPL_PF6MFP_INT5         (0x0fUL<<SYS_GPF_MFPL_PF6MFP_Pos) /*!< GPF_MFPL PF6 setting for INT5        */

/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function setting constant definitions abbreviation.                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define ACMP0_N_PB4              SYS_GPB_MFPL_PB4MFP_ACMP0_N          /*!< GPB_MFPL PB4 setting for ACMP0_N*/
#define ACMP0_O_PB7              SYS_GPB_MFPL_PB7MFP_ACMP0_O          /*!< GPB_MFPL PB7 setting for ACMP0_O*/
#define ACMP0_O_PC1              SYS_GPC_MFPL_PC1MFP_ACMP0_O          /*!< GPC_MFPL PC1 setting for ACMP0_O*/
#define ACMP0_O_PF0              SYS_GPF_MFPL_PF0MFP_ACMP0_O          /*!< GPF_MFPL PF0 setting for ACMP0_O*/
#define ACMP0_P0_PB2             SYS_GPB_MFPL_PB2MFP_ACMP0_P0         /*!< GPB_MFPL PB2 setting for ACMP0_P0*/
#define ACMP0_P1_PB3             SYS_GPB_MFPL_PB3MFP_ACMP0_P1         /*!< GPB_MFPL PB3 setting for ACMP0_P1*/
#define ACMP0_P2_PB12            SYS_GPB_MFPH_PB12MFP_ACMP0_P2        /*!< GPB_MFPH PB12 setting for ACMP0_P2*/
#define ACMP0_P3_PB13            SYS_GPB_MFPH_PB13MFP_ACMP0_P3        /*!< GPB_MFPH PB13 setting for ACMP0_P3*/
#define ACMP0_WLAT_PA7           SYS_GPA_MFPL_PA7MFP_ACMP0_WLAT       /*!< GPA_MFPL PA7 setting for ACMP0_WLAT*/
#define ACMP1_N_PB5              SYS_GPB_MFPL_PB5MFP_ACMP1_N          /*!< GPB_MFPL PB5 setting for ACMP1_N*/
#define ACMP1_O_PB6              SYS_GPB_MFPL_PB6MFP_ACMP1_O          /*!< GPB_MFPL PB6 setting for ACMP1_O*/
#define ACMP1_O_PC0              SYS_GPC_MFPL_PC0MFP_ACMP1_O          /*!< GPC_MFPL PC0 setting for ACMP1_O*/
#define ACMP1_O_PF1              SYS_GPF_MFPL_PF1MFP_ACMP1_O          /*!< GPF_MFPL PF1 setting for ACMP1_O*/
#define ACMP1_P0_PB3             SYS_GPB_MFPL_PB3MFP_ACMP1_P0         /*!< GPB_MFPL PB3 setting for ACMP1_P0*/
#define ACMP1_P1_PB4             SYS_GPB_MFPL_PB4MFP_ACMP1_P1         /*!< GPB_MFPL PB4 setting for ACMP1_P1*/
#define ACMP1_P2_PB12            SYS_GPB_MFPH_PB12MFP_ACMP1_P2        /*!< GPB_MFPH PB12 setting for ACMP1_P2*/
#define ACMP1_P3_PB13            SYS_GPB_MFPH_PB13MFP_ACMP1_P3        /*!< GPB_MFPH PB13 setting for ACMP1_P3*/
#define ACMP1_WLAT_PA6           SYS_GPA_MFPL_PA6MFP_ACMP1_WLAT       /*!< GPA_MFPL PA6 setting for ACMP1_WLAT*/
#define ACMP2_N_PB6              SYS_GPB_MFPL_PB6MFP_ACMP2_N          /*!< GPB_MFPL PB6 setting for ACMP2_N*/
#define ACMP2_O_PA1              SYS_GPA_MFPL_PA1MFP_ACMP2_O          /*!< GPA_MFPL PA1 setting for ACMP2_O*/
#define ACMP2_O_PF3              SYS_GPF_MFPL_PF3MFP_ACMP2_O          /*!< GPF_MFPL PF3 setting for ACMP2_O*/
#define ACMP2_P0_PB4             SYS_GPB_MFPL_PB4MFP_ACMP2_P0         /*!< GPB_MFPL PB4 setting for ACMP2_P0*/
#define ACMP2_P1_PB5             SYS_GPB_MFPL_PB5MFP_ACMP2_P1         /*!< GPB_MFPL PB5 setting for ACMP2_P1*/
#define ACMP2_P2_PB14            SYS_GPB_MFPH_PB14MFP_ACMP2_P2        /*!< GPB_MFPH PB14 setting for ACMP2_P2*/
#define ACMP2_P3_PB15            SYS_GPB_MFPH_PB15MFP_ACMP2_P3        /*!< GPB_MFPH PB15 setting for ACMP2_P3*/
#define ACMP2_WLAT_PA3           SYS_GPA_MFPL_PA3MFP_ACMP2_WLAT       /*!< GPA_MFPL PA3 setting for ACMP2_WLAT*/
#define ACMP2_WLAT_PF5           SYS_GPF_MFPL_PF5MFP_ACMP2_WLAT       /*!< GPF_MFPL PF5 setting for ACMP2_WLAT*/
#define ACMP3_N_PB7              SYS_GPB_MFPL_PB7MFP_ACMP3_N          /*!< GPB_MFPL PB7 setting for ACMP3_N*/
#define ACMP3_O_PA0              SYS_GPA_MFPL_PA0MFP_ACMP3_O          /*!< GPA_MFPL PA0 setting for ACMP3_O*/
#define ACMP3_O_PF2              SYS_GPF_MFPL_PF2MFP_ACMP3_O          /*!< GPF_MFPL PF2 setting for ACMP3_O*/
#define ACMP3_P0_PB5             SYS_GPB_MFPL_PB5MFP_ACMP3_P0         /*!< GPB_MFPL PB5 setting for ACMP3_P0*/
#define ACMP3_P1_PB6             SYS_GPB_MFPL_PB6MFP_ACMP3_P1         /*!< GPB_MFPL PB6 setting for ACMP3_P1*/
#define ACMP3_P2_PB14            SYS_GPB_MFPH_PB14MFP_ACMP3_P2        /*!< GPB_MFPH PB14 setting for ACMP3_P2*/
#define ACMP3_P3_PB15            SYS_GPB_MFPH_PB15MFP_ACMP3_P3        /*!< GPB_MFPH PB15 setting for ACMP3_P3*/
#define ACMP3_WLAT_PF4           SYS_GPF_MFPL_PF4MFP_ACMP3_WLAT       /*!< GPF_MFPL PF4 setting for ACMP3_WLAT*/
#define ACMP3_WLAT_PA2           SYS_GPA_MFPL_PA2MFP_ACMP3_WLAT       /*!< GPA_MFPL PA2 setting for ACMP3_WLAT*/
#define ADC0_CH0_PB0             SYS_GPB_MFPL_PB0MFP_ADC0_CH0         /*!< GPB_MFPL PB0 setting for ADC0_CH0*/
#define ADC0_CH1_PB1             SYS_GPB_MFPL_PB1MFP_ADC0_CH1         /*!< GPB_MFPL PB1 setting for ADC0_CH1*/
#define ADC0_CH10_PB10           SYS_GPB_MFPH_PB10MFP_ADC0_CH10       /*!< GPB_MFPH PB10 setting for ADC0_CH10*/
#define ADC0_CH11_PB11           SYS_GPB_MFPH_PB11MFP_ADC0_CH11       /*!< GPB_MFPH PB11 setting for ADC0_CH11*/
#define ADC0_CH12_PB12           SYS_GPB_MFPH_PB12MFP_ADC0_CH12       /*!< GPB_MFPH PB12 setting for ADC0_CH12*/
#define ADC0_CH13_PB13           SYS_GPB_MFPH_PB13MFP_ADC0_CH13       /*!< GPB_MFPH PB13 setting for ADC0_CH13*/
#define ADC0_CH14_PB14           SYS_GPB_MFPH_PB14MFP_ADC0_CH14       /*!< GPB_MFPH PB14 setting for ADC0_CH14*/
#define ADC0_CH15_PB15           SYS_GPB_MFPH_PB15MFP_ADC0_CH15       /*!< GPB_MFPH PB15 setting for ADC0_CH15*/
#define ADC0_CH2_PB2             SYS_GPB_MFPL_PB2MFP_ADC0_CH2         /*!< GPB_MFPL PB2 setting for ADC0_CH2*/
#define ADC0_CH3_PB3             SYS_GPB_MFPL_PB3MFP_ADC0_CH3         /*!< GPB_MFPL PB3 setting for ADC0_CH3*/
#define ADC0_CH4_PB4             SYS_GPB_MFPL_PB4MFP_ADC0_CH4         /*!< GPB_MFPL PB4 setting for ADC0_CH4*/
#define ADC0_CH5_PB5             SYS_GPB_MFPL_PB5MFP_ADC0_CH5         /*!< GPB_MFPL PB5 setting for ADC0_CH5*/
#define ADC0_CH6_PB6             SYS_GPB_MFPL_PB6MFP_ADC0_CH6         /*!< GPB_MFPL PB6 setting for ADC0_CH6*/
#define ADC0_CH7_PB7             SYS_GPB_MFPL_PB7MFP_ADC0_CH7         /*!< GPB_MFPL PB7 setting for ADC0_CH7*/
#define ADC0_CH8_PB8             SYS_GPB_MFPH_PB8MFP_ADC0_CH8         /*!< GPB_MFPH PB8 setting for ADC0_CH8*/
#define ADC0_CH9_PB9             SYS_GPB_MFPH_PB9MFP_ADC0_CH9         /*!< GPB_MFPH PB9 setting for ADC0_CH9*/
#define ADC0_ST_PF5              SYS_GPF_MFPL_PF5MFP_ADC0_ST          /*!< GPF_MFPL PF5 setting for ADC0_ST*/
#define ADC0_ST_PC1              SYS_GPC_MFPL_PC1MFP_ADC0_ST          /*!< GPC_MFPL PC1 setting for ADC0_ST*/
#define BPWM0_CH0_PA0            SYS_GPA_MFPL_PA0MFP_BPWM0_CH0        /*!< GPA_MFPL PA0 setting for BPWM0_CH0*/
#define BPWM0_CH0_PA11           SYS_GPA_MFPH_PA11MFP_BPWM0_CH0       /*!< GPA_MFPH PA11 setting for BPWM0_CH0*/
#define BPWM0_CH1_PA10           SYS_GPA_MFPH_PA10MFP_BPWM0_CH1       /*!< GPA_MFPH PA10 setting for BPWM0_CH1*/
#define BPWM0_CH1_PA1            SYS_GPA_MFPL_PA1MFP_BPWM0_CH1        /*!< GPA_MFPL PA1 setting for BPWM0_CH1*/
#define BPWM0_CH2_PA2            SYS_GPA_MFPL_PA2MFP_BPWM0_CH2        /*!< GPA_MFPL PA2 setting for BPWM0_CH2*/
#define BPWM0_CH2_PA9            SYS_GPA_MFPH_PA9MFP_BPWM0_CH2        /*!< GPA_MFPH PA9 setting for BPWM0_CH2*/
#define BPWM0_CH3_PA3            SYS_GPA_MFPL_PA3MFP_BPWM0_CH3        /*!< GPA_MFPL PA3 setting for BPWM0_CH3*/
#define BPWM0_CH3_PF3            SYS_GPF_MFPL_PF3MFP_BPWM0_CH3        /*!< GPF_MFPL PF3 setting for BPWM0_CH3*/
#define BPWM0_CH3_PA8            SYS_GPA_MFPH_PA8MFP_BPWM0_CH3        /*!< GPA_MFPH PA8 setting for BPWM0_CH3*/
#define BPWM0_CH4_PA4            SYS_GPA_MFPL_PA4MFP_BPWM0_CH4        /*!< GPA_MFPL PA4 setting for BPWM0_CH4*/
#define BPWM0_CH4_PF5            SYS_GPF_MFPL_PF5MFP_BPWM0_CH4        /*!< GPF_MFPL PF5 setting for BPWM0_CH4*/
#define BPWM0_CH5_PA5            SYS_GPA_MFPL_PA5MFP_BPWM0_CH5        /*!< GPA_MFPL PA5 setting for BPWM0_CH5*/
#define BPWM0_CH5_PF4            SYS_GPF_MFPL_PF4MFP_BPWM0_CH5        /*!< GPF_MFPL PF4 setting for BPWM0_CH5*/
#define BPWM1_CH0_PB11           SYS_GPB_MFPH_PB11MFP_BPWM1_CH0       /*!< GPB_MFPH PB11 setting for BPWM1_CH0*/
#define BPWM1_CH0_PF3            SYS_GPF_MFPL_PF3MFP_BPWM1_CH0        /*!< GPF_MFPL PF3 setting for BPWM1_CH0*/
#define BPWM1_CH0_PC7            SYS_GPC_MFPL_PC7MFP_BPWM1_CH0        /*!< GPC_MFPL PC7 setting for BPWM1_CH0*/
#define BPWM1_CH0_PF0            SYS_GPF_MFPL_PF0MFP_BPWM1_CH0        /*!< GPF_MFPL PF0 setting for BPWM1_CH0*/
#define BPWM1_CH1_PC6            SYS_GPC_MFPL_PC6MFP_BPWM1_CH1        /*!< GPC_MFPL PC6 setting for BPWM1_CH1*/
#define BPWM1_CH1_PF2            SYS_GPF_MFPL_PF2MFP_BPWM1_CH1        /*!< GPF_MFPL PF2 setting for BPWM1_CH1*/
#define BPWM1_CH1_PB10           SYS_GPB_MFPH_PB10MFP_BPWM1_CH1       /*!< GPB_MFPH PB10 setting for BPWM1_CH1*/
#define BPWM1_CH1_PF1            SYS_GPF_MFPL_PF1MFP_BPWM1_CH1        /*!< GPF_MFPL PF1 setting for BPWM1_CH1*/
#define BPWM1_CH2_PA7            SYS_GPA_MFPL_PA7MFP_BPWM1_CH2        /*!< GPA_MFPL PA7 setting for BPWM1_CH2*/
#define BPWM1_CH2_PB9            SYS_GPB_MFPH_PB9MFP_BPWM1_CH2        /*!< GPB_MFPH PB9 setting for BPWM1_CH2*/
#define BPWM1_CH3_PA6            SYS_GPA_MFPL_PA6MFP_BPWM1_CH3        /*!< GPA_MFPL PA6 setting for BPWM1_CH3*/
#define BPWM1_CH3_PB8            SYS_GPB_MFPH_PB8MFP_BPWM1_CH3        /*!< GPB_MFPH PB8 setting for BPWM1_CH3*/
#define BPWM1_CH4_PB7            SYS_GPB_MFPL_PB7MFP_BPWM1_CH4        /*!< GPB_MFPL PB7 setting for BPWM1_CH4*/
#define BPWM1_CH5_PB6            SYS_GPB_MFPL_PB6MFP_BPWM1_CH5        /*!< GPB_MFPL PB6 setting for BPWM1_CH5*/
#define BPWM2_CH0_PA5            SYS_GPA_MFPL_PA5MFP_BPWM2_CH0        /*!< GPA_MFPL PA5 setting for BPWM2_CH0*/
#define BPWM2_CH0_PF5            SYS_GPF_MFPL_PF5MFP_BPWM2_CH0        /*!< GPF_MFPL PF5 setting for BPWM2_CH0*/
#define BPWM2_CH0_PB5            SYS_GPB_MFPL_PB5MFP_BPWM2_CH0        /*!< GPB_MFPL PB5 setting for BPWM2_CH0*/
#define BPWM2_CH1_PF4            SYS_GPF_MFPL_PF4MFP_BPWM2_CH1        /*!< GPF_MFPL PF4 setting for BPWM2_CH1*/
#define BPWM2_CH1_PA4            SYS_GPA_MFPL_PA4MFP_BPWM2_CH1        /*!< GPA_MFPL PA4 setting for BPWM2_CH1*/
#define BPWM2_CH1_PB4            SYS_GPB_MFPL_PB4MFP_BPWM2_CH1        /*!< GPB_MFPL PB4 setting for BPWM2_CH1*/
#define BPWM2_CH1_PC1            SYS_GPC_MFPL_PC1MFP_BPWM2_CH1        /*!< GPC_MFPL PC1 setting for BPWM2_CH1*/
#define BPWM2_CH2_PA3            SYS_GPA_MFPL_PA3MFP_BPWM2_CH2        /*!< GPA_MFPL PA3 setting for BPWM2_CH2*/
#define BPWM2_CH2_PB3            SYS_GPB_MFPL_PB3MFP_BPWM2_CH2        /*!< GPB_MFPL PB3 setting for BPWM2_CH2*/
#define BPWM2_CH3_PA2            SYS_GPA_MFPL_PA2MFP_BPWM2_CH3        /*!< GPA_MFPL PA2 setting for BPWM2_CH3*/
#define BPWM2_CH3_PB2            SYS_GPB_MFPL_PB2MFP_BPWM2_CH3        /*!< GPB_MFPL PB2 setting for BPWM2_CH3*/
#define BPWM2_CH4_PB1            SYS_GPB_MFPL_PB1MFP_BPWM2_CH4        /*!< GPB_MFPL PB1 setting for BPWM2_CH4*/
#define BPWM2_CH4_PF6            SYS_GPF_MFPL_PF6MFP_BPWM2_CH4        /*!< GPF_MFPL PF6 setting for BPWM2_CH4*/
#define BPWM2_CH4_PA1            SYS_GPA_MFPL_PA1MFP_BPWM2_CH4        /*!< GPA_MFPL PA1 setting for BPWM2_CH4*/
#define BPWM2_CH5_PD15           SYS_GPD_MFPH_PD15MFP_BPWM2_CH5       /*!< GPD_MFPH PD15 setting for BPWM2_CH5*/
#define BPWM2_CH5_PA0            SYS_GPA_MFPL_PA0MFP_BPWM2_CH5        /*!< GPA_MFPL PA0 setting for BPWM2_CH5*/
#define BPWM2_CH5_PB0            SYS_GPB_MFPL_PB0MFP_BPWM2_CH5        /*!< GPB_MFPL PB0 setting for BPWM2_CH5*/
#define BPWM2_CH5_PF4            SYS_GPF_MFPL_PF4MFP_BPWM2_CH5        /*!< GPF_MFPL PF4 setting for BPWM2_CH5*/
#define BPWM3_CH0_PC5            SYS_GPC_MFPL_PC5MFP_BPWM3_CH0        /*!< GPC_MFPL PC5 setting for BPWM3_CH0*/
#define BPWM3_CH0_PB15           SYS_GPB_MFPH_PB15MFP_BPWM3_CH0       /*!< GPB_MFPH PB15 setting for BPWM3_CH0*/
#define BPWM3_CH1_PB14           SYS_GPB_MFPH_PB14MFP_BPWM3_CH1       /*!< GPB_MFPH PB14 setting for BPWM3_CH1*/
#define BPWM3_CH1_PC4            SYS_GPC_MFPL_PC4MFP_BPWM3_CH1        /*!< GPC_MFPL PC4 setting for BPWM3_CH1*/
#define BPWM3_CH2_PB13           SYS_GPB_MFPH_PB13MFP_BPWM3_CH2       /*!< GPB_MFPH PB13 setting for BPWM3_CH2*/
#define BPWM3_CH2_PC7            SYS_GPC_MFPL_PC7MFP_BPWM3_CH2        /*!< GPC_MFPL PC7 setting for BPWM3_CH2*/
#define BPWM3_CH2_PC3            SYS_GPC_MFPL_PC3MFP_BPWM3_CH2        /*!< GPC_MFPL PC3 setting for BPWM3_CH2*/
#define BPWM3_CH3_PB12           SYS_GPB_MFPH_PB12MFP_BPWM3_CH3       /*!< GPB_MFPH PB12 setting for BPWM3_CH3*/
#define BPWM3_CH3_PC0            SYS_GPC_MFPL_PC0MFP_BPWM3_CH3        /*!< GPC_MFPL PC0 setting for BPWM3_CH3*/
#define BPWM3_CH3_PC2            SYS_GPC_MFPL_PC2MFP_BPWM3_CH3        /*!< GPC_MFPL PC2 setting for BPWM3_CH3*/
#define BPWM3_CH3_PC6            SYS_GPC_MFPL_PC6MFP_BPWM3_CH3        /*!< GPC_MFPL PC6 setting for BPWM3_CH3*/
#define BPWM3_CH4_PC1            SYS_GPC_MFPL_PC1MFP_BPWM3_CH4        /*!< GPC_MFPL PC1 setting for BPWM3_CH4*/
#define BPWM3_CH4_PA7            SYS_GPA_MFPL_PA7MFP_BPWM3_CH4        /*!< GPA_MFPL PA7 setting for BPWM3_CH4*/
#define BPWM3_CH4_PB7            SYS_GPB_MFPL_PB7MFP_BPWM3_CH4        /*!< GPB_MFPL PB7 setting for BPWM3_CH4*/
#define BPWM3_CH4_PB1            SYS_GPB_MFPL_PB1MFP_BPWM3_CH4        /*!< GPB_MFPL PB1 setting for BPWM3_CH4*/
#define BPWM3_CH5_PB6            SYS_GPB_MFPL_PB6MFP_BPWM3_CH5        /*!< GPB_MFPL PB6 setting for BPWM3_CH5*/
#define BPWM3_CH5_PF4            SYS_GPF_MFPL_PF4MFP_BPWM3_CH5        /*!< GPF_MFPL PF4 setting for BPWM3_CH5*/
#define BPWM3_CH5_PC0            SYS_GPC_MFPL_PC0MFP_BPWM3_CH5        /*!< GPC_MFPL PC0 setting for BPWM3_CH5*/
#define BPWM3_CH5_PA6            SYS_GPA_MFPL_PA6MFP_BPWM3_CH5        /*!< GPA_MFPL PA6 setting for BPWM3_CH5*/
#define BPWM3_CH5_PB0            SYS_GPB_MFPL_PB0MFP_BPWM3_CH5        /*!< GPB_MFPL PB0 setting for BPWM3_CH5*/
#define CLKO_PF6                 SYS_GPF_MFPL_PF6MFP_CLKO             /*!< GPF_MFPL PF6 setting for CLKO*/
#define CLKO_PB14                SYS_GPB_MFPH_PB14MFP_CLKO            /*!< GPB_MFPH PB14 setting for CLKO*/
#define CLKO_PA3                 SYS_GPA_MFPL_PA3MFP_CLKO             /*!< GPA_MFPL PA3 setting for CLKO*/
#define DAC0_OUT_PB12            SYS_GPB_MFPH_PB12MFP_DAC0_OUT        /*!< GPB_MFPH PB12 setting for DAC0_OUT*/
#define DAC0_ST_PA10             SYS_GPA_MFPH_PA10MFP_DAC0_ST         /*!< GPA_MFPH PA10 setting for DAC0_ST*/
#define DAC0_ST_PA0              SYS_GPA_MFPL_PA0MFP_DAC0_ST          /*!< GPA_MFPL PA0 setting for DAC0_ST*/
#define DAC1_OUT_PB13            SYS_GPB_MFPH_PB13MFP_DAC1_OUT        /*!< GPB_MFPH PB13 setting for DAC1_OUT*/
#define DAC1_ST_PC14             SYS_GPC_MFPH_PC14MFP_DAC1_ST         /*!< GPC_MFPH PC14 setting for DAC1_ST*/
#define DAC1_ST_PA11             SYS_GPA_MFPH_PA11MFP_DAC1_ST         /*!< GPA_MFPH PA11 setting for DAC1_ST*/
#define DAC2_OUT_PB14            SYS_GPB_MFPH_PB14MFP_DAC2_OUT        /*!< GPB_MFPH PB14 setting for DAC2_OUT*/
#define DAC2_ST_PA9              SYS_GPA_MFPH_PA9MFP_DAC2_ST          /*!< GPA_MFPH PA9 setting for DAC2_ST*/
#define DAC2_ST_PA5              SYS_GPA_MFPL_PA5MFP_DAC2_ST          /*!< GPA_MFPL PA5 setting for DAC2_ST*/
#define DAC3_OUT_PB15            SYS_GPB_MFPH_PB15MFP_DAC3_OUT        /*!< GPB_MFPH PB15 setting for DAC3_OUT*/
#define DAC3_ST_PA4              SYS_GPA_MFPL_PA4MFP_DAC3_ST          /*!< GPA_MFPL PA4 setting for DAC3_ST*/
#define DAC3_ST_PA8              SYS_GPA_MFPH_PA8MFP_DAC3_ST          /*!< GPA_MFPH PA8 setting for DAC3_ST*/
#define I2C0_SCL_PA8             SYS_GPA_MFPH_PA8MFP_I2C0_SCL         /*!< GPA_MFPH PA8 setting for I2C0_SCL*/
#define I2C0_SCL_PB5             SYS_GPB_MFPL_PB5MFP_I2C0_SCL         /*!< GPB_MFPL PB5 setting for I2C0_SCL*/
#define I2C0_SCL_PB9             SYS_GPB_MFPH_PB9MFP_I2C0_SCL         /*!< GPB_MFPH PB9 setting for I2C0_SCL*/
#define I2C0_SCL_PF3             SYS_GPF_MFPL_PF3MFP_I2C0_SCL         /*!< GPF_MFPL PF3 setting for I2C0_SCL*/
#define I2C0_SCL_PA5             SYS_GPA_MFPL_PA5MFP_I2C0_SCL         /*!< GPA_MFPL PA5 setting for I2C0_SCL*/
#define I2C0_SCL_PC1             SYS_GPC_MFPL_PC1MFP_I2C0_SCL         /*!< GPC_MFPL PC1 setting for I2C0_SCL*/
#define I2C0_SCL_PC3             SYS_GPC_MFPL_PC3MFP_I2C0_SCL         /*!< GPC_MFPL PC3 setting for I2C0_SCL*/
#define I2C0_SCL_PF5             SYS_GPF_MFPL_PF5MFP_I2C0_SCL         /*!< GPF_MFPL PF5 setting for I2C0_SCL*/
#define I2C0_SDA_PA4             SYS_GPA_MFPL_PA4MFP_I2C0_SDA         /*!< GPA_MFPL PA4 setting for I2C0_SDA*/
#define I2C0_SDA_PF6             SYS_GPF_MFPL_PF6MFP_I2C0_SDA         /*!< GPF_MFPL PF6 setting for I2C0_SDA*/
#define I2C0_SDA_PC0             SYS_GPC_MFPL_PC0MFP_I2C0_SDA         /*!< GPC_MFPL PC0 setting for I2C0_SDA*/
#define I2C0_SDA_PC2             SYS_GPC_MFPL_PC2MFP_I2C0_SDA         /*!< GPC_MFPL PC2 setting for I2C0_SDA*/
#define I2C0_SDA_PF2             SYS_GPF_MFPL_PF2MFP_I2C0_SDA         /*!< GPF_MFPL PF2 setting for I2C0_SDA*/
#define I2C0_SDA_PB8             SYS_GPB_MFPH_PB8MFP_I2C0_SDA         /*!< GPB_MFPH PB8 setting for I2C0_SDA*/
#define I2C0_SDA_PB4             SYS_GPB_MFPL_PB4MFP_I2C0_SDA         /*!< GPB_MFPL PB4 setting for I2C0_SDA*/
#define I2C0_SDA_PF4             SYS_GPF_MFPL_PF4MFP_I2C0_SDA         /*!< GPF_MFPL PF4 setting for I2C0_SDA*/
#define I2C0_SMBAL_PC3           SYS_GPC_MFPL_PC3MFP_I2C0_SMBAL       /*!< GPC_MFPL PC3 setting for I2C0_SMBAL*/
#define I2C0_SMBAL_PA3           SYS_GPA_MFPL_PA3MFP_I2C0_SMBAL       /*!< GPA_MFPL PA3 setting for I2C0_SMBAL*/
#define I2C0_SMBSUS_PA2          SYS_GPA_MFPL_PA2MFP_I2C0_SMBSUS      /*!< GPA_MFPL PA2 setting for I2C0_SMBSUS*/
#define I2C0_SMBSUS_PC2          SYS_GPC_MFPL_PC2MFP_I2C0_SMBSUS      /*!< GPC_MFPL PC2 setting for I2C0_SMBSUS*/
#define I2C1_SCL_PC1             SYS_GPC_MFPL_PC1MFP_I2C1_SCL         /*!< GPC_MFPL PC1 setting for I2C1_SCL*/
#define I2C1_SCL_PA10            SYS_GPA_MFPH_PA10MFP_I2C1_SCL        /*!< GPA_MFPH PA10 setting for I2C1_SCL*/
#define I2C1_SCL_PA7             SYS_GPA_MFPL_PA7MFP_I2C1_SCL         /*!< GPA_MFPL PA7 setting for I2C1_SCL*/
#define I2C1_SCL_PA3             SYS_GPA_MFPL_PA3MFP_I2C1_SCL         /*!< GPA_MFPL PA3 setting for I2C1_SCL*/
#define I2C1_SCL_PB11            SYS_GPB_MFPH_PB11MFP_I2C1_SCL        /*!< GPB_MFPH PB11 setting for I2C1_SCL*/
#define I2C1_SCL_PB1             SYS_GPB_MFPL_PB1MFP_I2C1_SCL         /*!< GPB_MFPL PB1 setting for I2C1_SCL*/
#define I2C1_SCL_PF0             SYS_GPF_MFPL_PF0MFP_I2C1_SCL         /*!< GPF_MFPL PF0 setting for I2C1_SCL*/
#define I2C1_SCL_PC5             SYS_GPC_MFPL_PC5MFP_I2C1_SCL         /*!< GPC_MFPL PC5 setting for I2C1_SCL*/
#define I2C1_SCL_PB3             SYS_GPB_MFPL_PB3MFP_I2C1_SCL         /*!< GPB_MFPL PB3 setting for I2C1_SCL*/
#define I2C1_SDA_PC0             SYS_GPC_MFPL_PC0MFP_I2C1_SDA         /*!< GPC_MFPL PC0 setting for I2C1_SDA*/
#define I2C1_SDA_PB0             SYS_GPB_MFPL_PB0MFP_I2C1_SDA         /*!< GPB_MFPL PB0 setting for I2C1_SDA*/
#define I2C1_SDA_PA2             SYS_GPA_MFPL_PA2MFP_I2C1_SDA         /*!< GPA_MFPL PA2 setting for I2C1_SDA*/
#define I2C1_SDA_PC4             SYS_GPC_MFPL_PC4MFP_I2C1_SDA         /*!< GPC_MFPL PC4 setting for I2C1_SDA*/
#define I2C1_SDA_PB2             SYS_GPB_MFPL_PB2MFP_I2C1_SDA         /*!< GPB_MFPL PB2 setting for I2C1_SDA*/
#define I2C1_SDA_PA9             SYS_GPA_MFPH_PA9MFP_I2C1_SDA         /*!< GPA_MFPH PA9 setting for I2C1_SDA*/
#define I2C1_SDA_PB10            SYS_GPB_MFPH_PB10MFP_I2C1_SDA        /*!< GPB_MFPH PB10 setting for I2C1_SDA*/
#define I2C1_SDA_PA6             SYS_GPA_MFPL_PA6MFP_I2C1_SDA         /*!< GPA_MFPL PA6 setting for I2C1_SDA*/
#define I2C1_SDA_PF1             SYS_GPF_MFPL_PF1MFP_I2C1_SDA         /*!< GPF_MFPL PF1 setting for I2C1_SDA*/
#define I2C1_SMBAL_PC7           SYS_GPC_MFPL_PC7MFP_I2C1_SMBAL       /*!< GPC_MFPL PC7 setting for I2C1_SMBAL*/
#define I2C1_SMBAL_PB9           SYS_GPB_MFPH_PB9MFP_I2C1_SMBAL       /*!< GPB_MFPH PB9 setting for I2C1_SMBAL*/
#define I2C1_SMBAL_PF0           SYS_GPF_MFPL_PF0MFP_I2C1_SMBAL       /*!< GPF_MFPL PF0 setting for I2C1_SMBAL*/
#define I2C1_SMBSUS_PB8          SYS_GPB_MFPH_PB8MFP_I2C1_SMBSUS      /*!< GPB_MFPH PB8 setting for I2C1_SMBSUS*/
#define I2C1_SMBSUS_PC6          SYS_GPC_MFPL_PC6MFP_I2C1_SMBSUS      /*!< GPC_MFPL PC6 setting for I2C1_SMBSUS*/
#define I2C1_SMBSUS_PF1          SYS_GPF_MFPL_PF1MFP_I2C1_SMBSUS      /*!< GPF_MFPL PF1 setting for I2C1_SMBSUS*/
#define I2C2_SCL_PB3             SYS_GPB_MFPL_PB3MFP_I2C2_SCL         /*!< GPB_MFPL PB3 setting for I2C2_SCL*/
#define I2C2_SCL_PA11            SYS_GPA_MFPH_PA11MFP_I2C2_SCL        /*!< GPA_MFPH PA11 setting for I2C2_SCL*/
#define I2C2_SCL_PF0             SYS_GPF_MFPL_PF0MFP_I2C2_SCL         /*!< GPF_MFPL PF0 setting for I2C2_SCL*/
#define I2C2_SCL_PA1             SYS_GPA_MFPL_PA1MFP_I2C2_SCL         /*!< GPA_MFPL PA1 setting for I2C2_SCL*/
#define I2C2_SCL_PB13            SYS_GPB_MFPH_PB13MFP_I2C2_SCL        /*!< GPB_MFPH PB13 setting for I2C2_SCL*/
#define I2C2_SDA_PA0             SYS_GPA_MFPL_PA0MFP_I2C2_SDA         /*!< GPA_MFPL PA0 setting for I2C2_SDA*/
#define I2C2_SDA_PF1             SYS_GPF_MFPL_PF1MFP_I2C2_SDA         /*!< GPF_MFPL PF1 setting for I2C2_SDA*/
#define I2C2_SDA_PA10            SYS_GPA_MFPH_PA10MFP_I2C2_SDA        /*!< GPA_MFPH PA10 setting for I2C2_SDA*/
#define I2C2_SDA_PB2             SYS_GPB_MFPL_PB2MFP_I2C2_SDA         /*!< GPB_MFPL PB2 setting for I2C2_SDA*/
#define I2C2_SDA_PB12            SYS_GPB_MFPH_PB12MFP_I2C2_SDA        /*!< GPB_MFPH PB12 setting for I2C2_SDA*/
#define I2C2_SMBAL_PB15          SYS_GPB_MFPH_PB15MFP_I2C2_SMBAL      /*!< GPB_MFPH PB15 setting for I2C2_SMBAL*/
#define I2C2_SMBAL_PC3           SYS_GPC_MFPL_PC3MFP_I2C2_SMBAL       /*!< GPC_MFPL PC3 setting for I2C2_SMBAL*/
#define I2C2_SMBSUS_PB14         SYS_GPB_MFPH_PB14MFP_I2C2_SMBSUS     /*!< GPB_MFPH PB14 setting for I2C2_SMBSUS*/
#define I2C2_SMBSUS_PC2          SYS_GPC_MFPL_PC2MFP_I2C2_SMBSUS      /*!< GPC_MFPL PC2 setting for I2C2_SMBSUS*/
#define I3CS0_SCL_PA5            SYS_GPA_MFPL_PA5MFP_I3CS0_SCL        /*!< GPA_MFPL PA5 setting for I3CS0_SCL*/
#define I3CS0_SCL_PA1            SYS_GPA_MFPL_PA1MFP_I3CS0_SCL        /*!< GPA_MFPL PA1 setting for I3CS0_SCL*/
#define I3CS0_SDA_PA0            SYS_GPA_MFPL_PA0MFP_I3CS0_SDA        /*!< GPA_MFPL PA0 setting for I3CS0_SDA*/
#define I3CS0_SDA_PA4            SYS_GPA_MFPL_PA4MFP_I3CS0_SDA        /*!< GPA_MFPL PA4 setting for I3CS0_SDA*/
#define I3CS1_SCL_PA3            SYS_GPA_MFPL_PA3MFP_I3CS1_SCL        /*!< GPA_MFPL PA3 setting for I3CS1_SCL*/
#define I3CS1_SDA_PA2            SYS_GPA_MFPL_PA2MFP_I3CS1_SDA        /*!< GPA_MFPL PA2 setting for I3CS1_SDA*/
#define ICE_CLK_PF1              SYS_GPF_MFPL_PF1MFP_ICE_CLK          /*!< GPF_MFPL PF1 setting for ICE_CLK*/
#define ICE_DAT_PF0              SYS_GPF_MFPL_PF0MFP_ICE_DAT          /*!< GPF_MFPL PF0 setting for ICE_DAT*/
#define INT0_PA6                 SYS_GPA_MFPL_PA6MFP_INT0             /*!< GPA_MFPL PA6 setting for INT0*/
#define INT0_PB5                 SYS_GPB_MFPL_PB5MFP_INT0             /*!< GPB_MFPL PB5 setting for INT0*/
#define INT1_PB4                 SYS_GPB_MFPL_PB4MFP_INT1             /*!< GPB_MFPL PB4 setting for INT1*/
#define INT1_PD15                SYS_GPD_MFPH_PD15MFP_INT1            /*!< GPD_MFPH PD15 setting for INT1*/
#define INT1_PA7                 SYS_GPA_MFPL_PA7MFP_INT1             /*!< GPA_MFPL PA7 setting for INT1*/
#define INT2_PC6                 SYS_GPC_MFPL_PC6MFP_INT2             /*!< GPC_MFPL PC6 setting for INT2*/
#define INT2_PB3                 SYS_GPB_MFPL_PB3MFP_INT2             /*!< GPB_MFPL PB3 setting for INT2*/
#define INT3_PB2                 SYS_GPB_MFPL_PB2MFP_INT3             /*!< GPB_MFPL PB2 setting for INT3*/
#define INT3_PC7                 SYS_GPC_MFPL_PC7MFP_INT3             /*!< GPC_MFPL PC7 setting for INT3*/
#define INT4_PB6                 SYS_GPB_MFPL_PB6MFP_INT4             /*!< GPB_MFPL PB6 setting for INT4*/
#define INT4_PF2                 SYS_GPF_MFPL_PF2MFP_INT4             /*!< GPF_MFPL PF2 setting for INT4*/
#define INT4_PA8                 SYS_GPA_MFPH_PA8MFP_INT4             /*!< GPA_MFPH PA8 setting for INT4*/
#define INT5_PB7                 SYS_GPB_MFPL_PB7MFP_INT5             /*!< GPB_MFPL PB7 setting for INT5*/
#define INT5_PF6                 SYS_GPF_MFPL_PF6MFP_INT5             /*!< GPF_MFPL PF6 setting for INT5*/
#define LLSI0_OUT_PB15           SYS_GPB_MFPH_PB15MFP_LLSI0_OUT       /*!< GPB_MFPH PB15 setting for LLSI0_OUT*/
#define LLSI0_OUT_PC5            SYS_GPC_MFPL_PC5MFP_LLSI0_OUT        /*!< GPC_MFPL PC5 setting for LLSI0_OUT*/
#define LLSI1_OUT_PC4            SYS_GPC_MFPL_PC4MFP_LLSI1_OUT        /*!< GPC_MFPL PC4 setting for LLSI1_OUT*/
#define LLSI1_OUT_PB14           SYS_GPB_MFPH_PB14MFP_LLSI1_OUT       /*!< GPB_MFPH PB14 setting for LLSI1_OUT*/
#define LLSI2_OUT_PB13           SYS_GPB_MFPH_PB13MFP_LLSI2_OUT       /*!< GPB_MFPH PB13 setting for LLSI2_OUT*/
#define LLSI2_OUT_PC3            SYS_GPC_MFPL_PC3MFP_LLSI2_OUT        /*!< GPC_MFPL PC3 setting for LLSI2_OUT*/
#define LLSI3_OUT_PF6            SYS_GPF_MFPL_PF6MFP_LLSI3_OUT        /*!< GPF_MFPL PF6 setting for LLSI3_OUT*/
#define LLSI3_OUT_PB12           SYS_GPB_MFPH_PB12MFP_LLSI3_OUT       /*!< GPB_MFPH PB12 setting for LLSI3_OUT*/
#define LLSI3_OUT_PC2            SYS_GPC_MFPL_PC2MFP_LLSI3_OUT        /*!< GPC_MFPL PC2 setting for LLSI3_OUT*/
#define LLSI4_OUT_PA3            SYS_GPA_MFPL_PA3MFP_LLSI4_OUT        /*!< GPA_MFPL PA3 setting for LLSI4_OUT*/
#define LLSI4_OUT_PB5            SYS_GPB_MFPL_PB5MFP_LLSI4_OUT        /*!< GPB_MFPL PB5 setting for LLSI4_OUT*/
#define LLSI5_OUT_PA2            SYS_GPA_MFPL_PA2MFP_LLSI5_OUT        /*!< GPA_MFPL PA2 setting for LLSI5_OUT*/
#define LLSI5_OUT_PB4            SYS_GPB_MFPL_PB4MFP_LLSI5_OUT        /*!< GPB_MFPL PB4 setting for LLSI5_OUT*/
#define SPDH_HSA_PB1             SYS_GPB_MFPL_PB1MFP_SPDH_HSA         /*!< GPB_MFPL PB1 setting for SPDH_HSA*/
#define SPI0_CLK_PD2             SYS_GPD_MFPL_PD2MFP_SPI0_CLK         /*!< GPD_MFPL PD2 setting for SPI0_CLK*/
#define SPI0_CLK_PB14            SYS_GPB_MFPH_PB14MFP_SPI0_CLK        /*!< GPB_MFPH PB14 setting for SPI0_CLK*/
#define SPI0_CLK_PF5             SYS_GPF_MFPL_PF5MFP_SPI0_CLK         /*!< GPF_MFPL PF5 setting for SPI0_CLK*/
#define SPI0_CLK_PA2             SYS_GPA_MFPL_PA2MFP_SPI0_CLK         /*!< GPA_MFPL PA2 setting for SPI0_CLK*/
#define SPI0_I2SMCLK_PA4         SYS_GPA_MFPL_PA4MFP_SPI0_I2SMCLK     /*!< GPA_MFPL PA4 setting for SPI0_I2SMCLK*/
#define SPI0_I2SMCLK_PA5         SYS_GPA_MFPL_PA5MFP_SPI0_I2SMCLK     /*!< GPA_MFPL PA5 setting for SPI0_I2SMCLK*/
#define SPI0_I2SMCLK_PB0         SYS_GPB_MFPL_PB0MFP_SPI0_I2SMCLK     /*!< GPB_MFPL PB0 setting for SPI0_I2SMCLK*/
#define SPI0_I2SMCLK_PB11        SYS_GPB_MFPH_PB11MFP_SPI0_I2SMCLK    /*!< GPB_MFPH PB11 setting for SPI0_I2SMCLK*/
#define SPI0_I2SMCLK_PB2         SYS_GPB_MFPL_PB2MFP_SPI0_I2SMCLK     /*!< GPB_MFPL PB2 setting for SPI0_I2SMCLK*/
#define SPI0_I2SMCLK_PC14        SYS_GPC_MFPH_PC14MFP_SPI0_I2SMCLK    /*!< GPC_MFPH PC14 setting for SPI0_I2SMCLK*/
#define SPI0_MISO_PD1            SYS_GPD_MFPL_PD1MFP_SPI0_MISO        /*!< GPD_MFPL PD1 setting for SPI0_MISO*/
#define SPI0_MISO_PA1            SYS_GPA_MFPL_PA1MFP_SPI0_MISO        /*!< GPA_MFPL PA1 setting for SPI0_MISO*/
#define SPI0_MISO_PB13           SYS_GPB_MFPH_PB13MFP_SPI0_MISO       /*!< GPB_MFPH PB13 setting for SPI0_MISO*/
#define SPI0_MISO_PF4            SYS_GPF_MFPL_PF4MFP_SPI0_MISO        /*!< GPF_MFPL PF4 setting for SPI0_MISO*/
#define SPI0_MOSI_PB12           SYS_GPB_MFPH_PB12MFP_SPI0_MOSI       /*!< GPB_MFPH PB12 setting for SPI0_MOSI*/
#define SPI0_MOSI_PD0            SYS_GPD_MFPL_PD0MFP_SPI0_MOSI        /*!< GPD_MFPL PD0 setting for SPI0_MOSI*/
#define SPI0_MOSI_PA0            SYS_GPA_MFPL_PA0MFP_SPI0_MOSI        /*!< GPA_MFPL PA0 setting for SPI0_MOSI*/
#define SPI0_MOSI_PF6            SYS_GPF_MFPL_PF6MFP_SPI0_MOSI        /*!< GPF_MFPL PF6 setting for SPI0_MOSI*/
#define SPI0_SS_PD3              SYS_GPD_MFPL_PD3MFP_SPI0_SS          /*!< GPD_MFPL PD3 setting for SPI0_SS*/
#define SPI0_SS_PB0              SYS_GPB_MFPL_PB0MFP_SPI0_SS          /*!< GPB_MFPL PB0 setting for SPI0_SS*/
#define SPI0_SS_PB15             SYS_GPB_MFPH_PB15MFP_SPI0_SS         /*!< GPB_MFPH PB15 setting for SPI0_SS*/
#define SPI0_SS_PA3              SYS_GPA_MFPL_PA3MFP_SPI0_SS          /*!< GPA_MFPL PA3 setting for SPI0_SS*/
#define SPI1_CLK_PA9             SYS_GPA_MFPH_PA9MFP_SPI1_CLK         /*!< GPA_MFPH PA9 setting for SPI1_CLK*/
#define SPI1_CLK_PA7             SYS_GPA_MFPL_PA7MFP_SPI1_CLK         /*!< GPA_MFPL PA7 setting for SPI1_CLK*/
#define SPI1_CLK_PC1             SYS_GPC_MFPL_PC1MFP_SPI1_CLK         /*!< GPC_MFPL PC1 setting for SPI1_CLK*/
#define SPI1_CLK_PB3             SYS_GPB_MFPL_PB3MFP_SPI1_CLK         /*!< GPB_MFPL PB3 setting for SPI1_CLK*/
#define SPI1_I2SMCLK_PB1         SYS_GPB_MFPL_PB1MFP_SPI1_I2SMCLK     /*!< GPB_MFPL PB1 setting for SPI1_I2SMCLK*/
#define SPI1_I2SMCLK_PA5         SYS_GPA_MFPL_PA5MFP_SPI1_I2SMCLK     /*!< GPA_MFPL PA5 setting for SPI1_I2SMCLK*/
#define SPI1_I2SMCLK_PC4         SYS_GPC_MFPL_PC4MFP_SPI1_I2SMCLK     /*!< GPC_MFPL PC4 setting for SPI1_I2SMCLK*/
#define SPI1_MISO_PC7            SYS_GPC_MFPL_PC7MFP_SPI1_MISO        /*!< GPC_MFPL PC7 setting for SPI1_MISO*/
#define SPI1_MISO_PB5            SYS_GPB_MFPL_PB5MFP_SPI1_MISO        /*!< GPB_MFPL PB5 setting for SPI1_MISO*/
#define SPI1_MISO_PC3            SYS_GPC_MFPL_PC3MFP_SPI1_MISO        /*!< GPC_MFPL PC3 setting for SPI1_MISO*/
#define SPI1_MISO_PA10           SYS_GPA_MFPH_PA10MFP_SPI1_MISO       /*!< GPA_MFPH PA10 setting for SPI1_MISO*/
#define SPI1_MOSI_PB4            SYS_GPB_MFPL_PB4MFP_SPI1_MOSI        /*!< GPB_MFPL PB4 setting for SPI1_MOSI*/
#define SPI1_MOSI_PC2            SYS_GPC_MFPL_PC2MFP_SPI1_MOSI        /*!< GPC_MFPL PC2 setting for SPI1_MOSI*/
#define SPI1_MOSI_PC6            SYS_GPC_MFPL_PC6MFP_SPI1_MOSI        /*!< GPC_MFPL PC6 setting for SPI1_MOSI*/
#define SPI1_MOSI_PA11           SYS_GPA_MFPH_PA11MFP_SPI1_MOSI       /*!< GPA_MFPH PA11 setting for SPI1_MOSI*/
#define SPI1_MOSI_PC14           SYS_GPC_MFPH_PC14MFP_SPI1_MOSI       /*!< GPC_MFPH PC14 setting for SPI1_MOSI*/
#define SPI1_SS_PA8              SYS_GPA_MFPH_PA8MFP_SPI1_SS          /*!< GPA_MFPH PA8 setting for SPI1_SS*/
#define SPI1_SS_PA6              SYS_GPA_MFPL_PA6MFP_SPI1_SS          /*!< GPA_MFPL PA6 setting for SPI1_SS*/
#define SPI1_SS_PC0              SYS_GPC_MFPL_PC0MFP_SPI1_SS          /*!< GPC_MFPL PC0 setting for SPI1_SS*/
#define SPI1_SS_PB2              SYS_GPB_MFPL_PB2MFP_SPI1_SS          /*!< GPB_MFPL PB2 setting for SPI1_SS*/
#define SPI1_SS_PF5              SYS_GPF_MFPL_PF5MFP_SPI1_SS          /*!< GPF_MFPL PF5 setting for SPI1_SS*/
#define SPI2_CLK_PA2             SYS_GPA_MFPL_PA2MFP_SPI2_CLK         /*!< GPA_MFPL PA2 setting for SPI2_CLK*/
#define SPI2_CLK_PA10            SYS_GPA_MFPH_PA10MFP_SPI2_CLK        /*!< GPA_MFPH PA10 setting for SPI2_CLK*/
#define SPI2_I2SMCLK_PA5         SYS_GPA_MFPL_PA5MFP_SPI2_I2SMCLK     /*!< GPA_MFPL PA5 setting for SPI2_I2SMCLK*/
#define SPI2_I2SMCLK_PB0         SYS_GPB_MFPL_PB0MFP_SPI2_I2SMCLK     /*!< GPB_MFPL PB0 setting for SPI2_I2SMCLK*/
#define SPI2_MISO_PA1            SYS_GPA_MFPL_PA1MFP_SPI2_MISO        /*!< GPA_MFPL PA1 setting for SPI2_MISO*/
#define SPI2_MISO_PA9            SYS_GPA_MFPH_PA9MFP_SPI2_MISO        /*!< GPA_MFPH PA9 setting for SPI2_MISO*/
#define SPI2_MOSI_PA0            SYS_GPA_MFPL_PA0MFP_SPI2_MOSI        /*!< GPA_MFPL PA0 setting for SPI2_MOSI*/
#define SPI2_MOSI_PA8            SYS_GPA_MFPH_PA8MFP_SPI2_MOSI        /*!< GPA_MFPH PA8 setting for SPI2_MOSI*/
#define SPI2_SS_PA3              SYS_GPA_MFPL_PA3MFP_SPI2_SS          /*!< GPA_MFPL PA3 setting for SPI2_SS*/
#define SPI2_SS_PA11             SYS_GPA_MFPH_PA11MFP_SPI2_SS         /*!< GPA_MFPH PA11 setting for SPI2_SS*/
#define SPI_CLK_MUX_PA2          SYS_GPA_MFPL_PA2MFP_SPI_CLK_MUX      /*!< GPA_MFPL PA2 setting for SPI_CLK_MUX*/
#define SPI_MISO_MUX_PA1         SYS_GPA_MFPL_PA1MFP_SPI_MISO_MUX     /*!< GPA_MFPL PA1 setting for SPI_MISO_MUX*/
#define SPI_MOSI_MUX_PA0         SYS_GPA_MFPL_PA0MFP_SPI_MOSI_MUX     /*!< GPA_MFPL PA0 setting for SPI_MOSI_MUX*/
#define SPI_SS_MUX_PA3           SYS_GPA_MFPL_PA3MFP_SPI_SS_MUX       /*!< GPA_MFPL PA3 setting for SPI_SS_MUX*/
#define TM0_PB5                  SYS_GPB_MFPL_PB5MFP_TM0              /*!< GPB_MFPL PB5 setting for TM0*/
#define TM0_PC7                  SYS_GPC_MFPL_PC7MFP_TM0              /*!< GPC_MFPL PC7 setting for TM0*/
#define TM0_EXT_PA11             SYS_GPA_MFPH_PA11MFP_TM0_EXT         /*!< GPA_MFPH PA11 setting for TM0_EXT*/
#define TM0_EXT_PB15             SYS_GPB_MFPH_PB15MFP_TM0_EXT         /*!< GPB_MFPH PB15 setting for TM0_EXT*/
#define TM1_PB4                  SYS_GPB_MFPL_PB4MFP_TM1              /*!< GPB_MFPL PB4 setting for TM1*/
#define TM1_PC14                 SYS_GPC_MFPH_PC14MFP_TM1             /*!< GPC_MFPH PC14 setting for TM1*/
#define TM1_PC6                  SYS_GPC_MFPL_PC6MFP_TM1              /*!< GPC_MFPL PC6 setting for TM1*/
#define TM1_EXT_PA10             SYS_GPA_MFPH_PA10MFP_TM1_EXT         /*!< GPA_MFPH PA10 setting for TM1_EXT*/
#define TM1_EXT_PB14             SYS_GPB_MFPH_PB14MFP_TM1_EXT         /*!< GPB_MFPH PB14 setting for TM1_EXT*/
#define TM2_PD0                  SYS_GPD_MFPL_PD0MFP_TM2              /*!< GPD_MFPL PD0 setting for TM2*/
#define TM2_PB3                  SYS_GPB_MFPL_PB3MFP_TM2              /*!< GPB_MFPL PB3 setting for TM2*/
#define TM2_PA7                  SYS_GPA_MFPL_PA7MFP_TM2              /*!< GPA_MFPL PA7 setting for TM2*/
#define TM2_EXT_PB13             SYS_GPB_MFPH_PB13MFP_TM2_EXT         /*!< GPB_MFPH PB13 setting for TM2_EXT*/
#define TM2_EXT_PA9              SYS_GPA_MFPH_PA9MFP_TM2_EXT          /*!< GPA_MFPH PA9 setting for TM2_EXT*/
#define TM3_PD15                 SYS_GPD_MFPH_PD15MFP_TM3             /*!< GPD_MFPH PD15 setting for TM3*/
#define TM3_PA6                  SYS_GPA_MFPL_PA6MFP_TM3              /*!< GPA_MFPL PA6 setting for TM3*/
#define TM3_PF6                  SYS_GPF_MFPL_PF6MFP_TM3              /*!< GPF_MFPL PF6 setting for TM3*/
#define TM3_PB2                  SYS_GPB_MFPL_PB2MFP_TM3              /*!< GPB_MFPL PB2 setting for TM3*/
#define TM3_EXT_PA8              SYS_GPA_MFPH_PA8MFP_TM3_EXT          /*!< GPA_MFPH PA8 setting for TM3_EXT*/
#define TM3_EXT_PB1              SYS_GPB_MFPL_PB1MFP_TM3_EXT          /*!< GPB_MFPL PB1 setting for TM3_EXT*/
#define TM3_EXT_PB12             SYS_GPB_MFPH_PB12MFP_TM3_EXT         /*!< GPB_MFPH PB12 setting for TM3_EXT*/
#define UART0_RXD_PF2            SYS_GPF_MFPL_PF2MFP_UART0_RXD        /*!< GPF_MFPL PF2 setting for UART0_RXD*/
#define UART0_RXD_PA6            SYS_GPA_MFPL_PA6MFP_UART0_RXD        /*!< GPA_MFPL PA6 setting for UART0_RXD*/
#define UART0_RXD_PF1            SYS_GPF_MFPL_PF1MFP_UART0_RXD        /*!< GPF_MFPL PF1 setting for UART0_RXD*/
#define UART0_RXD_PD2            SYS_GPD_MFPL_PD2MFP_UART0_RXD        /*!< GPD_MFPL PD2 setting for UART0_RXD*/
#define UART0_RXD_PB12           SYS_GPB_MFPH_PB12MFP_UART0_RXD       /*!< GPB_MFPH PB12 setting for UART0_RXD*/
#define UART0_RXD_PB8            SYS_GPB_MFPH_PB8MFP_UART0_RXD        /*!< GPB_MFPH PB8 setting for UART0_RXD*/
#define UART0_RXD_PA4            SYS_GPA_MFPL_PA4MFP_UART0_RXD        /*!< GPA_MFPL PA4 setting for UART0_RXD*/
#define UART0_RXD_PA0            SYS_GPA_MFPL_PA0MFP_UART0_RXD        /*!< GPA_MFPL PA0 setting for UART0_RXD*/
#define UART0_TXD_PF0            SYS_GPF_MFPL_PF0MFP_UART0_TXD        /*!< GPF_MFPL PF0 setting for UART0_TXD*/
#define UART0_TXD_PA5            SYS_GPA_MFPL_PA5MFP_UART0_TXD        /*!< GPA_MFPL PA5 setting for UART0_TXD*/
#define UART0_TXD_PD3            SYS_GPD_MFPL_PD3MFP_UART0_TXD        /*!< GPD_MFPL PD3 setting for UART0_TXD*/
#define UART0_TXD_PB9            SYS_GPB_MFPH_PB9MFP_UART0_TXD        /*!< GPB_MFPH PB9 setting for UART0_TXD*/
#define UART0_TXD_PF3            SYS_GPF_MFPL_PF3MFP_UART0_TXD        /*!< GPF_MFPL PF3 setting for UART0_TXD*/
#define UART0_TXD_PA1            SYS_GPA_MFPL_PA1MFP_UART0_TXD        /*!< GPA_MFPL PA1 setting for UART0_TXD*/
#define UART0_TXD_PB13           SYS_GPB_MFPH_PB13MFP_UART0_TXD       /*!< GPB_MFPH PB13 setting for UART0_TXD*/
#define UART0_TXD_PA7            SYS_GPA_MFPL_PA7MFP_UART0_TXD        /*!< GPA_MFPL PA7 setting for UART0_TXD*/
#define UART0_nCTS_PA5           SYS_GPA_MFPL_PA5MFP_UART0_nCTS       /*!< GPA_MFPL PA5 setting for UART0_nCTS*/
#define UART0_nCTS_PC7           SYS_GPC_MFPL_PC7MFP_UART0_nCTS       /*!< GPC_MFPL PC7 setting for UART0_nCTS*/
#define UART0_nCTS_PB11          SYS_GPB_MFPH_PB11MFP_UART0_nCTS      /*!< GPB_MFPH PB11 setting for UART0_nCTS*/
#define UART0_nCTS_PB15          SYS_GPB_MFPH_PB15MFP_UART0_nCTS      /*!< GPB_MFPH PB15 setting for UART0_nCTS*/
#define UART0_nRTS_PB10          SYS_GPB_MFPH_PB10MFP_UART0_nRTS      /*!< GPB_MFPH PB10 setting for UART0_nRTS*/
#define UART0_nRTS_PB14          SYS_GPB_MFPH_PB14MFP_UART0_nRTS      /*!< GPB_MFPH PB14 setting for UART0_nRTS*/
#define UART0_nRTS_PC6           SYS_GPC_MFPL_PC6MFP_UART0_nRTS       /*!< GPC_MFPL PC6 setting for UART0_nRTS*/
#define UART0_nRTS_PA4           SYS_GPA_MFPL_PA4MFP_UART0_nRTS       /*!< GPA_MFPL PA4 setting for UART0_nRTS*/
#define UART1_RXD_PA8            SYS_GPA_MFPH_PA8MFP_UART1_RXD        /*!< GPA_MFPH PA8 setting for UART1_RXD*/
#define UART1_RXD_PF2            SYS_GPF_MFPL_PF2MFP_UART1_RXD        /*!< GPF_MFPL PF2 setting for UART1_RXD*/
#define UART1_RXD_PF1            SYS_GPF_MFPL_PF1MFP_UART1_RXD        /*!< GPF_MFPL PF1 setting for UART1_RXD*/
#define UART1_RXD_PB2            SYS_GPB_MFPL_PB2MFP_UART1_RXD        /*!< GPB_MFPL PB2 setting for UART1_RXD*/
#define UART1_RXD_PA2            SYS_GPA_MFPL_PA2MFP_UART1_RXD        /*!< GPA_MFPL PA2 setting for UART1_RXD*/
#define UART1_RXD_PB6            SYS_GPB_MFPL_PB6MFP_UART1_RXD        /*!< GPB_MFPL PB6 setting for UART1_RXD*/
#define UART1_TXD_PB7            SYS_GPB_MFPL_PB7MFP_UART1_TXD        /*!< GPB_MFPL PB7 setting for UART1_TXD*/
#define UART1_TXD_PF0            SYS_GPF_MFPL_PF0MFP_UART1_TXD        /*!< GPF_MFPL PF0 setting for UART1_TXD*/
#define UART1_TXD_PA9            SYS_GPA_MFPH_PA9MFP_UART1_TXD        /*!< GPA_MFPH PA9 setting for UART1_TXD*/
#define UART1_TXD_PB3            SYS_GPB_MFPL_PB3MFP_UART1_TXD        /*!< GPB_MFPL PB3 setting for UART1_TXD*/
#define UART1_TXD_PA3            SYS_GPA_MFPL_PA3MFP_UART1_TXD        /*!< GPA_MFPL PA3 setting for UART1_TXD*/
#define UART1_nCTS_PA1           SYS_GPA_MFPL_PA1MFP_UART1_nCTS       /*!< GPA_MFPL PA1 setting for UART1_nCTS*/
#define UART1_nCTS_PB9           SYS_GPB_MFPH_PB9MFP_UART1_nCTS       /*!< GPB_MFPH PB9 setting for UART1_nCTS*/
#define UART1_nRTS_PA0           SYS_GPA_MFPL_PA0MFP_UART1_nRTS       /*!< GPA_MFPL PA0 setting for UART1_nRTS*/
#define UART1_nRTS_PB8           SYS_GPB_MFPH_PB8MFP_UART1_nRTS       /*!< GPB_MFPH PB8 setting for UART1_nRTS*/
#define UART2_RXD_PF1            SYS_GPF_MFPL_PF1MFP_UART2_RXD        /*!< GPF_MFPL PF1 setting for UART2_RXD*/
#define UART2_RXD_PC0            SYS_GPC_MFPL_PC0MFP_UART2_RXD        /*!< GPC_MFPL PC0 setting for UART2_RXD*/
#define UART2_RXD_PF5            SYS_GPF_MFPL_PF5MFP_UART2_RXD        /*!< GPF_MFPL PF5 setting for UART2_RXD*/
#define UART2_RXD_PC4            SYS_GPC_MFPL_PC4MFP_UART2_RXD        /*!< GPC_MFPL PC4 setting for UART2_RXD*/
#define UART2_RXD_PF3            SYS_GPF_MFPL_PF3MFP_UART2_RXD        /*!< GPF_MFPL PF3 setting for UART2_RXD*/
#define UART2_RXD_PB4            SYS_GPB_MFPL_PB4MFP_UART2_RXD        /*!< GPB_MFPL PB4 setting for UART2_RXD*/
#define UART2_RXD_PB0            SYS_GPB_MFPL_PB0MFP_UART2_RXD        /*!< GPB_MFPL PB0 setting for UART2_RXD*/
#define UART2_TXD_PB1            SYS_GPB_MFPL_PB1MFP_UART2_TXD        /*!< GPB_MFPL PB1 setting for UART2_TXD*/
#define UART2_TXD_PF0            SYS_GPF_MFPL_PF0MFP_UART2_TXD        /*!< GPF_MFPL PF0 setting for UART2_TXD*/
#define UART2_TXD_PF4            SYS_GPF_MFPL_PF4MFP_UART2_TXD        /*!< GPF_MFPL PF4 setting for UART2_TXD*/
#define UART2_TXD_PC5            SYS_GPC_MFPL_PC5MFP_UART2_TXD        /*!< GPC_MFPL PC5 setting for UART2_TXD*/
#define UART2_TXD_PB5            SYS_GPB_MFPL_PB5MFP_UART2_TXD        /*!< GPB_MFPL PB5 setting for UART2_TXD*/
#define UART2_TXD_PC1            SYS_GPC_MFPL_PC1MFP_UART2_TXD        /*!< GPC_MFPL PC1 setting for UART2_TXD*/
#define UART2_nCTS_PC2           SYS_GPC_MFPL_PC2MFP_UART2_nCTS       /*!< GPC_MFPL PC2 setting for UART2_nCTS*/
#define UART2_nCTS_PF5           SYS_GPF_MFPL_PF5MFP_UART2_nCTS       /*!< GPF_MFPL PF5 setting for UART2_nCTS*/
#define UART2_nRTS_PC3           SYS_GPC_MFPL_PC3MFP_UART2_nRTS       /*!< GPC_MFPL PC3 setting for UART2_nRTS*/
#define UART2_nRTS_PF4           SYS_GPF_MFPL_PF4MFP_UART2_nRTS       /*!< GPF_MFPL PF4 setting for UART2_nRTS*/
#define VDET_P0_PB0              SYS_GPB_MFPL_PB0MFP_VDET_P0          /*!< GPB_MFPL PB0 setting for VDET_P0*/
#define VDET_P1_PB1              SYS_GPB_MFPL_PB1MFP_VDET_P1          /*!< GPB_MFPL PB1 setting for VDET_P1*/
#define X32_IN_PF5               SYS_GPF_MFPL_PF5MFP_X32_IN           /*!< GPF_MFPL PF5 setting for X32_IN*/
#define X32_OUT_PF4              SYS_GPF_MFPL_PF4MFP_X32_OUT          /*!< GPF_MFPL PF4 setting for X32_OUT*/
#define XT1_IN_PF3               SYS_GPF_MFPL_PF3MFP_XT1_IN           /*!< GPF_MFPL PF3 setting for XT1_IN*/
#define XT1_OUT_PF2              SYS_GPF_MFPL_PF2MFP_XT1_OUT          /*!< GPF_MFPL PF2 setting for XT1_OUT*/


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function setting mask constant definitions abbreviation.                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define ACMP0_N_PB4_Msk         SYS_GPB_MFPL_PB4MFP_Msk        /*!< ACMP0_N         PB4      MFP Mask */
#define ACMP0_O_PB7_Msk         SYS_GPB_MFPL_PB7MFP_Msk        /*!< ACMP0_O         PB7      MFP Mask */
#define ACMP0_O_PC1_Msk         SYS_GPC_MFPL_PC1MFP_Msk        /*!< ACMP0_O         PC1      MFP Mask */
#define ACMP0_O_PF0_Msk         SYS_GPF_MFPL_PF0MFP_Msk        /*!< ACMP0_O         PF0      MFP Mask */
#define ACMP0_P0_PB2_Msk        SYS_GPB_MFPL_PB2MFP_Msk        /*!< ACMP0_P0        PB2      MFP Mask */
#define ACMP0_P1_PB3_Msk        SYS_GPB_MFPL_PB3MFP_Msk        /*!< ACMP0_P1        PB3      MFP Mask */
#define ACMP0_P2_PB12_Msk       SYS_GPB_MFPH_PB12MFP_Msk       /*!< ACMP0_P2        PB12     MFP Mask */
#define ACMP0_P3_PB13_Msk       SYS_GPB_MFPH_PB13MFP_Msk       /*!< ACMP0_P3        PB13     MFP Mask */
#define ACMP0_WLAT_PA7_Msk      SYS_GPA_MFPL_PA7MFP_Msk        /*!< ACMP0_WLAT      PA7      MFP Mask */
#define ACMP1_N_PB5_Msk         SYS_GPB_MFPL_PB5MFP_Msk        /*!< ACMP1_N         PB5      MFP Mask */
#define ACMP1_O_PB6_Msk         SYS_GPB_MFPL_PB6MFP_Msk        /*!< ACMP1_O         PB6      MFP Mask */
#define ACMP1_O_PC0_Msk         SYS_GPC_MFPL_PC0MFP_Msk        /*!< ACMP1_O         PC0      MFP Mask */
#define ACMP1_O_PF1_Msk         SYS_GPF_MFPL_PF1MFP_Msk        /*!< ACMP1_O         PF1      MFP Mask */
#define ACMP1_P0_PB3_Msk        SYS_GPB_MFPL_PB3MFP_Msk        /*!< ACMP1_P0        PB3      MFP Mask */
#define ACMP1_P1_PB4_Msk        SYS_GPB_MFPL_PB4MFP_Msk        /*!< ACMP1_P1        PB4      MFP Mask */
#define ACMP1_P2_PB12_Msk       SYS_GPB_MFPH_PB12MFP_Msk       /*!< ACMP1_P2        PB12     MFP Mask */
#define ACMP1_P3_PB13_Msk       SYS_GPB_MFPH_PB13MFP_Msk       /*!< ACMP1_P3        PB13     MFP Mask */
#define ACMP1_WLAT_PA6_Msk      SYS_GPA_MFPL_PA6MFP_Msk        /*!< ACMP1_WLAT      PA6      MFP Mask */
#define ACMP2_N_PB6_Msk         SYS_GPB_MFPL_PB6MFP_Msk        /*!< ACMP2_N         PB6      MFP Mask */
#define ACMP2_O_PA1_Msk         SYS_GPA_MFPL_PA1MFP_Msk        /*!< ACMP2_O         PA1      MFP Mask */
#define ACMP2_O_PF3_Msk         SYS_GPF_MFPL_PF3MFP_Msk        /*!< ACMP2_O         PF3      MFP Mask */
#define ACMP2_P0_PB4_Msk        SYS_GPB_MFPL_PB4MFP_Msk        /*!< ACMP2_P0        PB4      MFP Mask */
#define ACMP2_P1_PB5_Msk        SYS_GPB_MFPL_PB5MFP_Msk        /*!< ACMP2_P1        PB5      MFP Mask */
#define ACMP2_P2_PB14_Msk       SYS_GPB_MFPH_PB14MFP_Msk       /*!< ACMP2_P2        PB14     MFP Mask */
#define ACMP2_P3_PB15_Msk       SYS_GPB_MFPH_PB15MFP_Msk       /*!< ACMP2_P3        PB15     MFP Mask */
#define ACMP2_WLAT_PA3_Msk      SYS_GPA_MFPL_PA3MFP_Msk        /*!< ACMP2_WLAT      PA3      MFP Mask */
#define ACMP2_WLAT_PF5_Msk      SYS_GPF_MFPL_PF5MFP_Msk        /*!< ACMP2_WLAT      PF5      MFP Mask */
#define ACMP3_N_PB7_Msk         SYS_GPB_MFPL_PB7MFP_Msk        /*!< ACMP3_N         PB7      MFP Mask */
#define ACMP3_O_PA0_Msk         SYS_GPA_MFPL_PA0MFP_Msk        /*!< ACMP3_O         PA0      MFP Mask */
#define ACMP3_O_PF2_Msk         SYS_GPF_MFPL_PF2MFP_Msk        /*!< ACMP3_O         PF2      MFP Mask */
#define ACMP3_P0_PB5_Msk        SYS_GPB_MFPL_PB5MFP_Msk        /*!< ACMP3_P0        PB5      MFP Mask */
#define ACMP3_P1_PB6_Msk        SYS_GPB_MFPL_PB6MFP_Msk        /*!< ACMP3_P1        PB6      MFP Mask */
#define ACMP3_P2_PB14_Msk       SYS_GPB_MFPH_PB14MFP_Msk       /*!< ACMP3_P2        PB14     MFP Mask */
#define ACMP3_P3_PB15_Msk       SYS_GPB_MFPH_PB15MFP_Msk       /*!< ACMP3_P3        PB15     MFP Mask */
#define ACMP3_WLAT_PF4_Msk      SYS_GPF_MFPL_PF4MFP_Msk        /*!< ACMP3_WLAT      PF4      MFP Mask */
#define ACMP3_WLAT_PA2_Msk      SYS_GPA_MFPL_PA2MFP_Msk        /*!< ACMP3_WLAT      PA2      MFP Mask */
#define ADC0_CH0_PB0_Msk        SYS_GPB_MFPL_PB0MFP_Msk        /*!< ADC0_CH0        PB0      MFP Mask */
#define ADC0_CH1_PB1_Msk        SYS_GPB_MFPL_PB1MFP_Msk        /*!< ADC0_CH1        PB1      MFP Mask */
#define ADC0_CH10_PB10_Msk      SYS_GPB_MFPH_PB10MFP_Msk       /*!< ADC0_CH10       PB10     MFP Mask */
#define ADC0_CH11_PB11_Msk      SYS_GPB_MFPH_PB11MFP_Msk       /*!< ADC0_CH11       PB11     MFP Mask */
#define ADC0_CH12_PB12_Msk      SYS_GPB_MFPH_PB12MFP_Msk       /*!< ADC0_CH12       PB12     MFP Mask */
#define ADC0_CH13_PB13_Msk      SYS_GPB_MFPH_PB13MFP_Msk       /*!< ADC0_CH13       PB13     MFP Mask */
#define ADC0_CH14_PB14_Msk      SYS_GPB_MFPH_PB14MFP_Msk       /*!< ADC0_CH14       PB14     MFP Mask */
#define ADC0_CH15_PB15_Msk      SYS_GPB_MFPH_PB15MFP_Msk       /*!< ADC0_CH15       PB15     MFP Mask */
#define ADC0_CH2_PB2_Msk        SYS_GPB_MFPL_PB2MFP_Msk        /*!< ADC0_CH2        PB2      MFP Mask */
#define ADC0_CH3_PB3_Msk        SYS_GPB_MFPL_PB3MFP_Msk        /*!< ADC0_CH3        PB3      MFP Mask */
#define ADC0_CH4_PB4_Msk        SYS_GPB_MFPL_PB4MFP_Msk        /*!< ADC0_CH4        PB4      MFP Mask */
#define ADC0_CH5_PB5_Msk        SYS_GPB_MFPL_PB5MFP_Msk        /*!< ADC0_CH5        PB5      MFP Mask */
#define ADC0_CH6_PB6_Msk        SYS_GPB_MFPL_PB6MFP_Msk        /*!< ADC0_CH6        PB6      MFP Mask */
#define ADC0_CH7_PB7_Msk        SYS_GPB_MFPL_PB7MFP_Msk        /*!< ADC0_CH7        PB7      MFP Mask */
#define ADC0_CH8_PB8_Msk        SYS_GPB_MFPH_PB8MFP_Msk        /*!< ADC0_CH8        PB8      MFP Mask */
#define ADC0_CH9_PB9_Msk        SYS_GPB_MFPH_PB9MFP_Msk        /*!< ADC0_CH9        PB9      MFP Mask */
#define ADC0_ST_PF5_Msk         SYS_GPF_MFPL_PF5MFP_Msk        /*!< ADC0_ST         PF5      MFP Mask */
#define ADC0_ST_PC1_Msk         SYS_GPC_MFPL_PC1MFP_Msk        /*!< ADC0_ST         PC1      MFP Mask */
#define BPWM0_CH0_PA0_Msk       SYS_GPA_MFPL_PA0MFP_Msk        /*!< BPWM0_CH0       PA0      MFP Mask */
#define BPWM0_CH0_PA11_Msk      SYS_GPA_MFPH_PA11MFP_Msk       /*!< BPWM0_CH0       PA11     MFP Mask */
#define BPWM0_CH1_PA10_Msk      SYS_GPA_MFPH_PA10MFP_Msk       /*!< BPWM0_CH1       PA10     MFP Mask */
#define BPWM0_CH1_PA1_Msk       SYS_GPA_MFPL_PA1MFP_Msk        /*!< BPWM0_CH1       PA1      MFP Mask */
#define BPWM0_CH2_PA2_Msk       SYS_GPA_MFPL_PA2MFP_Msk        /*!< BPWM0_CH2       PA2      MFP Mask */
#define BPWM0_CH2_PA9_Msk       SYS_GPA_MFPH_PA9MFP_Msk        /*!< BPWM0_CH2       PA9      MFP Mask */
#define BPWM0_CH3_PA3_Msk       SYS_GPA_MFPL_PA3MFP_Msk        /*!< BPWM0_CH3       PA3      MFP Mask */
#define BPWM0_CH3_PF3_Msk       SYS_GPF_MFPL_PF3MFP_Msk        /*!< BPWM0_CH3       PF3      MFP Mask */
#define BPWM0_CH3_PA8_Msk       SYS_GPA_MFPH_PA8MFP_Msk        /*!< BPWM0_CH3       PA8      MFP Mask */
#define BPWM0_CH4_PA4_Msk       SYS_GPA_MFPL_PA4MFP_Msk        /*!< BPWM0_CH4       PA4      MFP Mask */
#define BPWM0_CH4_PF5_Msk       SYS_GPF_MFPL_PF5MFP_Msk        /*!< BPWM0_CH4       PF5      MFP Mask */
#define BPWM0_CH5_PA5_Msk       SYS_GPA_MFPL_PA5MFP_Msk        /*!< BPWM0_CH5       PA5      MFP Mask */
#define BPWM0_CH5_PF4_Msk       SYS_GPF_MFPL_PF4MFP_Msk        /*!< BPWM0_CH5       PF4      MFP Mask */
#define BPWM1_CH0_PB11_Msk      SYS_GPB_MFPH_PB11MFP_Msk       /*!< BPWM1_CH0       PB11     MFP Mask */
#define BPWM1_CH0_PF3_Msk       SYS_GPF_MFPL_PF3MFP_Msk        /*!< BPWM1_CH0       PF3      MFP Mask */
#define BPWM1_CH0_PC7_Msk       SYS_GPC_MFPL_PC7MFP_Msk        /*!< BPWM1_CH0       PC7      MFP Mask */
#define BPWM1_CH0_PF0_Msk       SYS_GPF_MFPL_PF0MFP_Msk        /*!< BPWM1_CH0       PF0      MFP Mask */
#define BPWM1_CH1_PC6_Msk       SYS_GPC_MFPL_PC6MFP_Msk        /*!< BPWM1_CH1       PC6      MFP Mask */
#define BPWM1_CH1_PF2_Msk       SYS_GPF_MFPL_PF2MFP_Msk        /*!< BPWM1_CH1       PF2      MFP Mask */
#define BPWM1_CH1_PB10_Msk      SYS_GPB_MFPH_PB10MFP_Msk       /*!< BPWM1_CH1       PB10     MFP Mask */
#define BPWM1_CH1_PF1_Msk       SYS_GPF_MFPL_PF1MFP_Msk        /*!< BPWM1_CH1       PF1      MFP Mask */
#define BPWM1_CH2_PA7_Msk       SYS_GPA_MFPL_PA7MFP_Msk        /*!< BPWM1_CH2       PA7      MFP Mask */
#define BPWM1_CH2_PB9_Msk       SYS_GPB_MFPH_PB9MFP_Msk        /*!< BPWM1_CH2       PB9      MFP Mask */
#define BPWM1_CH3_PA6_Msk       SYS_GPA_MFPL_PA6MFP_Msk        /*!< BPWM1_CH3       PA6      MFP Mask */
#define BPWM1_CH3_PB8_Msk       SYS_GPB_MFPH_PB8MFP_Msk        /*!< BPWM1_CH3       PB8      MFP Mask */
#define BPWM1_CH4_PB7_Msk       SYS_GPB_MFPL_PB7MFP_Msk        /*!< BPWM1_CH4       PB7      MFP Mask */
#define BPWM1_CH5_PB6_Msk       SYS_GPB_MFPL_PB6MFP_Msk        /*!< BPWM1_CH5       PB6      MFP Mask */
#define BPWM2_CH0_PA5_Msk       SYS_GPA_MFPL_PA5MFP_Msk        /*!< BPWM2_CH0       PA5      MFP Mask */
#define BPWM2_CH0_PF5_Msk       SYS_GPF_MFPL_PF5MFP_Msk        /*!< BPWM2_CH0       PF5      MFP Mask */
#define BPWM2_CH0_PB5_Msk       SYS_GPB_MFPL_PB5MFP_Msk        /*!< BPWM2_CH0       PB5      MFP Mask */
#define BPWM2_CH1_PF4_Msk       SYS_GPF_MFPL_PF4MFP_Msk        /*!< BPWM2_CH1       PF4      MFP Mask */
#define BPWM2_CH1_PA4_Msk       SYS_GPA_MFPL_PA4MFP_Msk        /*!< BPWM2_CH1       PA4      MFP Mask */
#define BPWM2_CH1_PB4_Msk       SYS_GPB_MFPL_PB4MFP_Msk        /*!< BPWM2_CH1       PB4      MFP Mask */
#define BPWM2_CH1_PC1_Msk       SYS_GPC_MFPL_PC1MFP_Msk        /*!< BPWM2_CH1       PC1      MFP Mask */
#define BPWM2_CH2_PA3_Msk       SYS_GPA_MFPL_PA3MFP_Msk        /*!< BPWM2_CH2       PA3      MFP Mask */
#define BPWM2_CH2_PB3_Msk       SYS_GPB_MFPL_PB3MFP_Msk        /*!< BPWM2_CH2       PB3      MFP Mask */
#define BPWM2_CH3_PA2_Msk       SYS_GPA_MFPL_PA2MFP_Msk        /*!< BPWM2_CH3       PA2      MFP Mask */
#define BPWM2_CH3_PB2_Msk       SYS_GPB_MFPL_PB2MFP_Msk        /*!< BPWM2_CH3       PB2      MFP Mask */
#define BPWM2_CH4_PB1_Msk       SYS_GPB_MFPL_PB1MFP_Msk        /*!< BPWM2_CH4       PB1      MFP Mask */
#define BPWM2_CH4_PF6_Msk       SYS_GPF_MFPL_PF6MFP_Msk        /*!< BPWM2_CH4       PF6      MFP Mask */
#define BPWM2_CH4_PA1_Msk       SYS_GPA_MFPL_PA1MFP_Msk        /*!< BPWM2_CH4       PA1      MFP Mask */
#define BPWM2_CH5_PD15_Msk      SYS_GPD_MFPH_PD15MFP_Msk       /*!< BPWM2_CH5       PD15     MFP Mask */
#define BPWM2_CH5_PA0_Msk       SYS_GPA_MFPL_PA0MFP_Msk        /*!< BPWM2_CH5       PA0      MFP Mask */
#define BPWM2_CH5_PB0_Msk       SYS_GPB_MFPL_PB0MFP_Msk        /*!< BPWM2_CH5       PB0      MFP Mask */
#define BPWM2_CH5_PF4_Msk       SYS_GPF_MFPL_PF4MFP_Msk        /*!< BPWM2_CH5       PF4      MFP Mask */
#define BPWM3_CH0_PC5_Msk       SYS_GPC_MFPL_PC5MFP_Msk        /*!< BPWM3_CH0       PC5      MFP Mask */
#define BPWM3_CH0_PB15_Msk      SYS_GPB_MFPH_PB15MFP_Msk       /*!< BPWM3_CH0       PB15     MFP Mask */
#define BPWM3_CH1_PB14_Msk      SYS_GPB_MFPH_PB14MFP_Msk       /*!< BPWM3_CH1       PB14     MFP Mask */
#define BPWM3_CH1_PC4_Msk       SYS_GPC_MFPL_PC4MFP_Msk        /*!< BPWM3_CH1       PC4      MFP Mask */
#define BPWM3_CH2_PB13_Msk      SYS_GPB_MFPH_PB13MFP_Msk       /*!< BPWM3_CH2       PB13     MFP Mask */
#define BPWM3_CH2_PC7_Msk       SYS_GPC_MFPL_PC7MFP_Msk        /*!< BPWM3_CH2       PC7      MFP Mask */
#define BPWM3_CH2_PC3_Msk       SYS_GPC_MFPL_PC3MFP_Msk        /*!< BPWM3_CH2       PC3      MFP Mask */
#define BPWM3_CH3_PB12_Msk      SYS_GPB_MFPH_PB12MFP_Msk       /*!< BPWM3_CH3       PB12     MFP Mask */
#define BPWM3_CH3_PC0_Msk       SYS_GPC_MFPL_PC0MFP_Msk        /*!< BPWM3_CH3       PC0      MFP Mask */
#define BPWM3_CH3_PC2_Msk       SYS_GPC_MFPL_PC2MFP_Msk        /*!< BPWM3_CH3       PC2      MFP Mask */
#define BPWM3_CH3_PC6_Msk       SYS_GPC_MFPL_PC6MFP_Msk        /*!< BPWM3_CH3       PC6      MFP Mask */
#define BPWM3_CH4_PC1_Msk       SYS_GPC_MFPL_PC1MFP_Msk        /*!< BPWM3_CH4       PC1      MFP Mask */
#define BPWM3_CH4_PA7_Msk       SYS_GPA_MFPL_PA7MFP_Msk        /*!< BPWM3_CH4       PA7      MFP Mask */
#define BPWM3_CH4_PB7_Msk       SYS_GPB_MFPL_PB7MFP_Msk        /*!< BPWM3_CH4       PB7      MFP Mask */
#define BPWM3_CH4_PB1_Msk       SYS_GPB_MFPL_PB1MFP_Msk        /*!< BPWM3_CH4       PB1      MFP Mask */
#define BPWM3_CH5_PB6_Msk       SYS_GPB_MFPL_PB6MFP_Msk        /*!< BPWM3_CH5       PB6      MFP Mask */
#define BPWM3_CH5_PF4_Msk       SYS_GPF_MFPL_PF4MFP_Msk        /*!< BPWM3_CH5       PF4      MFP Mask */
#define BPWM3_CH5_PC0_Msk       SYS_GPC_MFPL_PC0MFP_Msk        /*!< BPWM3_CH5       PC0      MFP Mask */
#define BPWM3_CH5_PA6_Msk       SYS_GPA_MFPL_PA6MFP_Msk        /*!< BPWM3_CH5       PA6      MFP Mask */
#define BPWM3_CH5_PB0_Msk       SYS_GPB_MFPL_PB0MFP_Msk        /*!< BPWM3_CH5       PB0      MFP Mask */
#define CLKO_PF6_Msk            SYS_GPF_MFPL_PF6MFP_Msk        /*!< CLKO            PF6      MFP Mask */
#define CLKO_PB14_Msk           SYS_GPB_MFPH_PB14MFP_Msk       /*!< CLKO            PB14     MFP Mask */
#define CLKO_PA3_Msk            SYS_GPA_MFPL_PA3MFP_Msk        /*!< CLKO            PA3      MFP Mask */
#define DAC0_OUT_PB12_Msk       SYS_GPB_MFPH_PB12MFP_Msk       /*!< DAC0_OUT        PB12     MFP Mask */
#define DAC0_ST_PA10_Msk        SYS_GPA_MFPH_PA10MFP_Msk       /*!< DAC0_ST         PA10     MFP Mask */
#define DAC0_ST_PA0_Msk         SYS_GPA_MFPL_PA0MFP_Msk        /*!< DAC0_ST         PA0      MFP Mask */
#define DAC1_OUT_PB13_Msk       SYS_GPB_MFPH_PB13MFP_Msk       /*!< DAC1_OUT        PB13     MFP Mask */
#define DAC1_ST_PC14_Msk        SYS_GPC_MFPH_PC14MFP_Msk       /*!< DAC1_ST         PC14     MFP Mask */
#define DAC1_ST_PA11_Msk        SYS_GPA_MFPH_PA11MFP_Msk       /*!< DAC1_ST         PA11     MFP Mask */
#define DAC2_OUT_PB14_Msk       SYS_GPB_MFPH_PB14MFP_Msk       /*!< DAC2_OUT        PB14     MFP Mask */
#define DAC2_ST_PA9_Msk         SYS_GPA_MFPH_PA9MFP_Msk        /*!< DAC2_ST         PA9      MFP Mask */
#define DAC2_ST_PA5_Msk         SYS_GPA_MFPL_PA5MFP_Msk        /*!< DAC2_ST         PA5      MFP Mask */
#define DAC3_OUT_PB15_Msk       SYS_GPB_MFPH_PB15MFP_Msk       /*!< DAC3_OUT        PB15     MFP Mask */
#define DAC3_ST_PA4_Msk         SYS_GPA_MFPL_PA4MFP_Msk        /*!< DAC3_ST         PA4      MFP Mask */
#define DAC3_ST_PA8_Msk         SYS_GPA_MFPH_PA8MFP_Msk        /*!< DAC3_ST         PA8      MFP Mask */
#define I2C0_SCL_PA8_Msk        SYS_GPA_MFPH_PA8MFP_Msk        /*!< I2C0_SCL        PA8      MFP Mask */
#define I2C0_SCL_PB5_Msk        SYS_GPB_MFPL_PB5MFP_Msk        /*!< I2C0_SCL        PB5      MFP Mask */
#define I2C0_SCL_PB9_Msk        SYS_GPB_MFPH_PB9MFP_Msk        /*!< I2C0_SCL        PB9      MFP Mask */
#define I2C0_SCL_PF3_Msk        SYS_GPF_MFPL_PF3MFP_Msk        /*!< I2C0_SCL        PF3      MFP Mask */
#define I2C0_SCL_PA5_Msk        SYS_GPA_MFPL_PA5MFP_Msk        /*!< I2C0_SCL        PA5      MFP Mask */
#define I2C0_SCL_PC1_Msk        SYS_GPC_MFPL_PC1MFP_Msk        /*!< I2C0_SCL        PC1      MFP Mask */
#define I2C0_SCL_PC3_Msk        SYS_GPC_MFPL_PC3MFP_Msk        /*!< I2C0_SCL        PC3      MFP Mask */
#define I2C0_SCL_PF5_Msk        SYS_GPF_MFPL_PF5MFP_Msk        /*!< I2C0_SCL        PF5      MFP Mask */
#define I2C0_SDA_PA4_Msk        SYS_GPA_MFPL_PA4MFP_Msk        /*!< I2C0_SDA        PA4      MFP Mask */
#define I2C0_SDA_PF6_Msk        SYS_GPF_MFPL_PF6MFP_Msk        /*!< I2C0_SDA        PF6      MFP Mask */
#define I2C0_SDA_PC0_Msk        SYS_GPC_MFPL_PC0MFP_Msk        /*!< I2C0_SDA        PC0      MFP Mask */
#define I2C0_SDA_PC2_Msk        SYS_GPC_MFPL_PC2MFP_Msk        /*!< I2C0_SDA        PC2      MFP Mask */
#define I2C0_SDA_PF2_Msk        SYS_GPF_MFPL_PF2MFP_Msk        /*!< I2C0_SDA        PF2      MFP Mask */
#define I2C0_SDA_PB8_Msk        SYS_GPB_MFPH_PB8MFP_Msk        /*!< I2C0_SDA        PB8      MFP Mask */
#define I2C0_SDA_PB4_Msk        SYS_GPB_MFPL_PB4MFP_Msk        /*!< I2C0_SDA        PB4      MFP Mask */
#define I2C0_SDA_PF4_Msk        SYS_GPF_MFPL_PF4MFP_Msk        /*!< I2C0_SDA        PF4      MFP Mask */
#define I2C0_SMBAL_PC3_Msk      SYS_GPC_MFPL_PC3MFP_Msk        /*!< I2C0_SMBAL      PC3      MFP Mask */
#define I2C0_SMBAL_PA3_Msk      SYS_GPA_MFPL_PA3MFP_Msk        /*!< I2C0_SMBAL      PA3      MFP Mask */
#define I2C0_SMBSUS_PA2_Msk     SYS_GPA_MFPL_PA2MFP_Msk        /*!< I2C0_SMBSUS     PA2      MFP Mask */
#define I2C0_SMBSUS_PC2_Msk     SYS_GPC_MFPL_PC2MFP_Msk        /*!< I2C0_SMBSUS     PC2      MFP Mask */
#define I2C1_SCL_PC1_Msk        SYS_GPC_MFPL_PC1MFP_Msk        /*!< I2C1_SCL        PC1      MFP Mask */
#define I2C1_SCL_PA10_Msk       SYS_GPA_MFPH_PA10MFP_Msk       /*!< I2C1_SCL        PA10     MFP Mask */
#define I2C1_SCL_PA7_Msk        SYS_GPA_MFPL_PA7MFP_Msk        /*!< I2C1_SCL        PA7      MFP Mask */
#define I2C1_SCL_PA3_Msk        SYS_GPA_MFPL_PA3MFP_Msk        /*!< I2C1_SCL        PA3      MFP Mask */
#define I2C1_SCL_PB11_Msk       SYS_GPB_MFPH_PB11MFP_Msk       /*!< I2C1_SCL        PB11     MFP Mask */
#define I2C1_SCL_PB1_Msk        SYS_GPB_MFPL_PB1MFP_Msk        /*!< I2C1_SCL        PB1      MFP Mask */
#define I2C1_SCL_PF0_Msk        SYS_GPF_MFPL_PF0MFP_Msk        /*!< I2C1_SCL        PF0      MFP Mask */
#define I2C1_SCL_PC5_Msk        SYS_GPC_MFPL_PC5MFP_Msk        /*!< I2C1_SCL        PC5      MFP Mask */
#define I2C1_SCL_PB3_Msk        SYS_GPB_MFPL_PB3MFP_Msk        /*!< I2C1_SCL        PB3      MFP Mask */
#define I2C1_SDA_PC0_Msk        SYS_GPC_MFPL_PC0MFP_Msk        /*!< I2C1_SDA        PC0      MFP Mask */
#define I2C1_SDA_PB0_Msk        SYS_GPB_MFPL_PB0MFP_Msk        /*!< I2C1_SDA        PB0      MFP Mask */
#define I2C1_SDA_PA2_Msk        SYS_GPA_MFPL_PA2MFP_Msk        /*!< I2C1_SDA        PA2      MFP Mask */
#define I2C1_SDA_PC4_Msk        SYS_GPC_MFPL_PC4MFP_Msk        /*!< I2C1_SDA        PC4      MFP Mask */
#define I2C1_SDA_PB2_Msk        SYS_GPB_MFPL_PB2MFP_Msk        /*!< I2C1_SDA        PB2      MFP Mask */
#define I2C1_SDA_PA9_Msk        SYS_GPA_MFPH_PA9MFP_Msk        /*!< I2C1_SDA        PA9      MFP Mask */
#define I2C1_SDA_PB10_Msk       SYS_GPB_MFPH_PB10MFP_Msk       /*!< I2C1_SDA        PB10     MFP Mask */
#define I2C1_SDA_PA6_Msk        SYS_GPA_MFPL_PA6MFP_Msk        /*!< I2C1_SDA        PA6      MFP Mask */
#define I2C1_SDA_PF1_Msk        SYS_GPF_MFPL_PF1MFP_Msk        /*!< I2C1_SDA        PF1      MFP Mask */
#define I2C1_SMBAL_PC7_Msk      SYS_GPC_MFPL_PC7MFP_Msk        /*!< I2C1_SMBAL      PC7      MFP Mask */
#define I2C1_SMBAL_PB9_Msk      SYS_GPB_MFPH_PB9MFP_Msk        /*!< I2C1_SMBAL      PB9      MFP Mask */
#define I2C1_SMBAL_PF0_Msk      SYS_GPF_MFPL_PF0MFP_Msk        /*!< I2C1_SMBAL      PF0      MFP Mask */
#define I2C1_SMBSUS_PB8_Msk     SYS_GPB_MFPH_PB8MFP_Msk        /*!< I2C1_SMBSUS     PB8      MFP Mask */
#define I2C1_SMBSUS_PC6_Msk     SYS_GPC_MFPL_PC6MFP_Msk        /*!< I2C1_SMBSUS     PC6      MFP Mask */
#define I2C1_SMBSUS_PF1_Msk     SYS_GPF_MFPL_PF1MFP_Msk        /*!< I2C1_SMBSUS     PF1      MFP Mask */
#define I2C2_SCL_PB3_Msk        SYS_GPB_MFPL_PB3MFP_Msk        /*!< I2C2_SCL        PB3      MFP Mask */
#define I2C2_SCL_PA11_Msk       SYS_GPA_MFPH_PA11MFP_Msk       /*!< I2C2_SCL        PA11     MFP Mask */
#define I2C2_SCL_PF0_Msk        SYS_GPF_MFPL_PF0MFP_Msk        /*!< I2C2_SCL        PF0      MFP Mask */
#define I2C2_SCL_PA1_Msk        SYS_GPA_MFPL_PA1MFP_Msk        /*!< I2C2_SCL        PA1      MFP Mask */
#define I2C2_SCL_PB13_Msk       SYS_GPB_MFPH_PB13MFP_Msk       /*!< I2C2_SCL        PB13     MFP Mask */
#define I2C2_SDA_PA0_Msk        SYS_GPA_MFPL_PA0MFP_Msk        /*!< I2C2_SDA        PA0      MFP Mask */
#define I2C2_SDA_PF1_Msk        SYS_GPF_MFPL_PF1MFP_Msk        /*!< I2C2_SDA        PF1      MFP Mask */
#define I2C2_SDA_PA10_Msk       SYS_GPA_MFPH_PA10MFP_Msk       /*!< I2C2_SDA        PA10     MFP Mask */
#define I2C2_SDA_PB2_Msk        SYS_GPB_MFPL_PB2MFP_Msk        /*!< I2C2_SDA        PB2      MFP Mask */
#define I2C2_SDA_PB12_Msk       SYS_GPB_MFPH_PB12MFP_Msk       /*!< I2C2_SDA        PB12     MFP Mask */
#define I2C2_SMBAL_PB15_Msk     SYS_GPB_MFPH_PB15MFP_Msk       /*!< I2C2_SMBAL      PB15     MFP Mask */
#define I2C2_SMBAL_PC3_Msk      SYS_GPC_MFPL_PC3MFP_Msk        /*!< I2C2_SMBAL      PC3      MFP Mask */
#define I2C2_SMBSUS_PB14_Msk    SYS_GPB_MFPH_PB14MFP_Msk       /*!< I2C2_SMBSUS     PB14     MFP Mask */
#define I2C2_SMBSUS_PC2_Msk     SYS_GPC_MFPL_PC2MFP_Msk        /*!< I2C2_SMBSUS     PC2      MFP Mask */
#define I3CS0_SCL_PA5_Msk       SYS_GPA_MFPL_PA5MFP_Msk        /*!< I3CS0_SCL       PA5      MFP Mask */
#define I3CS0_SCL_PA1_Msk       SYS_GPA_MFPL_PA1MFP_Msk        /*!< I3CS0_SCL       PA1      MFP Mask */
#define I3CS0_SDA_PA0_Msk       SYS_GPA_MFPL_PA0MFP_Msk        /*!< I3CS0_SDA       PA0      MFP Mask */
#define I3CS0_SDA_PA4_Msk       SYS_GPA_MFPL_PA4MFP_Msk        /*!< I3CS0_SDA       PA4      MFP Mask */
#define I3CS1_SCL_PA3_Msk       SYS_GPA_MFPL_PA3MFP_Msk        /*!< I3CS1_SCL       PA3      MFP Mask */
#define I3CS1_SDA_PA2_Msk       SYS_GPA_MFPL_PA2MFP_Msk        /*!< I3CS1_SDA       PA2      MFP Mask */
#define ICE_CLK_PF1_Msk         SYS_GPF_MFPL_PF1MFP_Msk        /*!< ICE_CLK         PF1      MFP Mask */
#define ICE_DAT_PF0_Msk         SYS_GPF_MFPL_PF0MFP_Msk        /*!< ICE_DAT         PF0      MFP Mask */
#define INT0_PA6_Msk            SYS_GPA_MFPL_PA6MFP_Msk        /*!< INT0            PA6      MFP Mask */
#define INT0_PB5_Msk            SYS_GPB_MFPL_PB5MFP_Msk        /*!< INT0            PB5      MFP Mask */
#define INT1_PB4_Msk            SYS_GPB_MFPL_PB4MFP_Msk        /*!< INT1            PB4      MFP Mask */
#define INT1_PD15_Msk           SYS_GPD_MFPH_PD15MFP_Msk       /*!< INT1            PD15     MFP Mask */
#define INT1_PA7_Msk            SYS_GPA_MFPL_PA7MFP_Msk        /*!< INT1            PA7      MFP Mask */
#define INT2_PC6_Msk            SYS_GPC_MFPL_PC6MFP_Msk        /*!< INT2            PC6      MFP Mask */
#define INT2_PB3_Msk            SYS_GPB_MFPL_PB3MFP_Msk        /*!< INT2            PB3      MFP Mask */
#define INT3_PB2_Msk            SYS_GPB_MFPL_PB2MFP_Msk        /*!< INT3            PB2      MFP Mask */
#define INT3_PC7_Msk            SYS_GPC_MFPL_PC7MFP_Msk        /*!< INT3            PC7      MFP Mask */
#define INT4_PB6_Msk            SYS_GPB_MFPL_PB6MFP_Msk        /*!< INT4            PB6      MFP Mask */
#define INT4_PF2_Msk            SYS_GPF_MFPL_PF2MFP_Msk        /*!< INT4            PF2      MFP Mask */
#define INT4_PA8_Msk            SYS_GPA_MFPH_PA8MFP_Msk        /*!< INT4            PA8      MFP Mask */
#define INT5_PB7_Msk            SYS_GPB_MFPL_PB7MFP_Msk        /*!< INT5            PB7      MFP Mask */
#define INT5_PF6_Msk            SYS_GPF_MFPL_PF6MFP_Msk        /*!< INT5            PF6      MFP Mask */
#define LLSI0_OUT_PB15_Msk      SYS_GPB_MFPH_PB15MFP_Msk       /*!< LLSI0_OUT       PB15     MFP Mask */
#define LLSI0_OUT_PC5_Msk       SYS_GPC_MFPL_PC5MFP_Msk        /*!< LLSI0_OUT       PC5      MFP Mask */
#define LLSI1_OUT_PC4_Msk       SYS_GPC_MFPL_PC4MFP_Msk        /*!< LLSI1_OUT       PC4      MFP Mask */
#define LLSI1_OUT_PB14_Msk      SYS_GPB_MFPH_PB14MFP_Msk       /*!< LLSI1_OUT       PB14     MFP Mask */
#define LLSI2_OUT_PB13_Msk      SYS_GPB_MFPH_PB13MFP_Msk       /*!< LLSI2_OUT       PB13     MFP Mask */
#define LLSI2_OUT_PC3_Msk       SYS_GPC_MFPL_PC3MFP_Msk        /*!< LLSI2_OUT       PC3      MFP Mask */
#define LLSI3_OUT_PF6_Msk       SYS_GPF_MFPL_PF6MFP_Msk        /*!< LLSI3_OUT       PF6      MFP Mask */
#define LLSI3_OUT_PB12_Msk      SYS_GPB_MFPH_PB12MFP_Msk       /*!< LLSI3_OUT       PB12     MFP Mask */
#define LLSI3_OUT_PC2_Msk       SYS_GPC_MFPL_PC2MFP_Msk        /*!< LLSI3_OUT       PC2      MFP Mask */
#define LLSI4_OUT_PA3_Msk       SYS_GPA_MFPL_PA3MFP_Msk        /*!< LLSI4_OUT       PA3      MFP Mask */
#define LLSI4_OUT_PB5_Msk       SYS_GPB_MFPL_PB5MFP_Msk        /*!< LLSI4_OUT       PB5      MFP Mask */
#define LLSI5_OUT_PA2_Msk       SYS_GPA_MFPL_PA2MFP_Msk        /*!< LLSI5_OUT       PA2      MFP Mask */
#define LLSI5_OUT_PB4_Msk       SYS_GPB_MFPL_PB4MFP_Msk        /*!< LLSI5_OUT       PB4      MFP Mask */
#define SPDH_HSA_PB1_Msk        SYS_GPB_MFPL_PB1MFP_Msk        /*!< SPDH_HSA        PB1      MFP Mask */
#define SPI0_CLK_PD2_Msk        SYS_GPD_MFPL_PD2MFP_Msk        /*!< SPI0_CLK        PD2      MFP Mask */
#define SPI0_CLK_PB14_Msk       SYS_GPB_MFPH_PB14MFP_Msk       /*!< SPI0_CLK        PB14     MFP Mask */
#define SPI0_CLK_PF5_Msk        SYS_GPF_MFPL_PF5MFP_Msk        /*!< SPI0_CLK        PF5      MFP Mask */
#define SPI0_CLK_PA2_Msk        SYS_GPA_MFPL_PA2MFP_Msk        /*!< SPI0_CLK        PA2      MFP Mask */
#define SPI0_I2SMCLK_PA4_Msk    SYS_GPA_MFPL_PA4MFP_Msk        /*!< SPI0_I2SMCLK    PA4      MFP Mask */
#define SPI0_I2SMCLK_PA5_Msk    SYS_GPA_MFPL_PA5MFP_Msk        /*!< SPI0_I2SMCLK    PA5      MFP Mask */
#define SPI0_I2SMCLK_PB0_Msk    SYS_GPB_MFPL_PB0MFP_Msk        /*!< SPI0_I2SMCLK    PB0      MFP Mask */
#define SPI0_I2SMCLK_PB11_Msk   SYS_GPB_MFPH_PB11MFP_Msk       /*!< SPI0_I2SMCLK    PB11     MFP Mask */
#define SPI0_I2SMCLK_PB2_Msk    SYS_GPB_MFPL_PB2MFP_Msk        /*!< SPI0_I2SMCLK    PB2      MFP Mask */
#define SPI0_I2SMCLK_PC14_Msk   SYS_GPC_MFPH_PC14MFP_Msk       /*!< SPI0_I2SMCLK    PC14     MFP Mask */
#define SPI0_MISO_PD1_Msk       SYS_GPD_MFPL_PD1MFP_Msk        /*!< SPI0_MISO       PD1      MFP Mask */
#define SPI0_MISO_PA1_Msk       SYS_GPA_MFPL_PA1MFP_Msk        /*!< SPI0_MISO       PA1      MFP Mask */
#define SPI0_MISO_PB13_Msk      SYS_GPB_MFPH_PB13MFP_Msk       /*!< SPI0_MISO       PB13     MFP Mask */
#define SPI0_MISO_PF4_Msk       SYS_GPF_MFPL_PF4MFP_Msk        /*!< SPI0_MISO       PF4      MFP Mask */
#define SPI0_MOSI_PB12_Msk      SYS_GPB_MFPH_PB12MFP_Msk       /*!< SPI0_MOSI       PB12     MFP Mask */
#define SPI0_MOSI_PD0_Msk       SYS_GPD_MFPL_PD0MFP_Msk        /*!< SPI0_MOSI       PD0      MFP Mask */
#define SPI0_MOSI_PA0_Msk       SYS_GPA_MFPL_PA0MFP_Msk        /*!< SPI0_MOSI       PA0      MFP Mask */
#define SPI0_MOSI_PF6_Msk       SYS_GPF_MFPL_PF6MFP_Msk        /*!< SPI0_MOSI       PF6      MFP Mask */
#define SPI0_SS_PD3_Msk         SYS_GPD_MFPL_PD3MFP_Msk        /*!< SPI0_SS         PD3      MFP Mask */
#define SPI0_SS_PB0_Msk         SYS_GPB_MFPL_PB0MFP_Msk        /*!< SPI0_SS         PB0      MFP Mask */
#define SPI0_SS_PB15_Msk        SYS_GPB_MFPH_PB15MFP_Msk       /*!< SPI0_SS         PB15     MFP Mask */
#define SPI0_SS_PA3_Msk         SYS_GPA_MFPL_PA3MFP_Msk        /*!< SPI0_SS         PA3      MFP Mask */
#define SPI1_CLK_PA9_Msk        SYS_GPA_MFPH_PA9MFP_Msk        /*!< SPI1_CLK        PA9      MFP Mask */
#define SPI1_CLK_PA7_Msk        SYS_GPA_MFPL_PA7MFP_Msk        /*!< SPI1_CLK        PA7      MFP Mask */
#define SPI1_CLK_PC1_Msk        SYS_GPC_MFPL_PC1MFP_Msk        /*!< SPI1_CLK        PC1      MFP Mask */
#define SPI1_CLK_PB3_Msk        SYS_GPB_MFPL_PB3MFP_Msk        /*!< SPI1_CLK        PB3      MFP Mask */
#define SPI1_I2SMCLK_PB1_Msk    SYS_GPB_MFPL_PB1MFP_Msk        /*!< SPI1_I2SMCLK    PB1      MFP Mask */
#define SPI1_I2SMCLK_PA5_Msk    SYS_GPA_MFPL_PA5MFP_Msk        /*!< SPI1_I2SMCLK    PA5      MFP Mask */
#define SPI1_I2SMCLK_PC4_Msk    SYS_GPC_MFPL_PC4MFP_Msk        /*!< SPI1_I2SMCLK    PC4      MFP Mask */
#define SPI1_MISO_PC7_Msk       SYS_GPC_MFPL_PC7MFP_Msk        /*!< SPI1_MISO       PC7      MFP Mask */
#define SPI1_MISO_PB5_Msk       SYS_GPB_MFPL_PB5MFP_Msk        /*!< SPI1_MISO       PB5      MFP Mask */
#define SPI1_MISO_PC3_Msk       SYS_GPC_MFPL_PC3MFP_Msk        /*!< SPI1_MISO       PC3      MFP Mask */
#define SPI1_MISO_PA10_Msk      SYS_GPA_MFPH_PA10MFP_Msk       /*!< SPI1_MISO       PA10     MFP Mask */
#define SPI1_MOSI_PB4_Msk       SYS_GPB_MFPL_PB4MFP_Msk        /*!< SPI1_MOSI       PB4      MFP Mask */
#define SPI1_MOSI_PC2_Msk       SYS_GPC_MFPL_PC2MFP_Msk        /*!< SPI1_MOSI       PC2      MFP Mask */
#define SPI1_MOSI_PC6_Msk       SYS_GPC_MFPL_PC6MFP_Msk        /*!< SPI1_MOSI       PC6      MFP Mask */
#define SPI1_MOSI_PA11_Msk      SYS_GPA_MFPH_PA11MFP_Msk       /*!< SPI1_MOSI       PA11     MFP Mask */
#define SPI1_MOSI_PC14_Msk      SYS_GPC_MFPH_PC14MFP_Msk       /*!< SPI1_MOSI       PC14     MFP Mask */
#define SPI1_SS_PA8_Msk         SYS_GPA_MFPH_PA8MFP_Msk        /*!< SPI1_SS         PA8      MFP Mask */
#define SPI1_SS_PA6_Msk         SYS_GPA_MFPL_PA6MFP_Msk        /*!< SPI1_SS         PA6      MFP Mask */
#define SPI1_SS_PC0_Msk         SYS_GPC_MFPL_PC0MFP_Msk        /*!< SPI1_SS         PC0      MFP Mask */
#define SPI1_SS_PB2_Msk         SYS_GPB_MFPL_PB2MFP_Msk        /*!< SPI1_SS         PB2      MFP Mask */
#define SPI1_SS_PF5_Msk         SYS_GPF_MFPL_PF5MFP_Msk        /*!< SPI1_SS         PF5      MFP Mask */
#define SPI2_CLK_PA2_Msk        SYS_GPA_MFPL_PA2MFP_Msk        /*!< SPI2_CLK        PA2      MFP Mask */
#define SPI2_CLK_PA10_Msk       SYS_GPA_MFPH_PA10MFP_Msk       /*!< SPI2_CLK        PA10     MFP Mask */
#define SPI2_I2SMCLK_PA5_Msk    SYS_GPA_MFPL_PA5MFP_Msk        /*!< SPI2_I2SMCLK    PA5      MFP Mask */
#define SPI2_I2SMCLK_PB0_Msk    SYS_GPB_MFPL_PB0MFP_Msk        /*!< SPI2_I2SMCLK    PB0      MFP Mask */
#define SPI2_MISO_PA1_Msk       SYS_GPA_MFPL_PA1MFP_Msk        /*!< SPI2_MISO       PA1      MFP Mask */
#define SPI2_MISO_PA9_Msk       SYS_GPA_MFPH_PA9MFP_Msk        /*!< SPI2_MISO       PA9      MFP Mask */
#define SPI2_MOSI_PA0_Msk       SYS_GPA_MFPL_PA0MFP_Msk        /*!< SPI2_MOSI       PA0      MFP Mask */
#define SPI2_MOSI_PA8_Msk       SYS_GPA_MFPH_PA8MFP_Msk        /*!< SPI2_MOSI       PA8      MFP Mask */
#define SPI2_SS_PA3_Msk         SYS_GPA_MFPL_PA3MFP_Msk        /*!< SPI2_SS         PA3      MFP Mask */
#define SPI2_SS_PA11_Msk        SYS_GPA_MFPH_PA11MFP_Msk       /*!< SPI2_SS         PA11     MFP Mask */
#define SPI_CLK_MUX_PA2_Msk     SYS_GPA_MFPL_PA2MFP_Msk        /*!< SPI_CLK_MUX     PA2      MFP Mask */
#define SPI_MISO_MUX_PA1_Msk    SYS_GPA_MFPL_PA1MFP_Msk        /*!< SPI_MISO_MUX    PA1      MFP Mask */
#define SPI_MOSI_MUX_PA0_Msk    SYS_GPA_MFPL_PA0MFP_Msk        /*!< SPI_MOSI_MUX    PA0      MFP Mask */
#define SPI_SS_MUX_PA3_Msk      SYS_GPA_MFPL_PA3MFP_Msk        /*!< SPI_SS_MUX      PA3      MFP Mask */
#define TM0_PB5_Msk             SYS_GPB_MFPL_PB5MFP_Msk        /*!< TM0             PB5      MFP Mask */
#define TM0_PC7_Msk             SYS_GPC_MFPL_PC7MFP_Msk        /*!< TM0             PC7      MFP Mask */
#define TM0_EXT_PA11_Msk        SYS_GPA_MFPH_PA11MFP_Msk       /*!< TM0_EXT         PA11     MFP Mask */
#define TM0_EXT_PB15_Msk        SYS_GPB_MFPH_PB15MFP_Msk       /*!< TM0_EXT         PB15     MFP Mask */
#define TM1_PB4_Msk             SYS_GPB_MFPL_PB4MFP_Msk        /*!< TM1             PB4      MFP Mask */
#define TM1_PC14_Msk            SYS_GPC_MFPH_PC14MFP_Msk       /*!< TM1             PC14     MFP Mask */
#define TM1_PC6_Msk             SYS_GPC_MFPL_PC6MFP_Msk        /*!< TM1             PC6      MFP Mask */
#define TM1_EXT_PA10_Msk        SYS_GPA_MFPH_PA10MFP_Msk       /*!< TM1_EXT         PA10     MFP Mask */
#define TM1_EXT_PB14_Msk        SYS_GPB_MFPH_PB14MFP_Msk       /*!< TM1_EXT         PB14     MFP Mask */
#define TM2_PD0_Msk             SYS_GPD_MFPL_PD0MFP_Msk        /*!< TM2             PD0      MFP Mask */
#define TM2_PB3_Msk             SYS_GPB_MFPL_PB3MFP_Msk        /*!< TM2             PB3      MFP Mask */
#define TM2_PA7_Msk             SYS_GPA_MFPL_PA7MFP_Msk        /*!< TM2             PA7      MFP Mask */
#define TM2_EXT_PB13_Msk        SYS_GPB_MFPH_PB13MFP_Msk       /*!< TM2_EXT         PB13     MFP Mask */
#define TM2_EXT_PA9_Msk         SYS_GPA_MFPH_PA9MFP_Msk        /*!< TM2_EXT         PA9      MFP Mask */
#define TM3_PD15_Msk            SYS_GPD_MFPH_PD15MFP_Msk       /*!< TM3             PD15     MFP Mask */
#define TM3_PA6_Msk             SYS_GPA_MFPL_PA6MFP_Msk        /*!< TM3             PA6      MFP Mask */
#define TM3_PF6_Msk             SYS_GPF_MFPL_PF6MFP_Msk        /*!< TM3             PF6      MFP Mask */
#define TM3_PB2_Msk             SYS_GPB_MFPL_PB2MFP_Msk        /*!< TM3             PB2      MFP Mask */
#define TM3_EXT_PA8_Msk         SYS_GPA_MFPH_PA8MFP_Msk        /*!< TM3_EXT         PA8      MFP Mask */
#define TM3_EXT_PB1_Msk         SYS_GPB_MFPL_PB1MFP_Msk        /*!< TM3_EXT         PB1      MFP Mask */
#define TM3_EXT_PB12_Msk        SYS_GPB_MFPH_PB12MFP_Msk       /*!< TM3_EXT         PB12     MFP Mask */
#define UART0_RXD_PF2_Msk       SYS_GPF_MFPL_PF2MFP_Msk        /*!< UART0_RXD       PF2      MFP Mask */
#define UART0_RXD_PA6_Msk       SYS_GPA_MFPL_PA6MFP_Msk        /*!< UART0_RXD       PA6      MFP Mask */
#define UART0_RXD_PF1_Msk       SYS_GPF_MFPL_PF1MFP_Msk        /*!< UART0_RXD       PF1      MFP Mask */
#define UART0_RXD_PD2_Msk       SYS_GPD_MFPL_PD2MFP_Msk        /*!< UART0_RXD       PD2      MFP Mask */
#define UART0_RXD_PB12_Msk      SYS_GPB_MFPH_PB12MFP_Msk       /*!< UART0_RXD       PB12     MFP Mask */
#define UART0_RXD_PB8_Msk       SYS_GPB_MFPH_PB8MFP_Msk        /*!< UART0_RXD       PB8      MFP Mask */
#define UART0_RXD_PA4_Msk       SYS_GPA_MFPL_PA4MFP_Msk        /*!< UART0_RXD       PA4      MFP Mask */
#define UART0_RXD_PA0_Msk       SYS_GPA_MFPL_PA0MFP_Msk        /*!< UART0_RXD       PA0      MFP Mask */
#define UART0_TXD_PF0_Msk       SYS_GPF_MFPL_PF0MFP_Msk        /*!< UART0_TXD       PF0      MFP Mask */
#define UART0_TXD_PA5_Msk       SYS_GPA_MFPL_PA5MFP_Msk        /*!< UART0_TXD       PA5      MFP Mask */
#define UART0_TXD_PD3_Msk       SYS_GPD_MFPL_PD3MFP_Msk        /*!< UART0_TXD       PD3      MFP Mask */
#define UART0_TXD_PB9_Msk       SYS_GPB_MFPH_PB9MFP_Msk        /*!< UART0_TXD       PB9      MFP Mask */
#define UART0_TXD_PF3_Msk       SYS_GPF_MFPL_PF3MFP_Msk        /*!< UART0_TXD       PF3      MFP Mask */
#define UART0_TXD_PA1_Msk       SYS_GPA_MFPL_PA1MFP_Msk        /*!< UART0_TXD       PA1      MFP Mask */
#define UART0_TXD_PB13_Msk      SYS_GPB_MFPH_PB13MFP_Msk       /*!< UART0_TXD       PB13     MFP Mask */
#define UART0_TXD_PA7_Msk       SYS_GPA_MFPL_PA7MFP_Msk        /*!< UART0_TXD       PA7      MFP Mask */
#define UART0_nCTS_PA5_Msk      SYS_GPA_MFPL_PA5MFP_Msk        /*!< UART0_nCTS      PA5      MFP Mask */
#define UART0_nCTS_PC7_Msk      SYS_GPC_MFPL_PC7MFP_Msk        /*!< UART0_nCTS      PC7      MFP Mask */
#define UART0_nCTS_PB11_Msk     SYS_GPB_MFPH_PB11MFP_Msk       /*!< UART0_nCTS      PB11     MFP Mask */
#define UART0_nCTS_PB15_Msk     SYS_GPB_MFPH_PB15MFP_Msk       /*!< UART0_nCTS      PB15     MFP Mask */
#define UART0_nRTS_PB10_Msk     SYS_GPB_MFPH_PB10MFP_Msk       /*!< UART0_nRTS      PB10     MFP Mask */
#define UART0_nRTS_PB14_Msk     SYS_GPB_MFPH_PB14MFP_Msk       /*!< UART0_nRTS      PB14     MFP Mask */
#define UART0_nRTS_PC6_Msk      SYS_GPC_MFPL_PC6MFP_Msk        /*!< UART0_nRTS      PC6      MFP Mask */
#define UART0_nRTS_PA4_Msk      SYS_GPA_MFPL_PA4MFP_Msk        /*!< UART0_nRTS      PA4      MFP Mask */
#define UART1_RXD_PA8_Msk       SYS_GPA_MFPH_PA8MFP_Msk        /*!< UART1_RXD       PA8      MFP Mask */
#define UART1_RXD_PF2_Msk       SYS_GPF_MFPL_PF2MFP_Msk        /*!< UART1_RXD       PF2      MFP Mask */
#define UART1_RXD_PF1_Msk       SYS_GPF_MFPL_PF1MFP_Msk        /*!< UART1_RXD       PF1      MFP Mask */
#define UART1_RXD_PB2_Msk       SYS_GPB_MFPL_PB2MFP_Msk        /*!< UART1_RXD       PB2      MFP Mask */
#define UART1_RXD_PA2_Msk       SYS_GPA_MFPL_PA2MFP_Msk        /*!< UART1_RXD       PA2      MFP Mask */
#define UART1_RXD_PB6_Msk       SYS_GPB_MFPL_PB6MFP_Msk        /*!< UART1_RXD       PB6      MFP Mask */
#define UART1_TXD_PB7_Msk       SYS_GPB_MFPL_PB7MFP_Msk        /*!< UART1_TXD       PB7      MFP Mask */
#define UART1_TXD_PF0_Msk       SYS_GPF_MFPL_PF0MFP_Msk        /*!< UART1_TXD       PF0      MFP Mask */
#define UART1_TXD_PA9_Msk       SYS_GPA_MFPH_PA9MFP_Msk        /*!< UART1_TXD       PA9      MFP Mask */
#define UART1_TXD_PB3_Msk       SYS_GPB_MFPL_PB3MFP_Msk        /*!< UART1_TXD       PB3      MFP Mask */
#define UART1_TXD_PA3_Msk       SYS_GPA_MFPL_PA3MFP_Msk        /*!< UART1_TXD       PA3      MFP Mask */
#define UART1_nCTS_PA1_Msk      SYS_GPA_MFPL_PA1MFP_Msk        /*!< UART1_nCTS      PA1      MFP Mask */
#define UART1_nCTS_PB9_Msk      SYS_GPB_MFPH_PB9MFP_Msk        /*!< UART1_nCTS      PB9      MFP Mask */
#define UART1_nRTS_PA0_Msk      SYS_GPA_MFPL_PA0MFP_Msk        /*!< UART1_nRTS      PA0      MFP Mask */
#define UART1_nRTS_PB8_Msk      SYS_GPB_MFPH_PB8MFP_Msk        /*!< UART1_nRTS      PB8      MFP Mask */
#define UART2_RXD_PF1_Msk       SYS_GPF_MFPL_PF1MFP_Msk        /*!< UART2_RXD       PF1      MFP Mask */
#define UART2_RXD_PC0_Msk       SYS_GPC_MFPL_PC0MFP_Msk        /*!< UART2_RXD       PC0      MFP Mask */
#define UART2_RXD_PF5_Msk       SYS_GPF_MFPL_PF5MFP_Msk        /*!< UART2_RXD       PF5      MFP Mask */
#define UART2_RXD_PC4_Msk       SYS_GPC_MFPL_PC4MFP_Msk        /*!< UART2_RXD       PC4      MFP Mask */
#define UART2_RXD_PF3_Msk       SYS_GPF_MFPL_PF3MFP_Msk        /*!< UART2_RXD       PF3      MFP Mask */
#define UART2_RXD_PB4_Msk       SYS_GPB_MFPL_PB4MFP_Msk        /*!< UART2_RXD       PB4      MFP Mask */
#define UART2_RXD_PB0_Msk       SYS_GPB_MFPL_PB0MFP_Msk        /*!< UART2_RXD       PB0      MFP Mask */
#define UART2_TXD_PB1_Msk       SYS_GPB_MFPL_PB1MFP_Msk        /*!< UART2_TXD       PB1      MFP Mask */
#define UART2_TXD_PF0_Msk       SYS_GPF_MFPL_PF0MFP_Msk        /*!< UART2_TXD       PF0      MFP Mask */
#define UART2_TXD_PF4_Msk       SYS_GPF_MFPL_PF4MFP_Msk        /*!< UART2_TXD       PF4      MFP Mask */
#define UART2_TXD_PC5_Msk       SYS_GPC_MFPL_PC5MFP_Msk        /*!< UART2_TXD       PC5      MFP Mask */
#define UART2_TXD_PB5_Msk       SYS_GPB_MFPL_PB5MFP_Msk        /*!< UART2_TXD       PB5      MFP Mask */
#define UART2_TXD_PC1_Msk       SYS_GPC_MFPL_PC1MFP_Msk        /*!< UART2_TXD       PC1      MFP Mask */
#define UART2_nCTS_PC2_Msk      SYS_GPC_MFPL_PC2MFP_Msk        /*!< UART2_nCTS      PC2      MFP Mask */
#define UART2_nCTS_PF5_Msk      SYS_GPF_MFPL_PF5MFP_Msk        /*!< UART2_nCTS      PF5      MFP Mask */
#define UART2_nRTS_PC3_Msk      SYS_GPC_MFPL_PC3MFP_Msk        /*!< UART2_nRTS      PC3      MFP Mask */
#define UART2_nRTS_PF4_Msk      SYS_GPF_MFPL_PF4MFP_Msk        /*!< UART2_nRTS      PF4      MFP Mask */
#define VDET_P0_PB0_Msk         SYS_GPB_MFPL_PB0MFP_Msk        /*!< VDET_P0         PB0      MFP Mask */
#define VDET_P1_PB1_Msk         SYS_GPB_MFPL_PB1MFP_Msk        /*!< VDET_P1         PB1      MFP Mask */
#define X32_IN_PF5_Msk          SYS_GPF_MFPL_PF5MFP_Msk        /*!< X32_IN          PF5      MFP Mask */
#define X32_OUT_PF4_Msk         SYS_GPF_MFPL_PF4MFP_Msk        /*!< X32_OUT         PF4      MFP Mask */
#define XT1_IN_PF3_Msk          SYS_GPF_MFPL_PF3MFP_Msk        /*!< XT1_IN          PF3      MFP Mask */
#define XT1_OUT_PF2_Msk         SYS_GPF_MFPL_PF2MFP_Msk        /*!< XT1_OUT         PF2      MFP Mask */


/*@}*/ /* end of group SYS_EXPORTED_CONSTANTS */

/** @addtogroup SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function macro definitions.                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

#define SET_ACMP0_N_PB4()        SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP0_N_PB4_Msk)) | ACMP0_N_PB4              /*!< Set PB4 function to ACMP0_N          */
#define SET_ACMP0_O_PB7()        SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP0_O_PB7_Msk)) | ACMP0_O_PB7              /*!< Set PB7 function to ACMP0_O          */
#define SET_ACMP0_O_PC1()        SYS->GPC_MFPL = (SYS->GPC_MFPL & (~ACMP0_O_PC1_Msk)) | ACMP0_O_PC1              /*!< Set PC1 function to ACMP0_O          */
#define SET_ACMP0_O_PF0()        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~ACMP0_O_PF0_Msk)) | ACMP0_O_PF0              /*!< Set PF0 function to ACMP0_O          */
#define SET_ACMP0_P0_PB2()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP0_P0_PB2_Msk)) | ACMP0_P0_PB2            /*!< Set PB2 function to ACMP0_P0         */
#define SET_ACMP0_P1_PB3()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP0_P1_PB3_Msk)) | ACMP0_P1_PB3            /*!< Set PB3 function to ACMP0_P1         */
#define SET_ACMP0_P2_PB12()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ACMP0_P2_PB12_Msk)) | ACMP0_P2_PB12          /*!< Set PB12 function to ACMP0_P2        */
#define SET_ACMP0_P3_PB13()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ACMP0_P3_PB13_Msk)) | ACMP0_P3_PB13          /*!< Set PB13 function to ACMP0_P3        */
#define SET_ACMP0_WLAT_PA7()     SYS->GPA_MFPL = (SYS->GPA_MFPL & (~ACMP0_WLAT_PA7_Msk)) | ACMP0_WLAT_PA7        /*!< Set PA7 function to ACMP0_WLAT       */
#define SET_ACMP1_N_PB5()        SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP1_N_PB5_Msk)) | ACMP1_N_PB5              /*!< Set PB5 function to ACMP1_N          */
#define SET_ACMP1_O_PB6()        SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP1_O_PB6_Msk)) | ACMP1_O_PB6              /*!< Set PB6 function to ACMP1_O          */
#define SET_ACMP1_O_PC0()        SYS->GPC_MFPL = (SYS->GPC_MFPL & (~ACMP1_O_PC0_Msk)) | ACMP1_O_PC0              /*!< Set PC0 function to ACMP1_O          */
#define SET_ACMP1_O_PF1()        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~ACMP1_O_PF1_Msk)) | ACMP1_O_PF1              /*!< Set PF1 function to ACMP1_O          */
#define SET_ACMP1_P0_PB3()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP1_P0_PB3_Msk)) | ACMP1_P0_PB3            /*!< Set PB3 function to ACMP1_P0         */
#define SET_ACMP1_P1_PB4()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP1_P1_PB4_Msk)) | ACMP1_P1_PB4            /*!< Set PB4 function to ACMP1_P1         */
#define SET_ACMP1_P2_PB12()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ACMP1_P2_PB12_Msk)) | ACMP1_P2_PB12          /*!< Set PB12 function to ACMP1_P2        */
#define SET_ACMP1_P3_PB13()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ACMP1_P3_PB13_Msk)) | ACMP1_P3_PB13          /*!< Set PB13 function to ACMP1_P3        */
#define SET_ACMP1_WLAT_PA6()     SYS->GPA_MFPL = (SYS->GPA_MFPL & (~ACMP1_WLAT_PA6_Msk)) | ACMP1_WLAT_PA6        /*!< Set PA6 function to ACMP1_WLAT       */
#define SET_ACMP2_N_PB6()        SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP2_N_PB6_Msk)) | ACMP2_N_PB6              /*!< Set PB6 function to ACMP2_N          */
#define SET_ACMP2_O_PA1()        SYS->GPA_MFPL = (SYS->GPA_MFPL & (~ACMP2_O_PA1_Msk)) | ACMP2_O_PA1              /*!< Set PA1 function to ACMP2_O          */
#define SET_ACMP2_O_PF3()        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~ACMP2_O_PF3_Msk)) | ACMP2_O_PF3              /*!< Set PF3 function to ACMP2_O          */
#define SET_ACMP2_P0_PB4()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP2_P0_PB4_Msk)) | ACMP2_P0_PB4            /*!< Set PB4 function to ACMP2_P0         */
#define SET_ACMP2_P1_PB5()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP2_P1_PB5_Msk)) | ACMP2_P1_PB5            /*!< Set PB5 function to ACMP2_P1         */
#define SET_ACMP2_P2_PB14()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ACMP2_P2_PB14_Msk)) | ACMP2_P2_PB14          /*!< Set PB14 function to ACMP2_P2        */
#define SET_ACMP2_P3_PB15()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ACMP2_P3_PB15_Msk)) | ACMP2_P3_PB15          /*!< Set PB15 function to ACMP2_P3        */
#define SET_ACMP2_WLAT_PA3()     SYS->GPA_MFPL = (SYS->GPA_MFPL & (~ACMP2_WLAT_PA3_Msk)) | ACMP2_WLAT_PA3        /*!< Set PA3 function to ACMP2_WLAT       */
#define SET_ACMP2_WLAT_PF5()     SYS->GPF_MFPL = (SYS->GPF_MFPL & (~ACMP2_WLAT_PF5_Msk)) | ACMP2_WLAT_PF5        /*!< Set PF5 function to ACMP2_WLAT       */
#define SET_ACMP3_N_PB7()        SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP3_N_PB7_Msk)) | ACMP3_N_PB7              /*!< Set PB7 function to ACMP3_N          */
#define SET_ACMP3_O_PA0()        SYS->GPA_MFPL = (SYS->GPA_MFPL & (~ACMP3_O_PA0_Msk)) | ACMP3_O_PA0              /*!< Set PA0 function to ACMP3_O          */
#define SET_ACMP3_O_PF2()        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~ACMP3_O_PF2_Msk)) | ACMP3_O_PF2              /*!< Set PF2 function to ACMP3_O          */
#define SET_ACMP3_P0_PB5()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP3_P0_PB5_Msk)) | ACMP3_P0_PB5            /*!< Set PB5 function to ACMP3_P0         */
#define SET_ACMP3_P1_PB6()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ACMP3_P1_PB6_Msk)) | ACMP3_P1_PB6            /*!< Set PB6 function to ACMP3_P1         */
#define SET_ACMP3_P2_PB14()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ACMP3_P2_PB14_Msk)) | ACMP3_P2_PB14          /*!< Set PB14 function to ACMP3_P2        */
#define SET_ACMP3_P3_PB15()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ACMP3_P3_PB15_Msk)) | ACMP3_P3_PB15          /*!< Set PB15 function to ACMP3_P3        */
#define SET_ACMP3_WLAT_PF4()     SYS->GPF_MFPL = (SYS->GPF_MFPL & (~ACMP3_WLAT_PF4_Msk)) | ACMP3_WLAT_PF4        /*!< Set PF4 function to ACMP3_WLAT       */
#define SET_ACMP3_WLAT_PA2()     SYS->GPA_MFPL = (SYS->GPA_MFPL & (~ACMP3_WLAT_PA2_Msk)) | ACMP3_WLAT_PA2        /*!< Set PA2 function to ACMP3_WLAT       */
#define SET_ADC0_CH0_PB0()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ADC0_CH0_PB0_Msk)) | ADC0_CH0_PB0            /*!< Set PB0 function to ADC0_CH0         */
#define SET_ADC0_CH1_PB1()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ADC0_CH1_PB1_Msk)) | ADC0_CH1_PB1            /*!< Set PB1 function to ADC0_CH1         */
#define SET_ADC0_CH10_PB10()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ADC0_CH10_PB10_Msk)) | ADC0_CH10_PB10        /*!< Set PB10 function to ADC0_CH10       */
#define SET_ADC0_CH11_PB11()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ADC0_CH11_PB11_Msk)) | ADC0_CH11_PB11        /*!< Set PB11 function to ADC0_CH11       */
#define SET_ADC0_CH12_PB12()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ADC0_CH12_PB12_Msk)) | ADC0_CH12_PB12        /*!< Set PB12 function to ADC0_CH12       */
#define SET_ADC0_CH13_PB13()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ADC0_CH13_PB13_Msk)) | ADC0_CH13_PB13        /*!< Set PB13 function to ADC0_CH13       */
#define SET_ADC0_CH14_PB14()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ADC0_CH14_PB14_Msk)) | ADC0_CH14_PB14        /*!< Set PB14 function to ADC0_CH14       */
#define SET_ADC0_CH15_PB15()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ADC0_CH15_PB15_Msk)) | ADC0_CH15_PB15        /*!< Set PB15 function to ADC0_CH15       */
#define SET_ADC0_CH2_PB2()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ADC0_CH2_PB2_Msk)) | ADC0_CH2_PB2            /*!< Set PB2 function to ADC0_CH2         */
#define SET_ADC0_CH3_PB3()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ADC0_CH3_PB3_Msk)) | ADC0_CH3_PB3            /*!< Set PB3 function to ADC0_CH3         */
#define SET_ADC0_CH4_PB4()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ADC0_CH4_PB4_Msk)) | ADC0_CH4_PB4            /*!< Set PB4 function to ADC0_CH4         */
#define SET_ADC0_CH5_PB5()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ADC0_CH5_PB5_Msk)) | ADC0_CH5_PB5            /*!< Set PB5 function to ADC0_CH5         */
#define SET_ADC0_CH6_PB6()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ADC0_CH6_PB6_Msk)) | ADC0_CH6_PB6            /*!< Set PB6 function to ADC0_CH6         */
#define SET_ADC0_CH7_PB7()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~ADC0_CH7_PB7_Msk)) | ADC0_CH7_PB7            /*!< Set PB7 function to ADC0_CH7         */
#define SET_ADC0_CH8_PB8()       SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ADC0_CH8_PB8_Msk)) | ADC0_CH8_PB8            /*!< Set PB8 function to ADC0_CH8         */
#define SET_ADC0_CH9_PB9()       SYS->GPB_MFPH = (SYS->GPB_MFPH & (~ADC0_CH9_PB9_Msk)) | ADC0_CH9_PB9            /*!< Set PB9 function to ADC0_CH9         */
#define SET_ADC0_ST_PF5()        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~ADC0_ST_PF5_Msk)) | ADC0_ST_PF5              /*!< Set PF5 function to ADC0_ST          */
#define SET_ADC0_ST_PC1()        SYS->GPC_MFPL = (SYS->GPC_MFPL & (~ADC0_ST_PC1_Msk)) | ADC0_ST_PC1              /*!< Set PC1 function to ADC0_ST          */
#define SET_BPWM0_CH0_PA0()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM0_CH0_PA0_Msk)) | BPWM0_CH0_PA0          /*!< Set PA0 function to BPWM0_CH0        */
#define SET_BPWM0_CH0_PA11()     SYS->GPA_MFPH = (SYS->GPA_MFPH & (~BPWM0_CH0_PA11_Msk)) | BPWM0_CH0_PA11        /*!< Set PA11 function to BPWM0_CH0       */
#define SET_BPWM0_CH1_PA10()     SYS->GPA_MFPH = (SYS->GPA_MFPH & (~BPWM0_CH1_PA10_Msk)) | BPWM0_CH1_PA10        /*!< Set PA10 function to BPWM0_CH1       */
#define SET_BPWM0_CH1_PA1()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM0_CH1_PA1_Msk)) | BPWM0_CH1_PA1          /*!< Set PA1 function to BPWM0_CH1        */
#define SET_BPWM0_CH2_PA2()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM0_CH2_PA2_Msk)) | BPWM0_CH2_PA2          /*!< Set PA2 function to BPWM0_CH2        */
#define SET_BPWM0_CH2_PA9()      SYS->GPA_MFPH = (SYS->GPA_MFPH & (~BPWM0_CH2_PA9_Msk)) | BPWM0_CH2_PA9          /*!< Set PA9 function to BPWM0_CH2        */
#define SET_BPWM0_CH3_PA3()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM0_CH3_PA3_Msk)) | BPWM0_CH3_PA3          /*!< Set PA3 function to BPWM0_CH3        */
#define SET_BPWM0_CH3_PF3()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM0_CH3_PF3_Msk)) | BPWM0_CH3_PF3          /*!< Set PF3 function to BPWM0_CH3        */
#define SET_BPWM0_CH3_PA8()      SYS->GPA_MFPH = (SYS->GPA_MFPH & (~BPWM0_CH3_PA8_Msk)) | BPWM0_CH3_PA8          /*!< Set PA8 function to BPWM0_CH3        */
#define SET_BPWM0_CH4_PA4()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM0_CH4_PA4_Msk)) | BPWM0_CH4_PA4          /*!< Set PA4 function to BPWM0_CH4        */
#define SET_BPWM0_CH4_PF5()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM0_CH4_PF5_Msk)) | BPWM0_CH4_PF5          /*!< Set PF5 function to BPWM0_CH4        */
#define SET_BPWM0_CH5_PA5()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM0_CH5_PA5_Msk)) | BPWM0_CH5_PA5          /*!< Set PA5 function to BPWM0_CH5        */
#define SET_BPWM0_CH5_PF4()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM0_CH5_PF4_Msk)) | BPWM0_CH5_PF4          /*!< Set PF4 function to BPWM0_CH5        */
#define SET_BPWM1_CH0_PB11()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~BPWM1_CH0_PB11_Msk)) | BPWM1_CH0_PB11        /*!< Set PB11 function to BPWM1_CH0       */
#define SET_BPWM1_CH0_PF3()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM1_CH0_PF3_Msk)) | BPWM1_CH0_PF3          /*!< Set PF3 function to BPWM1_CH0        */
#define SET_BPWM1_CH0_PC7()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM1_CH0_PC7_Msk)) | BPWM1_CH0_PC7          /*!< Set PC7 function to BPWM1_CH0        */
#define SET_BPWM1_CH0_PF0()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM1_CH0_PF0_Msk)) | BPWM1_CH0_PF0          /*!< Set PF0 function to BPWM1_CH0        */
#define SET_BPWM1_CH1_PC6()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM1_CH1_PC6_Msk)) | BPWM1_CH1_PC6          /*!< Set PC6 function to BPWM1_CH1        */
#define SET_BPWM1_CH1_PF2()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM1_CH1_PF2_Msk)) | BPWM1_CH1_PF2          /*!< Set PF2 function to BPWM1_CH1        */
#define SET_BPWM1_CH1_PB10()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~BPWM1_CH1_PB10_Msk)) | BPWM1_CH1_PB10        /*!< Set PB10 function to BPWM1_CH1       */
#define SET_BPWM1_CH1_PF1()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM1_CH1_PF1_Msk)) | BPWM1_CH1_PF1          /*!< Set PF1 function to BPWM1_CH1        */
#define SET_BPWM1_CH2_PA7()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM1_CH2_PA7_Msk)) | BPWM1_CH2_PA7          /*!< Set PA7 function to BPWM1_CH2        */
#define SET_BPWM1_CH2_PB9()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~BPWM1_CH2_PB9_Msk)) | BPWM1_CH2_PB9          /*!< Set PB9 function to BPWM1_CH2        */
#define SET_BPWM1_CH3_PA6()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM1_CH3_PA6_Msk)) | BPWM1_CH3_PA6          /*!< Set PA6 function to BPWM1_CH3        */
#define SET_BPWM1_CH3_PB8()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~BPWM1_CH3_PB8_Msk)) | BPWM1_CH3_PB8          /*!< Set PB8 function to BPWM1_CH3        */
#define SET_BPWM1_CH4_PB7()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM1_CH4_PB7_Msk)) | BPWM1_CH4_PB7          /*!< Set PB7 function to BPWM1_CH4        */
#define SET_BPWM1_CH5_PB6()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM1_CH5_PB6_Msk)) | BPWM1_CH5_PB6          /*!< Set PB6 function to BPWM1_CH5        */
#define SET_BPWM2_CH0_PA5()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM2_CH0_PA5_Msk)) | BPWM2_CH0_PA5          /*!< Set PA5 function to BPWM2_CH0        */
#define SET_BPWM2_CH0_PF5()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM2_CH0_PF5_Msk)) | BPWM2_CH0_PF5          /*!< Set PF5 function to BPWM2_CH0        */
#define SET_BPWM2_CH0_PB5()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM2_CH0_PB5_Msk)) | BPWM2_CH0_PB5          /*!< Set PB5 function to BPWM2_CH0        */
#define SET_BPWM2_CH1_PF4()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM2_CH1_PF4_Msk)) | BPWM2_CH1_PF4          /*!< Set PF4 function to BPWM2_CH1        */
#define SET_BPWM2_CH1_PA4()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM2_CH1_PA4_Msk)) | BPWM2_CH1_PA4          /*!< Set PA4 function to BPWM2_CH1        */
#define SET_BPWM2_CH1_PB4()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM2_CH1_PB4_Msk)) | BPWM2_CH1_PB4          /*!< Set PB4 function to BPWM2_CH1        */
#define SET_BPWM2_CH1_PC1()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM2_CH1_PC1_Msk)) | BPWM2_CH1_PC1          /*!< Set PC1 function to BPWM2_CH1        */
#define SET_BPWM2_CH2_PA3()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM2_CH2_PA3_Msk)) | BPWM2_CH2_PA3          /*!< Set PA3 function to BPWM2_CH2        */
#define SET_BPWM2_CH2_PB3()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM2_CH2_PB3_Msk)) | BPWM2_CH2_PB3          /*!< Set PB3 function to BPWM2_CH2        */
#define SET_BPWM2_CH3_PA2()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM2_CH3_PA2_Msk)) | BPWM2_CH3_PA2          /*!< Set PA2 function to BPWM2_CH3        */
#define SET_BPWM2_CH3_PB2()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM2_CH3_PB2_Msk)) | BPWM2_CH3_PB2          /*!< Set PB2 function to BPWM2_CH3        */
#define SET_BPWM2_CH4_PB1()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM2_CH4_PB1_Msk)) | BPWM2_CH4_PB1          /*!< Set PB1 function to BPWM2_CH4        */
#define SET_BPWM2_CH4_PF6()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM2_CH4_PF6_Msk)) | BPWM2_CH4_PF6          /*!< Set PF6 function to BPWM2_CH4        */
#define SET_BPWM2_CH4_PA1()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM2_CH4_PA1_Msk)) | BPWM2_CH4_PA1          /*!< Set PA1 function to BPWM2_CH4        */
#define SET_BPWM2_CH5_PD15()     SYS->GPD_MFPH = (SYS->GPD_MFPH & (~BPWM2_CH5_PD15_Msk)) | BPWM2_CH5_PD15        /*!< Set PD15 function to BPWM2_CH5       */
#define SET_BPWM2_CH5_PA0()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM2_CH5_PA0_Msk)) | BPWM2_CH5_PA0          /*!< Set PA0 function to BPWM2_CH5        */
#define SET_BPWM2_CH5_PB0()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM2_CH5_PB0_Msk)) | BPWM2_CH5_PB0          /*!< Set PB0 function to BPWM2_CH5        */
#define SET_BPWM2_CH5_PF4()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM2_CH5_PF4_Msk)) | BPWM2_CH5_PF4          /*!< Set PF4 function to BPWM2_CH5        */
#define SET_BPWM3_CH0_PC5()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM3_CH0_PC5_Msk)) | BPWM3_CH0_PC5          /*!< Set PC5 function to BPWM3_CH0        */
#define SET_BPWM3_CH0_PB15()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~BPWM3_CH0_PB15_Msk)) | BPWM3_CH0_PB15        /*!< Set PB15 function to BPWM3_CH0       */
#define SET_BPWM3_CH1_PB14()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~BPWM3_CH1_PB14_Msk)) | BPWM3_CH1_PB14        /*!< Set PB14 function to BPWM3_CH1       */
#define SET_BPWM3_CH1_PC4()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM3_CH1_PC4_Msk)) | BPWM3_CH1_PC4          /*!< Set PC4 function to BPWM3_CH1        */
#define SET_BPWM3_CH2_PB13()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~BPWM3_CH2_PB13_Msk)) | BPWM3_CH2_PB13        /*!< Set PB13 function to BPWM3_CH2       */
#define SET_BPWM3_CH2_PC7()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM3_CH2_PC7_Msk)) | BPWM3_CH2_PC7          /*!< Set PC7 function to BPWM3_CH2        */
#define SET_BPWM3_CH2_PC3()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM3_CH2_PC3_Msk)) | BPWM3_CH2_PC3          /*!< Set PC3 function to BPWM3_CH2        */
#define SET_BPWM3_CH3_PB12()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~BPWM3_CH3_PB12_Msk)) | BPWM3_CH3_PB12        /*!< Set PB12 function to BPWM3_CH3       */
#define SET_BPWM3_CH3_PC0()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM3_CH3_PC0_Msk)) | BPWM3_CH3_PC0          /*!< Set PC0 function to BPWM3_CH3        */
#define SET_BPWM3_CH3_PC2()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM3_CH3_PC2_Msk)) | BPWM3_CH3_PC2          /*!< Set PC2 function to BPWM3_CH3        */
#define SET_BPWM3_CH3_PC6()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM3_CH3_PC6_Msk)) | BPWM3_CH3_PC6          /*!< Set PC6 function to BPWM3_CH3        */
#define SET_BPWM3_CH4_PC1()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM3_CH4_PC1_Msk)) | BPWM3_CH4_PC1          /*!< Set PC1 function to BPWM3_CH4        */
#define SET_BPWM3_CH4_PA7()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM3_CH4_PA7_Msk)) | BPWM3_CH4_PA7          /*!< Set PA7 function to BPWM3_CH4        */
#define SET_BPWM3_CH4_PB7()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM3_CH4_PB7_Msk)) | BPWM3_CH4_PB7          /*!< Set PB7 function to BPWM3_CH4        */
#define SET_BPWM3_CH4_PB1()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM3_CH4_PB1_Msk)) | BPWM3_CH4_PB1          /*!< Set PB1 function to BPWM3_CH4        */
#define SET_BPWM3_CH5_PB6()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM3_CH5_PB6_Msk)) | BPWM3_CH5_PB6          /*!< Set PB6 function to BPWM3_CH5        */
#define SET_BPWM3_CH5_PF4()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~BPWM3_CH5_PF4_Msk)) | BPWM3_CH5_PF4          /*!< Set PF4 function to BPWM3_CH5        */
#define SET_BPWM3_CH5_PC0()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~BPWM3_CH5_PC0_Msk)) | BPWM3_CH5_PC0          /*!< Set PC0 function to BPWM3_CH5        */
#define SET_BPWM3_CH5_PA6()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~BPWM3_CH5_PA6_Msk)) | BPWM3_CH5_PA6          /*!< Set PA6 function to BPWM3_CH5        */
#define SET_BPWM3_CH5_PB0()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~BPWM3_CH5_PB0_Msk)) | BPWM3_CH5_PB0          /*!< Set PB0 function to BPWM3_CH5        */
#define SET_CLKO_PF6()           SYS->GPF_MFPL = (SYS->GPF_MFPL & (~CLKO_PF6_Msk)) | CLKO_PF6                    /*!< Set PF6 function to CLKO             */
#define SET_CLKO_PB14()          SYS->GPB_MFPH = (SYS->GPB_MFPH & (~CLKO_PB14_Msk)) | CLKO_PB14                  /*!< Set PB14 function to CLKO            */
#define SET_CLKO_PA3()           SYS->GPA_MFPL = (SYS->GPA_MFPL & (~CLKO_PA3_Msk)) | CLKO_PA3                    /*!< Set PA3 function to CLKO             */
#define SET_DAC0_OUT_PB12()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~DAC0_OUT_PB12_Msk)) | DAC0_OUT_PB12          /*!< Set PB12 function to DAC0_OUT        */
#define SET_DAC0_ST_PA10()       SYS->GPA_MFPH = (SYS->GPA_MFPH & (~DAC0_ST_PA10_Msk)) | DAC0_ST_PA10            /*!< Set PA10 function to DAC0_ST         */
#define SET_DAC0_ST_PA0()        SYS->GPA_MFPL = (SYS->GPA_MFPL & (~DAC0_ST_PA0_Msk)) | DAC0_ST_PA0              /*!< Set PA0 function to DAC0_ST          */
#define SET_DAC1_OUT_PB13()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~DAC1_OUT_PB13_Msk)) | DAC1_OUT_PB13          /*!< Set PB13 function to DAC1_OUT        */
#define SET_DAC1_ST_PC14()       SYS->GPC_MFPH = (SYS->GPC_MFPH & (~DAC1_ST_PC14_Msk)) | DAC1_ST_PC14            /*!< Set PC14 function to DAC1_ST         */
#define SET_DAC1_ST_PA11()       SYS->GPA_MFPH = (SYS->GPA_MFPH & (~DAC1_ST_PA11_Msk)) | DAC1_ST_PA11            /*!< Set PA11 function to DAC1_ST         */
#define SET_DAC2_OUT_PB14()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~DAC2_OUT_PB14_Msk)) | DAC2_OUT_PB14          /*!< Set PB14 function to DAC2_OUT        */
#define SET_DAC2_ST_PA9()        SYS->GPA_MFPH = (SYS->GPA_MFPH & (~DAC2_ST_PA9_Msk)) | DAC2_ST_PA9              /*!< Set PA9 function to DAC2_ST          */
#define SET_DAC2_ST_PA5()        SYS->GPA_MFPL = (SYS->GPA_MFPL & (~DAC2_ST_PA5_Msk)) | DAC2_ST_PA5              /*!< Set PA5 function to DAC2_ST          */
#define SET_DAC3_OUT_PB15()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~DAC3_OUT_PB15_Msk)) | DAC3_OUT_PB15          /*!< Set PB15 function to DAC3_OUT        */
#define SET_DAC3_ST_PA4()        SYS->GPA_MFPL = (SYS->GPA_MFPL & (~DAC3_ST_PA4_Msk)) | DAC3_ST_PA4              /*!< Set PA4 function to DAC3_ST          */
#define SET_DAC3_ST_PA8()        SYS->GPA_MFPH = (SYS->GPA_MFPH & (~DAC3_ST_PA8_Msk)) | DAC3_ST_PA8              /*!< Set PA8 function to DAC3_ST          */
#define SET_I2C0_SCL_PA8()       SYS->GPA_MFPH = (SYS->GPA_MFPH & (~I2C0_SCL_PA8_Msk)) | I2C0_SCL_PA8            /*!< Set PA8 function to I2C0_SCL         */
#define SET_I2C0_SCL_PB5()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~I2C0_SCL_PB5_Msk)) | I2C0_SCL_PB5            /*!< Set PB5 function to I2C0_SCL         */
#define SET_I2C0_SCL_PB9()       SYS->GPB_MFPH = (SYS->GPB_MFPH & (~I2C0_SCL_PB9_Msk)) | I2C0_SCL_PB9            /*!< Set PB9 function to I2C0_SCL         */
#define SET_I2C0_SCL_PF3()       SYS->GPF_MFPL = (SYS->GPF_MFPL & (~I2C0_SCL_PF3_Msk)) | I2C0_SCL_PF3            /*!< Set PF3 function to I2C0_SCL         */
#define SET_I2C0_SCL_PA5()       SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I2C0_SCL_PA5_Msk)) | I2C0_SCL_PA5            /*!< Set PA5 function to I2C0_SCL         */
#define SET_I2C0_SCL_PC1()       SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C0_SCL_PC1_Msk)) | I2C0_SCL_PC1            /*!< Set PC1 function to I2C0_SCL         */
#define SET_I2C0_SCL_PC3()       SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C0_SCL_PC3_Msk)) | I2C0_SCL_PC3            /*!< Set PC3 function to I2C0_SCL         */
#define SET_I2C0_SCL_PF5()       SYS->GPF_MFPL = (SYS->GPF_MFPL & (~I2C0_SCL_PF5_Msk)) | I2C0_SCL_PF5            /*!< Set PF5 function to I2C0_SCL         */
#define SET_I2C0_SDA_PA4()       SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I2C0_SDA_PA4_Msk)) | I2C0_SDA_PA4            /*!< Set PA4 function to I2C0_SDA         */
#define SET_I2C0_SDA_PF6()       SYS->GPF_MFPL = (SYS->GPF_MFPL & (~I2C0_SDA_PF6_Msk)) | I2C0_SDA_PF6            /*!< Set PF6 function to I2C0_SDA         */
#define SET_I2C0_SDA_PC0()       SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C0_SDA_PC0_Msk)) | I2C0_SDA_PC0            /*!< Set PC0 function to I2C0_SDA         */
#define SET_I2C0_SDA_PC2()       SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C0_SDA_PC2_Msk)) | I2C0_SDA_PC2            /*!< Set PC2 function to I2C0_SDA         */
#define SET_I2C0_SDA_PF2()       SYS->GPF_MFPL = (SYS->GPF_MFPL & (~I2C0_SDA_PF2_Msk)) | I2C0_SDA_PF2            /*!< Set PF2 function to I2C0_SDA         */
#define SET_I2C0_SDA_PB8()       SYS->GPB_MFPH = (SYS->GPB_MFPH & (~I2C0_SDA_PB8_Msk)) | I2C0_SDA_PB8            /*!< Set PB8 function to I2C0_SDA         */
#define SET_I2C0_SDA_PB4()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~I2C0_SDA_PB4_Msk)) | I2C0_SDA_PB4            /*!< Set PB4 function to I2C0_SDA         */
#define SET_I2C0_SDA_PF4()       SYS->GPF_MFPL = (SYS->GPF_MFPL & (~I2C0_SDA_PF4_Msk)) | I2C0_SDA_PF4            /*!< Set PF4 function to I2C0_SDA         */
#define SET_I2C0_SMBAL_PC3()     SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C0_SMBAL_PC3_Msk)) | I2C0_SMBAL_PC3        /*!< Set PC3 function to I2C0_SMBAL       */
#define SET_I2C0_SMBAL_PA3()     SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I2C0_SMBAL_PA3_Msk)) | I2C0_SMBAL_PA3        /*!< Set PA3 function to I2C0_SMBAL       */
#define SET_I2C0_SMBSUS_PA2()    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I2C0_SMBSUS_PA2_Msk)) | I2C0_SMBSUS_PA2      /*!< Set PA2 function to I2C0_SMBSUS      */
#define SET_I2C0_SMBSUS_PC2()    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C0_SMBSUS_PC2_Msk)) | I2C0_SMBSUS_PC2      /*!< Set PC2 function to I2C0_SMBSUS      */
#define SET_I2C1_SCL_PC1()       SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C1_SCL_PC1_Msk)) | I2C1_SCL_PC1            /*!< Set PC1 function to I2C1_SCL         */
#define SET_I2C1_SCL_PA10()      SYS->GPA_MFPH = (SYS->GPA_MFPH & (~I2C1_SCL_PA10_Msk)) | I2C1_SCL_PA10          /*!< Set PA10 function to I2C1_SCL        */
#define SET_I2C1_SCL_PA7()       SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I2C1_SCL_PA7_Msk)) | I2C1_SCL_PA7            /*!< Set PA7 function to I2C1_SCL         */
#define SET_I2C1_SCL_PA3()       SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I2C1_SCL_PA3_Msk)) | I2C1_SCL_PA3            /*!< Set PA3 function to I2C1_SCL         */
#define SET_I2C1_SCL_PB11()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~I2C1_SCL_PB11_Msk)) | I2C1_SCL_PB11          /*!< Set PB11 function to I2C1_SCL        */
#define SET_I2C1_SCL_PB1()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~I2C1_SCL_PB1_Msk)) | I2C1_SCL_PB1            /*!< Set PB1 function to I2C1_SCL         */
#define SET_I2C1_SCL_PF0()       SYS->GPF_MFPL = (SYS->GPF_MFPL & (~I2C1_SCL_PF0_Msk)) | I2C1_SCL_PF0            /*!< Set PF0 function to I2C1_SCL         */
#define SET_I2C1_SCL_PC5()       SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C1_SCL_PC5_Msk)) | I2C1_SCL_PC5            /*!< Set PC5 function to I2C1_SCL         */
#define SET_I2C1_SCL_PB3()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~I2C1_SCL_PB3_Msk)) | I2C1_SCL_PB3            /*!< Set PB3 function to I2C1_SCL         */
#define SET_I2C1_SDA_PC0()       SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C1_SDA_PC0_Msk)) | I2C1_SDA_PC0            /*!< Set PC0 function to I2C1_SDA         */
#define SET_I2C1_SDA_PB0()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~I2C1_SDA_PB0_Msk)) | I2C1_SDA_PB0            /*!< Set PB0 function to I2C1_SDA         */
#define SET_I2C1_SDA_PA2()       SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I2C1_SDA_PA2_Msk)) | I2C1_SDA_PA2            /*!< Set PA2 function to I2C1_SDA         */
#define SET_I2C1_SDA_PC4()       SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C1_SDA_PC4_Msk)) | I2C1_SDA_PC4            /*!< Set PC4 function to I2C1_SDA         */
#define SET_I2C1_SDA_PB2()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~I2C1_SDA_PB2_Msk)) | I2C1_SDA_PB2            /*!< Set PB2 function to I2C1_SDA         */
#define SET_I2C1_SDA_PA9()       SYS->GPA_MFPH = (SYS->GPA_MFPH & (~I2C1_SDA_PA9_Msk)) | I2C1_SDA_PA9            /*!< Set PA9 function to I2C1_SDA         */
#define SET_I2C1_SDA_PB10()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~I2C1_SDA_PB10_Msk)) | I2C1_SDA_PB10          /*!< Set PB10 function to I2C1_SDA        */
#define SET_I2C1_SDA_PA6()       SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I2C1_SDA_PA6_Msk)) | I2C1_SDA_PA6            /*!< Set PA6 function to I2C1_SDA         */
#define SET_I2C1_SDA_PF1()       SYS->GPF_MFPL = (SYS->GPF_MFPL & (~I2C1_SDA_PF1_Msk)) | I2C1_SDA_PF1            /*!< Set PF1 function to I2C1_SDA         */
#define SET_I2C1_SMBAL_PC7()     SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C1_SMBAL_PC7_Msk)) | I2C1_SMBAL_PC7        /*!< Set PC7 function to I2C1_SMBAL       */
#define SET_I2C1_SMBAL_PB9()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~I2C1_SMBAL_PB9_Msk)) | I2C1_SMBAL_PB9        /*!< Set PB9 function to I2C1_SMBAL       */
#define SET_I2C1_SMBAL_PF0()     SYS->GPF_MFPL = (SYS->GPF_MFPL & (~I2C1_SMBAL_PF0_Msk)) | I2C1_SMBAL_PF0        /*!< Set PF0 function to I2C1_SMBAL       */
#define SET_I2C1_SMBSUS_PB8()    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~I2C1_SMBSUS_PB8_Msk)) | I2C1_SMBSUS_PB8      /*!< Set PB8 function to I2C1_SMBSUS      */
#define SET_I2C1_SMBSUS_PC6()    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C1_SMBSUS_PC6_Msk)) | I2C1_SMBSUS_PC6      /*!< Set PC6 function to I2C1_SMBSUS      */
#define SET_I2C1_SMBSUS_PF1()    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~I2C1_SMBSUS_PF1_Msk)) | I2C1_SMBSUS_PF1      /*!< Set PF1 function to I2C1_SMBSUS      */
#define SET_I2C2_SCL_PB3()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~I2C2_SCL_PB3_Msk)) | I2C2_SCL_PB3            /*!< Set PB3 function to I2C2_SCL         */
#define SET_I2C2_SCL_PA11()      SYS->GPA_MFPH = (SYS->GPA_MFPH & (~I2C2_SCL_PA11_Msk)) | I2C2_SCL_PA11          /*!< Set PA11 function to I2C2_SCL        */
#define SET_I2C2_SCL_PF0()       SYS->GPF_MFPL = (SYS->GPF_MFPL & (~I2C2_SCL_PF0_Msk)) | I2C2_SCL_PF0            /*!< Set PF0 function to I2C2_SCL         */
#define SET_I2C2_SCL_PA1()       SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I2C2_SCL_PA1_Msk)) | I2C2_SCL_PA1            /*!< Set PA1 function to I2C2_SCL         */
#define SET_I2C2_SCL_PB13()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~I2C2_SCL_PB13_Msk)) | I2C2_SCL_PB13          /*!< Set PB13 function to I2C2_SCL        */
#define SET_I2C2_SDA_PA0()       SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I2C2_SDA_PA0_Msk)) | I2C2_SDA_PA0            /*!< Set PA0 function to I2C2_SDA         */
#define SET_I2C2_SDA_PF1()       SYS->GPF_MFPL = (SYS->GPF_MFPL & (~I2C2_SDA_PF1_Msk)) | I2C2_SDA_PF1            /*!< Set PF1 function to I2C2_SDA         */
#define SET_I2C2_SDA_PA10()      SYS->GPA_MFPH = (SYS->GPA_MFPH & (~I2C2_SDA_PA10_Msk)) | I2C2_SDA_PA10          /*!< Set PA10 function to I2C2_SDA        */
#define SET_I2C2_SDA_PB2()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~I2C2_SDA_PB2_Msk)) | I2C2_SDA_PB2            /*!< Set PB2 function to I2C2_SDA         */
#define SET_I2C2_SDA_PB12()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~I2C2_SDA_PB12_Msk)) | I2C2_SDA_PB12          /*!< Set PB12 function to I2C2_SDA        */
#define SET_I2C2_SMBAL_PB15()    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~I2C2_SMBAL_PB15_Msk)) | I2C2_SMBAL_PB15      /*!< Set PB15 function to I2C2_SMBAL      */
#define SET_I2C2_SMBAL_PC3()     SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C2_SMBAL_PC3_Msk)) | I2C2_SMBAL_PC3        /*!< Set PC3 function to I2C2_SMBAL       */
#define SET_I2C2_SMBSUS_PB14()   SYS->GPB_MFPH = (SYS->GPB_MFPH & (~I2C2_SMBSUS_PB14_Msk)) | I2C2_SMBSUS_PB14    /*!< Set PB14 function to I2C2_SMBSUS     */
#define SET_I2C2_SMBSUS_PC2()    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~I2C2_SMBSUS_PC2_Msk)) | I2C2_SMBSUS_PC2      /*!< Set PC2 function to I2C2_SMBSUS      */
#define SET_I3CS0_SCL_PA5()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I3CS0_SCL_PA5_Msk)) | I3CS0_SCL_PA5          /*!< Set PA5 function to I3CS0_SCL        */
#define SET_I3CS0_SCL_PA1()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I3CS0_SCL_PA1_Msk)) | I3CS0_SCL_PA1          /*!< Set PA1 function to I3CS0_SCL        */
#define SET_I3CS0_SDA_PA0()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I3CS0_SDA_PA0_Msk)) | I3CS0_SDA_PA0          /*!< Set PA0 function to I3CS0_SDA        */
#define SET_I3CS0_SDA_PA4()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I3CS0_SDA_PA4_Msk)) | I3CS0_SDA_PA4          /*!< Set PA4 function to I3CS0_SDA        */
#define SET_I3CS1_SCL_PA3()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I3CS1_SCL_PA3_Msk)) | I3CS1_SCL_PA3          /*!< Set PA3 function to I3CS1_SCL        */
#define SET_I3CS1_SDA_PA2()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~I3CS1_SDA_PA2_Msk)) | I3CS1_SDA_PA2          /*!< Set PA2 function to I3CS1_SDA        */
#define SET_ICE_CLK_PF1()        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~ICE_CLK_PF1_Msk)) | ICE_CLK_PF1              /*!< Set PF1 function to ICE_CLK          */
#define SET_ICE_DAT_PF0()        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~ICE_DAT_PF0_Msk)) | ICE_DAT_PF0              /*!< Set PF0 function to ICE_DAT          */
#define SET_INT0_PA6()           SYS->GPA_MFPL = (SYS->GPA_MFPL & (~INT0_PA6_Msk)) | INT0_PA6                    /*!< Set PA6 function to INT0             */
#define SET_INT0_PB5()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~INT0_PB5_Msk)) | INT0_PB5                    /*!< Set PB5 function to INT0             */
#define SET_INT1_PB4()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~INT1_PB4_Msk)) | INT1_PB4                    /*!< Set PB4 function to INT1             */
#define SET_INT1_PD15()          SYS->GPD_MFPH = (SYS->GPD_MFPH & (~INT1_PD15_Msk)) | INT1_PD15                  /*!< Set PD15 function to INT1            */
#define SET_INT1_PA7()           SYS->GPA_MFPL = (SYS->GPA_MFPL & (~INT1_PA7_Msk)) | INT1_PA7                    /*!< Set PA7 function to INT1             */
#define SET_INT2_PC6()           SYS->GPC_MFPL = (SYS->GPC_MFPL & (~INT2_PC6_Msk)) | INT2_PC6                    /*!< Set PC6 function to INT2             */
#define SET_INT2_PB3()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~INT2_PB3_Msk)) | INT2_PB3                    /*!< Set PB3 function to INT2             */
#define SET_INT3_PB2()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~INT3_PB2_Msk)) | INT3_PB2                    /*!< Set PB2 function to INT3             */
#define SET_INT3_PC7()           SYS->GPC_MFPL = (SYS->GPC_MFPL & (~INT3_PC7_Msk)) | INT3_PC7                    /*!< Set PC7 function to INT3             */
#define SET_INT4_PB6()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~INT4_PB6_Msk)) | INT4_PB6                    /*!< Set PB6 function to INT4             */
#define SET_INT4_PF2()           SYS->GPF_MFPL = (SYS->GPF_MFPL & (~INT4_PF2_Msk)) | INT4_PF2                    /*!< Set PF2 function to INT4             */
#define SET_INT4_PA8()           SYS->GPA_MFPH = (SYS->GPA_MFPH & (~INT4_PA8_Msk)) | INT4_PA8                    /*!< Set PA8 function to INT4             */
#define SET_INT5_PB7()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~INT5_PB7_Msk)) | INT5_PB7                    /*!< Set PB7 function to INT5             */
#define SET_INT5_PF6()           SYS->GPF_MFPL = (SYS->GPF_MFPL & (~INT5_PF6_Msk)) | INT5_PF6                    /*!< Set PF6 function to INT5             */
#define SET_LLSI0_OUT_PB15()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~LLSI0_OUT_PB15_Msk)) | LLSI0_OUT_PB15        /*!< Set PB15 function to LLSI0_OUT       */
#define SET_LLSI0_OUT_PC5()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~LLSI0_OUT_PC5_Msk)) | LLSI0_OUT_PC5          /*!< Set PC5 function to LLSI0_OUT        */
#define SET_LLSI1_OUT_PC4()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~LLSI1_OUT_PC4_Msk)) | LLSI1_OUT_PC4          /*!< Set PC4 function to LLSI1_OUT        */
#define SET_LLSI1_OUT_PB14()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~LLSI1_OUT_PB14_Msk)) | LLSI1_OUT_PB14        /*!< Set PB14 function to LLSI1_OUT       */
#define SET_LLSI2_OUT_PB13()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~LLSI2_OUT_PB13_Msk)) | LLSI2_OUT_PB13        /*!< Set PB13 function to LLSI2_OUT       */
#define SET_LLSI2_OUT_PC3()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~LLSI2_OUT_PC3_Msk)) | LLSI2_OUT_PC3          /*!< Set PC3 function to LLSI2_OUT        */
#define SET_LLSI3_OUT_PF6()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~LLSI3_OUT_PF6_Msk)) | LLSI3_OUT_PF6          /*!< Set PF6 function to LLSI3_OUT        */
#define SET_LLSI3_OUT_PB12()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~LLSI3_OUT_PB12_Msk)) | LLSI3_OUT_PB12        /*!< Set PB12 function to LLSI3_OUT       */
#define SET_LLSI3_OUT_PC2()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~LLSI3_OUT_PC2_Msk)) | LLSI3_OUT_PC2          /*!< Set PC2 function to LLSI3_OUT        */
#define SET_LLSI4_OUT_PA3()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~LLSI4_OUT_PA3_Msk)) | LLSI4_OUT_PA3          /*!< Set PA3 function to LLSI4_OUT        */
#define SET_LLSI4_OUT_PB5()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~LLSI4_OUT_PB5_Msk)) | LLSI4_OUT_PB5          /*!< Set PB5 function to LLSI4_OUT        */
#define SET_LLSI5_OUT_PA2()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~LLSI5_OUT_PA2_Msk)) | LLSI5_OUT_PA2          /*!< Set PA2 function to LLSI5_OUT        */
#define SET_LLSI5_OUT_PB4()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~LLSI5_OUT_PB4_Msk)) | LLSI5_OUT_PB4          /*!< Set PB4 function to LLSI5_OUT        */
#define SET_SPDH_HSA_PB1()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SPDH_HSA_PB1_Msk)) | SPDH_HSA_PB1            /*!< Set PB1 function to SPDH_HSA         */
#define SET_SPI0_CLK_PD2()       SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SPI0_CLK_PD2_Msk)) | SPI0_CLK_PD2            /*!< Set PD2 function to SPI0_CLK         */
#define SET_SPI0_CLK_PB14()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SPI0_CLK_PB14_Msk)) | SPI0_CLK_PB14          /*!< Set PB14 function to SPI0_CLK        */
#define SET_SPI0_CLK_PF5()       SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SPI0_CLK_PF5_Msk)) | SPI0_CLK_PF5            /*!< Set PF5 function to SPI0_CLK         */
#define SET_SPI0_CLK_PA2()       SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI0_CLK_PA2_Msk)) | SPI0_CLK_PA2            /*!< Set PA2 function to SPI0_CLK         */
#define SET_SPI0_I2SMCLK_PA4()   SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI0_I2SMCLK_PA4_Msk)) | SPI0_I2SMCLK_PA4    /*!< Set PA4 function to SPI0_I2SMCLK     */
#define SET_SPI0_I2SMCLK_PA5()   SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI0_I2SMCLK_PA5_Msk)) | SPI0_I2SMCLK_PA5    /*!< Set PA5 function to SPI0_I2SMCLK     */
#define SET_SPI0_I2SMCLK_PB0()   SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SPI0_I2SMCLK_PB0_Msk)) | SPI0_I2SMCLK_PB0    /*!< Set PB0 function to SPI0_I2SMCLK     */
#define SET_SPI0_I2SMCLK_PB11()  SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SPI0_I2SMCLK_PB11_Msk)) | SPI0_I2SMCLK_PB11  /*!< Set PB11 function to SPI0_I2SMCLK    */
#define SET_SPI0_I2SMCLK_PB2()   SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SPI0_I2SMCLK_PB2_Msk)) | SPI0_I2SMCLK_PB2    /*!< Set PB2 function to SPI0_I2SMCLK     */
#define SET_SPI0_I2SMCLK_PC14()  SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SPI0_I2SMCLK_PC14_Msk)) | SPI0_I2SMCLK_PC14  /*!< Set PC14 function to SPI0_I2SMCLK    */
#define SET_SPI0_MISO_PD1()      SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SPI0_MISO_PD1_Msk)) | SPI0_MISO_PD1          /*!< Set PD1 function to SPI0_MISO        */
#define SET_SPI0_MISO_PA1()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI0_MISO_PA1_Msk)) | SPI0_MISO_PA1          /*!< Set PA1 function to SPI0_MISO        */
#define SET_SPI0_MISO_PB13()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SPI0_MISO_PB13_Msk)) | SPI0_MISO_PB13        /*!< Set PB13 function to SPI0_MISO       */
#define SET_SPI0_MISO_PF4()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SPI0_MISO_PF4_Msk)) | SPI0_MISO_PF4          /*!< Set PF4 function to SPI0_MISO        */
#define SET_SPI0_MOSI_PB12()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SPI0_MOSI_PB12_Msk)) | SPI0_MOSI_PB12        /*!< Set PB12 function to SPI0_MOSI       */
#define SET_SPI0_MOSI_PD0()      SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SPI0_MOSI_PD0_Msk)) | SPI0_MOSI_PD0          /*!< Set PD0 function to SPI0_MOSI        */
#define SET_SPI0_MOSI_PA0()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI0_MOSI_PA0_Msk)) | SPI0_MOSI_PA0          /*!< Set PA0 function to SPI0_MOSI        */
#define SET_SPI0_MOSI_PF6()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SPI0_MOSI_PF6_Msk)) | SPI0_MOSI_PF6          /*!< Set PF6 function to SPI0_MOSI        */
#define SET_SPI0_SS_PD3()        SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SPI0_SS_PD3_Msk)) | SPI0_SS_PD3              /*!< Set PD3 function to SPI0_SS          */
#define SET_SPI0_SS_PB0()        SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SPI0_SS_PB0_Msk)) | SPI0_SS_PB0              /*!< Set PB0 function to SPI0_SS          */
#define SET_SPI0_SS_PB15()       SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SPI0_SS_PB15_Msk)) | SPI0_SS_PB15            /*!< Set PB15 function to SPI0_SS         */
#define SET_SPI0_SS_PA3()        SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI0_SS_PA3_Msk)) | SPI0_SS_PA3              /*!< Set PA3 function to SPI0_SS          */
#define SET_SPI1_CLK_PA9()       SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SPI1_CLK_PA9_Msk)) | SPI1_CLK_PA9            /*!< Set PA9 function to SPI1_CLK         */
#define SET_SPI1_CLK_PA7()       SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI1_CLK_PA7_Msk)) | SPI1_CLK_PA7            /*!< Set PA7 function to SPI1_CLK         */
#define SET_SPI1_CLK_PC1()       SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SPI1_CLK_PC1_Msk)) | SPI1_CLK_PC1            /*!< Set PC1 function to SPI1_CLK         */
#define SET_SPI1_CLK_PB3()       SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SPI1_CLK_PB3_Msk)) | SPI1_CLK_PB3            /*!< Set PB3 function to SPI1_CLK         */
#define SET_SPI1_I2SMCLK_PB1()   SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SPI1_I2SMCLK_PB1_Msk)) | SPI1_I2SMCLK_PB1    /*!< Set PB1 function to SPI1_I2SMCLK     */
#define SET_SPI1_I2SMCLK_PA5()   SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI1_I2SMCLK_PA5_Msk)) | SPI1_I2SMCLK_PA5    /*!< Set PA5 function to SPI1_I2SMCLK     */
#define SET_SPI1_I2SMCLK_PC4()   SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SPI1_I2SMCLK_PC4_Msk)) | SPI1_I2SMCLK_PC4    /*!< Set PC4 function to SPI1_I2SMCLK     */
#define SET_SPI1_MISO_PC7()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SPI1_MISO_PC7_Msk)) | SPI1_MISO_PC7          /*!< Set PC7 function to SPI1_MISO        */
#define SET_SPI1_MISO_PB5()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SPI1_MISO_PB5_Msk)) | SPI1_MISO_PB5          /*!< Set PB5 function to SPI1_MISO        */
#define SET_SPI1_MISO_PC3()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SPI1_MISO_PC3_Msk)) | SPI1_MISO_PC3          /*!< Set PC3 function to SPI1_MISO        */
#define SET_SPI1_MISO_PA10()     SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SPI1_MISO_PA10_Msk)) | SPI1_MISO_PA10        /*!< Set PA10 function to SPI1_MISO       */
#define SET_SPI1_MOSI_PB4()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SPI1_MOSI_PB4_Msk)) | SPI1_MOSI_PB4          /*!< Set PB4 function to SPI1_MOSI        */
#define SET_SPI1_MOSI_PC2()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SPI1_MOSI_PC2_Msk)) | SPI1_MOSI_PC2          /*!< Set PC2 function to SPI1_MOSI        */
#define SET_SPI1_MOSI_PC6()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SPI1_MOSI_PC6_Msk)) | SPI1_MOSI_PC6          /*!< Set PC6 function to SPI1_MOSI        */
#define SET_SPI1_MOSI_PA11()     SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SPI1_MOSI_PA11_Msk)) | SPI1_MOSI_PA11        /*!< Set PA11 function to SPI1_MOSI       */
#define SET_SPI1_MOSI_PC14()     SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SPI1_MOSI_PC14_Msk)) | SPI1_MOSI_PC14        /*!< Set PC14 function to SPI1_MOSI       */
#define SET_SPI1_SS_PA8()        SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SPI1_SS_PA8_Msk)) | SPI1_SS_PA8              /*!< Set PA8 function to SPI1_SS          */
#define SET_SPI1_SS_PA6()        SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI1_SS_PA6_Msk)) | SPI1_SS_PA6              /*!< Set PA6 function to SPI1_SS          */
#define SET_SPI1_SS_PC0()        SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SPI1_SS_PC0_Msk)) | SPI1_SS_PC0              /*!< Set PC0 function to SPI1_SS          */
#define SET_SPI1_SS_PB2()        SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SPI1_SS_PB2_Msk)) | SPI1_SS_PB2              /*!< Set PB2 function to SPI1_SS          */
#define SET_SPI1_SS_PF5()        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SPI1_SS_PF5_Msk)) | SPI1_SS_PF5              /*!< Set PF5 function to SPI1_SS          */
#define SET_SPI2_CLK_PA2()       SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI2_CLK_PA2_Msk)) | SPI2_CLK_PA2            /*!< Set PA2 function to SPI2_CLK         */
#define SET_SPI2_CLK_PA10()      SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SPI2_CLK_PA10_Msk)) | SPI2_CLK_PA10          /*!< Set PA10 function to SPI2_CLK        */
#define SET_SPI2_I2SMCLK_PA5()   SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI2_I2SMCLK_PA5_Msk)) | SPI2_I2SMCLK_PA5    /*!< Set PA5 function to SPI2_I2SMCLK     */
#define SET_SPI2_I2SMCLK_PB0()   SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SPI2_I2SMCLK_PB0_Msk)) | SPI2_I2SMCLK_PB0    /*!< Set PB0 function to SPI2_I2SMCLK     */
#define SET_SPI2_MISO_PA1()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI2_MISO_PA1_Msk)) | SPI2_MISO_PA1          /*!< Set PA1 function to SPI2_MISO        */
#define SET_SPI2_MISO_PA9()      SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SPI2_MISO_PA9_Msk)) | SPI2_MISO_PA9          /*!< Set PA9 function to SPI2_MISO        */
#define SET_SPI2_MOSI_PA0()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI2_MOSI_PA0_Msk)) | SPI2_MOSI_PA0          /*!< Set PA0 function to SPI2_MOSI        */
#define SET_SPI2_MOSI_PA8()      SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SPI2_MOSI_PA8_Msk)) | SPI2_MOSI_PA8          /*!< Set PA8 function to SPI2_MOSI        */
#define SET_SPI2_SS_PA3()        SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI2_SS_PA3_Msk)) | SPI2_SS_PA3              /*!< Set PA3 function to SPI2_SS          */
#define SET_SPI2_SS_PA11()       SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SPI2_SS_PA11_Msk)) | SPI2_SS_PA11            /*!< Set PA11 function to SPI2_SS         */
#define SET_SPI_CLK_MUX_PA2()    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI_CLK_MUX_PA2_Msk)) | SPI_CLK_MUX_PA2      /*!< Set PA2 function to SPI_CLK_MUX      */
#define SET_SPI_MISO_MUX_PA1()   SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI_MISO_MUX_PA1_Msk)) | SPI_MISO_MUX_PA1    /*!< Set PA1 function to SPI_MISO_MUX     */
#define SET_SPI_MOSI_MUX_PA0()   SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI_MOSI_MUX_PA0_Msk)) | SPI_MOSI_MUX_PA0    /*!< Set PA0 function to SPI_MOSI_MUX     */
#define SET_SPI_SS_MUX_PA3()     SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SPI_SS_MUX_PA3_Msk)) | SPI_SS_MUX_PA3        /*!< Set PA3 function to SPI_SS_MUX       */
#define SET_TM0_PB5()            SYS->GPB_MFPL = (SYS->GPB_MFPL & (~TM0_PB5_Msk)) | TM0_PB5                      /*!< Set PB5 function to TM0              */
#define SET_TM0_PC7()            SYS->GPC_MFPL = (SYS->GPC_MFPL & (~TM0_PC7_Msk)) | TM0_PC7                      /*!< Set PC7 function to TM0              */
#define SET_TM0_EXT_PA11()       SYS->GPA_MFPH = (SYS->GPA_MFPH & (~TM0_EXT_PA11_Msk)) | TM0_EXT_PA11            /*!< Set PA11 function to TM0_EXT         */
#define SET_TM0_EXT_PB15()       SYS->GPB_MFPH = (SYS->GPB_MFPH & (~TM0_EXT_PB15_Msk)) | TM0_EXT_PB15            /*!< Set PB15 function to TM0_EXT         */
#define SET_TM1_PB4()            SYS->GPB_MFPL = (SYS->GPB_MFPL & (~TM1_PB4_Msk)) | TM1_PB4                      /*!< Set PB4 function to TM1              */
#define SET_TM1_PC14()           SYS->GPC_MFPH = (SYS->GPC_MFPH & (~TM1_PC14_Msk)) | TM1_PC14                    /*!< Set PC14 function to TM1             */
#define SET_TM1_PC6()            SYS->GPC_MFPL = (SYS->GPC_MFPL & (~TM1_PC6_Msk)) | TM1_PC6                      /*!< Set PC6 function to TM1              */
#define SET_TM1_EXT_PA10()       SYS->GPA_MFPH = (SYS->GPA_MFPH & (~TM1_EXT_PA10_Msk)) | TM1_EXT_PA10            /*!< Set PA10 function to TM1_EXT         */
#define SET_TM1_EXT_PB14()       SYS->GPB_MFPH = (SYS->GPB_MFPH & (~TM1_EXT_PB14_Msk)) | TM1_EXT_PB14            /*!< Set PB14 function to TM1_EXT         */
#define SET_TM2_PD0()            SYS->GPD_MFPL = (SYS->GPD_MFPL & (~TM2_PD0_Msk)) | TM2_PD0                      /*!< Set PD0 function to TM2              */
#define SET_TM2_PB3()            SYS->GPB_MFPL = (SYS->GPB_MFPL & (~TM2_PB3_Msk)) | TM2_PB3                      /*!< Set PB3 function to TM2              */
#define SET_TM2_PA7()            SYS->GPA_MFPL = (SYS->GPA_MFPL & (~TM2_PA7_Msk)) | TM2_PA7                      /*!< Set PA7 function to TM2              */
#define SET_TM2_EXT_PB13()       SYS->GPB_MFPH = (SYS->GPB_MFPH & (~TM2_EXT_PB13_Msk)) | TM2_EXT_PB13            /*!< Set PB13 function to TM2_EXT         */
#define SET_TM2_EXT_PA9()        SYS->GPA_MFPH = (SYS->GPA_MFPH & (~TM2_EXT_PA9_Msk)) | TM2_EXT_PA9              /*!< Set PA9 function to TM2_EXT          */
#define SET_TM3_PD15()           SYS->GPD_MFPH = (SYS->GPD_MFPH & (~TM3_PD15_Msk)) | TM3_PD15                    /*!< Set PD15 function to TM3             */
#define SET_TM3_PA6()            SYS->GPA_MFPL = (SYS->GPA_MFPL & (~TM3_PA6_Msk)) | TM3_PA6                      /*!< Set PA6 function to TM3              */
#define SET_TM3_PF6()            SYS->GPF_MFPL = (SYS->GPF_MFPL & (~TM3_PF6_Msk)) | TM3_PF6                      /*!< Set PF6 function to TM3              */
#define SET_TM3_PB2()            SYS->GPB_MFPL = (SYS->GPB_MFPL & (~TM3_PB2_Msk)) | TM3_PB2                      /*!< Set PB2 function to TM3              */
#define SET_TM3_EXT_PA8()        SYS->GPA_MFPH = (SYS->GPA_MFPH & (~TM3_EXT_PA8_Msk)) | TM3_EXT_PA8              /*!< Set PA8 function to TM3_EXT          */
#define SET_TM3_EXT_PB1()        SYS->GPB_MFPL = (SYS->GPB_MFPL & (~TM3_EXT_PB1_Msk)) | TM3_EXT_PB1              /*!< Set PB1 function to TM3_EXT          */
#define SET_TM3_EXT_PB12()       SYS->GPB_MFPH = (SYS->GPB_MFPH & (~TM3_EXT_PB12_Msk)) | TM3_EXT_PB12            /*!< Set PB12 function to TM3_EXT         */
#define SET_UART0_RXD_PF2()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART0_RXD_PF2_Msk)) | UART0_RXD_PF2          /*!< Set PF2 function to UART0_RXD        */
#define SET_UART0_RXD_PA6()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART0_RXD_PA6_Msk)) | UART0_RXD_PA6          /*!< Set PA6 function to UART0_RXD        */
#define SET_UART0_RXD_PF1()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART0_RXD_PF1_Msk)) | UART0_RXD_PF1          /*!< Set PF1 function to UART0_RXD        */
#define SET_UART0_RXD_PD2()      SYS->GPD_MFPL = (SYS->GPD_MFPL & (~UART0_RXD_PD2_Msk)) | UART0_RXD_PD2          /*!< Set PD2 function to UART0_RXD        */
#define SET_UART0_RXD_PB12()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~UART0_RXD_PB12_Msk)) | UART0_RXD_PB12        /*!< Set PB12 function to UART0_RXD       */
#define SET_UART0_RXD_PB8()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~UART0_RXD_PB8_Msk)) | UART0_RXD_PB8          /*!< Set PB8 function to UART0_RXD        */
#define SET_UART0_RXD_PA4()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART0_RXD_PA4_Msk)) | UART0_RXD_PA4          /*!< Set PA4 function to UART0_RXD        */
#define SET_UART0_RXD_PA0()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART0_RXD_PA0_Msk)) | UART0_RXD_PA0          /*!< Set PA0 function to UART0_RXD        */
#define SET_UART0_TXD_PF0()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART0_TXD_PF0_Msk)) | UART0_TXD_PF0          /*!< Set PF0 function to UART0_TXD        */
#define SET_UART0_TXD_PA5()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART0_TXD_PA5_Msk)) | UART0_TXD_PA5          /*!< Set PA5 function to UART0_TXD        */
#define SET_UART0_TXD_PD3()      SYS->GPD_MFPL = (SYS->GPD_MFPL & (~UART0_TXD_PD3_Msk)) | UART0_TXD_PD3          /*!< Set PD3 function to UART0_TXD        */
#define SET_UART0_TXD_PB9()      SYS->GPB_MFPH = (SYS->GPB_MFPH & (~UART0_TXD_PB9_Msk)) | UART0_TXD_PB9          /*!< Set PB9 function to UART0_TXD        */
#define SET_UART0_TXD_PF3()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART0_TXD_PF3_Msk)) | UART0_TXD_PF3          /*!< Set PF3 function to UART0_TXD        */
#define SET_UART0_TXD_PA1()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART0_TXD_PA1_Msk)) | UART0_TXD_PA1          /*!< Set PA1 function to UART0_TXD        */
#define SET_UART0_TXD_PB13()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~UART0_TXD_PB13_Msk)) | UART0_TXD_PB13        /*!< Set PB13 function to UART0_TXD       */
#define SET_UART0_TXD_PA7()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART0_TXD_PA7_Msk)) | UART0_TXD_PA7          /*!< Set PA7 function to UART0_TXD        */
#define SET_UART0_nCTS_PA5()     SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART0_nCTS_PA5_Msk)) | UART0_nCTS_PA5        /*!< Set PA5 function to UART0_nCTS       */
#define SET_UART0_nCTS_PC7()     SYS->GPC_MFPL = (SYS->GPC_MFPL & (~UART0_nCTS_PC7_Msk)) | UART0_nCTS_PC7        /*!< Set PC7 function to UART0_nCTS       */
#define SET_UART0_nCTS_PB11()    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~UART0_nCTS_PB11_Msk)) | UART0_nCTS_PB11      /*!< Set PB11 function to UART0_nCTS      */
#define SET_UART0_nCTS_PB15()    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~UART0_nCTS_PB15_Msk)) | UART0_nCTS_PB15      /*!< Set PB15 function to UART0_nCTS      */
#define SET_UART0_nRTS_PB10()    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~UART0_nRTS_PB10_Msk)) | UART0_nRTS_PB10      /*!< Set PB10 function to UART0_nRTS      */
#define SET_UART0_nRTS_PB14()    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~UART0_nRTS_PB14_Msk)) | UART0_nRTS_PB14      /*!< Set PB14 function to UART0_nRTS      */
#define SET_UART0_nRTS_PC6()     SYS->GPC_MFPL = (SYS->GPC_MFPL & (~UART0_nRTS_PC6_Msk)) | UART0_nRTS_PC6        /*!< Set PC6 function to UART0_nRTS       */
#define SET_UART0_nRTS_PA4()     SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART0_nRTS_PA4_Msk)) | UART0_nRTS_PA4        /*!< Set PA4 function to UART0_nRTS       */
#define SET_UART1_RXD_PA8()      SYS->GPA_MFPH = (SYS->GPA_MFPH & (~UART1_RXD_PA8_Msk)) | UART1_RXD_PA8          /*!< Set PA8 function to UART1_RXD        */
#define SET_UART1_RXD_PF2()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART1_RXD_PF2_Msk)) | UART1_RXD_PF2          /*!< Set PF2 function to UART1_RXD        */
#define SET_UART1_RXD_PF1()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART1_RXD_PF1_Msk)) | UART1_RXD_PF1          /*!< Set PF1 function to UART1_RXD        */
#define SET_UART1_RXD_PB2()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~UART1_RXD_PB2_Msk)) | UART1_RXD_PB2          /*!< Set PB2 function to UART1_RXD        */
#define SET_UART1_RXD_PA2()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART1_RXD_PA2_Msk)) | UART1_RXD_PA2          /*!< Set PA2 function to UART1_RXD        */
#define SET_UART1_RXD_PB6()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~UART1_RXD_PB6_Msk)) | UART1_RXD_PB6          /*!< Set PB6 function to UART1_RXD        */
#define SET_UART1_TXD_PB7()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~UART1_TXD_PB7_Msk)) | UART1_TXD_PB7          /*!< Set PB7 function to UART1_TXD        */
#define SET_UART1_TXD_PF0()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART1_TXD_PF0_Msk)) | UART1_TXD_PF0          /*!< Set PF0 function to UART1_TXD        */
#define SET_UART1_TXD_PA9()      SYS->GPA_MFPH = (SYS->GPA_MFPH & (~UART1_TXD_PA9_Msk)) | UART1_TXD_PA9          /*!< Set PA9 function to UART1_TXD        */
#define SET_UART1_TXD_PB3()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~UART1_TXD_PB3_Msk)) | UART1_TXD_PB3          /*!< Set PB3 function to UART1_TXD        */
#define SET_UART1_TXD_PA3()      SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART1_TXD_PA3_Msk)) | UART1_TXD_PA3          /*!< Set PA3 function to UART1_TXD        */
#define SET_UART1_nCTS_PA1()     SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART1_nCTS_PA1_Msk)) | UART1_nCTS_PA1        /*!< Set PA1 function to UART1_nCTS       */
#define SET_UART1_nCTS_PB9()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~UART1_nCTS_PB9_Msk)) | UART1_nCTS_PB9        /*!< Set PB9 function to UART1_nCTS       */
#define SET_UART1_nRTS_PA0()     SYS->GPA_MFPL = (SYS->GPA_MFPL & (~UART1_nRTS_PA0_Msk)) | UART1_nRTS_PA0        /*!< Set PA0 function to UART1_nRTS       */
#define SET_UART1_nRTS_PB8()     SYS->GPB_MFPH = (SYS->GPB_MFPH & (~UART1_nRTS_PB8_Msk)) | UART1_nRTS_PB8        /*!< Set PB8 function to UART1_nRTS       */
#define SET_UART2_RXD_PF1()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART2_RXD_PF1_Msk)) | UART2_RXD_PF1          /*!< Set PF1 function to UART2_RXD        */
#define SET_UART2_RXD_PC0()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~UART2_RXD_PC0_Msk)) | UART2_RXD_PC0          /*!< Set PC0 function to UART2_RXD        */
#define SET_UART2_RXD_PF5()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART2_RXD_PF5_Msk)) | UART2_RXD_PF5          /*!< Set PF5 function to UART2_RXD        */
#define SET_UART2_RXD_PC4()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~UART2_RXD_PC4_Msk)) | UART2_RXD_PC4          /*!< Set PC4 function to UART2_RXD        */
#define SET_UART2_RXD_PF3()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART2_RXD_PF3_Msk)) | UART2_RXD_PF3          /*!< Set PF3 function to UART2_RXD        */
#define SET_UART2_RXD_PB4()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~UART2_RXD_PB4_Msk)) | UART2_RXD_PB4          /*!< Set PB4 function to UART2_RXD        */
#define SET_UART2_RXD_PB0()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~UART2_RXD_PB0_Msk)) | UART2_RXD_PB0          /*!< Set PB0 function to UART2_RXD        */
#define SET_UART2_TXD_PB1()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~UART2_TXD_PB1_Msk)) | UART2_TXD_PB1          /*!< Set PB1 function to UART2_TXD        */
#define SET_UART2_TXD_PF0()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART2_TXD_PF0_Msk)) | UART2_TXD_PF0          /*!< Set PF0 function to UART2_TXD        */
#define SET_UART2_TXD_PF4()      SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART2_TXD_PF4_Msk)) | UART2_TXD_PF4          /*!< Set PF4 function to UART2_TXD        */
#define SET_UART2_TXD_PC5()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~UART2_TXD_PC5_Msk)) | UART2_TXD_PC5          /*!< Set PC5 function to UART2_TXD        */
#define SET_UART2_TXD_PB5()      SYS->GPB_MFPL = (SYS->GPB_MFPL & (~UART2_TXD_PB5_Msk)) | UART2_TXD_PB5          /*!< Set PB5 function to UART2_TXD        */
#define SET_UART2_TXD_PC1()      SYS->GPC_MFPL = (SYS->GPC_MFPL & (~UART2_TXD_PC1_Msk)) | UART2_TXD_PC1          /*!< Set PC1 function to UART2_TXD        */
#define SET_UART2_nCTS_PC2()     SYS->GPC_MFPL = (SYS->GPC_MFPL & (~UART2_nCTS_PC2_Msk)) | UART2_nCTS_PC2        /*!< Set PC2 function to UART2_nCTS       */
#define SET_UART2_nCTS_PF5()     SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART2_nCTS_PF5_Msk)) | UART2_nCTS_PF5        /*!< Set PF5 function to UART2_nCTS       */
#define SET_UART2_nRTS_PC3()     SYS->GPC_MFPL = (SYS->GPC_MFPL & (~UART2_nRTS_PC3_Msk)) | UART2_nRTS_PC3        /*!< Set PC3 function to UART2_nRTS       */
#define SET_UART2_nRTS_PF4()     SYS->GPF_MFPL = (SYS->GPF_MFPL & (~UART2_nRTS_PF4_Msk)) | UART2_nRTS_PF4        /*!< Set PF4 function to UART2_nRTS       */
#define SET_VDET_P0_PB0()        SYS->GPB_MFPL = (SYS->GPB_MFPL & (~VDET_P0_PB0_Msk)) | VDET_P0_PB0              /*!< Set PB0 function to VDET_P0          */
#define SET_VDET_P1_PB1()        SYS->GPB_MFPL = (SYS->GPB_MFPL & (~VDET_P1_PB1_Msk)) | VDET_P1_PB1              /*!< Set PB1 function to VDET_P1          */
#define SET_X32_IN_PF5()         SYS->GPF_MFPL = (SYS->GPF_MFPL & (~X32_IN_PF5_Msk)) | X32_IN_PF5                /*!< Set PF5 function to X32_IN           */
#define SET_X32_OUT_PF4()        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~X32_OUT_PF4_Msk)) | X32_OUT_PF4              /*!< Set PF4 function to X32_OUT          */
#define SET_XT1_IN_PF3()         SYS->GPF_MFPL = (SYS->GPF_MFPL & (~XT1_IN_PF3_Msk)) | XT1_IN_PF3                /*!< Set PF3 function to XT1_IN           */
#define SET_XT1_OUT_PF2()        SYS->GPF_MFPL = (SYS->GPF_MFPL & (~XT1_OUT_PF2_Msk)) | XT1_OUT_PF2              /*!< Set PF2 function to XT1_OUT          */
#define SET_GPIO_PA0()           SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(0xful << SYS_GPA_MFPL_PA0MFP_Pos)))          /*!< Set PA0 function to GPIO             */
#define SET_GPIO_PA1()           SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(0xful << SYS_GPA_MFPL_PA1MFP_Pos)))          /*!< Set PA1 function to GPIO             */
#define SET_GPIO_PA2()           SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(0xful << SYS_GPA_MFPL_PA2MFP_Pos)))          /*!< Set PA2 function to GPIO             */
#define SET_GPIO_PA3()           SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(0xful << SYS_GPA_MFPL_PA3MFP_Pos)))          /*!< Set PA3 function to GPIO             */
#define SET_GPIO_PA4()           SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(0xful << SYS_GPA_MFPL_PA4MFP_Pos)))          /*!< Set PA4 function to GPIO             */
#define SET_GPIO_PA5()           SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(0xful << SYS_GPA_MFPL_PA5MFP_Pos)))          /*!< Set PA5 function to GPIO             */
#define SET_GPIO_PA6()           SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(0xful << SYS_GPA_MFPL_PA6MFP_Pos)))          /*!< Set PA6 function to GPIO             */
#define SET_GPIO_PA7()           SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(0xful << SYS_GPA_MFPL_PA7MFP_Pos)))          /*!< Set PA7 function to GPIO             */
#define SET_GPIO_PA8()           SYS->GPA_MFPH = (SYS->GPA_MFPH & (~(0xful << SYS_GPA_MFPH_PA8MFP_Pos)))          /*!< Set PA8 function to GPIO             */
#define SET_GPIO_PA9()           SYS->GPA_MFPH = (SYS->GPA_MFPH & (~(0xful << SYS_GPA_MFPH_PA9MFP_Pos)))          /*!< Set PA9 function to GPIO             */
#define SET_GPIO_PA10()          SYS->GPA_MFPH = (SYS->GPA_MFPH & (~(0xful << SYS_GPA_MFPH_PA10MFP_Pos)))         /*!< Set PA10 function to GPIO            */
#define SET_GPIO_PA11()          SYS->GPA_MFPH = (SYS->GPA_MFPH & (~(0xful << SYS_GPA_MFPH_PA11MFP_Pos)))         /*!< Set PA11 function to GPIO            */
#define SET_GPIO_PB0()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~(0xful << SYS_GPB_MFPL_PB0MFP_Pos)))          /*!< Set PB0 function to GPIO             */
#define SET_GPIO_PB1()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~(0xful << SYS_GPB_MFPL_PB1MFP_Pos)))          /*!< Set PB1 function to GPIO             */
#define SET_GPIO_PB2()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~(0xful << SYS_GPB_MFPL_PB2MFP_Pos)))          /*!< Set PB2 function to GPIO             */
#define SET_GPIO_PB3()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~(0xful << SYS_GPB_MFPL_PB3MFP_Pos)))          /*!< Set PB3 function to GPIO             */
#define SET_GPIO_PB4()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~(0xful << SYS_GPB_MFPL_PB4MFP_Pos)))          /*!< Set PB4 function to GPIO             */
#define SET_GPIO_PB5()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~(0xful << SYS_GPB_MFPL_PB5MFP_Pos)))          /*!< Set PB5 function to GPIO             */
#define SET_GPIO_PB6()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~(0xful << SYS_GPB_MFPL_PB6MFP_Pos)))          /*!< Set PB6 function to GPIO             */
#define SET_GPIO_PB7()           SYS->GPB_MFPL = (SYS->GPB_MFPL & (~(0xful << SYS_GPB_MFPL_PB7MFP_Pos)))          /*!< Set PB7 function to GPIO             */
#define SET_GPIO_PB8()           SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(0xful << SYS_GPB_MFPH_PB8MFP_Pos)))          /*!< Set PB8 function to GPIO             */
#define SET_GPIO_PB9()           SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(0xful << SYS_GPB_MFPH_PB9MFP_Pos)))          /*!< Set PB9 function to GPIO             */
#define SET_GPIO_PB10()          SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(0xful << SYS_GPB_MFPH_PB10MFP_Pos)))         /*!< Set PB10 function to GPIO            */
#define SET_GPIO_PB11()          SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(0xful << SYS_GPB_MFPH_PB11MFP_Pos)))         /*!< Set PB11 function to GPIO            */
#define SET_GPIO_PB12()          SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(0xful << SYS_GPB_MFPH_PB12MFP_Pos)))         /*!< Set PB12 function to GPIO            */
#define SET_GPIO_PB13()          SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(0xful << SYS_GPB_MFPH_PB13MFP_Pos)))         /*!< Set PB13 function to GPIO            */
#define SET_GPIO_PB14()          SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(0xful << SYS_GPB_MFPH_PB14MFP_Pos)))         /*!< Set PB14 function to GPIO            */
#define SET_GPIO_PB15()          SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(0xful << SYS_GPB_MFPH_PB15MFP_Pos)))         /*!< Set PB15 function to GPIO            */
#define SET_GPIO_PC0()           SYS->GPC_MFPL = (SYS->GPC_MFPL & (~(0xful << SYS_GPC_MFPL_PC0MFP_Pos)))          /*!< Set PC0 function to GPIO             */
#define SET_GPIO_PC1()           SYS->GPC_MFPL = (SYS->GPC_MFPL & (~(0xful << SYS_GPC_MFPL_PC1MFP_Pos)))          /*!< Set PC1 function to GPIO             */
#define SET_GPIO_PC2()           SYS->GPC_MFPL = (SYS->GPC_MFPL & (~(0xful << SYS_GPC_MFPL_PC2MFP_Pos)))          /*!< Set PC2 function to GPIO             */
#define SET_GPIO_PC3()           SYS->GPC_MFPL = (SYS->GPC_MFPL & (~(0xful << SYS_GPC_MFPL_PC3MFP_Pos)))          /*!< Set PC3 function to GPIO             */
#define SET_GPIO_PC4()           SYS->GPC_MFPL = (SYS->GPC_MFPL & (~(0xful << SYS_GPC_MFPL_PC4MFP_Pos)))          /*!< Set PC4 function to GPIO             */
#define SET_GPIO_PC5()           SYS->GPC_MFPL = (SYS->GPC_MFPL & (~(0xful << SYS_GPC_MFPL_PC5MFP_Pos)))          /*!< Set PC5 function to GPIO             */
#define SET_GPIO_PC6()           SYS->GPC_MFPL = (SYS->GPC_MFPL & (~(0xful << SYS_GPC_MFPL_PC6MFP_Pos)))          /*!< Set PC6 function to GPIO             */
#define SET_GPIO_PC7()           SYS->GPC_MFPL = (SYS->GPC_MFPL & (~(0xful << SYS_GPC_MFPL_PC7MFP_Pos)))          /*!< Set PC7 function to GPIO             */
#define SET_GPIO_PC14()          SYS->GPC_MFPH = (SYS->GPC_MFPH & (~(0xful << SYS_GPC_MFPH_PC14MFP_Pos)))         /*!< Set PC14 function to GPIO            */
#define SET_GPIO_PD0()           SYS->GPD_MFPL = (SYS->GPD_MFPL & (~(0xful << SYS_GPD_MFPL_PD0MFP_Pos)))          /*!< Set PD0 function to GPIO             */
#define SET_GPIO_PD1()           SYS->GPD_MFPL = (SYS->GPD_MFPL & (~(0xful << SYS_GPD_MFPL_PD1MFP_Pos)))          /*!< Set PD1 function to GPIO             */
#define SET_GPIO_PD2()           SYS->GPD_MFPL = (SYS->GPD_MFPL & (~(0xful << SYS_GPD_MFPL_PD2MFP_Pos)))          /*!< Set PD2 function to GPIO             */
#define SET_GPIO_PD3()           SYS->GPD_MFPL = (SYS->GPD_MFPL & (~(0xful << SYS_GPD_MFPL_PD3MFP_Pos)))          /*!< Set PD3 function to GPIO             */
#define SET_GPIO_PD15()          SYS->GPD_MFPH = (SYS->GPD_MFPH & (~(0xful << SYS_GPD_MFPH_PD15MFP_Pos)))         /*!< Set PD15 function to GPIO            */
#define SET_GPIO_PF0()           SYS->GPF_MFPL = (SYS->GPF_MFPL & (~(0xful << SYS_GPF_MFPL_PF0MFP_Pos)))          /*!< Set PF0 function to GPIO             */
#define SET_GPIO_PF1()           SYS->GPF_MFPL = (SYS->GPF_MFPL & (~(0xful << SYS_GPF_MFPL_PF1MFP_Pos)))          /*!< Set PF1 function to GPIO             */
#define SET_GPIO_PF2()           SYS->GPF_MFPL = (SYS->GPF_MFPL & (~(0xful << SYS_GPF_MFPL_PF2MFP_Pos)))          /*!< Set PF2 function to GPIO             */
#define SET_GPIO_PF3()           SYS->GPF_MFPL = (SYS->GPF_MFPL & (~(0xful << SYS_GPF_MFPL_PF3MFP_Pos)))          /*!< Set PF3 function to GPIO             */
#define SET_GPIO_PF4()           SYS->GPF_MFPL = (SYS->GPF_MFPL & (~(0xful << SYS_GPF_MFPL_PF4MFP_Pos)))          /*!< Set PF4 function to GPIO             */
#define SET_GPIO_PF5()           SYS->GPF_MFPL = (SYS->GPF_MFPL & (~(0xful << SYS_GPF_MFPL_PF5MFP_Pos)))          /*!< Set PF5 function to GPIO             */
#define SET_GPIO_PF6()           SYS->GPF_MFPL = (SYS->GPF_MFPL & (~(0xful << SYS_GPF_MFPL_PF6MFP_Pos)))          /*!< Set PF6 function to GPIO             */


/**
  * @brief      Clear Brown-out detector interrupt flag
  * @param      None
  * @return     None
  * @details    This macro clear Brown-out detector interrupt flag.
  */
#define SYS_CLEAR_BOD_INT_FLAG()        (SYS->BODCTL |= SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Set Brown-out detector function to normal mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to normal mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_CLEAR_BOD_LPM()             (SYS->BODCTL &= ~SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD()               (SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD()                (SYS->BODCTL |= SYS_BODCTL_BODEN_Msk)

/**
  * @brief      Get Brown-out detector interrupt flag
  * @param      None
  * @retval     0   Brown-out detect interrupt flag is not set.
  * @retval     >=1 Brown-out detect interrupt flag is set.
  * @details    This macro get Brown-out detector interrupt flag.
  */
#define SYS_GET_BOD_INT_FLAG()          (SYS->BODCTL & SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Get Brown-out detector status
  * @param      None
  * @retval     0   System voltage is higher than BOD threshold voltage setting or BOD function is disabled.
  * @retval     >=1 System voltage is lower than BOD threshold voltage setting.
  * @details    This macro get Brown-out detector output status.
  *             If the BOD function is disabled, this function always return 0.
  */
#define SYS_GET_BOD_OUTPUT()            (SYS->BODCTL & SYS_BODCTL_BODOUT_Msk)

/**
  * @brief      Enable Brown-out detector interrupt function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector interrupt function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD_RST()           (SYS->BODCTL &= ~SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD_RST()            (SYS->BODCTL |= SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Set Brown-out detector function low power mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to low power mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LPM()               (SYS->BODCTL |= SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_4_5V
  *             - \ref SYS_BODCTL_BODVL_3_7V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_2_2V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LEVEL(u32Level)     (SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) | (u32Level))

/**
  * @brief      Get reset source is from Brown-out detector reset
  * @param      None
  * @retval     0   Previous reset source is not from Brown-out detector reset
  * @retval     >=1 Previous reset source is from Brown-out detector reset
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  */
#define SYS_IS_BOD_RST()                (SYS->RSTSTS & SYS_RSTSTS_BODRF_Msk)

/**
  * @brief      Get reset source is from CPU reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU reset
  * @retval     >=1 Previous reset source is from CPU reset
  * @details    This macro get previous reset source is from CPU reset.
  */
#define SYS_IS_CPU_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPURF_Msk)

/**
  * @brief      Get reset source is from LVR Reset
  * @param      None
  * @retval     0   Previous reset source is not from Low-Voltage-Reset
  * @retval     >=1 Previous reset source is from Low-Voltage-Reset
  * @details    This macro get previous reset source is from Low-Voltage-Reset.
  */
#define SYS_IS_LVR_RST()                (SYS->RSTSTS & SYS_RSTSTS_LVRF_Msk)

/**
  * @brief      Get reset source is from Power-on Reset
  * @param      None
  * @retval     0   Previous reset source is not from Power-on Reset
  * @retval     >=1 Previous reset source is from Power-on Reset
  * @details    This macro get previous reset source is from Power-on Reset.
  */
#define SYS_IS_POR_RST()                (SYS->RSTSTS & SYS_RSTSTS_PORF_Msk)

/**
  * @brief      Get reset source is from reset pin reset
  * @param      None
  * @retval     0   Previous reset source is not from reset pin reset
  * @retval     >=1 Previous reset source is from reset pin reset
  * @details    This macro get previous reset source is from reset pin reset.
  */
#define SYS_IS_RSTPIN_RST()             (SYS->RSTSTS & SYS_RSTSTS_PINRF_Msk)

/**
  * @brief      Get reset source is from system reset
  * @param      None
  * @retval     0   Previous reset source is not from system reset
  * @retval     >=1 Previous reset source is from system reset
  * @details    This macro get previous reset source is from system reset.
  */
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSTS & SYS_RSTSTS_MCURF_Msk)

/**
  * @brief      Get reset source is from window watch dog reset
  * @param      None
  * @retval     0   Previous reset source is not from window watch dog reset
  * @retval     >=1 Previous reset source is from window watch dog reset
  * @details    This macro get previous reset source is from window watch dog reset.
  */
#define SYS_IS_WDT_RST()                (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)

/**
  * @brief      Disable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_LVR()               (SYS->BODCTL &= ~SYS_BODCTL_LVREN_Msk)

/**
  * @brief      Enable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_LVR()                (SYS->BODCTL |= SYS_BODCTL_LVREN_Msk)

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_POR()               (SYS->PORCTL = 0x5AA5)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_POR()                (SYS->PORCTL = 0)

/**
  * @brief      Clear reset source flag
  * @param[in]  u32RstSrc is reset source. Including :
  *             - \ref SYS_RSTSTS_PORF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_LVRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_MCURF_Msk
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_CPULKRF_Msk
  * @return     None
  * @details    This macro clear reset source flag.
  */
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) ((SYS->RSTSTS) = (u32RstSrc) )




/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
__STATIC_INLINE void SYS_UnlockReg(void)
{
    uint32_t u32TimeOutCnt = __HIRC;

    do
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;

        if(--u32TimeOutCnt == 0) break;
    }
    while(SYS->REGLCTL == 0);
}

/**
  * @brief      Enable register write-protection function
  * @param      None
  * @return     None
  * @details    This function is used to enable register write-protection function.
  *             To lock the protected register to forbid write access.
  */
__STATIC_INLINE void SYS_LockReg(void)
{
    SYS->REGLCTL = 0;
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);
void SYS_SetVRef(uint32_t u32VRefCTL);


/*@}*/ /* end of group SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SYS_Driver */

/*@}*/ /* end of group Standard_Driver */


#ifdef __cplusplus
}
#endif

#endif //__SYS_H__

