/**************************************************************************//**
 * @file     adc.h
 * @version  V3.00
 * @brief    M2A23 series ADC driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup ADC_Driver ADC Driver
  @{
*/

/** @addtogroup ADC_EXPORTED_CONSTANTS ADC Exported Constants
  @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  ADCR Constant Definitions                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_ADCR_ADEN_CONVERTER_DISABLE   (0UL<<ADC_ADCR_ADEN_Pos)   /*!< ADC converter disable	*/
#define ADC_ADCR_ADEN_CONVERTER_ENABLE    (1UL<<ADC_ADCR_ADEN_Pos)   /*!< ADC converter enable	*/

#define ADC_ADCR_ADMD_SINGLE            (0UL<<ADC_ADCR_ADMD_Pos)     /*!< Single mode        			*/
#define ADC_ADCR_ADMD_BURST             (1UL<<ADC_ADCR_ADMD_Pos)     /*!< Burst mode              */
#define ADC_ADCR_ADMD_SINGLE_CYCLE      (2UL<<ADC_ADCR_ADMD_Pos)     /*!< Single cycle scan mode  */
#define ADC_ADCR_ADMD_CONTINUOUS        (3UL<<ADC_ADCR_ADMD_Pos)     /*!< Continuous scan mode    */

#define ADC_ADCR_DIFFEN_SINGLE_END      (0UL<<ADC_ADCR_DIFFEN_Pos)   /*!< Single end input mode   */
#define ADC_ADCR_DIFFEN_DIFFERENTIAL    (1UL<<ADC_ADCR_DIFFEN_Pos)   /*!< Differential input type */

#define ADC_ADCR_DMOF_UNSIGNED_OUTPUT   (0UL<<ADC_ADCR_DMOF_Pos)     /*!< Select the straight binary format as the output format of the conversion result	*/
#define ADC_ADCR_DMOF_TWOS_COMPLEMENT   (1UL<<ADC_ADCR_DMOF_Pos)     /*!< Select the 2's complement format as the output format of the conversion result	*/

#define ADC_ADCR_TRGS_DISABLE           (0UL<<ADC_ADCR_TRGS_Pos)     /*!< Disable triggering of A/D conversion by external STADC pin, PWM trigger, BPWM trigger, Timer trigger and ACMP trigger */
#define ADC_ADCR_TRGS_STADC             (1UL<<ADC_ADCR_TRGS_Pos)     /*!< A/D conversion is started by external STADC pin */
#define ADC_ADCR_TRGS_TIMER0            (2UL<<ADC_ADCR_TRGS_Pos)     /*!< A/D conversion is started by Timer0 overflow pulse trigger */
#define ADC_ADCR_TRGS_TIMER1            (3UL<<ADC_ADCR_TRGS_Pos)     /*!< A/D conversion is started by Timer1 overflow pulse trigger */
#define ADC_ADCR_TRGS_TIMER2            (4UL<<ADC_ADCR_TRGS_Pos)     /*!< A/D conversion is started by Timer2 overflow pulse trigger */
#define ADC_ADCR_TRGS_TIMER3            (5UL<<ADC_ADCR_TRGS_Pos)     /*!< A/D conversion is started by Timer3 overflow pulse trigger */
#define ADC_ADCR_TRGS_PWM               (6UL<<ADC_ADCR_TRGS_Pos)     /*!< A/D conversion is started by PWM trigger	 */
#define ADC_ADCR_TRGS_BPWM    		    (7UL<<ADC_ADCR_TRGS_Pos)     /*!< A/D conversion is started by BPWM trigger  */
#define ADC_ADCR_TRGS_ACMP0    		    (8UL<<ADC_ADCR_TRGS_Pos)     /*!< A/D conversion is started by ACMP0 trigger */
#define ADC_ADCR_TRGS_ACMP1            	(9UL<<ADC_ADCR_TRGS_Pos)     /*!< A/D conversion is started by ACMP1 trigger */

#define ADC_ADCR_TRGCOND_LOW_LEVEL      (0UL<<ADC_ADCR_TRGCOND_Pos)  /*!< STADC Low level active     */
#define ADC_ADCR_TRGCOND_HIGH_LEVEL     (1UL<<ADC_ADCR_TRGCOND_Pos)  /*!< STADC High level active    */
#define ADC_ADCR_TRGCOND_FALLING_EDGE   (2UL<<ADC_ADCR_TRGCOND_Pos)  /*!< STADC Falling edge active  */
#define ADC_ADCR_TRGCOND_RISING_EDGE    (3UL<<ADC_ADCR_TRGCOND_Pos)  /*!< STADC Rising edge active   */

#define ADC_ADCR_EXTSMPT(x)             ((x) << ADC_ADCR_EXTSMPT_Pos)  /*!< Configure the extend ADC sampling time, Extended Sampling Time = (EXTSMPT+1) x ADC_CLK period */

/*---------------------------------------------------------------------------------------------------------*/
/* ADCMPR Constant Definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_ADCMPR_CMPD(x)                    ((x) << ADC_ADCMPR_CMPD_Pos)          /*!< Compare value for compare function            */
#define ADC_ADCMPR_CMPMATCNT(x)               (((x)-1) << ADC_ADCMPR_CMPMATCNT_Pos) /*!< Match count for compare function              */
#define ADC_ADCMPR_CMPCH(x)                   ((x) << ADC_ADCMPR_CMPCH_Pos)         /*!< Compare channel for compare function          */
#define ADC_ADCMPR_CMPCOND_LESS_THAN          (0UL<<ADC_ADCMPR_CMPCOND_Pos)           /*!< The compare condition is "less than"          */
#define ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL   (1UL<<ADC_ADCMPR_CMPCOND_Pos)           /*!< The compare condition is "greater than or equal to" */
#define ADC_ADCMPR_CMPIE_INTERRUPT_ENABLE     (ADC_ADCMPR_CMPIE_Msk)                /*!< The compare function interrupt enable */

/*---------------------------------------------------------------------------------------------------------*/
/* ADC Interrupt Constant Definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_ADF_INT         		(ADC_ADSR0_ADF_Msk)		/*!< ADC convert complete interrupt	*/
#define ADC_CMP0_INT        		(ADC_ADSR0_CMPF0_Msk)	/*!< ADC comparator 0 interrupt 		*/
#define ADC_CMP1_INT        		(ADC_ADSR0_CMPF1_Msk)	/*!< ADC comparator 1 interrupt 		*/

/*---------------------------------------------------------------------------------------------------------*/
/* ADC Operation Mode Constant Definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_SINGLE_MODE         0   /*!< ADC single mode            */
#define ADC_BURST_MODE          1   /*!< ADC burst mode             */
#define ADC_SINGLE_CYCLE_MODE   2   /*!< ADC single-cycle scan mode */
#define ADC_CONTINUOUS_MODE   	3   /*!< ADC continuous scan mode   */

/*---------------------------------------------------------------------------------------------------------*/
/* ADC Trigger Condition Constant Definitions                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_LOW_LEVEL      			0		/*!< ADC external trigger condition is low level trigger    */
#define ADC_HIGH_LEVEL     			1   /*!< ADC external trigger condition is high level trigger   */
#define ADC_FALLING_EDGE   			2   /*!< ADC external trigger condition is falling edge trigger */
#define ADC_RISING_EDGE    			3   /*!< ADC external trigger condition is rising edge trigger  */

/*---------------------------------------------------------------------------------------------------------*/
/* ADC Compare Condition Constant Definitions                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_LESS_THAN       		0   /*!< ADC compare condition is "less than the compare value"                */
#define ADC_GREATER_OR_EQUAL   	1   /*!< ADC compare condition is "greater than or equal to the compare value" */

/*--------------------------------------------------------------------------------------------------*/
/* Define Error Code                                                                                */
/*--------------------------------------------------------------------------------------------------*/
#define ADC_TIMEOUT_ERR    	 	(-1)	/*!< ADC operation abort due to timeout error */

/*@}*/ /* end of group ADC_EXPORTED_CONSTANTS */

extern int32_t g_ADC_i32ErrCode;

/** @addtogroup ADC_EXPORTED_FUNCTIONS ADC Exported Functions
  @{
*/


/**
  * @brief Get conversion data of specified channel.
  * @param[in] adc The pointer of the specified ADC module.
  * @param[in] u32ChNum ADC Channel, valid value are from 0 to 7 and 29 to 30.
  * @return  16-bit data.
  * @details Read RSLT bit field to get conversion data.
  */
#define ADC_GET_CONVERSION_DATA(adc, u32ChNum) ((adc)->ADDR[(u32ChNum)] & ADC_ADDR_RSLT_Msk)

/**
  * @brief Return the user-specified interrupt flags.
  * @param[in] adc The pointer of the specified ADC module.
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *                    Valid values are:
  *                     - \ref ADC_ADF_INT          :Convert complete interrupt flag.
  *                     - \ref ADC_CMP0_INT         :Comparator 0 interrupt flag.
  *                     - \ref ADC_CMP1_INT         :Comparator 1 interrupt flag.
  * @return  User specified interrupt flags.
  * @details Get the status of the ADC interrupt flag.
  */
#define ADC_GET_INT_FLAG(adc, u32Mask) ((adc)->ADSR0 & (u32Mask))

/**
  * @brief This macro clear the selected interrupt status bits.
  * @param[in] adc The pointer of the specified ADC module.
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *                    Valid values are:
  *                     - \ref ADC_ADF_INT          :Convert complete interrupt flag.
  *                     - \ref ADC_CMP0_INT         :Comparator 0 interrupt flag.
  *                     - \ref ADC_CMP1_INT         :Comparator 1 interrupt flag.
  * @return  None
  * @details ADF(ADC_ADSR[0])/CMPF0(ADC_ADSR[1])/CMPF2(ADC_ADSR[2]) can be cleared by writing 1 to itself.
  */
#define ADC_CLR_INT_FLAG(adc, u32Mask) ((adc)->ADSR0 = (u32Mask))

/**
  * @brief Get the busy state of ADC.
  * @param[in] adc The pointer of the specified ADC module.
  * @retval 0 ADC is not busy.
  * @retval 1 ADC is busy.
  * @details BUSY(ADC_ADSR0[7]) is a mirror of ADST(ADC_ADCR[11]).
  */
#define ADC_IS_BUSY(adc) ((adc)->ADSR0 & ADC_ADSR0_BUSY_Msk ? 1 : 0)

/**
  * @brief Check if the ADC conversion data is over written or not.
  * @param[in] adc The pointer of the specified ADC module.
  * @param[in] u32ChNum ADC Channel, valid value are from 0 to 7 and 29 to 30.
  * @retval 0 ADC data is not overrun.
  * @retval 1 ADC data is overrun.
  * @details OVERRUN(ADC_ADSR2[30:0]) is the mirror of OVERRUN(ADC_ADDR0~30[16]) bits.
  */
#define ADC_IS_DATA_OVERRUN(adc, u32ChNum) (((adc)->ADSR2 & (1<<(u32ChNum))) ? 1 : 0)

/**
  * @brief Check if the ADC conversion data is valid or not.
  * @param[in] adc The pointer of the specified ADC module.
  * @param[in] u32ChNum ADC Channel, valid value are from 0 to 15 and 29 to 30.
  * @retval 0 ADC data is not valid.
  * @retval 1 ADC data is valid.
  * @details VALID(ADC_ADDR0~30[17]) is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_ADDR register is read.
  */
#define ADC_IS_DATA_VALID(adc, u32ChNum) ((adc)->ADSR1 & (0x1<<(u32ChNum)) ? 1 : 0)

/**
  * @brief Power down ADC module.
  * @param[in] adc The pointer of the specified ADC module.
  * @return None
  * @details Disable A/D converter analog circuit for saving power consumption.
  */
#define ADC_POWER_DOWN(adc) ((adc)->ADCR &= ~ADC_ADCR_ADEN_Msk)

/**
  * @brief Power on ADC module.
  * @param[in] adc The pointer of the specified ADC module.
  * @return None
  * @details Before starting A/D conversion function, ADEN(ADC_ADCR[0]) bit should be set to 1.
  */
#define ADC_POWER_ON(adc) ((adc)->ADCR |= ADC_ADCR_ADEN_Msk)

/**
  * @brief Configure the comparator 0 and enable it.
  * @param[in] adc The pointer of the specified ADC module.
  * @param[in] u32ChNum  Specifies the source channel, valid value are from 0 to 15.
  * @param[in] u32Condition Specifies the compare condition. Valid values are:
  *                          - \ref ADC_ADCMPR_CMPCOND_LESS_THAN            :The compare condition is "less than the compare value".
  *                          - \ref ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value".
  * @param[in] u32Data Specifies the compare value, valid value are between 0 ~ 0xFFF.
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16.
  * @return None
  * @details For example, ADC_ENABLE_CMP0(ADC, 5, ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 10);
  *          Means ADC will assert comparator 0 flag if channel 5 conversion result is greater than or
  *          equal to 0x800 for 10 times continuously.
  */
#define ADC_ENABLE_CMP0(adc, \
                        u32ChNum, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) ((adc)->ADCMPR[0] = ((u32ChNum) << ADC_ADCMPR_CMPCH_Pos) | \
                                                           (u32Condition) | \
                                                           ((u32Data) << ADC_ADCMPR_CMPD_Pos) | \
                                                           (((u32MatchCount) - 1) << ADC_ADCMPR_CMPMATCNT_Pos) |\
                                                           ADC_ADCMPR_CMPEN_Msk)

/**
  * @brief Disable comparator 0
  * @param[in] adc The pointer of the specified ADC module
  * @return None
  * @details Set CMPEN(ADC_ADCMPR0[0]) to 0 and reset comparator 0 configurations to disable ADC compare function.
  */
#define ADC_DISABLE_CMP0(adc) ((adc)->ADCMPR[0] = 0)

/**
  * @brief Configure the comparator 1 and enable it.
  * @param[in] adc The pointer of the specified ADC module.
  * @param[in] u32ChNum  Specifies the source channel, valid value are from 0 to 15.
  * @param[in] u32Condition Specifies the compare condition. Valid values are:
  *                          - \ref ADC_ADCMPR_CMPCOND_LESS_THAN            :The compare condition is "less than the compare value".
  *                          - \ref ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value".
  * @param[in] u32Data Specifies the compare value, valid value are between 0 ~ 0xFFF.
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16.
  * @return None
  * @details For example, ADC_ENABLE_CMP1(ADC, 5, ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 10);
  *          Means ADC will assert comparator 1 flag if channel 5 conversion result is greater than or
  *          equal to 0x800 for 10 times continuously.
  */
#define ADC_ENABLE_CMP1(adc, \
                        u32ChNum, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) ((adc)->ADCMPR[1] = ((u32ChNum) << ADC_ADCMPR_CMPCH_Pos) | \
                                                           (u32Condition) | \
                                                           ((u32Data) << ADC_ADCMPR_CMPD_Pos) | \
                                                           (((u32MatchCount) - 1) << ADC_ADCMPR_CMPMATCNT_Pos) |\
                                                           ADC_ADCMPR_CMPEN_Msk)

/**
  * @brief Disable comparator 1.
  * @param[in] adc The pointer of the specified ADC module.
  * @return None
  * @details Set CMPEN(ADC_ADCMPR1[0]) to 0 and reset comparator 1 configurations to disable ADC compare function.
  */
#define ADC_DISABLE_CMP1(adc) ((adc)->ADCMPR[1] = 0)

/**
  * @brief Enable the compare window mode.
  * @param[in] adc The pointer of the specified ADC module.
  * @return None
  * @details CMPF0 (ADSR0[1]) will be set when both ADC_CMP0 and ADC_CMP1 compared condition matched.
  * \hideinitializer
  */
#define ADC_ENABLE_CMP_WINDOW_MODE(adc)		((adc)->ADCMPR[0] |= ADC_ADCMPR_CMPWEN_Msk)

/**
  * @brief Disable the compare window mode.
  * @param[in] adc The pointer of the specified ADC module.
  * @return None
  * @details Disable the compare window mode for specified LPADC module.
  * \hideinitializer
  */
#define ADC_DISABLE_CMP_WINDOW_MODE(adc)	((adc)->ADCMPR[0] &= ~ADC_ADCMPR_CMPWEN_Msk)

/**
  * @brief Set ADC input channel.
  * @param[in] adc The pointer of the specified ADC module.
  * @param[in] u32Mask  Channel enable bit. Each bit corresponds to a input channel. Bit 0 is channel 0, bit 1 is channel 1..., bit 7 is channel 7.
  * @return None
  * @details Enabled channel will be converted while ADC starts.
  * @note In single mode, ADC can only convert 1 channel. If more than 1 channel are enabled, only the channel with smallest number will be converted.
  */
#define ADC_SET_INPUT_CHANNEL(adc, u32Mask) ((adc)->ADCHER = ((adc)->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (u32Mask))

/**
  * @brief Set the output format mode.
  * @param[in] adc The pointer of the specified ADC module.
  * @param[in] u32Format Decides the output format. Valid values are:
  *                      - \ref ADC_ADCR_DMOF_UNSIGNED_OUTPUT      : Select the straight binary format as the output format of the conversion result.
  *                      - \ref ADC_ADCR_DMOF_TWOS_COMPLEMENT      : Select the 2's complement format as the output format of the conversion result.
  * @return None
  * @details  The macro is used to set the output format of ADC differential input mode.
  * @note ADC compare function can not support 2's complement output format, u32Format should be set to 0.
  */
#define ADC_SET_DMOF(adc, u32Format) ((adc)->ADCR = ((adc)->ADCR & ~ADC_ADCR_DMOF_Msk) | (u32Format))

/**
  * @brief Start the A/D conversion.
  * @param[in] adc The pointer of the specified ADC module.
  * @return None
  * @details Set ADST bit to 1 to start the A/D conversion.
  */
#define ADC_START_CONV(adc) ((adc)->ADCR |= ADC_ADCR_ADST_Msk)

/**
  * @brief Stop the A/D conversion.
  * @param[in] adc The pointer of the specified ADC module.
  * @return None
  * @details ADST(ADC_ADCR[11]) will be cleared to 0 by hardware automatically at the ends of single mode and single-cycle scan mode.
  *          In continuous scan mode and burst mode, A/D conversion is continuously performed until software writes 0 to this bit.
  * @note When the ADST bit is cleared to 0, the ADST bit must be kept at 0 at least one ADC peripheral clock period
  *       before setting it to 1 again, otherwise the A/D converter may not work.
  *       If ADST bit is cleared to 0 when ADC is in converting, the BUSY bit will be cleared to 0 immediately,
  *       ADC will terminate the current conversion and enter idle state directly.
  */
#define ADC_STOP_CONV(adc) ((adc)->ADCR &= ~ADC_ADCR_ADST_Msk)

/**
  * @brief Enable PDMA transfer.
  * @param[in] adc The pointer of the specified ADC module
  * @return None
  * @details Enable PDMA to transfer the conversion data.
  * @note While enable PDMA transfer, software must set ADIE = 0 to disable interrupt.
  */
#define ADC_ENABLE_PDMA(adc) ((adc)->ADCR |= ADC_ADCR_PTEN_Msk)

/**
  * @brief Disable PDMA transfer.
  * @param[in] adc The pointer of the specified ADC module
  * @return None
  * @details Disable PDMA to transfer the conversion data.
  */
#define ADC_DISABLE_PDMA(adc) ((adc)->ADCR &= ~ADC_ADCR_PTEN_Msk)

/**
  * @brief Get PDMA current transfer data
  * @param[in] adc The pointer of the specified ADC module.
  * @return  PDMA current transfer data
  * \hideinitializer
  */
#define ADC_GET_PDMA_DATA(adc) ((adc)->ADPDMA & ADC_ADPDMA_CURDAT_Msk)


void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_SetExtendSampleTime(ADC_T *adc,
                             uint32_t u32ModuleNum,
                             uint32_t u32ExtendSampleTime);

/*@}*/ /* end of group ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ADC_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__ADC_H__

