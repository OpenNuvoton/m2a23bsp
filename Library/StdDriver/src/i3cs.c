/**************************************************************************//**
 * @file     i3cs.c
 * @version  V3.00
 * @brief    M2A23 series I3CS Controller (I3CS) driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup I3CS_Driver I3CS Driver
  @{
*/

/** @addtogroup I3CS_EXPORTED_FUNCTIONS I3CS Exported Functions
  @{
*/

/**
  * @brief      Open I3CS with Static Address and Select Dynamic Address Assignment Mode
  *
  * @param[in]  i3cs            The pointer of the specified I3CS module.
  * @param[in]  u8StaticAddr    7-bit slave address for I2C operation.
  * @param[in]  u32ModeSel      Initial mode selection to support ENTDAA CCC or Hot-Join generation. Valid values are:
  *                                 - \ref I3CS_SUPPORT_ENTDAA
  *                                 - \ref I3CS_SUPPORT_ADAPTIVE_HJ
  *                                 - \ref I3CS_SUPPORT_IMMEDIATE_HJ
  *
  * @return     None
  *
  * @details    This API is used to configure I3C Slave controller can receive ENTDAA CCC or generate a Hot-Join request.
  *             The 7-bit slave address is used for related I2C operations until I3C Slave controller has a valid Dynamic Address.
  * @note       This API also enables all interrupt status and configures TX, RX FIFO and Response, Command Queue threshold to 0.
  */
void I3CS_Open(I3CS_T *i3cs, uint8_t u8StaticAddr, uint32_t u32ModeSel)
{            
    /* Enable chip's I3CS function */
    SYS->SPDHCTL &= ~SYS_SPDHCTL_SPDHEN_Msk;
    
    if(u8StaticAddr == 0)
    {
        i3cs->DEVADDR &= ~I3CS_DEVADDR_SAVALID_Msk;
    }
    else
    {
        i3cs->DEVADDR &= ~I3CS_DEVADDR_SA_Msk;
        i3cs->DEVADDR |= (I3CS_DEVADDR_SAVALID_Msk | u8StaticAddr);
    }     
    
    /* Enable all interrupt status */
    i3cs->INTSTSEN = 0xFFFFFFFF;
    
    /* Set Tx/Rx/CmdQ/ReapQ threshold to 0 */
    i3cs->DBTHCTL = 0x0;
    i3cs->QUETHCTL = 0x0;
        
    /* Select the slave ability for enter I3CS mode */
    if(u32ModeSel == I3CS_SUPPORT_ENTDAA)
    {
        /* HJEN disabled: Slave supports ENTDAA CCC */
        i3cs->DEVCTL |= I3CS_DEVCTL_ADAPTIVE_Msk;
        i3cs->SLVEVNTS &= ~I3CS_SLVEVNTS_HJEN_Msk;
    }
    else if(u32ModeSel == I3CS_SUPPORT_ADAPTIVE_HJ)
    {
        /* Both ADAPTIVE and HJEN enabled: Slave generates a Hot-Join request until receive I3CS header 7'h7E on the bus */
        i3cs->DEVCTL |= I3CS_DEVCTL_ADAPTIVE_Msk;
        i3cs->SLVEVNTS |= I3CS_SLVEVNTS_HJEN_Msk;
    }
    else if(u32ModeSel == I3CS_SUPPORT_IMMEDIATE_HJ)
    {
        /* Only HJEN enabled: Slave generates a Hot-Join request immediately */
        i3cs->DEVCTL &= ~I3CS_DEVCTL_ADAPTIVE_Msk;
        i3cs->SLVEVNTS |= I3CS_SLVEVNTS_HJEN_Msk;
    }
    else
    {
        /* HJEN disabled: Slave supports ENTDAA CCC */
        i3cs->DEVCTL |= I3CS_DEVCTL_ADAPTIVE_Msk;
        i3cs->SLVEVNTS &= ~I3CS_SLVEVNTS_HJEN_Msk;
    }
}

/**
  * @brief      Reset and Resume I3CS Controller
  *
  * @param[in]  i3cs            The pointer of the specified I3CS module.
  * @param[in]  u32ResetMask    Software reset operation of I3CS module. Valid values are:
  *                                 - \ref I3CS_RESET_CMD_QUEUE
  *                                 - \ref I3CS_RESET_RESP_QUEUE
  *                                 - \ref I3CS_RESET_TX_BUF
  *                                 - \ref I3CS_RESET_RX_BUF
  *                                 - \ref I3CS_RESET_ALL_QUEUE_AND_BUF
  * @param[in]  u32EnableResume Enable resume I3C Slave controller. Valid values are TRUE and FALSE.
  *
  * @retval     I3CS_STS_NO_ERR      No error
  * @retval     I3CS_TIMEOUT_ERR     Function time-out
  *
  * @details    This API is used to reset specified FIFO and Queue or resume the I3C Slave controller from the halt state.
  * @note       THe application can reset Queues and FIFO only when the Slave controller is disabled or 
  *             when the Slave controller is in Halt state(I3CS_CCCDEVS[9] SLVBUSY = 1) after any error occurred from the I3C Master request.
  */
int32_t I3CS_ResetAndResume(I3CS_T *i3cs, uint32_t u32ResetMask, uint32_t u32EnableResume)
{
    uint8_t u8InHaltState = 0;
    volatile uint32_t u32Timeout;
    
    if(I3CS_IS_SLAVE_BUSY(i3cs))
        u8InHaltState = 1;
        
    if(u32ResetMask)
    {
        if(u8InHaltState == 0)
        {
            /* Disable I3CS controller for reset buffer and queue */
            if(I3CS_Disable(i3cs) != I3CS_STS_NO_ERR)
                return I3CS_TIMEOUT_ERR;
        }

        /* Reset specify source */
        i3cs->RSTCTL = u32ResetMask;
        u32Timeout = (SystemCoreClock / 1000);
        while((i3cs->RSTCTL != 0) && (--u32Timeout)) {}
        if(u32Timeout == 0)
            return I3CS_TIMEOUT_ERR;
                    
        if(u8InHaltState == 0)
        {
            /* Enable I3CS controller again */
            if(I3CS_Enable(i3cs) != I3CS_STS_NO_ERR)
                return I3CS_TIMEOUT_ERR;
        }
    }
    
    if(u32EnableResume || u8InHaltState)
    {
        /* The application has to take necessary action to handle the error condition and 
            then set RESUME bit to resume the controller. */
        /* Slave will receive GETSTATUS CCC to clear specify status in I3CS_CCCDEVS register. */
        i3cs->DEVCTL |= I3CS_DEVCTL_RESUME_Msk;
        while((i3cs->DEVCTL&I3CS_DEVCTL_RESUME_Msk) == I3CS_DEVCTL_RESUME_Msk) {}
            
        /* RESUME bit is auto-cleared once the controller is ready to accept new transfers. */
    }    
    
    return I3CS_STS_NO_ERR;
}

/**
  * @brief      Get Resopnse Status and Receive Data
  *
  * @param[in]  i3cs            The pointer of the specified I3CS module.
  * @param[out] pRespQ          The response data structure to get the response data.
  *
  * @retval     I3CS_STS_NO_ERR          No error
  * @retval     I3CS_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3CS_STS_RESPQ_EMPTY     No Response Queue data
  * @return     Response error status
  *
  * @details    This API is used to get the response data and the received data on Master write operation.
  */
int32_t I3CS_ParseRespQueue(I3CS_T *i3cs, uint32_t *pu32RespQ)
{
    uint8_t qn, u8RespQCnt;
    
    /* Check if RespQ buffer is empty */
    if(pu32RespQ == NULL)
        return I3CS_STS_INVALID_INPUT;
    
    /* Check if RespQ is empty */
    if(I3CS_IS_RESPQ_EMPTY(i3cs))
        return I3CS_STS_RESPQ_EMPTY;
    
    u8RespQCnt = I3CS_GET_RESPQ_THLD(i3cs) + 1;
    
    qn = 0; // Queue number
    do {
        pu32RespQ[qn] = I3CS_GET_RESP_DATA(i3cs);
        if(!I3CS_IS_RESP_NO_ERR(pu32RespQ[qn]))
            return (pu32RespQ[qn] & I3CS_RESPQUE_ERRSTS_Msk);
        
        qn++;
        u8RespQCnt--;
    }while(u8RespQCnt);
    
    return I3CS_STS_NO_ERR;
}

/**
  * @brief      Set Command Queue and Transmit Data for Master Read
  *
  * @param[in]  i3cs            The pointer of the specified I3CS module.
  * @param[in]  u8TID           Specified Transmit Transaction ID in Command Queue.
  * @param[in]  pu32TxBuf       The buffer to send the data to transmit FIFO.
  * @param[in]  u16WriteBytes   The byte number of TX data.
  *
  * @retval     I3CS_STS_NO_ERR          No error
  * @retval     I3CS_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3CS_STS_CMDQ_FULL       Command Queue is full
  * @retval     I3CS_STS_TX_FULL         TX FIFO is full
  *
  * @details    This API is used to prepare a Command Queue and TX response data for Master read operation.
  */
int32_t I3CS_SetCmdQueueAndData(I3CS_T *i3cs, uint8_t u8TID, uint32_t *pu32TxBuf, uint16_t u16WriteBytes)
{
    uint32_t i;
    
    /* Check if write bytes is exceeded */
    if(u16WriteBytes > (I3CS_DEVICE_TX_BUF_CNT *4 ))
        return I3CS_STS_INVALID_INPUT;

    /* Check if CmdQ is full */
    if(I3CS_IS_CMDQ_FULL(i3cs))
        return I3CS_STS_CMDQ_FULL;
        
    if(pu32TxBuf != NULL)
    {
        for(i=0; i<((u16WriteBytes+3)/4); i++)
        {
            /* Check if TX buffer is full */
            if(I3CS_IS_TX_FULL(i3cs))
                return I3CS_STS_TX_FULL;
            
            i3cs->TXRXDAT = pu32TxBuf[i];
        }
    }
    
    i3cs->CMDQUE = ((u8TID << I3CS_CMDQUE_TID_Pos) | (u16WriteBytes << I3CS_CMDQUE_DATLEN_Pos));
        
    return I3CS_STS_NO_ERR;
}

/**
  * @brief      Generate IBI Request
  *
  * @param[in]  i3cs            The pointer of the specified I3CS module.
  * @param[in]  u8MandatoryData The mandatory data byte.
  * @param[in]  u32PayloadData  The payload data.
  * @param[in]  u8PayloadLen    The byte number of payload data. The maximum length is 4 bytes.
  *
  * @retval     I3CS_STS_NO_ERR              No error, IBI request accepted by the Master
  * @retval     I3CS_STS_INVALID_STATE       Invalid state
  * @retval     I3CS_STS_INVALID_INPUT       Invalid input parameter
  *
  * @details    This API is used to generate an IBI request on the bus.
  */
int32_t I3CS_SendIBIRequest(I3CS_T *i3cs, uint8_t u8MandatoryData, uint32_t u32PayloadData, uint8_t u8PayloadLen)
{    
    /* Check if Controller is in busy state */
    if(I3CS_IS_SLAVE_BUSY(i3cs))
        return I3CS_STS_INVALID_STATE;
        
    /* Check if SIR function enabled */
    if(!(i3cs->SLVEVNTS&I3CS_SLVEVNTS_SIREN_Msk))
        return I3CS_STS_INVALID_STATE;
        
    /* Check if payload length > 4-bytes */
    if(u8PayloadLen > 4)
        return I3CS_STS_INVALID_INPUT;

    /* Program IBI payload data, payload length and MDB */
    i3cs->SIR = (u8PayloadLen << I3CS_SIR_DATLEN_Pos) | (u8MandatoryData << I3CS_SIR_MDB_Pos) | (0 << I3CS_SIR_CTL_Pos);
    i3cs->SIRDAT = u32PayloadData;
        
    /* Trigger IBI request */
    /* SIR_EN bit be cleared automatically after the Master accepts the IBI request or Slave unable to issue the IBI request */
    i3cs->SIR |= I3CS_SIR_EN_Msk;
    
    return I3CS_STS_NO_ERR;
}

/**
  * @brief      Disable HJ Generation
  *
  * @param[in]  i3cs                The pointer of the specified I3CS module.
  *
  * @retval     I3CS_STS_NO_ERR      No error
  *
  * @details    This API is used to disable the HJ generation.
  * @note       I3C Slave controller can recognize the ENTDAA CCC after disabling the HJ generation.
  */
int32_t I3CS_DisableHJRequest(I3CS_T *i3cs)
{
    /* HJEN disabled: Slave supports ENTDAA CCC */
    i3cs->SLVEVNTS &= ~I3CS_SLVEVNTS_HJEN_Msk;
    i3cs->DEVCTL |= I3CS_DEVCTL_ADAPTIVE_Msk;
    
    return I3CS_STS_NO_ERR;
}

/**
  * @brief      I3CS Response Error Recovery
  *
  * @param[in]  i3cs            The pointer of the specified I3CS module.
  * @param[in]  u32RespStatus   Response error status from the response queue.
  *
  * @return     I3CS_STS_NO_ERR
  *
  * @details    This API is used to perform error recovery then the I3C Slave controller can leave Halt(Busy) state.
  * @note       The RESUME operation is completed after a GETSTATUS CCC is received or the specified Master operation is successfully.
  */
int32_t I3CS_RespErrorRecovery(I3CS_T *i3cs, uint32_t u32RespStatus)
{
    if(u32RespStatus != I3CS_STS_NO_ERR)
    {
        if(I3CS_IS_SLAVE_BUSY(i3cs))
        {
            switch(u32RespStatus)
            {
                case I3CS_RESP_CRC_ERR:
                case I3CS_RESP_PARITY_ERR:
                case I3CS_RESP_FRAME_ERRR:
                case I3CS_RESP_FLOW_ERR:
                    /* Reset RX FIFO -> apply resume */
                    I3CS_ResetAndResume(i3cs, I3CS_RESET_RX_BUF, TRUE);
                    break;
                
                case I3CS_RESP_MASTER_TERMINATE_ERR:
                    /* Reset TX FIFO and CMDQ Queue -> apply resume */
                    I3CS_ResetAndResume(i3cs, (I3CS_RESET_TX_BUF | I3CS_RESET_CMD_QUEUE), TRUE);
                    break;
                
                default:
                    /* Reset all FIFO and Queue */
                    I3CS_ResetAndResume(i3cs, I3CS_RESET_ALL_QUEUE_AND_BUF, FALSE);
                    break;
            }
        }
    }
    
    return I3CS_STS_NO_ERR;
}

/*@}*/ /* end of group I3CS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I3CS_Driver */

/*@}*/ /* end of group Standard_Driver */
