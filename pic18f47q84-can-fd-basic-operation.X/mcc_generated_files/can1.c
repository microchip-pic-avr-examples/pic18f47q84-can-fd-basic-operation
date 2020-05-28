/**
  CAN1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    can1.c

  @Summary
    This is the generated driver implementation file for the CAN1 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides implementations for driver APIs for CAN1.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.0
        Device            :  PIC18F47Q84
        Driver Version    :  1.1.1
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.10 and above or later
        MPLAB             :  MPLAB X 5.35
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include <xc.h>
#include <stdint.h>
#include <string.h>
#include "can1.h"

#define RX_FIFO_MSG_DATA                (32U)
#define NUM_OF_RX_FIFO                  (2U)

#define SID_LOW_WIDTH                   (8U)
#define SID_HIGH_MASK                   (0x07U)
#define EID_LOW_WIDTH                   (5U)
#define EID_LOW_POSN                    (3U)
#define EID_LOW_MASK                    (0xF8U)
#define EID_MID_WIDTH                   (8U)
#define EID_HIGH_WIDTH                  (5U)
#define EID_HIGH_MASK                   (0x1FU)
#define IDE_POSN                        (4U)
#define RTR_POSN                        (5U)
#define BRS_POSN                        (6U)
#define FDF_POSN                        (7U)

#define DLCToPayloadBytes(x)            (DLC_BYTES[(x)])
#define PLSIZEToPayloadBytes(x)         (DLCToPayloadBytes(8u + (x)))

struct CAN_FIFOREG
{
    uint8_t CONL;
    uint8_t CONH;
    uint8_t CONU;
    uint8_t CONT;
    uint8_t STAL;
    uint8_t STAH;
    uint8_t STAU;
    uint8_t STAT;
    uint32_t UA;
};

typedef enum
{
    CAN_RX_MSG_NOT_AVAILABLE = 0U,
    CAN_RX_MSG_AVAILABLE = 1U,
    CAN_RX_MSG_OVERFLOW = 8U
} CAN_RX_FIFO_STATUS;


struct CAN1_RX_FIFO
{
    CAN1_RX_FIFO_CHANNELS channel;
    volatile uint8_t fifoHead;
};

//CAN RX FIFO Message object data field 
static volatile uint8_t rxMsgData[RX_FIFO_MSG_DATA];

static struct CAN1_RX_FIFO rxFifos[] = 
{
    {FIFO1, 0u},
    {FIFO2, 0u}
};

static volatile struct CAN_FIFOREG * const FIFO = (struct CAN_FIFOREG *)&C1TXQCONL;
static const uint8_t DLC_BYTES[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

static void (*CAN1_FIFO1NotEmptyHandler)(void);
static void (*CAN1_FIFO2NotEmptyHandler)(void);

static void DefaultFIFO1NotEmptyHandler(void)
{
    CAN_MSG_OBJ EchoMessage;
    while(1)
    {
        if(CAN1_ReceivedMessageCountGet() > 0) 
        {
            if(true == CAN1_Receive(&EchoMessage)) //receive the message
            {
                break;
            }
        }
    }
    EchoMessage.msgId=0x222; //Change the ID to 0x222 then echo the message back out
if(CAN_TX_FIFO_AVAILABLE == (CAN1_TransmitFIFOStatusGet(TXQ) & CAN_TX_FIFO_AVAILABLE))
    {
        CAN1_Transmit(TXQ, &EchoMessage);
    }
}

static void DefaultFIFO2NotEmptyHandler(void)
{
    CAN_MSG_OBJ InternalMessage;
    uint8_t DataIndex=0;
    while(1)
    {
        if(CAN1_ReceivedMessageCountGet() > 0) 
        {
            if(true == CAN1_Receive(&InternalMessage)) //receive the message
            {
                break;
            }
        }
    }
    DataIndex=InternalMessage.data[0]/32;
    LATA=InternalMessage.data[DataIndex];
}

void CAN1_RX_FIFO_ResetInfo(void)
{
    uint8_t index;

    for (index = 0; index < NUM_OF_RX_FIFO; index++)
    {
        rxFifos[index].fifoHead = 0;
    }
}

static void CAN1_RX_FIFO_Configuration(void)
{
    // TXEN disabled; RTREN disabled; RXTSEN disabled; TXATIE disabled; RXOVIE disabled; TFERFFIE disabled; TFHRFHIE disabled; TFNRFNIE enabled; 
    C1FIFOCON1L = 0x01;
    
    // FRESET enabled; TXREQ disabled; UINC disabled; 
    C1FIFOCON1H = 0x04;
    
    // TXAT Unlimited number of retransmission attempts; TXPRI 1; 
    C1FIFOCON1U = 0x60;
    
    // PLSIZE 32; FSIZE 6; 
    C1FIFOCON1T = 0xA5;
    
    // TXEN disabled; RTREN disabled; RXTSEN disabled; TXATIE disabled; RXOVIE disabled; TFERFFIE disabled; TFHRFHIE disabled; TFNRFNIE enabled; 
    C1FIFOCON2L = 0x01;
    
    // FRESET enabled; TXREQ disabled; UINC disabled; 
    C1FIFOCON2H = 0x04;
    
    // TXAT Unlimited number of retransmission attempts; TXPRI 1; 
    C1FIFOCON2U = 0x60;
    
    // PLSIZE 32; FSIZE 6; 
    C1FIFOCON2T = 0xA5;
    
    CAN1_SetFIFO1NotEmptyHandler(DefaultFIFO1NotEmptyHandler);
    CAN1_SetFIFO2NotEmptyHandler(DefaultFIFO2NotEmptyHandler);
    
    C1INTUbits.RXIE = 1;
    
    PIR4bits.CANRXIF = 0;
    PIE4bits.CANRXIE = 1;
}

static void CAN1_RX_FIFO_FilterMaskConfiguration(void)
{
    // FLTEN0 enabled; F0BP FIFO 1; 
    C1FLTOBJ0L = 0x11;
    C1FLTOBJ0H = 0x01;
    C1FLTOBJ0U = 0x00;
    C1FLTOBJ0T = 0x00;
    C1MASK0L = 0xFF;
    C1MASK0H = 0x07;
    C1MASK0U = 0x00;
    C1MASK0T = 0x40;
    C1FLTCON0L = 0x81; 
    
    // FLTEN1 enabled; F1BP FIFO 2; 
    C1FLTOBJ1L = 0x85;
    C1FLTOBJ1H = 0xAD;
    C1FLTOBJ1U = 0x03;
    C1FLTOBJ1T = 0x00;
    C1MASK1L = 0xFF;
    C1MASK1H = 0x2F;
    C1MASK1U = 0x80;
    C1MASK1T = 0x53;
    C1FLTCON0H = 0x82; 
    
}

static void CAN1_TX_FIFO_Configuration(void)
{
    // TXATIE disabled; TXQEIE disabled; TXQNIE disabled; 
    C1TXQCONL = 0x00;
    
    // FRESET enabled; UINC disabled; 
    C1TXQCONH = 0x04;
    
    // TXAT 3; TXPRI 1; 
    C1TXQCONU = 0x60;
    
    // PLSIZE 32; FSIZE 6; 
    C1TXQCONT = 0xA5;
    
}

static void CAN1_BitRateConfiguration(void)
{
    // SJW 11; 
    C1NBTCFGL = 0x0B;
    
    // TSEG2 11; 
    C1NBTCFGH = 0x0B;
    
    // TSEG1 50; 
    C1NBTCFGU = 0x32;
    
    // BRP 0; 
    C1NBTCFGT = 0x00;
    
    // SJW 2; 
    C1DBTCFGL = 0x02;
    
    // TSEG2 2; 
    C1DBTCFGH = 0x02;
    
    // TSEG1 11; 
    C1DBTCFGU = 0x0B;
    
    // BRP 0; 
    C1DBTCFGT = 0x00;
    
    // TDCO 12; 
    C1TDCH = 0x0C;
    
    // TDCMOD Auto; 
    C1TDCU = 0x02;
}


void CAN1_Initialize(void)
{
    /* Enable the CAN module */
    C1CONHbits.ON = 1;
    
    if (CAN_OP_MODE_REQUEST_SUCCESS == CAN1_OperationModeSet(CAN_CONFIGURATION_MODE))
    {
        /* Initialize the C1FIFOBA with the start address of the CAN FIFO message object area. */
        C1FIFOBA = 0x3800;
        
        // CLKSEL0 disabled; PXEDIS enabled; ISOCRCEN enabled; DNCNT 0; 
        C1CONL = 0x60;

        // ON enabled; FRZ disabled; SIDL disabled; BRSDIS disabled; WFT T11 Filter; WAKFIL enabled; 
        C1CONH = 0x87;

        // TXQEN enabled; STEF disabled; SERR2LOM disabled; ESIGM disabled; RTXAT disabled; 
        C1CONU = 0x10;

        CAN1_BitRateConfiguration();
        CAN1_TX_FIFO_Configuration();
        CAN1_RX_FIFO_Configuration();
        CAN1_RX_FIFO_FilterMaskConfiguration();
        CAN1_RX_FIFO_ResetInfo();
        CAN1_OperationModeSet(CAN_NORMAL_FD_MODE);    
    }
}

CAN_OP_MODE_STATUS CAN1_OperationModeSet(const CAN_OP_MODES requestMode)
{
    CAN_OP_MODE_STATUS status = CAN_OP_MODE_REQUEST_SUCCESS;
    CAN_OP_MODES opMode = CAN1_OperationModeGet();

    if (CAN_CONFIGURATION_MODE == opMode
            || CAN_DISABLE_MODE == requestMode
            || CAN_CONFIGURATION_MODE == requestMode)
    {
        C1CONTbits.REQOP = requestMode;
        
        while (C1CONUbits.OPMOD != requestMode)
        {
            //This condition is avoiding the system error case endless loop
            if (1 == C1INTHbits.SERRIF)
            {
                status = CAN_OP_MODE_SYS_ERROR_OCCURED;
                break;
            }
        }
    }
    else
    {
        status = CAN_OP_MODE_REQUEST_FAIL;
    }

    return status;
}

CAN_OP_MODES CAN1_OperationModeGet(void)
{
    return C1CONUbits.OPMOD;
}

static uint8_t GetRxFifoDepth(uint8_t validChannel)
{
    return 1U + (FIFO[validChannel].CONT & _C1FIFOCON1T_FSIZE_MASK);
}

static CAN_RX_FIFO_STATUS GetRxFifoStatus(uint8_t validChannel)
{
    return FIFO[validChannel].STAL & (CAN_RX_MSG_AVAILABLE | CAN_RX_MSG_OVERFLOW);
}

static void ReadMessageFromFifo(uint8_t *rxFifoObj, CAN_MSG_OBJ *rxCanMsg)
{
    uint32_t msgId;
    uint8_t status = rxFifoObj[4];
    const uint8_t payloadOffsetBytes =
              4U    // ID
            + 1U    // FDF, BRS, RTR, ...
            + 1U    // FILHIT, ...
            + 2U;   // Unimplemented
    
    rxCanMsg->field.dlc = status;
    rxCanMsg->field.idType = (status & (1UL << IDE_POSN)) ? CAN_FRAME_EXT : CAN_FRAME_STD;
    rxCanMsg->field.frameType = (status & (1UL << RTR_POSN)) ? CAN_FRAME_RTR : CAN_FRAME_DATA;
    rxCanMsg->field.brs = (status & (1UL << BRS_POSN)) ? CAN_BRS_MODE : CAN_NON_BRS_MODE;
    rxCanMsg->field.formatType = (status & (1UL << FDF_POSN)) ? CAN_FRAME_EXT : CAN_FRAME_STD;
       
    msgId = rxFifoObj[1] & SID_HIGH_MASK;
    msgId <<= SID_LOW_WIDTH;
    msgId |= rxFifoObj[0];
    if (CAN_FRAME_EXT == rxCanMsg->field.idType)
    {
        msgId <<= EID_HIGH_WIDTH;
        msgId |= (rxFifoObj[3] & EID_HIGH_MASK);
        msgId <<= EID_MID_WIDTH;
        msgId |= rxFifoObj[2];
        msgId <<= EID_LOW_WIDTH;
        msgId |= (rxFifoObj[1] & EID_LOW_MASK) >> EID_LOW_POSN;
    }
    rxCanMsg->msgId = msgId;
    
    memcpy(rxMsgData, rxFifoObj + payloadOffsetBytes, DLCToPayloadBytes(rxCanMsg->field.dlc));
    rxCanMsg->data = rxMsgData;
}

static bool Receive(uint8_t index, CAN1_RX_FIFO_CHANNELS channel, CAN_MSG_OBJ *rxCanMsg)
{
    bool status = false;
    CAN_RX_FIFO_STATUS rxMsgStatus = GetRxFifoStatus(channel);

    if (CAN_RX_MSG_AVAILABLE == (rxMsgStatus & CAN_RX_MSG_AVAILABLE))
    {
        uint8_t *rxFifoObj = (uint8_t *) FIFO[channel].UA;

        if (rxFifoObj != NULL)
        {
            ReadMessageFromFifo(rxFifoObj, rxCanMsg);
            FIFO[channel].CONH |= _C1FIFOCON1H_UINC_MASK;

            rxFifos[index].fifoHead += 1;
            if (rxFifos[index].fifoHead >= GetRxFifoDepth(channel))
            {
                rxFifos[index].fifoHead = 0;
            }

            if (CAN_RX_MSG_OVERFLOW == (rxMsgStatus & CAN_RX_MSG_OVERFLOW))
            {
                FIFO[channel].STAL &= ~_C1FIFOSTA1L_RXOVIF_MASK;
            }

            status = true;
        }
    }
    
    return status;
}

bool CAN1_Receive(CAN_MSG_OBJ *rxCanMsg)
{
    uint8_t index;
    bool status = false;
    
    for (index = 0; index < NUM_OF_RX_FIFO; index++)
    {
        status = Receive(index, rxFifos[index].channel, rxCanMsg);
        
        if (status)
        {
            break;
        }
    }
    
    return status;
}

bool CAN1_ReceiveFrom(const CAN1_RX_FIFO_CHANNELS channel, CAN_MSG_OBJ *rxCanMsg)
{
    uint8_t index;
    bool status = false;
    
    for (index = 0; index < NUM_OF_RX_FIFO; index++)
    {
        if (channel == rxFifos[index].channel)
        {
            status = Receive(index, channel, rxCanMsg);
            break;
        }
    }
    
    return status;
}

uint8_t CAN1_ReceivedMessageCountGet(void)
{
    uint8_t index, totalMsgObj = 0;

    for (index = 0; index < NUM_OF_RX_FIFO; index++)
    {
        CAN1_RX_FIFO_CHANNELS channel = rxFifos[index].channel;
        CAN_RX_FIFO_STATUS rxMsgStatus = GetRxFifoStatus(channel);

        if (CAN_RX_MSG_AVAILABLE == (rxMsgStatus & CAN_RX_MSG_AVAILABLE))
        {
            uint8_t numOfMsg, fifoDepth = GetRxFifoDepth(channel);
            
            if (CAN_RX_MSG_OVERFLOW == (rxMsgStatus & CAN_RX_MSG_OVERFLOW))
            {
                numOfMsg = fifoDepth;
            }
            else
            {
                uint8_t fifoTail = FIFO[channel].STAH & _C1FIFOSTA1H_FIFOCI_MASK;
                uint8_t fifoHead = rxFifos[index].fifoHead;

                if (fifoTail < fifoHead)
                {
                    numOfMsg = ((fifoTail + fifoDepth) - fifoHead); // wrap
                }
                else if (fifoTail > fifoHead)
                {
                    numOfMsg = fifoTail - fifoHead;
                }
                else
                {
                    numOfMsg = fifoDepth;
                }
            }

            totalMsgObj += numOfMsg;
        }
    }

    return totalMsgObj;
}

static bool isTxChannel(uint8_t channel) 
{
    return channel < 4u && (FIFO[channel].CONL & _C1FIFOCON1L_TXEN_MASK);
}

static CAN_TX_FIFO_STATUS GetTxFifoStatus(uint8_t validChannel)
{
    return (FIFO[validChannel].STAL & _C1FIFOSTA1L_TFNRFNIF_MASK);
}

static void WriteMessageToFifo(uint8_t *txFifoObj, CAN_MSG_OBJ *txCanMsg)
{
    uint32_t msgId = txCanMsg->msgId;
    uint8_t status;
    const uint8_t payloadOffsetBytes =
              4U    // ID
            + 1U    // FDF, BRS, RTR, ...
            + 1U    // SEQ[6:0], ESI
            + 2U;   // SEQ
    
    if (CAN_FRAME_EXT == txCanMsg->field.idType)
    {
        txFifoObj[1] = (msgId << EID_LOW_POSN) & EID_LOW_MASK;
        msgId >>= EID_LOW_WIDTH;
        txFifoObj[2] = msgId;
        msgId >>= EID_MID_WIDTH;
        txFifoObj[3] = (msgId & EID_HIGH_MASK);
        msgId >>= EID_HIGH_WIDTH;
    }
    else
    {
        txFifoObj[1] = txFifoObj[2] = txFifoObj[3] = 0;
    }
    
    txFifoObj[0] = msgId;
    msgId >>= SID_LOW_WIDTH;
    txFifoObj[1] |= (msgId & SID_HIGH_MASK);
    
    status = txCanMsg->field.dlc;
    status |= (txCanMsg->field.idType << IDE_POSN);
    status |= (txCanMsg->field.frameType << RTR_POSN);
    status |= (txCanMsg->field.brs << BRS_POSN);
    status |= (txCanMsg->field.formatType << FDF_POSN);
    txFifoObj[4] = status;

    if (CAN_FRAME_DATA == txCanMsg->field.frameType)
    {
        memcpy(txFifoObj + payloadOffsetBytes, txCanMsg->data, DLCToPayloadBytes(txCanMsg->field.dlc));
    }
}

static CAN_TX_MSG_REQUEST_STATUS ValidateTransmission(uint8_t validChannel, CAN_MSG_OBJ *txCanMsg)
{
    CAN_TX_MSG_REQUEST_STATUS txMsgStatus = CAN_TX_MSG_REQUEST_SUCCESS;
    CAN_MSG_FIELD field = txCanMsg->field;
    uint8_t plsize = (FIFO[validChannel].CONT & _C1FIFOCON1T_PLSIZE_MASK) >> _C1FIFOCON1T_PLSIZE_POSN;
    
    if (CAN_BRS_MODE == field.brs && (1 == C1CONHbits.BRSDIS || CAN_NORMAL_2_0_MODE == CAN1_OperationModeGet()))
    {
        txMsgStatus |= CAN_TX_MSG_REQUEST_BRS_ERROR;
    }
    
    if (field.dlc > DLC_8 && (CAN_2_0_FORMAT == field.formatType || CAN_NORMAL_2_0_MODE == CAN1_OperationModeGet()))
    {
        txMsgStatus |= CAN_TX_MSG_REQUEST_DLC_EXCEED_ERROR;
    }
    
    if (DLCToPayloadBytes(field.dlc) > PLSIZEToPayloadBytes(plsize))
    {
        txMsgStatus |= CAN_TX_MSG_REQUEST_DLC_EXCEED_ERROR;
    }
    
    if (CAN_TX_FIFO_FULL == GetTxFifoStatus(validChannel))
    {
        txMsgStatus |= CAN_TX_MSG_REQUEST_FIFO_FULL;
    }
    
    return txMsgStatus;
}

CAN_TX_MSG_REQUEST_STATUS CAN1_Transmit(const CAN1_TX_FIFO_CHANNELS fifoChannel, CAN_MSG_OBJ *txCanMsg)
{
    CAN_TX_MSG_REQUEST_STATUS status = CAN_TX_MSG_REQUEST_FIFO_FULL;
    
    if (isTxChannel(fifoChannel))
    {
        status = ValidateTransmission(fifoChannel, txCanMsg);
        if (CAN_TX_MSG_REQUEST_SUCCESS == status)
        {
            uint8_t *txFifoObj = (uint8_t *) FIFO[fifoChannel].UA;
            
            if (txFifoObj != NULL)
            {
                WriteMessageToFifo(txFifoObj, txCanMsg);
                FIFO[fifoChannel].CONH |= (_C1FIFOCON1H_TXREQ_MASK | _C1FIFOCON1H_UINC_MASK);
            }
        }
    }
    
    return status;
}

CAN_TX_FIFO_STATUS CAN1_TransmitFIFOStatusGet(const CAN1_TX_FIFO_CHANNELS fifoChannel)
{
    CAN_TX_FIFO_STATUS status = CAN_TX_FIFO_FULL;
    
    if (isTxChannel(fifoChannel)) 
    {
        status = GetTxFifoStatus(fifoChannel);
    }
    
    return status;
}

bool CAN1_IsBusOff(void)
{
    return C1TRECUbits.TXBO;
}

bool CAN1_IsRxErrorPassive(void)
{
    return C1TRECUbits.RXBP;
}

bool CAN1_IsRxErrorWarning(void)
{
    return C1TRECUbits.RXWARN;
}

bool CAN1_IsRxErrorActive(void)
{
    return !CAN1_IsRxErrorPassive();
}

bool CAN1_IsTxErrorPassive(void)
{
    return C1TRECUbits.TXBP;
}

bool CAN1_IsTxErrorWarning(void)
{
    return C1TRECUbits.TXWARN;
}

bool CAN1_IsTxErrorActive(void)
{
    return !CAN1_IsTxErrorPassive();
}

void CAN1_Sleep(void)
{
    C1INTHbits.WAKIF = 0;
    C1INTTbits.WAKIE = 1;
    
    CAN1_OperationModeSet(CAN_DISABLE_MODE);
}


void CAN1_SetFIFO1NotEmptyHandler(void (*handler)(void))
{
    CAN1_FIFO1NotEmptyHandler = handler;
}

void CAN1_SetFIFO2NotEmptyHandler(void (*handler)(void))
{
    CAN1_FIFO2NotEmptyHandler = handler;
}


void CAN1_RXI_ISR(void)
{
    if (1 == C1FIFOSTA1Lbits.TFNRFNIF)
    {
        CAN1_FIFO1NotEmptyHandler();
        // flag readonly
    }
    
    if (1 == C1FIFOSTA2Lbits.TFNRFNIF)
    {
        CAN1_FIFO2NotEmptyHandler();
        // flag readonly
    }
    
}

