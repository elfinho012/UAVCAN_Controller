/********************************** (C) COPYRIGHT *******************************
 * File Name          : UAVCAN4PX4.c
 * Author             : Your Name
 * Version            : V1.0.0
 * Date               : Current Date
 * Description        : UAVCAN implementation for PX4 flight controller using CH32V20x
*********************************************************************************
 * Copyright (c) 2023 Your Company
 * Attention: This software is designed for CH32V20x microcontrollers
*******************************************************************************/

#include "UAVCAN4PX4.h"
#include "ch32v20x_can.h"
#include "ch32v20x_rcc.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_crc.h"
#include <string.h>

// UAVCAN error codes
typedef enum {
    UAVCAN_ERROR_INVALID_NODE_ID = 1,
    UAVCAN_ERROR_CAN_INIT_FAILED,
    UAVCAN_ERROR_CRC_FAILED,
    UAVCAN_ERROR_INVALID_TRANSFER
} UAVCAN_ErrorCodes_t;

// Static variables
static uint8_t uavcan_node_id = UAVCAN_NODE_ID_MAX; // Invalid ID by default
static uint32_t uavcan_transfer_id = 0;
static UAVCAN_MessageHandler_t message_handler = NULL;
static UAVCAN_ErrorHandler_t error_handler = NULL;

// Local function prototypes
static void CAN_GPIO_Config(void);
static void CAN_Mode_Config(uint32_t baudrate);
static bool CAN_SendFrame(uint32_t id, const uint8_t* data, uint8_t len);
static bool CAN_ReceiveFrame(uint32_t* id, uint8_t* data, uint8_t* len);
static void UAVCAN_ParseTransfer(const CanRxMsg* frame, UAVCAN_Transfer_t* transfer);
static void UAVCAN_ComposeTransfer(const UAVCAN_Transfer_t* transfer, CanTxMsg* frame);
static bool UAVCAN_ProcessReceivedTransfer(const UAVCAN_Transfer_t* transfer);
static uint32_t UAVCAN_GetTimeMicroseconds(void);

/*********************************************************************
 * @fn      UAVCAN_Init
 *
 * @brief   Initialize UAVCAN protocol stack and CAN interface
 *********************************************************************/
void UAVCAN_Init(uint8_t node_id, uint32_t can_bitrate)
{
    // Validate node ID
    if(node_id > UAVCAN_NODE_ID_MAX)
    {
        if(error_handler) error_handler(UAVCAN_ERROR_INVALID_NODE_ID);
        return;
    }

    uavcan_node_id = node_id;
    uavcan_transfer_id = 0;

    // Initialize CAN interface
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    CAN_GPIO_Config();
    CAN_Mode_Config(can_bitrate);

    // Initialize CRC module for UAVCAN CRC checks
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
    CRC_ResetDR();
}

/*********************************************************************
 * @fn      UAVCAN_Process
 *
 * @brief   Process incoming UAVCAN messages
 *********************************************************************/
void UAVCAN_Process(void)
{
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    UAVCAN_Transfer_t transfer;

    while(CAN_ReceiveFrame(&id, data, &len))
    {
        CanRxMsg frame;
        frame.StdId = id;
        frame.DLC = len;
        memcpy(frame.Data, data, len);

        UAVCAN_ParseTransfer(&frame, &transfer);

        if(message_handler)
        {
            message_handler(&transfer);
        }

        UAVCAN_ProcessReceivedTransfer(&transfer);
    }
}

/*********************************************************************
 * @fn      UAVCAN_SendHeartbeat
 *
 * @brief   Send UAVCAN heartbeat message
 *********************************************************************/
bool UAVCAN_SendHeartbeat(UAVCAN_NodeMode_t mode, UAVCAN_NodeHealth_t health)
{
    UAVCAN_Transfer_t transfer = {
        .timestamp_us = UAVCAN_GetTimeMicroseconds(),
        .priority = UAVCAN_PRIORITY_NORMAL,
        .transfer_type = UAVCAN_TRANSFER_TYPE_BROADCAST,
        .source_node_id = uavcan_node_id,
        .destination_node_id = UAVCAN_NODE_ID_MAX,
        .data_type_id = UAVCAN_MSG_TYPE_HEARTBEAT,
        .transfer_id = uavcan_transfer_id++,
        .payload_size = 7
    };

    // Heartbeat payload: uptime (4 bytes), mode (1 byte), health (1 byte), vendor-specific (1 byte)
    uint32_t uptime_sec = transfer.timestamp_us / 1000000;
    transfer.payload[0] = (uptime_sec >> 0) & 0xFF;
    transfer.payload[1] = (uptime_sec >> 8) & 0xFF;
    transfer.payload[2] = (uptime_sec >> 16) & 0xFF;
    transfer.payload[3] = (uptime_sec >> 24) & 0xFF;
    transfer.payload[4] = mode;
    transfer.payload[5] = health;
    transfer.payload[6] = 0; // Vendor-specific status

    return UAVCAN_SendTransfer(&transfer);
}

/*********************************************************************
 * @fn      UAVCAN_SendESCCommand
 *
 * @brief   Send ESC command message
 *********************************************************************/
bool UAVCAN_SendESCCommand(const UAVCAN_ESC_Command_t* command)
{
    UAVCAN_Transfer_t transfer = {
        .timestamp_us = UAVCAN_GetTimeMicroseconds(),
        .priority = UAVCAN_PRIORITY_HIGH,
        .transfer_type = UAVCAN_TRANSFER_TYPE_BROADCAST,
        .source_node_id = uavcan_node_id,
        .destination_node_id = UAVCAN_NODE_ID_MAX,
        .data_type_id = UAVCAN_MSG_TYPE_ESC_COMMAND,
        .transfer_id = uavcan_transfer_id++,
        .payload_size = 2 + command->count * 2
    };

    // ESC command payload: command count (1 byte), reserved (1 byte), commands (2 bytes each)
    transfer.payload[0] = command->count;
    transfer.payload[1] = 0; // Reserved

    for(uint8_t i = 0; i < command->count; i++)
    {
        transfer.payload[2 + i*2] = (command->command[i] >> 0) & 0xFF;
        transfer.payload[3 + i*2] = (command->command[i] >> 8) & 0xFF;
    }

    return UAVCAN_SendTransfer(&transfer);
}

/*********************************************************************
 * @fn      UAVCAN_SendNodeInfo
 *
 * @brief   Send node information message
 *********************************************************************/
bool UAVCAN_SendNodeInfo(const UAVCAN_NodeInfo_t* node_info)
{
    UAVCAN_Transfer_t transfer = {
        .timestamp_us = UAVCAN_GetTimeMicroseconds(),
        .priority = UAVCAN_PRIORITY_NORMAL,
        .transfer_type = UAVCAN_TRANSFER_TYPE_RESPONSE,
        .source_node_id = uavcan_node_id,
        .destination_node_id = node_info->node_id,
        .data_type_id = UAVCAN_MSG_TYPE_GET_NODE_INFO,
        .transfer_id = uavcan_transfer_id++,
        .payload_size = sizeof(UAVCAN_NodeInfo_t)
    };

    memcpy(transfer.payload, node_info, sizeof(UAVCAN_NodeInfo_t));
    return UAVCAN_SendTransfer(&transfer);
}

/*********************************************************************
 * @fn      UAVCAN_SendTransfer
 *
 * @brief   Send UAVCAN transfer
 *********************************************************************/
bool UAVCAN_SendTransfer(const UAVCAN_Transfer_t* transfer)
{
    CanTxMsg frame;
    UAVCAN_ComposeTransfer(transfer, &frame);
    return CAN_SendFrame(frame.StdId, frame.Data, frame.DLC);
}

/*********************************************************************
 * @fn      UAVCAN_RegisterMessageHandler
 *
 * @brief   Register message handler callback
 *********************************************************************/
void UAVCAN_RegisterMessageHandler(UAVCAN_MessageHandler_t handler)
{
    message_handler = handler;
}

/*********************************************************************
 * @fn      UAVCAN_RegisterErrorHandler
 *
 * @brief   Register error handler callback
 *********************************************************************/
void UAVCAN_RegisterErrorHandler(UAVCAN_ErrorHandler_t handler)
{
    error_handler = handler;
}

/*********************************************************************
 * @fn      UAVCAN_CRC16
 *
 * @brief   Calculate UAVCAN CRC16
 *********************************************************************/
uint16_t UAVCAN_CRC16(const uint8_t* data, uint16_t length)
{
    CRC_ResetDR();

    // Use the library function to calculate CRC by each byte
    for(uint16_t i = 0; i < length; i++)
    {
        CRC_CalcCRC8bits(data[i]);
    }

    return CRC_GetCRC();
}

/*********************************************************************
 * @fn      UAVCAN_ProcessReceivedTransfer
 *
 * @brief   Process received UAVCAN transfer
 *********************************************************************/
static bool UAVCAN_ProcessReceivedTransfer(const UAVCAN_Transfer_t* transfer)
{
    // Placeholder: Add your message processing logic here
    return true;
}

/*********************************************************************
 * @fn      UAVCAN_GetTimeMicroseconds
 *
 * @brief   Get current time in microseconds
 *********************************************************************/
static uint32_t UAVCAN_GetTimeMicroseconds(void)
{
    // This should be replaced with a real timer implementation
    static uint32_t dummy_time = 0;
    return dummy_time++; // Increment for testing
}

/*********************************************************************
 * @fn      CAN_GPIO_Config
 *
 * @brief   Configure CAN GPIO pins
 *********************************************************************/
static void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Enable GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure CAN RX pin (PA11)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure CAN TX pin (PA12)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      CAN_Mode_Config
 *
 * @brief   Configure CAN interface mode and bitrate
 *********************************************************************/
static void CAN_Mode_Config(uint32_t baudrate)
{
    CAN_InitTypeDef CAN_InitStructure = {0};

    // CAN register init
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    // CAN cell init
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

    // Calculate timing parameters based on baudrate
    if(baudrate == 1000000)
    {
        CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
        CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
        CAN_InitStructure.CAN_Prescaler = 2;
    }
    else if(baudrate == 500000)
    {
        CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
        CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
        CAN_InitStructure.CAN_Prescaler = 4;
    }
    else // Default to 250000
    {
        CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
        CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
        CAN_InitStructure.CAN_Prescaler = 8;
    }

    CAN_Init(CAN1, &CAN_InitStructure);

    // Configure filter to accept all messages
    CAN_FilterInitTypeDef filter;
    filter.CAN_FilterNumber = 0;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = 0x0000;
    filter.CAN_FilterIdLow = 0x0000;
    filter.CAN_FilterMaskIdHigh = 0x0000;
    filter.CAN_FilterMaskIdLow = 0x0000;
    filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);
}

/*********************************************************************
 * @fn      CAN_SendFrame
 *
 * @brief   Send CAN frame
 *********************************************************************/
static bool CAN_SendFrame(uint32_t id, const uint8_t* data, uint8_t len)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = id;
    TxMessage.ExtId = 0;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = len;

    for(uint8_t i = 0; i < len; i++)
    {
        TxMessage.Data[i] = data[i];
    }

    uint8_t mailbox = CAN_Transmit(CAN1, &TxMessage);
    if(mailbox == CAN_TxStatus_NoMailBox)
    {
        return false;
    }

    // Wait for transmission to complete
    while(CAN_TransmitStatus(CAN1, mailbox) != CAN_TxStatus_Ok);
    return true;
}

/*********************************************************************
 * @fn      CAN_ReceiveFrame
 *
 * @brief   Receive CAN frame
 *********************************************************************/
static bool CAN_ReceiveFrame(uint32_t* id, uint8_t* data, uint8_t* len)
{
    if(CAN_MessagePending(CAN1, CAN_FIFO0) == 0)
    {
        return false;
    }

    CanRxMsg RxMessage;
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

    *id = RxMessage.StdId;
    *len = RxMessage.DLC;
    for(uint8_t i = 0; i < RxMessage.DLC; i++)
    {
        data[i] = RxMessage.Data[i];
    }

    return true;
}

/*********************************************************************
 * @fn      UAVCAN_ParseTransfer
 *
 * @brief   Parse CAN frame into UAVCAN transfer
 *********************************************************************/
static void UAVCAN_ParseTransfer(const CanRxMsg* frame, UAVCAN_Transfer_t* transfer)
{
    // Parse CAN ID into UAVCAN fields
    transfer->priority = (frame->StdId >> 26) & 0x7;
    transfer->data_type_id = (frame->StdId >> 8) & 0xFFFF;
    transfer->source_node_id = frame->StdId & 0x7F;

    // Default values
    transfer->destination_node_id = UAVCAN_NODE_ID_MAX;
    transfer->transfer_type = UAVCAN_TRANSFER_TYPE_BROADCAST;
    transfer->transfer_id = 0;

    // Copy payload
    transfer->payload_size = frame->DLC;
    memcpy(transfer->payload, frame->Data, frame->DLC);

    // Get timestamp
    transfer->timestamp_us = UAVCAN_GetTimeMicroseconds();
}

/*********************************************************************
 * @fn      UAVCAN_ComposeTransfer
 *
 * @brief   Compose UAVCAN transfer into CAN frame
 *********************************************************************/
static void UAVCAN_ComposeTransfer(const UAVCAN_Transfer_t* transfer, CanTxMsg* frame)
{
    // Compose CAN ID from UAVCAN fields
    frame->StdId = (transfer->priority << 26) |
                  (transfer->data_type_id << 8) |
                  (transfer->source_node_id);

    // Set frame data
    frame->DLC = transfer->payload_size;
    memcpy(frame->Data, transfer->payload, transfer->payload_size);
}
