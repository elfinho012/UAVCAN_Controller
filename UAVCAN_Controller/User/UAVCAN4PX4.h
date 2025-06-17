/********************************** (C) COPYRIGHT *******************************
 * File Name          : UAVCAN4PX4.h
 * Author             : Your Name
 * Version            : V1.0.0
 * Date               : Current Date
 * Description        : UAVCAN implementation for PX4 flight controller using CH32V20x
 *                      Supports latest UAVCAN protocol version compatible with PX4
*********************************************************************************
 * Copyright (c) 2023 Your Company
 * Attention: This software is designed for CH32V20x microcontrollers
*******************************************************************************/

#ifndef __UAVCAN4PX4_H
#define __UAVCAN4PX4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32v20x.h"
#include "ch32v20x_can.h"
#include "ch32v20x_crc.h"
#include <stdint.h>
#include <stdbool.h>

// UAVCAN constants and definitions
#define UAVCAN_MAX_TRANSFER_SIZE          128
#define UAVCAN_MTU_CAN_CLASSIC            8
#define UAVCAN_PROTOCOL_VERSION           1.0
#define UAVCAN_NODE_ID_MAX                127

// UAVCAN priority levels
typedef enum {
    UAVCAN_PRIORITY_HIGHEST = 0,
    UAVCAN_PRIORITY_HIGH,
    UAVCAN_PRIORITY_NORMAL,
    UAVCAN_PRIORITY_LOW,
    UAVCAN_PRIORITY_LOWEST
} UAVCAN_Priority_t;

// UAVCAN transfer types
typedef enum {
    UAVCAN_TRANSFER_TYPE_BROADCAST,
    UAVCAN_TRANSFER_TYPE_REQUEST,
    UAVCAN_TRANSFER_TYPE_RESPONSE,
    UAVCAN_TRANSFER_TYPE_SERVICE
} UAVCAN_TransferType_t;

// UAVCAN message types
typedef enum {
    UAVCAN_MSG_TYPE_HEARTBEAT = 0,
    UAVCAN_MSG_TYPE_GET_NODE_INFO = 1,
    UAVCAN_MSG_TYPE_RESTART_NODE = 2,
    UAVCAN_MSG_TYPE_ESC_COMMAND = 3,
    UAVCAN_MSG_TYPE_ESC_STATUS = 4,
    UAVCAN_MSG_TYPE_GNSS_FIX = 5,
    UAVCAN_MSG_TYPE_BATTERY_INFO = 6,
    UAVCAN_MSG_TYPE_SERVO_COMMAND = 7,
    UAVCAN_MSG_TYPE_SERVO_STATUS = 8
} UAVCAN_MessageType_t;

// UAVCAN node modes
typedef enum {
    UAVCAN_NODE_MODE_OPERATIONAL = 0,
    UAVCAN_NODE_MODE_INITIALIZATION,
    UAVCAN_NODE_MODE_MAINTENANCE,
    UAVCAN_NODE_MODE_SOFTWARE_UPDATE,
    UAVCAN_NODE_MODE_OFFLINE
} UAVCAN_NodeMode_t;

// UAVCAN node health status
typedef enum {
    UAVCAN_NODE_HEALTH_OK = 0,
    UAVCAN_NODE_HEALTH_WARNING,
    UAVCAN_NODE_HEALTH_ERROR,
    UAVCAN_NODE_HEALTH_CRITICAL
} UAVCAN_NodeHealth_t;

// UAVCAN transfer descriptor
typedef struct {
    uint32_t timestamp_us;
    uint8_t priority;
    uint8_t transfer_type;
    uint8_t source_node_id;
    uint8_t destination_node_id;
    uint16_t data_type_id;
    uint16_t transfer_id;
    uint8_t payload[UAVCAN_MAX_TRANSFER_SIZE];
    uint16_t payload_size;
} UAVCAN_Transfer_t;

// UAVCAN node information
typedef struct {
    uint8_t node_id;
    uint8_t mode;
    uint8_t health;
    uint32_t uptime_sec;
    char name[50];
    uint8_t software_version[4];
    uint8_t hardware_version[4];
    uint64_t unique_id;
} UAVCAN_NodeInfo_t;

// UAVCAN ESC command
typedef struct {
    uint16_t command[8];  // RPM or PWM values for up to 8 ESCs
    uint8_t count;        // Number of active ESCs
} UAVCAN_ESC_Command_t;

// UAVCAN ESC status
typedef struct {
    uint32_t error_count;
    float voltage;
    float current;
    float temperature;
    int32_t rpm;
    uint8_t power_rating_pct;
    uint8_t esc_index;
} UAVCAN_ESC_Status_t;

// UAVCAN GNSS fix
typedef struct {
    double latitude;
    double longitude;
    float altitude;
    float velocity[3];
    float position_covariance[9];
    uint8_t sats_used;
    uint8_t fix_type;
    uint64_t timestamp_usec;
} UAVCAN_GNSS_Fix_t;

// UAVCAN battery info
typedef struct {
    float voltage;
    float current;
    float temperature;
    uint16_t capacity_wh;
    uint8_t state_of_charge_pct;
    uint16_t remaining_capacity_wh;
} UAVCAN_BatteryInfo_t;

// Callback function types
typedef void (*UAVCAN_MessageHandler_t)(const UAVCAN_Transfer_t* transfer);
typedef void (*UAVCAN_ErrorHandler_t)(uint32_t error_code);

// Function prototypes
void UAVCAN_Init(uint8_t node_id, uint32_t can_bitrate);
void UAVCAN_Process(void);
bool UAVCAN_SendHeartbeat(UAVCAN_NodeMode_t mode, UAVCAN_NodeHealth_t health);
bool UAVCAN_SendESCCommand(const UAVCAN_ESC_Command_t* command);
bool UAVCAN_SendNodeInfo(const UAVCAN_NodeInfo_t* node_info);
bool UAVCAN_SendTransfer(const UAVCAN_Transfer_t* transfer);
void UAVCAN_RegisterMessageHandler(UAVCAN_MessageHandler_t handler);
void UAVCAN_RegisterErrorHandler(UAVCAN_ErrorHandler_t handler);

// Helper functions
uint16_t UAVCAN_CRC16(const uint8_t* data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __UAVCAN4PX4_H */
