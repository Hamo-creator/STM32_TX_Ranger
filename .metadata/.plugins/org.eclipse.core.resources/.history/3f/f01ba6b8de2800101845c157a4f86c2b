/*
 * crsf.h
 *
 *  Created on: Apr 5, 2025
 *      Author: Khalil
 */

#ifndef CRSF_H
#define CRSF_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

extern UART_HandleTypeDef huart1;

// Protocol configuration
#define CRSF_MAX_CHANNEL              16
#define CRSF_FRAME_SIZE_MAX           64
#define RADIO_ADDRESS                 0xEA
#define TYPE_CHANNELS                 0x16

#define CRSF_DIGITAL_CHANNEL_MIN      172
#define CRSF_DIGITAL_CHANNEL_MAX      1811

#define CRSF_TIME_NEEDED_PER_FRAME_US 1100
#define SERIAL_BAUDRATE               115200
#define CRSF_TIME_BETWEEN_FRAMES_US   4000

#define CRSF_PAYLOAD_OFFSET           offsetof(crsfFrameDef_t, type)
#define CRSF_MSP_RX_BUF_SIZE          128
#define CRSF_MSP_TX_BUF_SIZE          128
#define CRSF_PAYLOAD_SIZE_MAX         60
#define CRSF_PACKET_LENGTH            22
#define CRSF_PACKET_SIZE              26
#define CRSF_FRAME_LENGTH             24
#define CRSF_CMD_PACKET_SIZE          8

// ELRS command constants
#define ELRS_ADDRESS                  0xEE
#define ELRS_PKT_RATE_COMMAND         0x01
#define ELRS_TLM_RATIO_COMMAND        0x02
#define ELRS_SWITCH_MODE_COMMAND      0x03
#define ELRS_MODEL_MATCH_COMMAND      0x04
#define ELRS_POWER_COMMAND            0x06
#define ELRS_DYNAMIC_POWER_COMMAND    0x07
#define ELRS_WIFI_COMMAND             0x0F
#define ELRS_BIND_COMMAND             0x11
#define ELRS_START_COMMAND            0x04
#define TYPE_SETTINGS_WRITE           0x2D
#define ADDR_RADIO                    0xEA

typedef enum {
    STATE_STARTUP,
    STATE_SEND_PKT_RATE,
    STATE_SEND_POWER,
    STATE_SEND_DYNAMIC,
    STATE_NORMAL
} CRSF_State;

// === Function declarations (formerly class methods) ===

// Initialize CRSF protocol (e.g., UART or variables)
void CRSF_Begin(void);

// Prepares a CRSF data packet from channel values
void CRSF_PrepareDataPacket(uint8_t packet[], int16_t channels[]);

// Prepares a CRSF command packet
void CRSF_PrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value);

// Writes a CRSF packet over UART
void CRSF_WritePacket(uint8_t packet[], uint8_t packetLength);

#endif // CRSF_PROTOCOL_H

