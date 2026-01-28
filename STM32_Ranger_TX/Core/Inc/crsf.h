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
#include <stdbool.h>
#include <stddef.h>

extern UART_HandleTypeDef huart1;

#define UART_RX_BUFFER_SIZE 	128
extern uint8_t uartRxBuf[UART_RX_BUFFER_SIZE];

extern volatile uint8_t crsf_tx_busy;

extern volatile uint8_t g_last_crsf_packet_type;

#define CRSF_PACKET_TIMEOUT_MS     100
#define CRSF_FAILSAFE_STAGE1_MS    300
// Protocol configuration
#define CRSF_MAX_PACKET_SIZE  		  64
#define CRSF_NUM_CHANNELS		      16
#define CRSF_MAX_CHANNEL              16
#define CRSF_FRAME_SIZE_MAX           64
#define RADIO_ADDRESS                 0xEA
#define TYPE_CHANNELS                 0x16

#define CRSF_DIGITAL_CHANNEL_MIN      172
#define CRSF_DIGITAL_CHANNEL_MID	  992
#define CRSF_DIGITAL_CHANNEL_MAX      1811

#define CRSF_TIME_NEEDED_PER_FRAME_US 1100
#define SERIAL_BAUDRATE               420000
#define CRSF_TIME_BETWEEN_FRAMES_US   4000

#define CRSF_PAYLOAD_OFFSET           offsetof(crsfFrameDef_t, type)
#define CRSF_MSP_RX_BUF_SIZE          128
#define CRSF_MSP_TX_BUF_SIZE          128
#define CRSF_PAYLOAD_SIZE_MAX         60
#define CRSF_PACKET_LENGTH            22
#define CRSF_PACKET_SIZE              26
#define CRSF_FRAME_LENGTH             24
#define CRSF_CMD_PACKET_SIZE          8
#define CRSF_FRAMETYPE_CMD			  0x28
#define CRSF_FRAMETYPE_SET_RC_CHANNELS_PACKED       0x2B

#define CRSF_FRAMETYPE_BATTERY_SENSOR	0x08
#define CRSF_FRAMETYPE_LINK_STATISTICS	0x14
#define CRSF_FRAMETYPE_GPS				0x02
#define CRSF_FRAMETYPE_FLIGHT_MODE		0x21
#define CRSF_FRAMETYPE_ATTITUDE			0x1E

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
#define ADDR_FC						  0xC8

typedef struct {
    UART_HandleTypeDef *huart;

    uint8_t rxBuf[CRSF_FRAME_SIZE_MAX];
    uint8_t rxBufPos;
    uint32_t baud;
    uint32_t lastReceive;
    uint32_t lastChannelsPacket;
    volatile bool rx_packet_ready;      // Flag set when a full packet is received
    bool linkIsUp;
    uint32_t passthroughBaud;
    int channels[CRSF_MAX_CHANNEL];
    int telemetry_channels[CRSF_MAX_CHANNEL];
    uint32_t last_channels_packet_ms;   // Timestamp of last RC channels packet
    
    crsf_link_statistics_t linkStatistics;
    crsf_sensor_battery_t batterySensor;
    crsf_sensor_gps_t gpsSensor;

    // TX State
    volatile bool tx_busy;              // Flag indicating if UART TX is busy
    volatile bool rx_busy;				// FLag indicating if UART RX is busy
    volatile bool idlecallback;         // Flag indicating if call back should be called
    volatile bool uart_error_occurred;  // Flag indicating if UART error occurred

    // Event Handlers
    void (*onLinkUp)(void);
    void (*onLinkDown)(void);
    void (*onOobData)(uint8_t b);
    void (*onPacketChannels)(void);
} CrsfSerial_HandleTypeDef;

typedef enum {
    STATE_STARTUP,
    STATE_SEND_PKT_RATE,
    STATE_SEND_POWER,
    STATE_SEND_DYNAMIC,
    STATE_NORMAL
} CRSF_State;

typedef struct crsf_sensor_battery_s
{
    uint32_t voltage;  // V * 10 big endian
    uint32_t current;  // A * 10 big endian
    uint32_t capacity; // mah big endian
    uint32_t remaining; // %
} PACKED crsf_sensor_battery_t;

typedef struct {
    uint8_t uplink_rssi_ant1;    // RSSI in dBm * -1 (e.g., 60 = -60dBm)
    uint8_t uplink_rssi_ant2;
    uint8_t uplink_lq;           // Link Quality (0 - 100)
    int8_t  uplink_snr;          // Signal-to-Noise Ratio
    uint8_t active_antenna;      // 0 or 1
    uint8_t rf_mode;             // 0=4Hz, 1=50Hz, 2=150Hz, etc.
    uint8_t uplink_tx_power;     // 0=0mW, 1=10mW, 2=25mW, etc.
    uint8_t downlink_rssi;
    uint8_t downlink_lq;
    int8_t  downlink_snr;
} crsf_link_statistics_t;

typedef struct crsf_sensor_gps_s
{
    int32_t latitude;   // degree / 10,000,000 big endian
    int32_t longitude;  // degree / 10,000,000 big endian
    uint16_t groundspeed;  // km/h / 10 big endian
    uint16_t heading;   // GPS heading, degree/100 big endian
    uint16_t altitude;  // meters, +1000m big endian
    uint8_t satellites; // satellites
} PACKED crsf_sensor_gps_t;

// === Function declarations (formerly class methods) ===
void CRSF_SetRxMode(void);

void CRSF_SetTxMode(void);

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len);

// Prepares a CRSF data packet from channel values
void CRSF_PrepareDataPacket(uint8_t packet[], int16_t channels[]);

// Prepares a CRSF command packet
void CRSF_PrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value);

// Writes a CRSF packet over UART
//void CRSF_WritePacket(uint8_t packet[], uint8_t packetLength);
uint8_t CRSF_WritePacket(uint8_t packet[], uint8_t packetLength);

// CRSF Loop function
void CrsfSerial_Loop(CrsfSerial_HandleTypeDef *hcrsf);

/**
 * @brief Processes a single byte received from UART.
 *        This function should be called from the UART RX callback (e.g., HAL_UARTEx_RxEventCallback).
 * @param handle Pointer to the CRSF handle structure.
 * @param byte The received byte.
 */
void ProcessByte(CrsfSerial_HandleTypeDef *hcrsf, uint8_t b);

/**
 * @brief Sends a CRSF telemetry poll packet.
 *        Typically used on the Transmitter side.
 * @param handle Pointer to the CRSF handle structure.
 * @return HAL_StatusTypeDef HAL_OK if packet is sent, HAL_BUSY if TX is busy.
 */
HAL_StatusTypeDef Crsf_SendTelemetryPoll(CrsfSerial_HandleTypeDef *hcrsf);

#endif // CRSF_PROTOCOL_H

