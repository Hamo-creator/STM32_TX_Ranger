/*
 * crsf.c
 *
 *  Created on: Apr 5, 2025
 *      Author: Khalil
 */

#include "crsf.h"
#include "main.h"  // for UART handle


volatile uint8_t crsf_tx_busy = 0;
volatile uint32_t telemetry_count = 0;

extern uint16_t oldPos;
extern uint8_t  uartRxBuf[];

// Replace with your actual UART handle
extern UART_HandleTypeDef huart1;
extern CrsfSerial_HandleTypeDef hcrsf;

// Human readable RF mode
const char* ELRS_Modes[] = {
    "4Hz", "50Hz", "150Hz", "250Hz", "500Hz", "F500", "F1000", "D500", "D250"
};

// crc implementation from CRSF protocol document rev7
static uint8_t crsf_crc8tab[256] = {

    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,

    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,

    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,

    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,

    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,

    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,

    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,

    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,

    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,

    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,

    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,

    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,

    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,

    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,

    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,

    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};


// === CRC8 Calculation ===
//static uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

uint8_t crc8_dvb_s2(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    }
    return crc;
}

void CRSF_SetRxMode(void)
{
    HAL_GPIO_WritePin(TX_RX_EN_GPIO_Port, TX_RX_EN_Pin, GPIO_PIN_SET);   // Enable receiver
    hcrsf.rx_busy = true;
}

void CRSF_SetTxMode(void)
{
    HAL_GPIO_WritePin(TX_RX_EN_GPIO_Port, TX_RX_EN_Pin, GPIO_PIN_RESET); // Disable receiver → TX active
    //hcrsf.rx_busy = false;
}

// === Packet Preparation: Channels ===
void CRSF_PrepareDataPacket(uint8_t packet[], int16_t channels[]) {
    packet[0] = ELRS_ADDRESS;
    packet[1] = 24;
    packet[2] = TYPE_CHANNELS;

    packet[3]  = (uint8_t)(channels[0] & 0x07FF);
    packet[4]  = (uint8_t)((channels[0] >> 8) | (channels[1] << 3));
    packet[5]  = (uint8_t)((channels[1] >> 5) | (channels[2] << 6));
    packet[6]  = (uint8_t)((channels[2] >> 2));
    packet[7]  = (uint8_t)((channels[2] >> 10) | (channels[3] << 1));
    packet[8]  = (uint8_t)((channels[3] >> 7) | (channels[4] << 4));
    packet[9]  = (uint8_t)((channels[4] >> 4) | (channels[5] << 7));
    packet[10] = (uint8_t)((channels[5] >> 1));
    packet[11] = (uint8_t)((channels[5] >> 9) | (channels[6] << 2));
    packet[12] = (uint8_t)((channels[6] >> 6) | (channels[7] << 5));
    packet[13] = (uint8_t)((channels[7] >> 3));
    packet[14] = (uint8_t)(channels[8]);
    packet[15] = (uint8_t)((channels[8] >> 8) | (channels[9] << 3));
    packet[16] = (uint8_t)((channels[9] >> 5) | (channels[10] << 6));
    packet[17] = (uint8_t)((channels[10] >> 2));
    packet[18] = (uint8_t)((channels[10] >> 10) | (channels[11] << 1));
    packet[19] = (uint8_t)((channels[11] >> 7) | (channels[12] << 4));
    packet[20] = (uint8_t)((channels[12] >> 4) | (channels[13] << 7));
    packet[21] = (uint8_t)((channels[13] >> 1));
    packet[22] = (uint8_t)((channels[13] >> 9) | (channels[14] << 2));
    packet[23] = (uint8_t)((channels[14] >> 6) | (channels[15] << 5));
    packet[24] = (uint8_t)((channels[15] >> 3));

    packet[25] = crsf_crc8(&packet[2], packet[1] - 1);
}

// === Packet Preparation: Command ===
void CRSF_PrepareCmdPacket(uint8_t packetCmd[], uint8_t command, uint8_t value) {
    packetCmd[0] = ELRS_ADDRESS;
    packetCmd[1] = 6;
    packetCmd[2] = TYPE_SETTINGS_WRITE;
    packetCmd[3] = ELRS_ADDRESS;
    packetCmd[4] = ADDR_RADIO;
    packetCmd[5] = command;
    packetCmd[6] = value;
    packetCmd[7] = crsf_crc8(&packetCmd[2], packetCmd[1] - 1);
}


// === UART Transmission ===
uint8_t CRSF_WritePacket(uint8_t packet[], uint8_t packetLength)
{
    // 1. Check if the UART is already busy.
    if (crsf_tx_busy) {
        return HAL_BUSY;
    }

    HAL_UART_DMAStop(&huart1);
    CRSF_SetTxMode();

	hcrsf.rx_busy = false;
    hcrsf.tx_busy = true;
    hcrsf.idlecallback = false;
	
    // 6. Start the DMA transfer.
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, packet, packetLength);
	/*
	if (status == HAL_OK) {// Wait for TX to finish (or use the Timer method we discussed)
        // Then immediately clear the echo and go back to RX
        while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
        
        CRSF_SetRxMode();
        
        // RESTART DMA: This clears any "echo" bytes from the RX buffer
        HAL_UART_DMAStop(&huart1);
        HAL_UART_Receive_DMA(&huart1, uartRxBuf, UART_RX_BUFFER_SIZE);
    }
	*/
	return status;
}

static void ShiftBuffer(CrsfSerial_HandleTypeDef *hcrsf, uint8_t cnt) {
    if (cnt >= hcrsf->rxBufPos) {
        hcrsf->rxBufPos = 0;
        return;
    }
    memmove(hcrsf->rxBuf, hcrsf->rxBuf + cnt, hcrsf->rxBufPos - cnt);
    hcrsf->rxBufPos -= cnt;
}

void ProcessByte(CrsfSerial_HandleTypeDef *hcrsf, uint8_t b) {
    if (hcrsf->rxBufPos >= sizeof(hcrsf->rxBuf)) {
        hcrsf->rxBufPos = 0; // reset on overflow
    }

	hcrsf->rxBuf[hcrsf->rxBufPos++] = b;
    if (hcrsf->rxBufPos >= 2) {
        uint8_t len = hcrsf->rxBuf[1];
		if (len < 3 || CRSF_MAX_PACKET_SIZE) {
			ShiftBuffer(hcrsf, 1);
			return;
		}
		// Check if the full packet has arrived
        if (hcrsf->rxBufPos >= len + 2) {
			// CRC is calculated on Type + Payload (excludes Addr, Len, and CRC itself)
            uint8_t crc = crc8_dvb_s2(hcrsf->rxBuf + 2, len - 1);
            if (crc == hcrsf->rxBuf[len + 1]) {
                HandlePacket(hcrsf, len);
                ShiftBuffer(hcrsf, len + 2);// Successfully processed
            } else {	// CRC mismatch, discard this packet
                ShiftBuffer(hcrsf, 1);
            }
            // ShiftBuffer(hcrsf, len + 2); // Shift buffer past the processed/discarded packet
        }
    }
}

// This is where you handle telemetry frames from the RadioMaster TX
void HandlePacket(CrsfSerial_HandleTypeDef *hcrsf, uint8_t len)
{
	telemetry_count++;
    uint8_t type = hcrsf->rxBuf[2];  // Frame type byte

    switch (type) {
    case CRSF_FRAMETYPE_BATTERY_SENSOR:	//BATTERY Telemetry
        {
            // Voltage: 2 bytes, Big Endian (deciVolts)
            hcrsf->telemetry_channels[3] = (hcrsf->rxBuf[3] << 8) | hcrsf->rxBuf[4];
			// Current: 2 bytes, Big Endian (10mA units)
            hcrsf->telemetry_channels[4] = (hcrsf->rxBuf[5] << 8) | hcrsf->rxBuf[6];
			// Capacity: 3 bytes, Big Endian (mAh)
            hcrsf->telemetry_channels[5] = ((hcrsf->rxBuf[7] << 16) | hcrsf->rxBuf[8] << 8) | hcrsf->rxBuf[9];
			// hcrsf->batterySensor.voltage = (hcrsf->rxBuf[3] << 8) | hcrsf->rxBuf[4];
			// hcrsf->batterySensor.current = (hcrsf->rxBuf[5] << 8) | hcrsf->rxBuf[6];
			// hcrsf->batterySensor.capacity = ((hcrsf-<rxBuf[7] << 16) | (hcrsf->rxBuf[8] << 8 )) | hcrsf->rxBuf[9];
            //uint16_t voltage = (hcrsf->rxBuf[3] << 8) | hcrsf->rxBuf[4];
            //uint16_t current = (hcrsf->rxBuf[5] << 8) | hcrsf->rxBuf[6];
            // Store or print it
            //printf("Telemetry: Battery %u dV, %u cA\r\n", voltage, current);
        }
        break;

    case CRSF_FRAMETYPE_LINK_STATISTICS: // LINK_STATISTICS
        {
			// Use the handle's member 'linkStatistics' to store the data
            memcpy(&hcrsf->linkStatistics, payload, sizeof(crsf_link_statistics_t));
			
            // Map to your global display variables
            display_rssi = (int16_t)hcrsf->linkStatistics.uplink_rssi_ant1 * -1;
            display_lq = hcrsf->linkStatistics.uplink_lq;
			
			// Get the RF Mode string safely
            const char* mode_name = "N/A";
            if (hcrsf->linkStatistics.rf_mode < (sizeof(ELRS_Modes) / sizeof(char*))) {
                mode_name = ELRS_Modes[hcrsf->linkStatistics.rf_mode];
            }

            if (hcrsf->onPacketLinkStatistics) {
                hcrsf->onPacketLinkStatistics(&hcrsf->linkStatistics);
            }
            break;
        }
        break;

    case CRSF_FRAMETYPE_ATTITUDE: // ATTITUDE
        {
            int16_t pitch = (hcrsf->rxBuf[3] << 8) | hcrsf->rxBuf[4];
            int16_t roll  = (hcrsf->rxBuf[5] << 8) | hcrsf->rxBuf[6];
            int16_t yaw   = (hcrsf->rxBuf[7] << 8) | hcrsf->rxBuf[8];
            printf("Attitude: P=%d, R=%d, Y=%d\r\n", pitch, roll, yaw);
        }
        break;

    default:
        // Unknown / unhandled frame type
        break;
    }
}

void CrsfSerial_Loop(CrsfSerial_HandleTypeDef *hcrsf) {
    if (hcrsf->linkIsUp && HAL_GetTick() - hcrsf->lastChannelsPacket > CRSF_FAILSAFE_STAGE1_MS) {
        if (hcrsf->onLinkDown){
        	hcrsf->onLinkDown();
        }
        hcrsf->linkIsUp = false;
    }
}

void CrsfSerial_UART_IdleCallback(CrsfSerial_HandleTypeDef *hcrsf)
{
    uint16_t dmaPos = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(hcrsf->huart->hdmarx);
    uint16_t len;

    hcrsf->idlecallback = true;
	// Safety check: if dmaPos is exactly at the limit, wrap it
    // if (dmaPos >= UART_RX_BUFFER_SIZE) dmaPos = 0;

    if (dmaPos != oldPos) {
        if (dmaPos > oldPos) {
			// Linear read: data is between oldPos and dmaPos
            /*
			for (uint16_t i = oldPos; i < dmaPos; i++) {
                ProcessByte(hcrsf, uartRxBuf[i]);
            }
			*/
            len = dmaPos - oldPos;
            for (uint16_t i = 0; i < len; i++) {
                ProcessByte(hcrsf, uartRxBuf[oldPos + i]);
            }
        } else {
            len = UART_RX_BUFFER_SIZE - oldPos;
            for (uint16_t i = 0; i < len; i++) {
                ProcessByte(hcrsf, uartRxBuf[oldPos + i]);
            }
            for (uint16_t i = 0; i < dmaPos; i++) {
                ProcessByte(hcrsf, uartRxBuf[i]);
            }
        }
		/*
		else {
            // Wrapped read: data is from oldPos to end, AND from start to dmaPos
            // Part 1: From oldPos to end of buffer
            for (uint16_t i = oldPos; i < UART_RX_BUFFER_SIZE; i++) {
                ProcessByte(hcrsf, uartRxBuf[i]);
            }
            // Part 2: From start of buffer to dmaPos
            for (uint16_t i = 0; i < dmaPos; i++) {
                ProcessByte(hcrsf, uartRxBuf[i]);
            }
		}
		*/

        oldPos = dmaPos;
        if (oldPos >= UART_RX_BUFFER_SIZE) oldPos = 0;
    }
}

// This is called when the DMA has received data (IDLE line detected)
/*void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {//CrsfSerial_HandleTypeDef hcrsf;
    // Assuming a single CRSF handle for simplicity, adapt for multiple if needed
   // extern CrsfHandle_t crsf_handle_tx; // For TX side

    //if (huart->Instance == crsf_handle_tx.huart->Instance) {
    if (huart->Instance == USART1) {
        // This is the TX side receiving telemetry from FC
//    	hcrsf.rx_busy = false;
        // Process bytes from the DMA buffer
        for (uint16_t i = 0; i < Size; i++) {
        	ProcessByte(&hcrsf, huart->pRxBuffPtr[i]);
        }
        // Re-arm DMA reception
        HAL_UARTEx_ReceiveToIdle_DMA(huart, huart->pRxBuffPtr, hcrsf.rxBufPos);

        //CRSF_SetTxMode();
    }
}*/

// This is called when the UART transmission is truly complete (TC flag set)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    //extern CrsfHandle_t crsf_handle_tx; // For TX side

    if (huart->Instance == USART1) {
        // TX side: Transmission complete, switch back to RX mode
    	CRSF_SetRxMode();
    	hcrsf.tx_busy = false;
    	//hcrsf.rx_busy = false;
		// oldPos = 0;
		// 2. RESTART receiving now that the line is clear
        // This ensures the next byte from the Ranger goes to uartRxBuf[0]
        HAL_UART_Receive_DMA(&huart1, uartRxBuf, UART_RX_BUFFER_SIZE);
    }
}

