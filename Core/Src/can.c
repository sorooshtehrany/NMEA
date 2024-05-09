/**
  ******************************************************************************
  * @file           : can.c
  * @brief          : this file manages can communication
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "can.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define CAN_PAYLOAD_SIZE 8  // Maximum size of CAN payload

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Send GPS data over CAN in 8-byte chunks.
 *
 * This function transmits the GPS data received from the module over the CAN bus.
 * It divides the data into 8-byte chunks, configures the CAN header, and sends
 * each chunk as a separate CAN message. The function waits for each message to
 * be transmitted before sending the next one. It starts and stops the CAN
 * peripheral to enable and complete the data transmission.
 *
 * @param gpsData Pointer to the GPS data to be transmitted.
 * @param dataLength The length of the GPS data in bytes.
 */
void sendGPSDataOverCAN(uint8_t* gpsData, uint16_t dataLength)
{
    uint8_t i;
    uint8_t canTxData[CAN_PAYLOAD_SIZE];
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    // Start the CAN transmit
    HAL_CAN_Start(&hcan1);

    for (i = 0; i < dataLength; i += CAN_PAYLOAD_SIZE)
    {
        // Copy 8 bytes of GPS data to the CAN TX buffer
        if ((i+8) <= dataLength) memcpy(canTxData, &gpsData[i], CAN_PAYLOAD_SIZE);
		else 		 			 memcpy(canTxData, &gpsData[i], dataLength - i);

        // Set the CAN TX header properties
        TxHeader.DLC = CAN_PAYLOAD_SIZE;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.StdId = 0x161;

        // Transmit the CAN message
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, canTxData, &TxMailbox);

        // Wait for the CAN message to be transmitted
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3);
    }

    // Stop the CAN transmit
    HAL_CAN_Stop(&hcan1);
}