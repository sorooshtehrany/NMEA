
/**
  ******************************************************************************
  * @file           : can.h
  * @brief          : Header for can communication             
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
	void sendGPSDataOverCAN(uint8_t* gpsData, uint8_t dataLength);
	
#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
