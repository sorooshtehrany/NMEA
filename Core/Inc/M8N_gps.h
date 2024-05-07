
/**
  ******************************************************************************
  * @file           : M8N_gps.h
  * @brief          : Header for M8N_gps file.                   
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __M8N_GPS_H
#define __M8N_GPS_H

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

void uartRxInit(void);
void M8N_refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* __M8N_GPS_H */