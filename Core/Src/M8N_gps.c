/**
  ******************************************************************************
  * @file           : M8N_gps.c
  * @brief          : this file parses the gps packets
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "M8N_gps.h"
#include <string.h>
#include <stdio.h>
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#difine MAX_LENGTH  100// 100 for example
#difine READY_TO_PARSE 255;
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
uint8_t Rx_data                    // gets the uart rx
uint8_t Rx_buffer[2][MAX_LENGTH];  // double buffering
uint8_t Buff_changer;              // switches between 2 buffers
uint8_t Buff_counter;              // counts the index of buffering array 
/* Private function prototypes -----------------------------------------------*/
static uint8_t parseNMEA(uint8_t* data);
/* Private user code ---------------------------------------------------------*/



/**
 * @brief Parse an NMEA sentence received from the GPS module.
 *
 * This function takes an NMEA sentence as input and parses it to validate the
 * sentence structure and checksum. It then checks the sentence type and
 * verifies that the number of data fields matches the expected format for
 * each type of NMEA sentence.
 *
 * @param data Pointer to the NMEA sentence data.
 * @return 1 if the NMEA sentence is valid and parsed successfully, 0 otherwise.
 */
static uint8_t parseNMEA(uint8_t* data) 
{
    if (!data || data[0] != '$' || !strchr(data, '*'))
    {
        // Invalid sentence (empty, missing '$', or missing checksum)
        return 0;
    }

        
    uint8_t num_fields = 1;
    uint8_t calculated_checksum = 0;
    uint8_t char_counter = 1;
    // Separate data fields based on commas
    while (data[char_counter] != '*' && char_counter < MAX_LENGTH)
    {
        calculated_checksum ^= data[char_counter];
        if (data[char_counter++] == ',')
        {            
            num_fields++;
        }        
    }    
    uint8_t data_checksum = (uint8_t)strtoul(&data[char_counter + 1], NULL, 16);
    if (calculated_checksum != data_checksum)
    {
        // checksum error
        return 0;
    }
    
    // Packet 1
    uint8_t flag = 0;    
    if (strncmp(&data[1], "GPGGA", 5) == 0)
    {
        if (num_fields == 15)
        {
            flag = 1;
        }
    }

    // Packet 2
    else if (strncmp(&data[1], "$GPGSA", 5) == 0)
    {
        if (num_fields == 18)
        {
            flag = 1;
        }
    }

    // Packet 3
    else if (strncmp(&data[1], "$GPGSV", 5) == 0)
    {
        if (num_fields == 20)
        {
            flag = 1;
        }
    }

    // Packet 4
    else if (strncmp(&data[1], "$GPGSV", 5) == 0)
    {
        if (num_fields == 20)
        {
            flag = 1;
        }
    }

    // Packet 5
    else if (strncmp(&data[1], "$GPGSV", 5) == 0)
    {
        if (num_fields == 16)
        {
            flag = 1;
        }
    }

    // Packet 6
    else if (strncmp(&data[1], "$GPRMC", 5) == 0)
    {
        if (num_fields == 13)
        {
            flag = 1;
        }
    }

    // Packet 7
    else if (strncmp(&data[1], "$GPGGA", 5) == 0)
    {
        if (num_fields == 15)
        {
            flag = 1;
        }
    }

    // Packet 8
    else if (strncmp(&data[1], "$GPGSA", 5) == 0)
    {
        if (num_fields == 18)
        {
            flag = 1;
        }
    }

    // Packet 9
    else if (strncmp(&data[1], "$GPGSV", 5) == 0)
    {
        if (num_fields == 20)
        {
            flag = 1;
        }
    }

    // Packet 10
    else if (strncmp(&data[1], "$GPGSV", 5) == 0)
    {
        if (num_fields == 20)
        {
            flag = 1;
        }
    }

    // Packet 11
    else if (strncmp(&data[1], "$GPGSV", 5) == 0)
    {
        if (num_fields == 16)
        {
            flag = 1;
        }
    }

    // Packet 12
    else if (strncmp(&data[1], "$GPRMC", 5) == 0)
    {
        if (num_fields == 13)
        {
            flag = 1;
        }
    }
    if (flag == 0)
    {
        //packet error 
        return 0;
    }
    return 1;
}



/**
 * @brief Initialize UART2 reception in interrupt mode.
 *
 * This function sets up UART2 to receive a single byte into 'Rx_data' and
 * resets the 'Buff_counter' and 'Buff_changer' variables.
 */
void uartRxInit(void)
{
  HAL_UART_Receive_IT(&huart2, &Rx_data, 1);  
  Buff_counter = 0; 
  Buff_changer = 0;
}


/**
 * @brief Callback function for UART2 reception complete event.
 *
 * This function is called by the HAL UART driver when a byte of data has been
 * received on the UART2 interface. It processes the received byte and stores
 * it in the 'Rx_buffer' array based on the current state of the 'Buff_counter'
 * and 'Buff_changer' variables. The function also resets the UART2 reception
 * to receive the next byte.
 *
 * @param huart Pointer to the UART handle.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
  if (Buff_counter == 0)  
  {
    if (Rx_data == '$')
    {
      Buff_counter = 1;
      Rx_buffer[Buff_changer][0] =  '$'; 
    }
  }
  else if(Rx_buffer[Buff_changer][Buff_counter-2] != '*')
  {
    Rx_buffer[Buff_changer][Buff_counter++] = Rx_data;
  }
  else
  {
    Rx_buffer[Buff_changer][Buff_counter] = Rx_data;
    Rx_buffer[Buff_changer][Buff_counter + 1] = 0;
    Buff_counter = READY_TO_PARSE; 
  }
  HAL_UART_Receive_IT(&huart2, &Rx_data, 1); 
}

/**
 * @brief Refresh the M8N GPS module processing.
 *
 * It checks if a complete NMEA message has been received
 * (indicated by the 'Buff_counter' variable being equal to 'READY_TO_PARSE').
 */
uint8_t* M8N_refresh(void)
{
    if (Buff_counter == READY_TO_PARSE)
    {
         Buff_counter = 0; // ready to next buffering
        if (parseNMEA(Rx_buffer[Buff_changer]))
        {            
            if (Buff_changer == 0) 
            {
                Buff_changer = 1;
                return Rx_buffer[0];
            }            
            else
            {
                Buff_changer = 0; 
                return Rx_buffer[1];
            }                
        }
    }
    return NULL;
}