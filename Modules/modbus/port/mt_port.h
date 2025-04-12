/**
  ******************************************************************************
  * @file           : mt_port.h
  * @brief          : Additional porting data
  * @author         : MicroTechnics (microtechnics.ru)
  ******************************************************************************
  */
#ifndef MT_PORT_H
#define MT_PORT_H
/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
/* Declarations and definitions ----------------------------------------------*/
/* Functions -----------------------------------------------------------------*/
void MT_PORT_SetTimerModule(TIM_HandleTypeDef* timer);
void MT_PORT_SetUartModule(UART_HandleTypeDef* uart);
#endif // #ifndef MT_PORT_H