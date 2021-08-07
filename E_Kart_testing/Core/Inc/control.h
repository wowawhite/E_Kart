/*
 * control.h - one line comments only!
 *  Created on: 28.11.2018
 *  Updated on: 2021.07 - Added RCP-Mode
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "main.h"


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void Motor_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Emergency_Stop(void);

#endif /* CONTROL_H_ */
