/*
 * control.h
 *
 *  Created on: 28.11.2018
 *      Author: melf_
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "Parameter.h"
#include "main.h"


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void Motor_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Emergency_Stop();

#endif /* CONTROL_H_ */
