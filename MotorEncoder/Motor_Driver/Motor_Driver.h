/*
 * Motor_Driver.h
 *
 *  Created on: Nov 6, 2023
 *      Author: win 10
 */

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
typedef struct {
  int16_t speed;
  uint8_t forward;
} VNH3SP30_t;
void VNH3SP30_Init(VNH3SP30_t* driver);

void VNH3SP30_SetSpeed(VNH3SP30_t* driver, int16_t speed);

void VNH3SP30_Brake(VNH3SP30_t* driver, int16_t brakePower);

uint8_t VNH3SP30_GetStatus(VNH3SP30_t* driver);

int16_t VNH3SP30_GetCurrent(VNH3SP30_t* driver);


#endif /* MOTOR_DRIVER_H_ */
