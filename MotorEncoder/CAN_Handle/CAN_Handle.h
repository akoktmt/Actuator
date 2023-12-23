/*
 * CAN_Handle.h
 *
 *  Created on: Dec 22, 2023
 *      Author: win 10
 */

#ifndef CAN_HANDLE_H_
#define CAN_HANDLE_H_
#include "main.h"
void CAN_HandleSenđata(const uint32_t arbitration_id,
                           const uint8_t* data, const uint8_t size);
void CAN_Config_filtering(void);

#endif /* CAN_HANDLE_H_ */
