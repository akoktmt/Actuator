/*
 * EncoderVelocity.h
 *
 *  Created on: Nov 10, 2023
 *      Author: win 10
 */

#ifndef ENCODERVELOCITY_H_
#define ENCODERVELOCITY_H_
#include <stdint.h>
#include "main.h"

#define MAX_COUNT 65536
typedef struct {
	int32_t last_counter_value;
	float velocity;
	uint32_t Lastick;
} Encoder;


void Encoder_Config(void);
void Encoder_Init(Encoder *Encoder);
uint16_t Encoder_Read();
void Encoder_CaculateSpeed(Encoder *Encoder, uint32_t Time);
float Get_Speed(Encoder *Encoder);
#endif /* ENCODERVELOCITY_H_ */
