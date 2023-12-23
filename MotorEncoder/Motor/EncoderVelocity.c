/*
 * EncoderVelocity.c
 *
 *  Created on: Nov 10, 2023
 *      Author: win 10
 */

#include "EncoderVelocity.h"
#include "tim.h"

void Encoder_Init(Encoder *Encoder) {

	Encoder->Lastick=HAL_GetTick();
	Encoder->velocity=0;
	Encoder->last_counter_value=0;

}

int32_t encoder_count = 0;
int32_t encoder_last_count=0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
     int16_t capture = __HAL_TIM_GET_COUNTER(htim);
     if (htim->Instance == TIM2)
     {
         if (capture < encoder_last_count)
         {
             encoder_count++;
         }
         else
         {
             encoder_count--;
         }

         if (encoder_count < 0)
         {
             encoder_count += MAX_COUNT;
         }

         if (encoder_count >= MAX_COUNT)
         {
             encoder_count -= MAX_COUNT;
         }
         encoder_last_count = capture;
     }
}
void Encoder_CaculateSpeed(Encoder *Encoder, uint32_t Time){

	 if ((HAL_GetTick() - Encoder->Lastick) >= Time)
	 {
			int delta_counter = encoder_count - Encoder->last_counter_value;
			  	if(encoder_count > Encoder->last_counter_value) {

			  		delta_counter = encoder_count - Encoder->last_counter_value;
			  	}
			  	else {
			  		delta_counter = Encoder->last_counter_value - encoder_count;
			  	}
			  	  float  rpm = ((float)(delta_counter) / 400)/(Time/1000.0);
			  	   Encoder->Lastick = HAL_GetTick();
		           Encoder->velocity = rpm*(2 * 3.14159265359 * (0.07-0.045));
		           Encoder->last_counter_value=encoder_count;
	 }
}

float Get_Speed(Encoder *Encoder){
	return Encoder->velocity;
}


