/*
 * disp.h
 *
 *  Created on: May 26, 2024
 *      Author: Jewel James
 */

#ifndef INC_DISP_H_
#define INC_DISP_H_

#include "main.h"

#define set_dis_1_high()		(HAL_GPIO_WritePin(DISP_1_GPIO_Port, DISP_1_Pin, GPIO_PIN_RESET))
#define set_dis_2_high()		(HAL_GPIO_WritePin(DISP_2_GPIO_Port, DISP_2_Pin, GPIO_PIN_RESET))
#define set_dis_3_high()		(HAL_GPIO_WritePin(DISP_3_GPIO_Port, DISP_3_Pin, GPIO_PIN_RESET))
#define set_dis_4_high()		(HAL_GPIO_WritePin(DISP_4_GPIO_Port, DISP_4_Pin, GPIO_PIN_RESET))
#define set_dis_5_high()		(HAL_GPIO_WritePin(DISP_5_GPIO_Port, DISP_5_Pin, GPIO_PIN_RESET))
#define set_dis_6_high()		(HAL_GPIO_WritePin(DISP_6_GPIO_Port, DISP_6_Pin, GPIO_PIN_RESET))
#define set_dis_1_low()			(HAL_GPIO_WritePin(DISP_1_GPIO_Port, DISP_1_Pin, GPIO_PIN_SET))
#define set_dis_2_low()			(HAL_GPIO_WritePin(DISP_2_GPIO_Port, DISP_2_Pin, GPIO_PIN_SET))
#define set_dis_3_low()			(HAL_GPIO_WritePin(DISP_3_GPIO_Port, DISP_3_Pin, GPIO_PIN_SET))
#define set_dis_4_low()			(HAL_GPIO_WritePin(DISP_4_GPIO_Port, DISP_4_Pin, GPIO_PIN_SET))
#define set_dis_5_low()			(HAL_GPIO_WritePin(DISP_5_GPIO_Port, DISP_5_Pin, GPIO_PIN_SET))
#define set_dis_6_low()			(HAL_GPIO_WritePin(DISP_6_GPIO_Port, DISP_6_Pin, GPIO_PIN_SET))
#define set_A_high()			(HAL_GPIO_WritePin(DISP_A_GPIO_Port, DISP_A_Pin, GPIO_PIN_SET))
#define set_B_high()			(HAL_GPIO_WritePin(DISP_B_GPIO_Port, DISP_B_Pin, GPIO_PIN_SET))
#define set_C_high()			(HAL_GPIO_WritePin(DISP_C_GPIO_Port, DISP_C_Pin, GPIO_PIN_SET))
#define set_D_high()			(HAL_GPIO_WritePin(DISP_D_GPIO_Port, DISP_D_Pin, GPIO_PIN_SET))
#define set_E_high()			(HAL_GPIO_WritePin(DISP_E_GPIO_Port, DISP_E_Pin, GPIO_PIN_SET))
#define set_F_high()			(HAL_GPIO_WritePin(DISP_F_GPIO_Port, DISP_F_Pin, GPIO_PIN_SET))
#define set_G_high()			(HAL_GPIO_WritePin(DISP_G_GPIO_Port, DISP_G_Pin, GPIO_PIN_SET))
#define set_A_low()				(HAL_GPIO_WritePin(DISP_A_GPIO_Port, DISP_A_Pin, GPIO_PIN_RESET))
#define set_B_low()				(HAL_GPIO_WritePin(DISP_B_GPIO_Port, DISP_B_Pin, GPIO_PIN_RESET))
#define set_C_low()				(HAL_GPIO_WritePin(DISP_C_GPIO_Port, DISP_C_Pin, GPIO_PIN_RESET))
#define set_D_low()				(HAL_GPIO_WritePin(DISP_D_GPIO_Port, DISP_D_Pin, GPIO_PIN_RESET))
#define set_E_low()				(HAL_GPIO_WritePin(DISP_E_GPIO_Port, DISP_E_Pin, GPIO_PIN_RESET))
#define set_F_low()				(HAL_GPIO_WritePin(DISP_F_GPIO_Port, DISP_F_Pin, GPIO_PIN_RESET))
#define set_G_low()				(HAL_GPIO_WritePin(DISP_G_GPIO_Port, DISP_G_Pin, GPIO_PIN_RESET))

#define DISP_COUNT				6

extern uint8_t char_idxs[];

void disp_set(uint8_t disp_no, uint8_t val_idx);
void disp_routine(uint8_t disp_no);
void disp_alloff(void);
void disp_no(uint16_t no);
void disp_text(char* txt);
void display_value(uint8_t dis_no, uint8_t val_idx);

#endif /* INC_DISP_H_ */
