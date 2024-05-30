/*
 * disp.c
 *
 *  Created on: May 26, 2024
 *      Author: Jewel James
 */


#include "main.h"
#include "disp.h"

volatile uint8_t value_dis_1,value_dis_2,value_dis_3,value_dis_4,value_dis_5,value_dis_6 ;
volatile uint8_t seg_A, seg_B,seg_C,seg_D,seg_E,seg_F,seg_G,seg_DP;

uint8_t char_idxs[] = {1, 2, 3, 4, 5, 6};

uint8_t digit_code_map[]={
// GFEDCBA  Segments      7-segment map:
  0b00111111, // 0   "0"          AAA
  0b00000110, // 1   "1"         F   B
  0b01011011, // 2   "2"         F   B
  0b01001111, // 3   "3"          GGG
  0b01100110, // 4   "4"         E   C
  0b01101101, // 5   "5"         E   C
  0b01111101, // 6   "6"          DDD
  0b00000111, // 7   "7"
  0b01111111, // 8   "8"
  0b01101111, // 9   "9"
  0b00000000, // 32  ' '  BLANK
  0b01110111, // 65  'A'
  0b01111100, // 66  'b'
  0b00111001, // 67  'C'
  0b01011110, // 68  'd'
  0b01111001, // 69  'E'
  0b01110001, // 70  'F'
  0b00111101, // 71  'G'
  0b01110110, // 72  'H'
  0b00110000, // 73  'I'
  0b00001110, // 74  'J'
  0b01110110, // 75  'K'  Same as 'H'
  0b00111000, // 76  'L'
  0b00000000, // 77  'M'  NO DISPLAY
  0b01010100, // 78  'n'
  0b00111111, // 79  'O'
  0b01110011, // 80  'P'
  0b01100111, // 81  'q'
  0b01010000, // 82  'r'
  0b01101101, // 83  'S'
  0b01111000, // 84  't'
  0b00111110, // 85  'U'
  0b00111110, // 86  'V'  Same as 'U'
  0b00000000, // 87  'W'  NO DISPLAY
  0b01110110, // 88  'X'  Same as 'H'
  0b01101110, // 89  'y'
  0b01011011, // 90  'Z'  Same as '2'
  0b01000000, // 45  '-'  DASH
  0b10000000, // 46  '.'  PERIOD
  0b01100011, // 42 '*'  DEGREE ..
  0b00001000, // 95 '_'  UNDERSCORE
};

void load_val(uint8_t dis_no) {
	switch(dis_no) {
		case 1:
			seg_DP = 0b10000000 & value_dis_1;
			seg_G = (0b01000000 & value_dis_1);
			seg_F = (0b00100000 & value_dis_1);
			seg_E = (0b00010000 & value_dis_1);
			seg_D = (0b00001000 & value_dis_1);
			seg_C = (0b00000100 & value_dis_1);
			seg_B = (0b00000010 & value_dis_1);
			seg_A = (0b00000001 & value_dis_1);
			break;
		case 2:
			seg_DP = 0b10000000 & value_dis_2;
			seg_G = (0b01000000 & value_dis_2);
			seg_F = (0b00100000 & value_dis_2);
			seg_E = (0b00010000 & value_dis_2);
			seg_D = (0b00001000 & value_dis_2);
			seg_C = (0b00000100 & value_dis_2);
			seg_B = (0b00000010 & value_dis_2);
			seg_A = (0b00000001 & value_dis_2);
			break;
		case 3:
			seg_DP = 0b10000000 & value_dis_3;
			seg_G = (0b01000000 & value_dis_3);
			seg_F = (0b00100000 & value_dis_3);
			seg_E = (0b00010000 & value_dis_3);
			seg_D = (0b00001000 & value_dis_3);
			seg_C = (0b00000100 & value_dis_3);
			seg_B = (0b00000010 & value_dis_3);
			seg_A = (0b00000001 & value_dis_3);
			break;
		case 4:
			seg_DP = 0b10000000 & value_dis_4;
			seg_G = (0b01000000 & value_dis_4);
			seg_F = (0b00100000 & value_dis_4);
			seg_E = (0b00010000 & value_dis_4);
			seg_D = (0b00001000 & value_dis_4);
			seg_C = (0b00000100 & value_dis_4);
			seg_B = (0b00000010 & value_dis_4);
			seg_A = (0b00000001 & value_dis_4);
			break ;
		case 5:
			seg_DP = 0b10000000 & value_dis_5;
			seg_G = (0b01000000 & value_dis_5);
			seg_F = (0b00100000 & value_dis_5);
			seg_E = (0b00010000 & value_dis_5);
			seg_D = (0b00001000 & value_dis_5);
			seg_C = (0b00000100 & value_dis_5);
			seg_B = (0b00000010 & value_dis_5);
			seg_A = (0b00000001 & value_dis_5);
			break;
		case 6:
			seg_DP = 0b10000000 & value_dis_6;
			seg_G = (0b01000000 & value_dis_6);
			seg_F = (0b00100000 & value_dis_6);
			seg_E = (0b00010000 & value_dis_6);
			seg_D = (0b00001000 & value_dis_6);
			seg_C = (0b00000100 & value_dis_6);
			seg_B = (0b00000010 & value_dis_6);
			seg_A = (0b00000001 & value_dis_6);
			break;
	}
}

void display_value(uint8_t dis_no, uint8_t val_idx) {
	switch(dis_no) {
		case 1:
			value_dis_1 = digit_code_map[val_idx];
			load_val(dis_no);
			set_dis_1_high();
			set_dis_2_low();
			set_dis_3_low();
			set_dis_4_low();
			set_dis_5_low();
			set_dis_6_low();
			seg_A ? set_A_high() : set_A_low();
		    seg_B ? set_B_high() : set_B_low();
		    seg_C ? set_C_high() : set_C_low();
		    seg_D ? set_D_high() : set_D_low();
		    seg_E ? set_E_high() : set_E_low();
		    seg_F ? set_F_high() : set_F_low();
		    seg_G ? set_G_high() : set_G_low();
			break ;
		case 2:
			value_dis_2 = digit_code_map[val_idx];
			load_val(dis_no);
			set_dis_2_high();
			set_dis_1_low();
			set_dis_3_low();
			set_dis_4_low();
			set_dis_5_low();
			set_dis_6_low();
			seg_A ? set_A_high() : set_A_low();
		    seg_B ? set_B_high() : set_B_low();
		    seg_C ? set_C_high() : set_C_low();
		    seg_D ? set_D_high() : set_D_low();
		    seg_E ? set_E_high() : set_E_low();
		    seg_F ? set_F_high() : set_F_low();
		    seg_G ? set_G_high() : set_G_low();
			break ;
		case 3 :
			value_dis_3 = digit_code_map[val_idx];
			load_val(dis_no);
			set_dis_3_high();
			set_dis_1_low();
			set_dis_2_low();
			set_dis_4_low();
			set_dis_5_low();
			set_dis_6_low();
			seg_A ? set_A_high() : set_A_low();
		    seg_B ? set_B_high() : set_B_low();
		    seg_C ? set_C_high() : set_C_low();
		    seg_D ? set_D_high() : set_D_low();
		    seg_E ? set_E_high() : set_E_low();
		    seg_F ? set_F_high() : set_F_low();
		    seg_G ? set_G_high() : set_G_low();
			break ;
		case 4 :
			value_dis_4 = digit_code_map[val_idx];
			load_val(dis_no);
			set_dis_4_high();
			set_dis_1_low();
			set_dis_2_low();
			set_dis_3_low();
			set_dis_5_low();
			set_dis_6_low();
			seg_A ? set_A_high() : set_A_low();
		    seg_B ? set_B_high() : set_B_low();
		    seg_C ? set_C_high() : set_C_low();
		    seg_D ? set_D_high() : set_D_low();
		    seg_E ? set_E_high() : set_E_low();
		    seg_F ? set_F_high() : set_F_low();
		    seg_G ? set_G_high() : set_G_low();
			break ;
		case 5 :
			value_dis_5 = digit_code_map[val_idx];
			load_val(dis_no);
			set_dis_5_high();
			set_dis_1_low();
			set_dis_2_low();
			set_dis_3_low();
			set_dis_4_low();
			set_dis_6_low();
			seg_A ? set_A_high() : set_A_low();
		    seg_B ? set_B_high() : set_B_low();
		    seg_C ? set_C_high() : set_C_low();
		    seg_D ? set_D_high() : set_D_low();
		    seg_E ? set_E_high() : set_E_low();
		    seg_F ? set_F_high() : set_F_low();
		    seg_G ? set_G_high() : set_G_low();
			break ;
		case 6:
			value_dis_6 = digit_code_map[val_idx];
			load_val(dis_no);
			set_dis_6_high();
			set_dis_1_low();
			set_dis_2_low();
			set_dis_3_low();
			set_dis_4_low();
			set_dis_5_low();
			seg_A ? set_A_high() : set_A_low();
		    seg_B ? set_B_high() : set_B_low();
		    seg_C ? set_C_high() : set_C_low();
		    seg_D ? set_D_high() : set_D_low();
		    seg_E ? set_E_high() : set_E_low();
		    seg_F ? set_F_high() : set_F_low();
		    seg_G ? set_G_high() : set_G_low();
			break ;
		}
	}

void disp_alloff() {
	set_dis_1_low();
	set_dis_2_low();
	set_dis_3_low();
	set_dis_4_low();
	set_dis_5_low();
	set_dis_6_low();
}
void disp_no(uint16_t no) {
	// Turn all displays off
	uint8_t temp = 0;
	if(no > 999999)
		return;
	char_idxs[0] = 10;
	char_idxs[1] = 10;
	char_idxs[2] = 10;
	char_idxs[3] = 10;
	char_idxs[4] = 10;
	char_idxs[5] = 10;
	if(no == 0) {
		char_idxs[5] = 0;
		return;
	}
	for(size_t i = 0; no; i++) {
		temp = no % 10;
		char_idxs[6-i-1] = temp;
		no = no / 10;
	}
}

