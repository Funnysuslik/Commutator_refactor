#ifndef SETTING_H_
#define SETTING_H_

#include "stm32f4xx_hal.h"

float sens_set(int max_curr);
void sens_V(uint8_t MM_address, float*  MM_nom_dec, float MM_nom_dec_prev);
float sens_I(uint8_t MM_address, float m);
void dpot_set1_4(uint8_t DP_address, float Vset, float MM_nom_cur_dec, float* MM_nom_delta, float MM_nom_dec_old, float MM_nom_dec);
void dpot_set5_7(uint8_t DP_address, float Vset, float MM_nom_cur_dec, float* MM_nom_delta, float MM_nom_dec_old, float MM_nom_dec);

#endif
