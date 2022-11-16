#include "setting.h" 
//#include "stm32f4xx_hal.h"
#include "math.h"

static uint8_t MM_read_V  = 0x8B;
static uint8_t MM_read_I =  0x89;
static float R = 0.001f;


extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

float sens_set(int max_curr)
{
	float m, Current_LSB, CAL, jank;
	int32_t bCur, mCur, rCur, bPwr, mPwr, rPwr;
  uint16_t CAL_LSB;
  uint8_t CUR_LSB [3];
  //----------------------------------------------------------------------
  CUR_LSB[0] = 0xD4;
  Current_LSB = max_curr / powl(2, 15);
  CAL = 0.00512f / (Current_LSB * R);
  m = 1 / Current_LSB;
	mCur = (uint32_t) m;
//calculate and round curr and power coeffs from CalData
/***********************************************************************************************/
/*For example, assume a Current_LSB of 0.75 mA/bit is selected for a given application. 
The value for m is calculated by inverting the LSB value (for this case, m = 1 / 0.00075 = 1333.333).
Moving the decimal point so the value of m is maximized and remains within the required range of –32768 to
32767 is preferable because this value of m is relatively small and contains decimal information. Moving the
decimal point one place to the right results in a final m value of 13333 with an R value of –1 resulting from the
shift in decimal location. Moving the decimal point to maximize the value of m is critical to minimize rounding
errors. The m coefficient for power can be calculated by applying 1 / (25 × Current_LSB). For this example, the
value for the m power coefficient is calculated to be 53.333. Again (to maximize accuracy), the decimal location
is shifted by 2 to the right to give a final m value of 5333 with an R coefficient of –2. Care must be taken to adjust
the exponent coefficient, R, such that the value of m remains within the range of –32768 to 32767. However,
rounding errors resulting from the limitations on the value of m can be mitigated by carefully selecting a slightly
higher current LSB size. For example, if a Current_LSB of 1 mA/bit is selected instead of 0.75 mA/bit, the
calculated value for m is 1 / 0.001 or 1000; because this value is a whole number there is no rounding errors and
the value for R is 0. Positive values for R signify the number of times the decimal point is shifted to the left,
whereas negative values for R signify the number of decimal point shifts to the right.*/
/***********************************************************************************************/
	if((modff(mCur, &jank) != 0) & (mCur > 32768 & mCur*10 < 32768) || (mCur < -32768 & mCur/10 > -32768))
	{
		while((mCur > 32768 & mCur*10 < 32768) || (mCur < -32768 & mCur/10 > -32768))
		{
			if(mCur > 32768 || mCur/10 > -32768)
			{
				m /= 10;
				rCur ++;
				mCur = (uint32_t) m;
			}
			else if (mCur < -32768 || mCur*10 < 32768)
			{
				m *= 10;
				rCur --;
				mCur = (uint32_t)m;
			}
		}
	}
	float check_mPwr  = 1 / (CAL * 25);
	mPwr = (uint32_t) check_mPwr;
	if((modff(check_mPwr, &jank) != 0) & (mPwr > 32768 & mPwr*10 < 32768) || (mPwr < -32768 & mPwr/10 > -32768))
	{
		while((mPwr > 32768 & mPwr*10 < 32768) || (mPwr < -32768 & mPwr/10 > -32768))
		{
			if(mPwr > 32768 || mPwr/10 > -32768)
			{
				check_mPwr /= 10;
				rPwr ++;
				mPwr = (int32_t)mPwr;
			}
			else if (mPwr < -32768 || mPwr*10 < 32768)
			{
				check_mPwr *= 10;
				rPwr --;
				mPwr = (int32_t)mPwr;
			}
		}
	}
    CAL_LSB = CAL;
    CUR_LSB [1] =  CAL_LSB;
    CUR_LSB [2] =  CAL_LSB>>8;
    HAL_I2C_Master_Transmit(&hi2c1, 0x80, CUR_LSB, 3, 1000);   // set INA233 MFR_CALIBRATION register for current measurement
    HAL_I2C_Master_Transmit(&hi2c1, 0x82, CUR_LSB, 3, 1000);   // set INA233 MFR_CALIBRATION register for current measurement
    HAL_I2C_Master_Transmit(&hi2c1, 0x84, CUR_LSB, 3, 1000);   // set INA233 MFR_CALIBRATION register for current measurement
    HAL_I2C_Master_Transmit(&hi2c1, 0x86, CUR_LSB, 3, 1000);   // set INA233 MFR_CALIBRATION register for current measurement
    HAL_I2C_Master_Transmit(&hi2c1, 0x88, CUR_LSB, 3, 1000);   // set INA233 MFR_CALIBRATION register for current measurement
    HAL_I2C_Master_Transmit(&hi2c1, 0x8A, CUR_LSB, 3, 1000);   // set INA233 MFR_CALIBRATION register for current measurement
    HAL_I2C_Master_Transmit(&hi2c1, 0x8C, CUR_LSB, 3, 1000);   // set INA233 MFR_CALIBRATION register for current measurement
    HAL_I2C_Master_Transmit(&hi2c1, 0x8E, CUR_LSB, 3, 1000);   // set INA233 MFR_CALIBRATION register for current measurement
    //HAL_I2C_Mem_Read(&hi2c2, 0x80, CUR_LSB[0], 1, CUR_LSB_check, 2, 100);
    return m;
}

void sens_V(uint8_t MM_address, float*  MM_nom_dec, float MM_nom_dec_prev)
{   
    int16_t MM_nom_16;
    uint8_t MM_nom [2];
    //----------------------------------------------------------------------
    // Шунт датчик №1------------------------------------------------------
    HAL_I2C_Master_Transmit(&hi2c1, MM_address, &MM_read_V, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c1, MM_address, MM_nom, 2, 1000);
    MM_nom_16= (MM_nom[1]<<8) | MM_nom[0] ;
    MM_nom_dec[0] = ((float)MM_nom_16 / 100.0f) / 8.0f;
    MM_nom_dec[1] = MM_nom_dec [0] - MM_nom_dec_prev;
}
//
float sens_I(uint8_t MM_address, float m)
{
    int16_t MM_nom_cur_16;
    uint8_t MM_nom_cur [2];
    float MM_nom_cur_dec;
    // измерение тока
    //HAL_I2C_Mem_Read(&hi2c2, MM_address, MM_read_shunt, 1, MM_shunt, 2, 100);
    //MM_shunt16= (MM_shunt[1]<<8) | MM_shunt[0] ;
		HAL_I2C_Master_Transmit(&hi2c1, MM_address, &MM_read_I, 1, 1000);
		HAL_I2C_Master_Receive(&hi2c1, MM_address, MM_nom_cur, 2, 1000);
    MM_nom_cur_16= ((int16_t)(MM_nom_cur[1]<<8)) | MM_nom_cur[0] ;
    MM_nom_cur_dec = ((float)MM_nom_cur_16 /*+ 600.00f*/) / m * (1);
    //
    //----------------------------------------------------------------------
    return MM_nom_cur_dec;
}

//void Buzzer(float time)
//{
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//	TIM2->CCR3 = 11500;
//	HAL_Delay(time);
//	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
//}//


