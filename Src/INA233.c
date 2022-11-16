#define __INA233_
#include "INA233.h"
#include <math.h>

uint32_t defTimeout = 1000;
float result;
//static float INA233[8];
static struct Coefficients Coeffs;

float GetRealVoltage(uint16_t RawVoltage)
{	 
	result = (1 / Coeffs.mVBus) * (RawVoltage * pow(10, -Coeffs.rVBus) - Coeffs.bVBus);
	if (result == NULL) return 0.0f;
	else return result;
}
float GetRealCurrent(uint16_t RawCurrent)
{		
	return (1 / Coeffs.mCur) * (RawCurrent * pow(10, -Coeffs.rCur) - Coeffs.bCur);
}
float GetRealPower(uint16_t RawPower)
{		
	return (1 / Coeffs.mPwr) * (RawPower * pow(10, -Coeffs.rPwr) - Coeffs.bPwr);
}

//Calculate all coeffs and set Callibration reg in INA233
void INA233_Init(I2C_HandleTypeDef* ina233, uint16_t DeviceAddress)
{
	uint8_t transmition[5];
	float C_LSB = 0.0f, CAL = 0.0f, jank;
	
//calculate coeffs..
	Coeffs.mVShunt = m_vs;
	Coeffs.rVShunt = R_vs;
	Coeffs.bVShunt = b_vs;
	Coeffs.mVBus = m_vb;
	Coeffs.rVBus = R_vb;
	Coeffs.bVBus = b_vb;
	Coeffs.mCur = 0;
	Coeffs.rCur = 0;
	Coeffs.bCur = b_c;
	Coeffs.mPwr = 0;
	Coeffs.rPwr = 0;
	Coeffs.bPwr = b_p;

//Get calibration value	
//calculate current and power lest significant bit and over coefficients
  C_LSB = (INA233_DEFAULT_MaxCurrent) / pow(2, 15);
//  P_LSB = 25 * C_LSB;
	
//calculate CALIBRATION for INA233
	CAL = 0.00512f / (INA233_DEFAULT_RShunt * C_LSB);
	
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
	float check_mCur = 1 / C_LSB;
	Coeffs.mCur = (uint32_t) check_mCur;
	if((modff(check_mCur, &jank) != 0) & (Coeffs.mCur > 32768 & Coeffs.mCur*10 < 32768) || (Coeffs.mCur < -32768 & Coeffs.mCur/10 > -32768))
	{
		while((Coeffs.mCur > 32768 & Coeffs.mCur*10 < 32768) || (Coeffs.mCur < -32768 & Coeffs.mCur/10 > -32768))
		{
			if(Coeffs.mCur > 32768 || Coeffs.mCur/10 > -32768)
			{
				check_mCur /= 10;
				Coeffs.rCur ++;
				Coeffs.mCur = (uint32_t) check_mCur;
			}
			else if (Coeffs.mCur < -32768 || Coeffs.mCur*10 < 32768)
			{
				check_mCur *= 10;
				Coeffs.rCur --;
				Coeffs.mCur = (uint32_t)check_mCur;
			}
		}
	}
	float check_mPwr  = 1 / (C_LSB * 25);
	Coeffs.mPwr = (uint32_t) check_mPwr;
	if((modff(check_mPwr, &jank) != 0) & (Coeffs.mPwr > 32768 & Coeffs.mPwr*10 < 32768) || (Coeffs.mPwr < -32768 & Coeffs.mPwr/10 > -32768))
	{
		while((Coeffs.mPwr > 32768 & Coeffs.mPwr*10 < 32768) || (Coeffs.mPwr < -32768 & Coeffs.mPwr/10 > -32768))
		{
			if(Coeffs.mPwr > 32768 || Coeffs.mPwr/10 > -32768)
			{
				check_mPwr /= 10;
				Coeffs.rPwr ++;
				Coeffs.mPwr = (int32_t)Coeffs.mPwr;
			}
			else if (Coeffs.mPwr < -32768 || Coeffs.mPwr*10 < 32768)
			{
				check_mPwr *= 10;
				Coeffs.rPwr --;
				Coeffs.mPwr = (int32_t)Coeffs.mPwr;
			}
		}
	}
	
//convert Data to uint8 massive with calibration
	transmition[0] = INA233_CLEAR_FAULTS;
	transmition[1] = INA233_RESTORE_DEFAULT_ALL;
	transmition[2] = INA233_MFR_CALIBRATION;
	transmition[3] = (uint16_t) CAL;
	transmition[4] = (uint16_t) CAL >> 8;

//sending register + data itself
	HAL_I2C_Master_Transmit(ina233, (uint16_t)DeviceAddress, transmition, (uint16_t) sizeof(transmition), defTimeout);
}

//Get Voltage Value
float ReadInVoltage(I2C_HandleTypeDef* ina233, uint16_t DeviceAddress)
{
	uint8_t _8BitData[INA233_READ_VIN_SIZE];
	uint16_t _16BitData;
	
// Get Raw Value of Voltage
//sending reg were we want to read data
	HAL_I2C_Master_Transmit(ina233, (uint16_t)DeviceAddress, (uint8_t*) INA233_READ_VIN, 1, defTimeout);
	
//reading data itself
	HAL_I2C_Master_Receive(ina233, (uint16_t)DeviceAddress, _8BitData, INA233_READ_VIN_SIZE, defTimeout);
	
	_16BitData = (_8BitData[1] << 8) | _8BitData[0];
	return GetRealVoltage(_16BitData);
}
//Get Current Value
float ReadInCurrent(I2C_HandleTypeDef* ina233, uint16_t DeviceAddress)
{
	uint8_t _8BitData[INA233_READ_IN_SIZE];
	uint16_t _16BitData;
	
// Get Raw Value of Voltage
//sending reg were we want to read data
	HAL_I2C_Master_Transmit(ina233, (uint16_t)DeviceAddress, (uint8_t*) INA233_READ_IN, 1, defTimeout);	
	
//reading data itself
	HAL_I2C_Master_Receive(ina233, (uint16_t)DeviceAddress, _8BitData, INA233_READ_IN_SIZE, defTimeout);
	
	_16BitData = ((uint16_t)_8BitData[1] << 8) | _8BitData[0];
	return GetRealCurrent(_16BitData);
}
//Get Power Value
float ReadInPower(I2C_HandleTypeDef* ina233, uint16_t DeviceAddress)
{
	uint8_t _8BitData[INA233_READ_PIN_SIZE];
	uint16_t _16BitData;
	
// Get Raw Value of Voltage
//sending reg were we want to read data
	HAL_I2C_Master_Transmit(ina233,(uint16_t)DeviceAddress, (uint8_t*) INA233_READ_PIN, 1, defTimeout);	
	
//reading data itself
	HAL_I2C_Master_Receive(ina233, (uint16_t)DeviceAddress, _8BitData, INA233_READ_PIN_SIZE, defTimeout);
	
	_16BitData = ((uint16_t)_8BitData[1] << 8) | _8BitData[0];
	return GetRealPower(_16BitData);
}
