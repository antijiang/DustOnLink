#ifndef __STORE_H__
#define __STORE_H__

#include "Store.h"
#include "DustSensor.h"
#include "M051Series.h"

void StoreCleanVoltage(void)
{
	Flash_Write(STORE_OFFSET_CLEANVOLTAGE, CleanVoltage);
}

void ReadCleanVoltage(void)
{
	uint32_t dist = 0;
	if (Flash_Read(STORE_OFFSET_CLEANVOLTAGE, &dist))
	{
		if (dist == 0xffffffff)
		{
			CleanVoltage = 360;
		}
		else
			CleanVoltage = dist;
	}
	else
	{
		CleanVoltage = 360;
	}
}

void StoreSensitivity(void)
{
	uint32_t *p = (uint32_t*) (&Sensitivity);
	Flash_Write(STORE_OFFSET_SENSITIVITY, *p);
}

void ReadSensitivity(void)
{
	uint32_t dist = 0;
	if (Flash_Read(STORE_OFFSET_SENSITIVITY, &dist))
	{
		if (dist == 0xffffffff)
		{
			Sensitivity = 5;
		}
		else
			Sensitivity = *(float*) &dist;
	}
	else
	{
		Sensitivity = 5;
	}
}

void StoreCAndAD_A(void)
{
	Flash_Write(STORE_OFFSET_AD_A, Calibrate_A_AD);
	Flash_Write(STORE_OFFSET_C_A, Calibrate_A_Concentration);
}

void ReadCAndAD_A(void)
{
	uint32_t dist_AD, dist_C;
	if (Flash_Read(STORE_OFFSET_AD_A, &dist_AD))
	{
		Calibrate_A_AD = dist_AD;
	}
	else
	{
		Calibrate_A_AD = 0xffff;
	}

	if (Flash_Read(STORE_OFFSET_C_A, &dist_C))
	{
		Calibrate_A_Concentration = dist_C;
	}
	else
	{
		Calibrate_A_Concentration = 0xffff;
	}
}

void StoreCAndAD_B(void)
{
	Flash_Write(STORE_OFFSET_AD_B, Calibrate_B_AD);
	Flash_Write(STORE_OFFSET_C_B, Calibrate_B_Concentration);
}

void ReadCAndAD_B(void)
{
	uint32_t dist_AD, dist_C;
	if (Flash_Read(STORE_OFFSET_AD_B, &dist_AD))
	{
		Calibrate_B_AD = dist_AD;
	}
	else
	{
		Calibrate_B_AD = 0xffff;
	}

	if (Flash_Read(STORE_OFFSET_C_B, &dist_C))
	{
		Calibrate_B_Concentration = dist_C;
	}
	else
	{
		Calibrate_B_Concentration = 0xffff;
	}
}

void StoreDustN(void)
{
	uint32_t *p = (uint32_t*) (&Dust_Calc_N);
	Flash_Write(STORE_OFFSET_DUST_N, *p);
}

void ReadDustN(void)
{
	uint32_t dist = 0;
	if (Flash_Read(STORE_OFFSET_DUST_N, &dist))
	{
		if (dist == 0xffffffff)
		{
			Dust_Calc_N = 10;
		}
		else
			Dust_Calc_N = *(float*) &dist;
	}
	else
	{
		Dust_Calc_N = 10;
	}
}

void StoreKn1(void)
{
	uint32_t *p = (uint32_t*) (&Kn1);
	Flash_Write(STORE_OFFSET_KN1, *p);
}

void ReadKn1(void)
{
	uint32_t dist = 0;
	if (Flash_Read(STORE_OFFSET_KN1, &dist))
	{
		if (dist == 0xffffffff)
		{
			Kn1 = 1;
		}
		else
			Kn1 = *(float*) &dist;
	}
	else
	{
		Kn1 = 1;
	}
}

void StoreKn2(void)
{
	uint32_t *p = (uint32_t*) (&Kn2);
	Flash_Write(STORE_OFFSET_KN2, *p);
}

void ReadKn2(void)
{
	uint32_t dist = 0;
	if (Flash_Read(STORE_OFFSET_KN2, &dist))
	{
		if (dist == 0xffffffff)
		{
			Kn2 = 1;
		}
		else
			Kn2 = *(float*) &dist;
	}
	else
	{
		Kn2 = 1;
	}
}

#endif
