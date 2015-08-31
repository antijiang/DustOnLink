#include "DustSensor.h"

int main(void)
{
    DustSensor_Init();
    
    while(1)
    {
        DustSensor_Process();
			  	{
					//	extern volatile uint16_t CurrentConcentration_Smoke ;
					//			extern void SetDustIndicateLED(uint16_t);
						//		SetDustIndicateLED(CurrentConcentration_Smoke);
					}
    }
}


