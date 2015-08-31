#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "DustSensor.h"

#if 1
#define	LED_1	P07		//Ũ�ȵ�
#define	LED_2	P01
#define	LED_3	P00
#define	LED_4	P06   //Ũ�ȸ�ʱ�� �������˸

#else
#define	LED_1	P01		//Ũ�ȵ�
#define	LED_2	P07
#define	LED_3	P00
#define	LED_4	P06   //Ũ�ȸ�ʱ�� �������˸


#endif
#define	DUSTOUT_CTRL	P05

#ifdef  ADDR_USE_SWITCH

//��������Դ���ƣ�ͨ��IO��
#define	AUTOMODE_OFF	0    //һֱ��
#define	AUTOMODE_ON		1   //һֱ��
#define	AUTOMODE_AUTO	2   //��ģʽ���ݻҳ��yֵ�Զ������� 

uint16_t	Ind_DustP1=POINT_DUST_1,Ind_DustP2=POINT_DUST_2,Ind_DustP3=POINT_DUST_3,Ind_DustP4=POINT_DUST_3;
uint16_t	AutoOnDustPoint=DEFAULT_AUTOONLEVEL; //�Զ����ص��yֵ
uint16_t	AutoMode=AUTOMODE_AUTO;  //0:off 1:on  2:AUTO  ��������Դ����
extern uint64_t sys_timer;
void flashled4()
{
	static uint8_t led4on;
	//if(sys_timer%500==0)
	{
	if(led4on==1){led4on=0;LED_4=0;}
	else{		led4on=1;LED_4=1;	}
  }
}
void SetDustIndicateLED(uint16_t dustvalue)
{
	
	static int lastdustvalue=0;
	uint8_t dir;
	if(dustvalue>lastdustvalue)dir=1;
	else dir=0;
	lastdustvalue=dustvalue;
	LED_4=0;LED_3=0;LED_2=0;LED_1=0;
	
	//�˹�����
	if(AutoMode==AUTOMODE_ON)DUSTOUT_CTRL=1;
	else if(AutoMode==AUTOMODE_OFF) DUSTOUT_CTRL=0;
	if(dir) 
	{

		// ����
		if(dustvalue>Ind_DustP4){
		  //	flash led4 
			flashled4();
			
		}else
		
		if(dustvalue>Ind_DustP3){
		
		LED_4=1;
		
		
		}else if(dustvalue>Ind_DustP2){
		
		LED_3=1;
			
		}else if(dustvalue>Ind_DustP1){
		
		LED_2=1;
			
		}
		else{
			
			LED_1=1;
			
		}
		
		if(dustvalue>AutoOnDustPoint&&AutoMode==AUTOMODE_AUTO)
		{
			DUSTOUT_CTRL=1;
			
		}
	
  }
	else
	{
		
				if(dustvalue<POINT_DUST_1A){
					LED_1=1;
					
					
				}else if(dustvalue<POINT_DUST_2A){
				
				LED_2=1;
					
				
				}else if(dustvalue<POINT_DUST_3A){
				
				LED_3=1;
					
				}else if(dustvalue<POINT_DUST_4A){
				
				LED_4=1;
					
				}
				else{
					flashled4();
					
				}
				if((AutoMode==AUTOMODE_AUTO)&&(dustvalue<(AutoOnDustPoint-AutoOnDustPoint*10/100)))
				//	if((AutoMode==AUTOMODE_AUTO)&&(dustvalue<(AutoOnDustPoint)))
				{
				
					DUSTOUT_CTRL=0;
					
				
				}
			}
	
	
}


#endif
