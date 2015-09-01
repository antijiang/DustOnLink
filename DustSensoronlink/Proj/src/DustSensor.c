#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "DustSensor.h"
#include "Flash.h"
#include "SerialTransfer.h"

#ifndef BOARDTYPE_MINI
#include "LCD_Driver.h"
#endif
//#define	DEBUG_DUST
#ifdef	DEBUG_DUST
#define	DPRINTF(a) printf a
#else
#define	DPRINTF(a)
#endif

#ifdef  ADDR_USE_SWITCH
uint32_t PulseCNT = 0;
#endif
#define	MIN_UPDATE_VDD 4700
#define	MAX_UPDATE_VDD 5300
volatile uint8_t Sec_ReadCount = 0, Min_ReadCount = 0, Update_ReadCount = 0,
    Dust_N_ReadCount = 0, Total_ReadCount = 0;
volatile uint16_t Sec_Data[DATA_SIZE], Min_Data[60],
    Update_Data[UPDATE_CYCLIC_COUNT], Dust_N_Data[CYCLIC_TIME];
#ifdef	REF25_VDD
uint16_t Vdd_Data[DATA_SIZE];
uint8_t Vdd_ReadCount = 0;
uint16_t Vdd_Aver1S;		//电源电压数字采样值
uint16_t VddRel_MV = 0;  //转换为真实电压值，单位mv
#endif

#ifdef	TEMP_DETECT
uint16_t Vtemp_Data[DATA_SIZE];
uint8_t Vtemp_ReadCount = 0;
uint16_t Vtemp_Aver1S;		//电源电压数字采样值
uint16_t gTemp0, gTempn;			//温度AD采样
uint16_t gTemp0V, gTempnV;		//对应温度的灰尘电压

#endif
// 用于更新无尘电压
volatile uint16_t WorkingTime = 0;
volatile uint16_t TurnUpCleanVolConfirmTimes = 0;
volatile uint16_t First_Value = 0;
volatile uint16_t Last_Value = 0;  // 最后的值
volatile uint8_t Readed_Machine_State = 0x05;  // 5: 连接错误 0：关闭 1：打开
volatile _Bool Last_Machine_State = FALSE;
volatile _Bool Current_Machine_State = FALSE;
volatile _Bool First_Start = FALSE;
volatile _Bool AutoOutPutCurrentSmoke = FALSE;

volatile uint8_t ReceiveStarted = FALSE, CommandReceived = FALSE;
volatile uint8_t Receive_Seek = 0;
volatile uint8_t SerialBuffer[SERIAL_BUFF_SIZE];
char ReadBuffer[SERIAL_BUFF_SIZE];
volatile uint8_t Gear = 0; // 默认0挡
volatile uint16_t CurrentConcentration_Smoke = 0;
volatile uint16_t CurrentConcentration_Dust = 0;
volatile uint16_t CyclicAver = 0;
uint16_t CyclicAveraftercalibrate; //温度补偿后的电压
volatile uint32_t DustFiveSecAver = 0;
volatile uint16_t TurnDownCleanVolConfirmTimes = 0;

volatile uint32_t Address_Count = 0;
volatile uint8_t Address = 33;
volatile uint16_t VB1;
volatile uint16_t VB2;

volatile uint16_t VB1Temp;
volatile uint16_t VB2Temp;
volatile uint16_t CleanVoltageTemp;

volatile enum UART_OUTPUT_MOD CurrentUARTMod = UARTMOD_DEBUG;

volatile uint16_t CleanVoltage;
volatile float Sensitivity;
volatile float Dust_Calc_N;
volatile uint16_t Calibrate_A_AD;  //电压采样值
volatile uint16_t Calibrate_A_Concentration; // 浓度值ug/m3
volatile uint16_t Calibrate_B_AD;
volatile uint16_t Calibrate_B_Concentration;
volatile float Kn1;
volatile float Kn2;

uint8_t DustPortOrUartMode = 0;  //工作在读取端口高低判定除尘器是否工作 0：读端口  1：读串口
extern uint16_t Ind_DustP1, Ind_DustP2, Ind_DustP3, Ind_DustP4;
extern uint16_t AutoOnDustPoint;
extern uint16_t AutoMode;

volatile uint16_t Calibrate_N_Concentration;

/* 对16位数据根据提供的参数进行循环滤波 */
uint16_t
Cyclic_Aver(uint16_t* data, uint8_t data_length, uint8_t startpos,
    uint8_t cyclic_length, volatile uint8_t* p_totalCnt);

/* 计算尘的浓度 参数为: 所有尘数据, 尘数据大小, 平均值 */
uint32_t
Calc_Dust_Aver(uint16_t* dustData, uint8_t length, uint16_t secAver);

/* 计算两条直线的K与b值 */
void
Calc_Virtual_KandB(void);

//计算缓冲区平均值
uint32_t
Util_CalcAvg(uint16_t *buf, uint8_t length)
{
  uint8_t i;
  uint32_t Vavg = 0;
  for (i = 0; i < length; i++)
    {
      Vavg += *buf;
      buf++;
    }
  Vavg = Vavg / length;
  return Vavg;
}

//===============================================================================
//温度校准后的采样准
uint16_t
CalTempCalibrate(void)
{
  float_t k, deltVcal;
  uint16_t tmpcalib;

  tmpcalib = CyclicAver;
  if (gTempn == gTemp0)
    return tmpcalib;

  k = ((float) gTempnV - gTemp0V) / ((float) gTempn - gTemp0);

  if (gTempnV == 0xffff || gTemp0V == 0xffff || gTempn == 0xffff
      || gTemp0 == 0xffff)
    return tmpcalib;
  deltVcal = -k * (Vtemp_Aver1S - gTemp0) + VB1;
  if (deltVcal < 0)
    deltVcal = 0;
  VB1Temp = (uint16_t) deltVcal;

  deltVcal = -k * (Vtemp_Aver1S - gTemp0) + VB2;
  if (deltVcal < 0)
    deltVcal = 0;
  VB2Temp = (uint16_t) deltVcal;

  deltVcal = -k * ((float) Vtemp_Aver1S - gTemp0) + CyclicAver;
  if (deltVcal < 0)
    deltVcal = 0;
  tmpcalib = (uint16_t) deltVcal;

  return tmpcalib;

}
void
StoreCleanVoltage(void)
{
  Flash_Write(STORE_OFFSET_CLEANVOLTAGE, CleanVoltage);
}

void
ReadCleanVoltage(void)
{
  uint32_t dist = 0;
  if (Flash_Read(STORE_OFFSET_CLEANVOLTAGE, &dist))
    {
      if (dist == 0xffffffff)
        {
          CleanVoltage = CLEANVOLTAGE;
        }
      else
        CleanVoltage = dist;
    }
  else
    {
      CleanVoltage = CLEANVOLTAGE;
    }
}

void
StoreSensitivity(void)
{
  uint32_t *p = (uint32_t*) (&Sensitivity);
  Flash_Write(STORE_OFFSET_SENSITIVITY, *p);
}

void
ReadSensitivity(void)
{
  uint32_t dist = 0;
  if (Flash_Read(STORE_OFFSET_SENSITIVITY, &dist))
    {
      if (dist == 0xffffffff)
        {
          Sensitivity = SENSITIVITY;
        }
      else
        Sensitivity = *(float*) &dist;
    }
  else
    {
      Sensitivity = SENSITIVITY;
    }
}

void
StoreCAndAD_A(void)
{
  Flash_Write(STORE_OFFSET_AD_A, Calibrate_A_AD);
  Flash_Write(STORE_OFFSET_C_A, Calibrate_A_Concentration);
}

void
ReadCAndAD_A(void)
{
  uint32_t dist_AD, dist_C;
  if (Flash_Read(STORE_OFFSET_AD_A, &dist_AD))
    {
      Calibrate_A_AD = dist_AD;
    }
  else
    {
      Calibrate_A_AD = CALIBRATE_A_AD;
    }

  if (Flash_Read(STORE_OFFSET_C_A, &dist_C))
    {
      Calibrate_A_Concentration = dist_C;
    }
  else
    {
      Calibrate_A_Concentration = CALIBRATE_A_CONCENTRATION;
    }
}

void
StoreCAndAD_B(void)
{
  Flash_Write(STORE_OFFSET_AD_B, Calibrate_B_AD);
  Flash_Write(STORE_OFFSET_C_B, Calibrate_B_Concentration);
}

void
ReadCAndAD_B(void)
{
  uint32_t dist_AD, dist_C;
  if (Flash_Read(STORE_OFFSET_AD_B, &dist_AD))
    {
      Calibrate_B_AD = dist_AD;
    }
  else
    {
      Calibrate_B_AD = CALIBRATE_B_AD;
    }

  if (Flash_Read(STORE_OFFSET_C_B, &dist_C))
    {
      Calibrate_B_Concentration = dist_C;
    }
  else
    {
      Calibrate_B_Concentration = CALIBRATE_B_CONCENTRATION;
    }
}

void
StoreDustN(void)
{
  uint32_t *p = (uint32_t*) (&Dust_Calc_N);
  Flash_Write(STORE_OFFSET_DUST_N, *p);
}

void
ReadDustN(void)
{
  uint32_t dist = 0;
  if (Flash_Read(STORE_OFFSET_DUST_N, &dist))
    {
      if (dist == 0xffffffff)
        {
          Dust_Calc_N = DUST_CALC_N;
        }
      else
        Dust_Calc_N = *(float*) &dist;
    }
  else
    {
      Dust_Calc_N = DUST_CALC_N;
    }
}

void
StoreKn1(void)
{
  uint32_t *p = (uint32_t*) (&Kn1);
  Flash_Write(STORE_OFFSET_KN1, *p);
}

void
ReadKn1(void)
{
  uint32_t dist = 0;
  if (Flash_Read(STORE_OFFSET_KN1, &dist))
    {
      if (dist == 0xffffffff)
        {
          Kn1 = KN1;
        }
      else
        Kn1 = *(float*) &dist;
    }
  else
    {
      Kn1 = KN1;
    }
}

void
StoreKn2(void)
{
  uint32_t *p = (uint32_t*) (&Kn2);
  Flash_Write(STORE_OFFSET_KN2, *p);
}

void
ReadKn2(void)
{
  uint32_t dist = 0;
  if (Flash_Read(STORE_OFFSET_KN2, &dist))
    {
      if (dist == 0xffffffff)
        {
          Kn2 = KN2;
        }
      else
        Kn2 = *(float*) &dist;
    }
  else
    {
      Kn2 = KN2;
    }
}

void
StoreAutoOnLevel(void)
{

  Flash_Write(STORE_OFFSET_AUTOONLEVEL, AutoOnDustPoint);
}

void
ReadAutoOnlevel(void)
{
  uint32_t dist = 0;
  if (Flash_Read(STORE_OFFSET_AUTOONLEVEL, &dist))
    {
      if (dist == 0xffffffff)
        {
          AutoOnDustPoint = DEFAULT_AUTOONLEVEL;
        }
      else
        AutoOnDustPoint = dist;
    }
  else
    {
      AutoOnDustPoint = DEFAULT_AUTOONLEVEL;
    }
}
void
StoreDispIND(void)
{

  Flash_Write(STORE_OFFSET_DUSTINDICATE1, Ind_DustP1);
  Flash_Write(STORE_OFFSET_DUSTINDICATE2, Ind_DustP2);
  Flash_Write(STORE_OFFSET_DUSTINDICATE3, Ind_DustP3);
  Flash_Write(STORE_OFFSET_DUSTINDICATE4, Ind_DustP4);
}

void
ReadDispIND(void)
{
  uint32_t dist = 0;
  if (Flash_Read(STORE_OFFSET_DUSTINDICATE1, &dist))
    {
      if (dist == 0xffffffff)
        {
          Ind_DustP1 = POINT_DUST_1;
        }
      else
        Ind_DustP1 = dist;
    }
  else
    {
      Ind_DustP1 = POINT_DUST_1;
    }
  if (Flash_Read(STORE_OFFSET_DUSTINDICATE2, &dist))
    {
      if (dist == 0xffffffff)
        {
          Ind_DustP2 = POINT_DUST_2;
        }
      else
        Ind_DustP2 = dist;
    }
  else
    {
      Ind_DustP2 = POINT_DUST_2;
    }
  if (Flash_Read(STORE_OFFSET_DUSTINDICATE3, &dist))
    {
      if (dist == 0xffffffff)
        {
          Ind_DustP3 = POINT_DUST_3;
        }
      else
        Ind_DustP3 = dist;
    }
  else
    {
      Ind_DustP3 = POINT_DUST_3;
    }
  if (Flash_Read(STORE_OFFSET_DUSTINDICATE4, &dist))
    {
      if (dist == 0xffffffff)
        {
          Ind_DustP4 = POINT_DUST_4;
        }
      else
        Ind_DustP4 = dist;
    }
  else
    {
      Ind_DustP4 = POINT_DUST_4;
    }
}

#ifdef	TEMP_DETECT
void
StoreTemp(void)
{

  Flash_Write(STORE_OFFSET_TEMP0, gTemp0);
  Flash_Write(STORE_OFFSET_TEMPN, gTempn);
  Flash_Write(STORE_OFFSET_TEMP0_V, gTemp0V);
  Flash_Write(STORE_OFFSET_TEMPN_V, gTempnV);
}

void
ReadTemp(void)
{
  uint32_t dist = 0;
  if (Flash_Read(STORE_OFFSET_TEMP0, &dist))
    {

      gTemp0 = dist;
    }
  else
    {
      gTemp0 = 0xffff;
    }

  if (Flash_Read(STORE_OFFSET_TEMPN, &dist))
    {

      gTempn = dist;
    }
  else
    {
      gTempn = 0xffff;
    }

  if (Flash_Read(STORE_OFFSET_TEMP0_V, &dist))
    {

      gTemp0V = dist;
    }
  else
    {
      gTemp0V = 0xffff;
    }

  if (Flash_Read(STORE_OFFSET_TEMPN_V, &dist))
    {

      gTempnV = dist;
    }
  else
    {
      gTempnV = 0xffff;
    }
}

#endif
void
SYS_Init(void)
{
  SYS_UnlockReg();

  /* Enable Internal RC clock */SYSCLK ->PWRCON |= SYSCLK_PWRCON_IRC22M_EN_Msk;
  /* Waiting for IRC22M clock ready */
  SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_IRC22M_STB_Msk);
  /* Switch HCLK clock source to internal RC */SYSCLK ->CLKSEL0 =
      SYSCLK_CLKSEL0_HCLK_IRC22M;
  /* Enable PLL and Set PLL frequency */SYSCLK ->PLLCON =
      SYSCLK_PLLCON_50MHz_IRC22M;

  /* Waiting for clock ready */
  SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_PLL_STB_Msk);

  /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */SYSCLK ->CLKSEL0 =
      SYSCLK_CLKSEL0_STCLK_HCLK_DIV2 | SYSCLK_CLKSEL0_HCLK_PLL;

  /* Enable IP clock */SYSCLK ->APBCLK = SYSCLK_APBCLK_UART0_EN_Msk | /*SYSCLK_APBCLK_SPI0_EN_Msk |*/
  SYSCLK_APBCLK_PWM23_EN_Msk | SYSCLK_APBCLK_PWM45_EN_Msk
      | SYSCLK_APBCLK_ADC_EN_Msk | SYSCLK_APBCLK_TMR0_EN_Msk;
  /* IP clock source */SYSCLK ->CLKSEL1 = SYSCLK_CLKSEL1_UART_IRC22M
      | SYSCLK_CLKSEL1_PWM23_IRC22M //|SYSCLK_CLKSEL1_PWM01_IRC22M
      | SYSCLK_CLKSEL1_ADC_IRC22M | SYSCLK_CLKSEL1_TMR0_IRC22M;
  /* IP clock2 source */SYSCLK ->CLKSEL2 = SYSCLK_CLKSEL2_PWM45_IRC22M
      | SYSCLK_CLKSEL2_PWM67_IRC22M;

  /*---------------------------------------------------------------------------------------------------------*/
  /* Init I/O Multi-function                                                                                 */
  /*---------------------------------------------------------------------------------------------------------*/
  /* Set P3 multi-function pins for UART0 RXD and TXD  */
#ifndef BOARDTYPE_MINI
  SYS->P1_MFP |= SYS_MFP_P10_AIN0 | SYS_MFP_P17_AIN7;
#else
  SYS ->P1_MFP |= SYS_MFP_P10_AIN0 | SYS_MFP_P13_AIN3 | SYS_MFP_P14_AIN4
      | SYS_MFP_P17_AIN7;
#endif
  SYS ->P2_MFP |= SYS_MFP_P22_PWM2 | SYS_MFP_P24_PWM4 | SYS_MFP_P26_PWM6; //jxd
  SYS ->P3_MFP |= SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0 | SYS_MFP_P34_T0;
  /* Lock protected registers */
  SYS_LockReg();
}
#ifdef  ADDR_USE_SWITCH

void
GPIOP0P1_IRQHandler(void)
{

  if (P0 ->ISRC & (1 << 7))
    {
      /* Re-enable debounce function */
      P0 ->DBEN |= GPIO_DBEN_ENABLE(7);
      /* Clear the interrupt P07 */P0 ->ISRC = 1 << 7;

      // printf("P1.3 Interrupt!\n");

      if (PulseCNT != 0xffffffff)
        PulseCNT++;
    }

}
#endif
void
GPIOP2P3P4_IRQHandler(void)
{
  /* Re-enable debounce function */
  // P4->DBEN |= GPIO_DBEN_ENABLE(5);
  /* Clear the interrupt */
//	P4->ISRC = 1 << 5;
}

void
EINT0_IRQHandler(void)
{
  /* Re-enable debounce function */
//   P3->DBEN |= GPIO_DBEN_ENABLE(2);
//   P3->ISRC = 1 << 2;
}

void
EINT1_IRQHandler(void)
{
  /* Re-enable debounce function */
  // P3->DBEN |= GPIO_DBEN_ENABLE(3);
  //  P3->ISRC = 1 << 3;
}

void
UART0_IRQHandler(void)
{
  /* 当前接收到的字节 */
  uint8_t rcvByte;
  rcvByte = UART0 ->DATA;

  /* 清除中断标志 */UART0 ->ISR |= UART_ISR_RDA_INT_Msk;
  if (CurrentUARTMod == UARTMOD_DEBUG || CurrentUARTMod == UARTMOD_CAL)
    {
      /* 已经接收到起始字节 */
      if (ReceiveStarted)
        {
          /* 没有超过缓冲区大小 */
          if (Receive_Seek < SERIAL_BUFF_SIZE)
            {
              SerialBuffer[Receive_Seek++] = rcvByte;
              /* 接收到结束字节 */
              if (rcvByte == END_BYTE)
                {
                  Receive_Seek = 0;
                  ReceiveStarted = FALSE;
                  /* 置接收标志位 */
                  CommandReceived = TRUE;
                }
            }
          else // 超出缓冲区大小
            {
              /* 丢弃已接收的数据 */
              Receive_Seek = 0;
              ReceiveStarted = FALSE;
            }
        }
      else // 还未接收到起始字节
        {
          /* 接收到起始字节 */
          if (rcvByte == START_BYTE)
            {
              Receive_Seek = 0;
              /* 开始接收 */
              SerialBuffer[Receive_Seek++] = rcvByte;
              ReceiveStarted = TRUE;
            }
        }
    }

  //{STX】【S】【N】【N】【N】【X】【X】【ETX】
  else if (CurrentUARTMod == UARTMOD_DATA_LINK)
    {
      /* 已经接收到起始字节 */
      if (ReceiveStarted)
        {
          /* 没有超过缓冲区大小 */
          if (Receive_Seek < SERIAL_BUFF_SIZE)
            {
              SerialBuffer[Receive_Seek++] = rcvByte;
              /* 接收到结束字节 */
              if (rcvByte == CR_BYTE)
                {
                  Receive_Seek = 0;
                  ReceiveStarted = FALSE;
                  /* 置接收标志位 */
                  CommandReceived = TRUE;

                }
            }
          else // 超出缓冲区大小
            {
              /* 丢弃已接收的数据 */
              Receive_Seek = 0;
              ReceiveStarted = FALSE;
            }
        }
      else // 还未接收到起始字节
        {
          /* 接收到起始字节 */
          if (rcvByte == STX_BYTE)
            {
              Receive_Seek = 0;
              /* 开始接收 */
              SerialBuffer[Receive_Seek++] = rcvByte;
              ReceiveStarted = TRUE;
            }
        }
    }
    {

      if (CommandReceived)
        memcpy((char *) &ReadBuffer, (char *) &SerialBuffer, SERIAL_BUFF_SIZE);

    }
}

void
UART0_Init(void)
{
  UART0 ->BAUD =
      UART_BAUD_MODE2 | UART_BAUD_DIV_MODE2(__IRC22M, UART_BAUD_RATE);
  _UART_SET_DATA_FORMAT(UART0,
      UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1);

  /* 打开UART接收中断 */
  _UART_ENABLE_INT(UART0, UART_IER_RDA_IEN_Msk);
  NVIC_EnableIRQ(UART0_IRQn);
}

void
PWMA_IRQHandler(void)
{
  _PWM_CLEAR_TIMER_PERIOD_INT_FLAG(PWMA, PWM_CH2);
  StartTMR();
}
#ifdef	TEMP_DETECT
void
ADC_CompleteTemp(void)
{
  /* 获取ADC转换结果 */
  uint16_t value = (ADC ->ADDR[TEMP_ADC_CHANEL] & 0xFFFUL);
  Vtemp_Data[Vtemp_ReadCount++] = value;

  // 清除转换完成标志
  _ADC_CLEAR_ADC_INT_FLAG();
  if (Vtemp_ReadCount >= DATA_SIZE)
    {
      Vtemp_ReadCount = 0;
    }

}
#endif
#ifdef	REF25_VDD
void
ADC_Complete25(void)
{
  /* 获取ADC转换结果 */
  uint16_t value = (ADC ->ADDR[REF25_ADC_CHANEL] & 0xFFFUL);
  Vdd_Data[Vdd_ReadCount++] = value;
  // 清除转换完成标志
  _ADC_CLEAR_ADC_INT_FLAG();
  if (Vdd_ReadCount >= DATA_SIZE)
    {
      Vdd_ReadCount = 0;
    }

}
#endif
#ifdef	ADDR_USE_SWITCH
void
PWMB_IRQHandler(void)
{
  _PWM_CLEAR_TIMER_PERIOD_INT_FLAG(PWMB, PWM_CH0);
  StartTMR();
#ifdef	REF25_VDD
  StartADC(REF25_ADC_CHANEL, 0);
  ADC_Complete25();

  ADC ->ADCHER = 0x1 << SENSOR_ADC_CHANEL; //稳定 ，避免延时，先切换到其他通道 灰尘

#endif

#ifdef	TEMP_DETECT
  ADC ->ADCHER = 0x200 | (0x1 << TEMP_ADC_CHANEL); //
  StartADC(TEMP_ADC_CHANEL, 0x200);
  ADC_CompleteTemp();

  ADC ->ADCHER = 0x1 << SENSOR_ADC_CHANEL; //稳定 ，避免延时，先切换到其他通道 灰尘
#endif

}
#endif
#ifdef	ADDR_USE_SWITCH
void
PWMLEDO_init(void) //使用PWMB中断
{
  _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMB, PWM_CH0);
  _PWM_SET_TIMER_PRESCALE(PWMB, PWM_CH0, 26); // Divided by 27
  _PWM_SET_TIMER_CLOCK_DIV(PWMB, PWM_CH0, PWM_CSR_DIV2); // __IRC22M / (27 * 2) = 409600

  //PWM Freq = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1));
  PWMB ->CNR0 = __IRC22M / (27 * 2) / PWM_FREQ - 1;
  // 设置PWM占空比
  PWMG_SHIFT(0);

  /* Enable PWM Output pin */
  _PWM_ENABLE_PWM_OUT(PWMB, PWM_CH0);

  /* Enable Timer period Interrupt */
  //_PWM_ENABLE_TIMER_PERIOD_INT(PWMA, PWM_CH2);
  NVIC_EnableIRQ((IRQn_Type) (PWMB_IRQn));

  /* Enable PWM2 Timer */
  _PWM_ENABLE_TIMER(PWMB, PWM_CH0);
}

/* 档位切换. pwm占空比 = 0.32 / 2 ^ 档位
 Todo: 自动计算定时器延时 */
void
PWMG_SHIFT(uint8_t gear)
{
  Gear = gear;
  PWMB ->CMR0 = (PWMB ->CNR0 + 1) * PWM_DEFAULT_DUTY_RATIO / pow(2, gear) - 1;

  AdaptTMRDelay();
}

void
Start_PWMG_INT(void)
{
  /* 打开PWM中断 */
  _PWM_ENABLE_TIMER_PERIOD_INT(PWMB, PWM_CH0);
}

void
Stop_PWMG_INT(void)
{
  /* 关闭PWM中断 */
  _PWM_DISABLE_TIMER_PERIOD_INT(PWMB, PWM_CH0);
}

// PWM_OUTPUT 频率大概在44K左右
void
PWMOV_init(void)
{
  _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMB, PWM_CH2);
  _PWM_SET_TIMER_PRESCALE(PWMB, PWM_CH2, 1); // Divided by 1
  _PWM_SET_TIMER_CLOCK_DIV(PWMB, PWM_CH2, PWM_CSR_DIV1); // __IRC22M / (1) = 22118400

  //PWM Freq = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1));
  PWMB ->CNR2 = PWM_OUTPUT_PRECISION - 1;
  //PWMB->CMR0 = 100;

  /* Enable PWM Output pin */
  _PWM_ENABLE_PWM_OUT(PWMB, PWM_CH2);

  /* Enable PWM4 Timer */
  _PWM_ENABLE_TIMER(PWMB, PWM_CH2);
}

void
SetOV_Enable(uint16_t value)
{
  if (CurrentConcentration_Smoke == 0)
    {
      //    _PWM_DISABLE_PWM_OUT(PWMB, PWM_CH3); jxd
    }
  else
    {
      uint16_t outValue =
          value > (SENSOR_OUTPUT_RANGE) ? SENSOR_OUTPUT_RANGE : value;
      PWMB ->CMR2 = (PWMB ->CNR2 + 1) * outValue / SENSOR_OUTPUT_RANGE - 1;
      _PWM_ENABLE_PWM_OUT(PWMB, PWM_CH2);
    }
}
#else
void PWMLEDO_init(void)
  {
    _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMA, PWM_CH2);
    _PWM_SET_TIMER_PRESCALE(PWMA, PWM_CH2, 26); // Divided by 27
    _PWM_SET_TIMER_CLOCK_DIV(PWMA, PWM_CH2, PWM_CSR_DIV2);// __IRC22M / (27 * 2) = 409600

    //PWM Freq = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1));
    PWMA->CNR2 = __IRC22M / (27 * 2) / PWM_FREQ - 1;
    // 设置PWM占空比
    PWMG_SHIFT(0);

    /* Enable PWM Output pin */
    _PWM_ENABLE_PWM_OUT(PWMA, PWM_CH2);

    /* Enable Timer period Interrupt */
    //_PWM_ENABLE_TIMER_PERIOD_INT(PWMA, PWM_CH2);
    NVIC_EnableIRQ((IRQn_Type)(PWMA_IRQn));

    /* Enable PWM2 Timer */
    _PWM_ENABLE_TIMER(PWMA, PWM_CH2);
  }

/* 档位切换. pwm占空比 = 0.32 / 2 ^ 档位
 Todo: 自动计算定时器延时 */
void PWMG_SHIFT(uint8_t gear)
  {
    Gear = gear;
    PWMA->CMR2 = (PWMA->CNR2 + 1) * DEFAULT_DUTY_RATIO / pow(2, gear) - 1;

    AdaptTMRDelay();
  }

void Start_PWMG_INT(void)
  {
    /* 打开PWM中断 */
    _PWM_ENABLE_TIMER_PERIOD_INT(PWMA, PWM_CH2);
  }

void Stop_PWMG_INT(void)
  {
    /* 关闭PWM中断 */
    _PWM_DISABLE_TIMER_PERIOD_INT(PWMA, PWM_CH2);
  }

// PWM_OUTPUT 频率大概在44K左右
void PWMOV_init(void)
  {
    _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMB, PWM_CH0);
    _PWM_SET_TIMER_PRESCALE(PWMB, PWM_CH0, 1); // Divided by 1
    _PWM_SET_TIMER_CLOCK_DIV(PWMB, PWM_CH0, PWM_CSR_DIV1);// __IRC22M / (1) = 22118400

    //PWM Freq = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1));
    PWMB->CNR0 = PWM_OUTPUT_PRECISION - 1;
    //PWMB->CMR0 = 100;

    /* Enable PWM Output pin */
    _PWM_ENABLE_PWM_OUT(PWMB, PWM_CH0);

    /* Enable PWM4 Timer */
    _PWM_ENABLE_TIMER(PWMB, PWM_CH0);
  }

void SetOV_Enable(uint16_t value)
  {
    if (CurrentConcentration_Smoke == 0)
      {
        _PWM_DISABLE_PWM_OUT(PWMB, PWM_CH0);
      }
    else
      {
        uint16_t outValue = value > (SENSOR_OUTPUT_RANGE) ? SENSOR_OUTPUT_RANGE : value;
        PWMB->CMR0 = (PWMB->CNR0 + 1) * outValue / SENSOR_OUTPUT_RANGE - 1;
        _PWM_ENABLE_PWM_OUT(PWMB, PWM_CH0);
      }
  }

#endif
void
ADC_Auto_Cal(void)
{
  ADC ->ADCALR = ADC_ADCALR_CALEN_Msk;
  while (!(ADC ->ADCALR & ADC_ADCALR_CALDONE_Msk))
    ;
}

void
ADC_Complete(void)
{
  uint16_t value;
  static int firstADPWM = 1;  //第一次有可能开中断的时间晚了，导致立马中断，采样位置错误
  if (firstADPWM)
    {
      firstADPWM = 0;
      return;
    }
  /* 获取ADC转换结果 */
  value = (ADC ->ADDR[SENSOR_ADC_CHANEL] & 0xFFFUL);
  Sec_Data[Sec_ReadCount++] = value;
  // 清除转换完成标志
  _ADC_CLEAR_ADC_INT_FLAG();

  /* 超出缓冲区 */
  if (Sec_ReadCount >= DATA_SIZE)
    {
      /* 暂时关闭PWM中断(数据采集)进行复杂处理 */
      Stop_PWMG_INT();
      firstADPWM = 1;
    }

  //    if(CurrentUARTMod == UARTMOD_DATA_AD)
  //    {
  //        _UART_SENDBYTE(UART0, value>>8);
  //        _UART_SENDBYTE(UART0, value);
  //    }
}

void
ADC_Init(void)
{
  /* 设置ADC转换通道 */
  ADC ->ADCHER = 0x1 << SENSOR_ADC_CHANEL;
}

void
StartADC(uint8_t adc_channel, uint16_t hbit)
{
  ADC ->ADCHER = (0x1 << adc_channel) | hbit; //jxd
  /* 单次转换 */ADC ->ADCR = ADC_ADCR_ADEN_Msk | ADC_ADCR_ADST_Msk;
  /* 等待转换完成 */
  while (ADC ->ADCR & ADC_ADCR_ADST_Msk)
    ;

}
uint64_t sys_timer = 0;
void
TMR0_IRQHandler(void)
{
  /* 清除中断标志 */
  _TIMER_CLEAR_CMP_INT_FLAG(TIMER0);
  /* 启动AD转换 */
  StartADC(SENSOR_ADC_CHANEL, 0);

  ADC_Complete();
#ifdef	REF25_VDD
  ADC ->ADCHER = //0x200|
      (0x1 << REF25_ADC_CHANEL); //稳定 ，避免延时,提早切换,先切换到其他通道

#endif
  sys_timer++;

}

void
TMR_Init(void)
{
  /* Reset and stop TIMER0 counting first */
  _TIMER_RESET(TIMER0);

  /* Enable TIMER0 NVIC */
  NVIC_EnableIRQ(TMR0_IRQn);

  /*                                    |
   *           +--+                   +-|+
   *           |  | 中断              | ||
   *           |  |                   | ||
   *     ------+  +-------------------+ |+--------
   *              |---------9.96ms------|  采样
   *              |---------10.0ms-------| 周期
   *
   * 设置延时0.996ms( = 10ms - 0.32ms + 0.28ms)
   */
  //AdaptTMRDelay();
}

void
AdaptTMRDelay(void)
{
  // 停止定时器, 丢弃当前值.
  _TIMER_RESET(TIMER0);
  TIMER0 ->TCMPR = __IRC22M / 1000.0
      * (10 * (1 - PWM_DEFAULT_DUTY_RATIO / (pow(2, Gear)))
          + TMR_DEFAULT_DELAY_TIME / (pow(2, Gear))) /*ms*/;
}

void
StartTMR(void)
{
  /* Start TIMER0 counting */
  TIMER0 ->TCSR =
      TIMER_TCSR_CEN_Msk | TIMER_TCSR_IE_Msk | TIMER_TCSR_MODE_ONESHOT
          | TIMER_TCSR_TDR_EN_Msk | TIMER_TCSR_PRESCALE(1);
}

void
SendCalInfoTemp()
{

#ifdef	TEMP_DETECT
  //temp0
  _UART_SENDBYTE(UART0, gTemp0 >> 8);
  _UART_SENDBYTE(UART0, gTemp0);
  //temp1
  _UART_SENDBYTE(UART0, gTempn >> 8);
  _UART_SENDBYTE(UART0, gTempn);

  //temp0V
  _UART_SENDBYTE(UART0, gTemp0V >> 8);
  _UART_SENDBYTE(UART0, gTemp0V);

  //tempNV
  _UART_SENDBYTE(UART0, gTempnV >> 8);
  _UART_SENDBYTE(UART0, gTempnV);

#endif

}

void
SendCalInfo(void)
{
  //K:
  uint32_t* sens = (uint32_t*) &Sensitivity;
  uint32_t* d_N = (uint32_t*) &Dust_Calc_N;
  uint32_t* d_kn1 = (uint32_t*) &Kn1;
  uint32_t* d_kn2 = (uint32_t*) &Kn2;

  _UART_SENDBYTE(UART0, *sens >> 24);
  _UART_SENDBYTE(UART0, *sens >> 16);
  _UART_SENDBYTE(UART0, *sens >> 8);
  _UART_SENDBYTE(UART0, *sens);
  //B:
  _UART_SENDBYTE(UART0, CleanVoltage >> 8);
  _UART_SENDBYTE(UART0, CleanVoltage);
  //C_A:
  _UART_SENDBYTE(UART0, Calibrate_A_Concentration >> 8);
  _UART_SENDBYTE(UART0, Calibrate_A_Concentration);
  //AD_A:
  _UART_SENDBYTE(UART0, Calibrate_A_AD >> 8);
  _UART_SENDBYTE(UART0, Calibrate_A_AD);
  //C_B:
  _UART_SENDBYTE(UART0, Calibrate_B_Concentration >> 8);
  _UART_SENDBYTE(UART0, Calibrate_B_Concentration);
  //AD_B:
  _UART_SENDBYTE(UART0, Calibrate_B_AD >> 8);
  _UART_SENDBYTE(UART0, Calibrate_B_AD);
  //N:
  _UART_SENDBYTE(UART0, *d_N >> 24);
  _UART_SENDBYTE(UART0, *d_N >> 16);
  _UART_SENDBYTE(UART0, *d_N >> 8);
  _UART_SENDBYTE(UART0, *d_N);
  //Kn1
  _UART_SENDBYTE(UART0, *d_kn1 >> 24);
  _UART_SENDBYTE(UART0, *d_kn1 >> 16);
  _UART_SENDBYTE(UART0, *d_kn1 >> 8);
  _UART_SENDBYTE(UART0, *d_kn1);
  //Kn2
  _UART_SENDBYTE(UART0, *d_kn2 >> 24);
  _UART_SENDBYTE(UART0, *d_kn2 >> 16);
  _UART_SENDBYTE(UART0, *d_kn2 >> 8);
  _UART_SENDBYTE(UART0, *d_kn2);

}

void
SendSensorValues(void)
{
//    float temp;
//    float humi;
//    float ion;
//    float radi;
//    uint32_t* ptemp = (uint32_t*)(&temp);
//    uint32_t* phumi = (uint32_t*)(&humi);
//    uint32_t* pion = (uint32_t*)(&ion);
//    uint32_t* pradi = (uint32_t*)(&radi);
//
//    GetTemperature(&temp);
//    GetHum(&humi);
//    GetIonCount(&ion);
//    GetDoseRatio(&radi);

//    //Smoke:
//    _UART_SENDBYTE(UART0, 0);
//    _UART_SENDBYTE(UART0, 0);
//    _UART_SENDBYTE(UART0, CurrentConcentration_Smoke >> 8);
//    _UART_SENDBYTE(UART0, CurrentConcentration_Smoke);
//    //Dust:
//    _UART_SENDBYTE(UART0, 0);
//    _UART_SENDBYTE(UART0, 0);
//    _UART_SENDBYTE(UART0, CurrentConcentration_Dust >> 8);
//    _UART_SENDBYTE(UART0, CurrentConcentration_Dust);
//    //Temp
//    _UART_SENDBYTE(UART0, *ptemp >> 24);
//    _UART_SENDBYTE(UART0, *ptemp >> 16);
//    _UART_SENDBYTE(UART0, *ptemp >> 8);
//    _UART_SENDBYTE(UART0, *ptemp);
//    //Humi
//    _UART_SENDBYTE(UART0, *phumi >> 24);
//    _UART_SENDBYTE(UART0, *phumi >> 16);
//    _UART_SENDBYTE(UART0, *phumi >> 8);
//    _UART_SENDBYTE(UART0, *phumi);
//    //Ion
//    _UART_SENDBYTE(UART0, *pion >> 24);
//    _UART_SENDBYTE(UART0, *pion >> 16);
//    _UART_SENDBYTE(UART0, *pion >> 8);
//    _UART_SENDBYTE(UART0, *pion);
//    //Radi
//    _UART_SENDBYTE(UART0, *pradi >> 24);
//    _UART_SENDBYTE(UART0, *pradi >> 16);
//    _UART_SENDBYTE(UART0, *pradi >> 8);
//    _UART_SENDBYTE(UART0, *pradi);
}

void
Calc_Virtual_KandB(void)
{
  if (Calibrate_B_Concentration != 0xffff && Kn1 != 1)
    {
      VB1 = Calibrate_B_AD - (Calibrate_B_Concentration * (Sensitivity * Kn1));
    }
  else
    {
      VB1 = CleanVoltage;
    }

  if (Calibrate_B_Concentration != 0xffff && Kn2 != 1)
    {
      VB2 = Calibrate_B_AD - (Calibrate_B_Concentration * (Sensitivity * Kn2));
    }
  else
    {
      VB2 = CleanVoltage;
    }
}

uint32_t
Calc_Dust_Aver(uint16_t* dustData, uint8_t length, uint16_t secAver)
{
  uint8_t i;
  uint32_t dust_Count = 0;
  for (i = 0; i < length; i++)
    {
      dust_Count += dustData[i];
    }
  dust_Count -= (secAver * length);
  return dust_Count / Sensitivity;
}

#if 0
// 注意: 每个使用的地方应该有一个单独的p_totalCnt!!!

//*p_totalcnt 循环区有效的准备好的参与平均值的数据的总数
//循环缓冲区cyclic_length: 样板个数 来求平均值
//data 缓冲区指针
//data_length ：缓冲区的长度
uint16_t Cyclic_Aver(uint16_t* data, uint8_t data_length, uint8_t startpos, uint8_t cyclic_length, volatile uint8_t* p_totalCnt)
  {
    // 计算最近cyclic_length秒的均值
    uint32_t result = 0;
    uint8_t i;
    if (*p_totalCnt < cyclic_length)
      {
        (*p_totalCnt)++;
      }
    else
    for (i = 0; i < *p_totalCnt; i++)
      {
        if (startpos > 0)
          {
            startpos--;
          }
        else
          {
            //case startpos=0
            startpos = data_length - 1;
          }
        result += Min_Data[startpos];
      }
    result /= *p_totalCnt;
    return result;
  }

#else
// 注意: 每个使用的地方应该有一个单独的p_totalCnt!!!

//*p_totalcnt 循环区有效的准备好的参与平均值的数据的总数
//循环缓冲区cyclic_length: 样板个数 来求平均值
//data 缓冲区指针
//data_length ：整个缓冲区的长度
uint16_t
Cyclic_Aver(uint16_t* data, uint8_t data_length, uint8_t startpos,
    uint8_t cyclic_length, volatile uint8_t* p_totalCnt)
{
  // 计算最近cyclic_length秒的均值
  uint32_t result = 0;
  uint8_t i;
  uint8_t t = *p_totalCnt;
  if (t < cyclic_length)
    {
      //do nothing
    }
  else
    t = cyclic_length;
  //t取了用作平均数的 总个数，数据采用未满时取* p_totalCnt，取满采用
  for (i = 0; i < t; i++)
    {
      if (startpos > 0)
        {
          startpos--;
        }
      else
        {
          //case startpos=0
          startpos = data_length - 1;
        }
      result += Min_Data[startpos];
    }
  result /= t;
  return result;
}
#endif
void
LoadDatas(void)
{
  ReadCleanVoltage();
  ReadSensitivity();
  ReadCAndAD_A();
  ReadCAndAD_B();
  ReadDustN();
  ReadKn2();
  ReadKn1();
  ReadAutoOnlevel();
  ReadDispIND();
  ReadTemp();
}

void
MyDelayms(uint32_t ms)
{
  uint32_t i, tick;
  for (i = 0; i < ms; i++)
    for (tick = 0; tick < 5000; tick++)
      ;
}
#ifdef	ADDR_USE_SWITCH

//脉冲检测，周期为1Khz，中断里检测
void
GetSensorAddr(void)
{
#if 0
  P0->PMD=(GPIO_PMD_PMD0_INPUT)|GPIO_PMD_PMD1_INPUT //|GPIO_PMD_PMD4_INPUT
  |GPIO_PMD_PMD5_INPUT|GPIO_PMD_PMD6_INPUT|GPIO_PMD_PMD7_INPUT;
#else

  P0 ->PMD = (GPIO_PMD_PMD0_INPUT) | GPIO_PMD_PMD1_INPUT //|GPIO_PMD_PMD4_INPUT
      | GPIO_PMD_PMD5_INPUT | GPIO_PMD_PMD6_INPUT | GPIO_PMD_PMD7_QUASI;

#endif
  /* Debounce function control */GPIO ->DBNCECON = GPIO_DBNCECON_ICLK_ON
      | GPIO_DBNCECON_DBCLKSRC_HCLK | GPIO_DBNCECON_DBCLKSEL_32768;
  P0 ->DBEN = GPIO_DBEN_ENABLE(7);

  /* Set p0.3 as falling edge trigger and enable its interrupt */
  GPIO_EnableInt(P0, 7, GPIO_INT_FALLING);
  NVIC_EnableIRQ(GPIO_P0P1_IRQn);

  MyDelayms(1000);
//#define	TEST_CALMOD
#ifdef	TEST_CALMOD
  PulseCNT=30;
#endif
  if (PulseCNT > 10)
    {

      Address = 0;
      /*	if(P04)Address=1;
       if(P05)Address|=0x2;
       if(P06)Address|=0x4;
       if(P00)Address|=0x8;
       if(P01)Address|=0x10;
       */
      if (P04 )
        Address |= 0x10;
      if (P05 )
        Address |= 0x08;
      if (P06 )
        Address |= 0x04;
      if (P00 )
        Address |= 0x02;
      if (P01 )
        Address |= 0x01;

      Address++; ////1地址用于复制的源
//test
#ifdef	TEST_CALMOD
      Address=2;
#endif

      DPRINTF(("Addr=%d",Address));
      CurrentUARTMod = UARTMOD_CAL;
    }
  else if (PulseCNT <= 3)
    {

      CurrentUARTMod = UARTMOD_DATA_LINK;
      //p0. 5 6 7 1 0  输出  0.4  for input 检测开机否
      P0 ->PMD = GPIO_PMD_PMD0_OUTPUT | GPIO_PMD_PMD1_OUTPUT //|GPIO_PMD_PMD4_OUTPUT
          | GPIO_PMD_PMD5_OUTPUT | GPIO_PMD_PMD6_OUTPUT | GPIO_PMD_PMD7_OUTPUT;

    }

  //p0. 5 6 7 1 0  输出  0.4for input 检测开机否

  //P0->PMD=GPIO_PMD_PMD0_OUTPUT|GPIO_PMD_PMD1_OUTPUT//|GPIO_PMD_PMD4_OUTPUT
  //|GPIO_PMD_PMD5_OUTPUT|GPIO_PMD_PMD6_OUTPUT|GPIO_PMD_PMD7_OUTPUT;
  GPIO_DisableInt(P0, 7);

}
#else

void GetSensorAddr(void)
  {
    uint16_t adcValue;
    uint16_t adcMod;
    uint8_t i;

    MyDelayms(3000);

    for (i = 0; i < 20; i++)
      {
        ADC->ADCHER = 0x01 << 0;
        /* 单次转换 */
        ADC->ADCR = ADC_ADCR_ADEN_Msk | ADC_ADCR_ADST_Msk;
        /* 等待转换完成 */
        while (ADC->ADCR & ADC_ADCR_ADST_Msk);
        /* 获取ADC转换结果 */
        Address_Count += ADC->ADDR[0] & 0xFFFUL;
        MyDelayms(1);
      }
    adcValue = Address_Count / 20;
    adcMod = adcValue % (ADC_MAX_VALUE / 25);
    if ((adcMod < (ADC_MAX_VALUE / 125)) || (adcMod >((ADC_MAX_VALUE / 25) - (ADC_MAX_VALUE / 125))))
      {
        Address = (adcValue + (ADC_MAX_VALUE / 125)) / (ADC_MAX_VALUE / 25);
        DPRINTF(("Current Address: %d\r\n", Address));
        // 根据地址进入不同的模式
        if ((Address != 0) && (Address <= 20))
          {
            CurrentUARTMod = UARTMOD_CAL;
          }
        else
          {
            CurrentUARTMod = UARTMOD_DATA_LINK;
          }
      }
    else
      {
        DPRINTF(("Get Address Error! Ad: %d\r\n", adcValue）);
            // 读取地址错误也进入正常输出模式
            CurrentUARTMod = UARTMOD_DATA_LINK;
          }
        // 清除转换完成标志
        _ADC_CLEAR_ADC_INT_FLAG();
      }
#endif
// 按指定格式输出到LCD, 数值自动右对齐
void
PrintToLCD(uint8_t line, char* baseChars, uint32_t value, uint8_t offset)
{
#ifndef BOARDTYPE_MINI
  // 计算右对齐的代码
  if (value < 1000)
    {
      if (value < 100)
        {
          if (value < 10)
            {
              offset += 3;
            }
          else
            {
              offset += 2;
            }
        }
      else
        {
          offset += 1;
        }
    }

  LCD_Print(line, baseChars);
  sprintf(baseChars + offset, "%d", value);
  LCD_Print(line, baseChars);
#endif
}

void
DustSensor_Init(void)
{
  /* Init System, IP clock and multi-function I/O */
  SYS_Init();
  /* Init UART0 for printf */
  UART0_Init();

  /* Init Flash */
  Flash_Init();

  /* 重新写入参数 */
  // StoreCleanVoltage();
  // StoreSensitivity();
  /* 载入Flash中的参数 */
  LoadDatas();

  /* 计算KB值 */
  Calc_Virtual_KandB();

  printf("StartingQCL008... \r\n");
#ifdef	TEMP_DETECT
  SYS ->TEMPCR = 1;
#endif
#ifndef BOARDTYPE_MINI
  /* Init SPI0 and LCD */
  LCD_Init();
  LCD_EnableBackLight();
  LCD_ClearScreen();

  LCD_Print(0, " DUSTTESTOR");
  LCD_Print(1, "Particle_density");
#endif

  /* ADC自校正 */
  ADC_Auto_Cal();

  /* 读取地址, 并根据地址设置模式 */
  GetSensorAddr();

  ADC_Init();
  TMR_Init();
  PWMLEDO_init();
  PWMOV_init();

  Start_PWMG_INT();
}

//发送字符串
void
Uartcmd(char *p)
{

  _UART_SENDBYTE(UART0, STX_BYTE);

  while (*p != NULL)
    {
      _UART_SENDBYTE(UART0, *p);
      p++;
    }
//	_UART_SENDBYTE(UART0, NULL);//null
//	_UART_SENDBYTE(UART0, ETX_BYTE);
}

const char S_READMSTATUS[] = "RSTATUS\r";
//本机发送  读取主机的工作状态
void
ReadStatus(void)
{

  /*	_UART_SENDBYTE(UART0, 0xa5);
   _UART_SENDBYTE(UART0, 0xa5);
   _UART_SENDBYTE(UART0, 0xf0);
   _UART_SENDBYTE(UART0, 0x0f);
   _UART_SENDBYTE(UART0, 0x0a);
   */
  if (CurrentUARTMod != UARTMOD_CAL)

    Uartcmd((char *) S_READMSTATUS);

}

//1错误
void
ACK(char err)
{

  char buf[20];
  //,i=0;

  if (err == 0)
    sprintf(buf, "%s", "OK\r");
  else
    sprintf(buf, "%s\r", "ERR");
  Uartcmd(buf);

}
const char S_AUTOMODE[] = "AMODE:%d\r";
void
GetAutoMode(void *p1)
{

  char buf[50];

  sprintf(buf, S_AUTOMODE, AutoMode);
  Uartcmd(buf);
}
void
SetAutoMode(void *p1)
{
  int a;
  char err = 0;
  if (p1 != NULL)
    {

      a = atoi((char *) p1);
      if (a == 1 || a == 2 || a == 0)
        AutoMode = a;

      else
        err = 1;
    }
  else
    err = 1;
  ACK(err);

}
const char S_DISPOINT_ALL[] = "INDV:%d,%d,%d,%d\r";
void
Getdisplevel(void *p1)
{
  char buf[50];

  sprintf(buf, S_DISPOINT_ALL, Ind_DustP1, Ind_DustP2, Ind_DustP3, Ind_DustP4);
  Uartcmd(buf);
}
void
Setdisplevel(void *p1)
{

  char *s1 = p1;
  char *s2;

  char err = 0;
  if (s1 != NULL)
    {
      s2 = strtok(s1, ", ");
      Ind_DustP1 = atoi((char *) s2);
      s2 = strtok(NULL, ", ");
      Ind_DustP2 = atoi((char *) s2);
      s2 = strtok(NULL, ", ");
      Ind_DustP3 = atoi((char *) s2);
      s2 = strtok(NULL, ", ");
      Ind_DustP4 = atoi((char *) s2);
      StoreDispIND();

    }
  else
    err = 1;
  ACK(err);
}
const char S_AUTOLEVEL[] = "AV:%d\r";
void
GetAutonlevel(void *p1)
{
  char buf[20];  //,i=0;

  sprintf(buf, S_AUTOLEVEL, AutoOnDustPoint);

  Uartcmd(buf);

}
void
SetAutonlevel(void *p1)
{

  int a;
  char err = 0;
  if (p1 != NULL)
    {
      a = atoi((char *) p1);
      if (a < 1000)
        {

          AutoOnDustPoint = a;
          StoreAutoOnLevel();
        }
      else
        err = 1;
    }
  else
    err = 1;
  ACK(err);

}
//开显示
void
DispOn(void *p1)
{

  int a;
  char err = 0;
  if (p1 != NULL)
    {

      a = atoi((char *) p1);
      if (a == 1)
        AutoOutPutCurrentSmoke = TRUE;
      else if (a == 0)
        AutoOutPutCurrentSmoke = FALSE;
      else
        err = 1;
    }
  else
    err = 1;
  ACK(err);

}
//切换到串口模式读取
void
SwitchMode(void * p1)
{
  int a;
  char err = 0;
  if (p1 != NULL)
    {

      a = atoi((char *) p1);
      if (a == 1)
        DustPortOrUartMode = 1;
      else if (a == 0)
        DustPortOrUartMode = 0;
      else
        err = 1;
    }
  else
    err = 1;
  ACK(err);

}
//除尘机状态
void
SetMStatus(void *p1)
{
  int a;
  char err = 0;
  if (p1 != NULL)
    {

      a = atoi((char *) p1);
      if (a == 1)
        Readed_Machine_State = 1;
      else if (a == 0)
        Readed_Machine_State = 0;
      else
        err = 1;
    }
  else
    err = 1;
  ACK(err);
}

const char S_DUSTLEVEL[] = "DV:%d\r";
//发送灰尘值到主机
void
ReadMDUST(void *p1)
{
  char buf[20];  //,i=0;

  sprintf(buf, S_DUSTLEVEL, CurrentConcentration_Smoke);

  Uartcmd(buf);

}

//设置校准模式，后跟地址
void
SetCALMode(void * p1)
{
  int a;
  char err = 0;
  if (p1 != NULL)
    {

      a = atoi((char *) p1);
      if (a >= 1 && a <= 30)
        {
          Address = a;
          CurrentUARTMod = UARTMOD_CAL;
        }
      else
        CurrentUARTMod = UARTMOD_DATA_LINK;
    }
  else
    err = 1;
  ACK(err);

}
struct _command
{
  char *cmd; //命令字符串

  char *cmd_breief;
  void
  (*fn1)(void *p1);
};

const struct _command Scommand[] =
  {

    { "S_DISPON", "O", DispOn },  //开显示
        { "S_UARTMODE", " ", SwitchMode }, //串口模式 or IO
        { "S_STATUS", " ", SetMStatus }, //设置除尘机状态
        { "G_RDUST", " ", ReadMDUST },   //读灰尘
        { "S_AONLEVEL", " ", SetAutonlevel },   //设置自动开的值
        { "G_AONLEVEL", " ", GetAutonlevel },   //读取自动开的值
        { "S_DSPOINT", " ", Setdisplevel },   //
        { "G_DSPOINT", " ", Getdisplevel },   //读取显示的点阀值
        { "S_AUTOMODE", " ", SetAutoMode },   //设置自动模式
        { "G_AUTOMODE", " ", GetAutoMode },   //主机;读取自动模式
        { "S_CALMODE", " ", SetCALMode },   //设置到校准模式，重上电失效

        { 0, 0, 0 }, };

void
ProcessInterfaceCommand(void)
{
  char Buf[SERIAL_BUFF_SIZE];
  const char *s1;
  char *s2;
  char *s3;
  char i;

  if (CommandReceived)
    {
      memcpy((char *) &Buf, (char *) &ReadBuffer + 1, SERIAL_BUFF_SIZE); //去掉头部02
      CommandReceived = FALSE;
      s2 = strtok(Buf, ": \x03\r");
      s3 = strtok(NULL, ": \x03\r");   //取参数
      i = 0;
      s1 = Scommand[i].cmd;
      while (s1 != NULL)
        {
          if (strcmp(s1, s2) == 0)
            {
              Scommand[i].fn1(s3);
              break;
            }
          i++;
          s1 = Scommand[i].cmd;

        }

    }
}

void
ProcessSerialCommand(void)
{
  if (CommandReceived)
    {
      uint8_t *commandBytes = NULL;

      CommandReceived = FALSE;
#ifdef	EXT_CMD
      uint8_t readPos;
#endif

      /*动态创建一个有数据长度+3(类型,长度,校验码)个uint8_t元素的数组*/
      commandBytes = (uint8_t*) malloc(
          sizeof(uint8_t) * (GetDataLength((uint8_t*) SerialBuffer) + 3));

      if (TranslateData((uint8_t*) SerialBuffer, commandBytes))
        {
          if (CheckDATA(commandBytes))
            {
//              uint32_t fTemp;
              /* 保存当前的模式, 切换到关闭自动输出的模式处理完命令后切换回原来的模式 */
              enum UART_OUTPUT_MOD temp_mod = CurrentUARTMod;
              CurrentUARTMod = UARTMOD_NO_AUTO_OUTPUT;
#ifdef		EXT_CMD
              readPos = (Min_ReadCount ? Min_ReadCount : 60) - 1;
#endif
              switch (commandBytes[0])
                // 类型
                {
              default:
                break;

#ifdef TEMP_DETECT
              case CMD_CALIBRATETEMP0:
                if (commandBytes[3] == Address
                    || (Address != 1 && commandBytes[3] == 0))
                  {
                    gTemp0 = Vtemp_Aver1S;
                    gTemp0V = CyclicAver;
                    //	 if (commandBytes[3] == Address)  SendCalInfo();
                    StoreTemp();
                  }
                break;
              case CMD_CALIBRATETEMPN:
                if (commandBytes[3] == Address
                    || (Address != 1 && commandBytes[3] == 0))
                  {
                    gTempn = Vtemp_Aver1S;
                    gTempnV = CyclicAver;
                    //if (commandBytes[3] == Address)     SendCalInfo();
                    StoreTemp();

                  }
                break;
              case CMD_GETCURRENTCALVCMD:
                if (commandBytes[3] == Address)
                  {

                    _UART_SENDBYTE(UART0, CyclicAveraftercalibrate >> 8);
                    _UART_SENDBYTE(UART0, CyclicAveraftercalibrate);
                  }

                break;
                case CMD_GETCURRENTTEMPCMD:
                if (commandBytes[3] == Address)
                  {

                    _UART_SENDBYTE(UART0, Vtemp_Aver1S >> 8);
                    _UART_SENDBYTE(UART0, Vtemp_Aver1S);
                  }

                break;

#endif
#ifdef EXT_CMD
                case CMD_GET_SEC_VALUE:
                if (commandBytes[3] == Address)
                  {
                    readPos = (Sec_ReadCount ? Sec_ReadCount : DATA_SIZE) - 1;
                    _UART_SENDBYTE(UART0, Sec_Data[readPos] >> 8);
                    _UART_SENDBYTE(UART0, Sec_Data[readPos]);
                  }
                break;
                case CMD_GET_AVER_VALUE:
                if (commandBytes[3] == Address)
                  {
                    _UART_SENDBYTE(UART0, Min_Data[readPos] >> 8);
                    _UART_SENDBYTE(UART0, Min_Data[readPos]);
                  }
                break;
#endif
                case CMD_SET_UART_OUTPUT_MOD: //
                if (commandBytes[3] == Address || commandBytes[3] == 0)
                  {
                    temp_mod = (enum UART_OUTPUT_MOD)commandBytes[4];
                  }
                break;

                case CMD_GET_CALIBRATE_VOL:
                if (commandBytes[3] == Address)
                  {
                    _UART_SENDBYTE(UART0, CyclicAver >> 8);
                    _UART_SENDBYTE(UART0, CyclicAver);
                  }
                break;
                case CMD_GET_CURRENT_SMOKE:
                if (commandBytes[3] == Address)
                  {
                    _UART_SENDBYTE(UART0, CurrentConcentration_Smoke >> 8);
                    _UART_SENDBYTE(UART0, CurrentConcentration_Smoke);
                  }
                break;
#ifdef EXT_CMD
                case CMD_GET_CURRENT_DUST:
                if (commandBytes[3] == Address)
                  {
                    _UART_SENDBYTE(UART0, CurrentConcentration_Dust >> 8);
                    _UART_SENDBYTE(UART0, CurrentConcentration_Dust);
                  }
                break;

                case CMD_GET_SENSOR_VALUES:
                SendSensorValues();
                break;
#endif

                case CMD_GET_CALIBRATE_INFO:
                if (commandBytes[3] == Address)
                  {
                    SendCalInfo();
                  }
                break;
#ifdef TEMP_DETECT
                case CMD_GET_CALIBRATE_TEMP:
                if (commandBytes[3] == Address)
                  {
                    SendCalInfoTemp();
                  }
                break;
#endif

                case CMD_CALIBRATE_A:
                //todo
                if (commandBytes[3] == Address || (Address != 1 && commandBytes[3] == 0))
                  {
                    Calibrate_A_Concentration = commandBytes[4] << 8 | commandBytes[5];
                    Calibrate_A_AD = CyclicAver;
                    StoreCAndAD_A();
                    if (Calibrate_A_Concentration == 0)
                      {
                        CleanVoltage = Calibrate_A_AD;
                      }
                    StoreCleanVoltage();
                    if (Calibrate_B_AD == 0xffff &&
                        Calibrate_B_Concentration == 0xffff &&
                        Calibrate_A_Concentration != 0)
                      {
                        // 计算校准值
                        Sensitivity = (float)(Calibrate_A_AD - CleanVoltage) /
                        (Calibrate_A_Concentration - 0);
                      }
                    else if (Calibrate_B_AD != 0xffff && Calibrate_A_AD != Calibrate_B_AD &&
                        Calibrate_B_Concentration != 0xffff && Calibrate_B_Concentration != Calibrate_A_Concentration)
                      {
                        // 计算校准值
                        Sensitivity = (float)(Calibrate_B_AD - Calibrate_A_AD) /
                        (Calibrate_B_Concentration - Calibrate_A_Concentration);
                        CleanVoltage = Calibrate_A_AD - Calibrate_A_Concentration * Sensitivity;
                      }
                    // 保存校准值
                    StoreCleanVoltage();
                    StoreSensitivity();
                    Calc_Virtual_KandB();
                    if (commandBytes[3] == Address)
                    SendCalInfo();
                  }
                break;
                case CMD_CALIBRATE_B:
                //todo
                if (commandBytes[3] == Address || (Address != 1 && commandBytes[3] == 0))
                  {
                    Calibrate_B_Concentration = commandBytes[4] << 8 | commandBytes[5];
                    Calibrate_B_AD = CyclicAver;
                    StoreCAndAD_B();
                    if (Calibrate_B_Concentration == 0)
                      {
                        CleanVoltage = Calibrate_B_AD;
                      }
                    StoreCleanVoltage();
                    if (Calibrate_A_AD == 0xffff &&
                        Calibrate_A_Concentration == 0xffff &&
                        Calibrate_B_Concentration != 0)
                      {
                        // 计算校准值
                        Sensitivity = (float)(Calibrate_B_AD - CleanVoltage) /
                        (Calibrate_B_Concentration - 0);
                      }
                    else if (Calibrate_A_AD != 0xffff && Calibrate_B_AD != Calibrate_A_AD &&
                        Calibrate_A_Concentration != 0xffff && Calibrate_A_Concentration != Calibrate_B_Concentration)
                      {
                        // 计算校准值
                        Sensitivity = (float)(Calibrate_A_AD - Calibrate_B_AD) /
                        (Calibrate_A_Concentration - Calibrate_B_Concentration);
                        CleanVoltage = Calibrate_B_AD - Calibrate_B_Concentration * Sensitivity;
                      }
                    // 保存校准值
                    StoreCleanVoltage();
                    StoreSensitivity();
                    Calc_Virtual_KandB();
                    if (commandBytes[3] == Address)
                    SendCalInfo();
                  }
                break;
#ifdef EXT_CMD
                case CMD_CAL_N:
                if (commandBytes[3] == Address || (Address != 1 && commandBytes[3] == 0))
                  {
                    Calibrate_N_Concentration = commandBytes[4] << 8 | commandBytes[5];
                    Dust_Calc_N = (float)DustFiveSecAver / (Calibrate_N_Concentration - CurrentConcentration_Smoke);
                    if (commandBytes[3] == Address)
                    SendCalInfo();
                    StoreDustN();
                  }
                break;
                case CMD_SHIFT:
                if (commandBytes[3] == Address || commandBytes[3] == 0)
                  {
                    PWMG_SHIFT(commandBytes[3]);
                  }
                break;
                case CMD_SET_PWM_PERIOD:
                if (commandBytes[3] == Address || commandBytes[3] == 0)
                  {
                    PWMA->CMR2 = commandBytes[3] << 24 |
                    commandBytes[4] << 16 |
                    commandBytes[5] << 8 |
                    commandBytes[6];
                  }
                break;
                case CMD_GET_PWM_PERIOD:
                if (commandBytes[3] == Address)
                  {
                    _UART_SENDBYTE(UART0, PWMA->CMR2 >> 24);
                    _UART_SENDBYTE(UART0, PWMA->CMR2 >> 16);
                    _UART_SENDBYTE(UART0, PWMA->CMR2 >> 8);
                    _UART_SENDBYTE(UART0, PWMA->CMR2);
                  }
                break;
                case CMD_SET_TMR_DELAY:
                if (commandBytes[3] == Address || commandBytes[3] == 0)
                  {
                    _TIMER_RESET(TIMER0);
                    TIMER0->TCMPR = commandBytes[4] << 24 |
                    commandBytes[5] << 16 |
                    commandBytes[6] << 8 |
                    commandBytes[7];
                  }
                break;
                case CMD_GET_TMR_DELAY:
                if (commandBytes[3] == Address)
                  {
                    _UART_SENDBYTE(UART0, TIMER0->TCMPR >> 24);
                    _UART_SENDBYTE(UART0, TIMER0->TCMPR >> 16);
                    _UART_SENDBYTE(UART0, TIMER0->TCMPR >> 8);
                    _UART_SENDBYTE(UART0, TIMER0->TCMPR);
                  }
                break;
                case CMD_SET_K:
                if (commandBytes[3] == Address || commandBytes[3] == 0)
                  {
                    fTemp = commandBytes[4] << 24 |
                    commandBytes[5] << 16 |
                    commandBytes[6] << 8 |
                    commandBytes[7];
                    Sensitivity = *(float*)&fTemp;
                    StoreSensitivity();
                    Calc_Virtual_KandB();
                    if (commandBytes[3] == Address)
                    SendCalInfo();
                  }
                break;
                case CMD_SET_Kn1:
                if (commandBytes[3] == Address || commandBytes[3] == 0)
                  {
                    fTemp = commandBytes[4] << 24 |
                    commandBytes[5] << 16 |
                    commandBytes[6] << 8 |
                    commandBytes[7];
                    Kn1 = *(float*)&fTemp;
                    StoreKn1();
                    Calc_Virtual_KandB();
                    if (commandBytes[3] == Address)
                    SendCalInfo();
                  }
                break;
                case CMD_SET_Kn2:
                if (commandBytes[3] == Address || commandBytes[3] == 0)
                  {
                    fTemp = commandBytes[4] << 24 |
                    commandBytes[5] << 16 |
                    commandBytes[6] << 8 |
                    commandBytes[7];
                    Kn2 = *(float*)&fTemp;
                    StoreKn2();
                    Calc_Virtual_KandB();
                    if (commandBytes[3] == Address)
                    SendCalInfo();
                  }
                break;
                case CMD_SET_B:
                if (commandBytes[3] == Address || commandBytes[3] == 0)
                  {
                    CleanVoltage = commandBytes[4] << 8 |
                    commandBytes[5];
                    StoreCleanVoltage();
                    Calc_Virtual_KandB();
                    if (commandBytes[3] == Address)
                    SendCalInfo();
                  }
                break;
                case CMD_SET_CAL_N:
                if (commandBytes[3] == Address || commandBytes[3] == 0)
                  {
                    fTemp = commandBytes[4] << 24 |
                    commandBytes[5] << 16 |
                    commandBytes[6] << 8 |
                    commandBytes[7];
                    Dust_Calc_N = *(float*)&fTemp;
                    StoreDustN();
                    if (commandBytes[3] == Address)
                    SendCalInfo();
                  }
                break;

#endif
              }
              CurrentUARTMod = temp_mod;
            }
        }
      free(commandBytes);
    }
}

void
Display_Min_Aver(uint16_t Min_Aver)
{
  char strVal[LCD_WIDTH] = "Min_Value:      ";
  PrintToLCD(1, strVal, Min_Aver, 10);
  //if (CurrentUARTMod != UARTMOD_CAL) DPRINTF(("Min_Value: %d, Total: %d\r\n", Min_Aver, Total_ReadCount));
}

void
TryUpdateClean(uint16_t AVR1minvalue)
{
  if (CurrentUARTMod == UARTMOD_CAL)
    Current_Machine_State = 0; //2015.0606
  else
    {
      // 提高无尘电压
      // 获取机器工作状态
      Current_Machine_State = GetMachineWorking();
      // 判断是否初次启动
      if (Current_Machine_State && !Last_Machine_State)
        First_Start = TRUE;
      Last_Machine_State = Current_Machine_State;
    }

  if (Current_Machine_State == 0)
    {
      // 机器关闭状态
      TurnUpCleanVolConfirmTimes = 0;
    }
  else
    {
      // 如果机器开启
      if (Last_Value == 0)
        Last_Value = AVR1minvalue;
#ifdef	DEBUG_DUST
      DPRINTF(("Machine is working, FirstStart: %d\r\n", First_Start));
      DPRINTF(("minValue: %d\r\n", min_Aver));
      DPRINTF(("FirstValue: %d\r\nLast_Value: %d\r\nTurnUpCleanVolConfirmTimes: %d\r\n",
              First_Value, Last_Value, TurnUpCleanVolConfirmTimes));
      DPRINTF(("Clean Voltage: %d \r\n", CleanVoltage));
#endif
      // 如果灰尘值大于一定值
      if (AVR1minvalue
          <= CleanVoltage + CLEAN_UP_UPDATE_THRESHOLD * Sensitivity)
//不大于门限
        {
          Last_Value = 0;
          TurnUpCleanVolConfirmTimes = 0;
        }
      else
        {
          // 与最后一次获取值相差的百分比
          float percent =
              (AVR1minvalue > Last_Value) ?
                  ((AVR1minvalue - Last_Value) / (float) Last_Value) :
                  ((Last_Value - AVR1minvalue) / (float) Last_Value);
          if (First_Start)
            {
              First_Value = AVR1minvalue;
              First_Start = FALSE;
            }
          if (First_Value < AVR1minvalue)
            First_Value = AVR1minvalue;
          // 设置最后获取到的值
          if (percent > CLEAN_UP_DISTINGUISH_RANGE)
            {
              Last_Value = AVR1minvalue;
              TurnUpCleanVolConfirmTimes = 0;
            }
          else
            {
#ifdef	REF25_VDD
              if (VddRel_MV > MAX_UPDATE_VDD) //电源电压升高了
                {
                  TurnUpCleanVolConfirmTimes = 0;
                }
              else
#endif
                TurnUpCleanVolConfirmTimes++;
            }

          DPRINTF(("Current Change: %f%%\r\n", percent * 100));
          if (TurnUpCleanVolConfirmTimes >= CLEAN_UP_CONFIRM_TIMES)
            {
              if ((AVR1minvalue < First_Value)
                  && (((First_Value - AVR1minvalue) / (float) First_Value)
                      > CLEAN_UP_FINAL_RANGE))
                {
                  uint16_t subValue = ((Last_Value - CleanVoltage)
                      * CLEAN_UP_CALI_VALUE);
                  if (subValue < 3 * Sensitivity)
                    subValue = 3 * Sensitivity;
                  CleanVoltage = (uint16_t)(CleanVoltage + subValue);
                  First_Value = AVR1minvalue;
                  StoreCleanVoltage();
                  Calc_Virtual_KandB();
                  if (CurrentUARTMod == UARTMOD_DATA_LINK)
                    {
                      /**/
                      CurrentUARTMod = UARTMOD_DEBUG;
                      DPRINTF(("CLEAN_VOLTAGE Updated to: %d\r\n", CleanVoltage));
                      CurrentUARTMod = UARTMOD_DATA_LINK;
                      /**/
                    }
                }
              TurnUpCleanVolConfirmTimes = 0;
            }
        }
    }
}

_Bool
GetMachineWorking(void)
{
  _Bool up = FALSE;
  if (Readed_Machine_State == 0x01)
    {
      up = TRUE;
      Readed_Machine_State = 0x05;
    }
  return up;
}
#ifdef REF25_VDD
//计算电源电压值
void
CalVDDAVG()
{
  uint16_t sec_Aver;
  sec_Aver = Util_CalcAvg(Vdd_Data, DATA_SIZE);
  Vdd_Aver1S = sec_Aver;
  //电源电压  Vdd_Aver1S=v25/vdd*4096   vdd==(int) ((uint32_t) 2500*4096/Vdd_Aver1S);
  VddRel_MV = (uint16_t)((uint32_t) 2500 * 4096 / Vdd_Aver1S);
  DPRINTF(("VDD=%dmV\r\n", VddRel_MV));
  //        DPRINTF(("VDD=%dmV\r\n", Vdd_Aver1S));
}
#endif

#ifdef TEMP_DETECT
void
CalTempVoltageAVG()
{
  uint16_t sec_Aver, i;
  uint32_t sec_Count = 0;
  // 温度代表的电压
  sec_Aver = Util_CalcAvg(Vtemp_Data, DATA_SIZE);
  //Vtemp_Aver1S=sec_Aver;
  Vtemp_Aver1S = (uint16_t)((uint32_t) sec_Aver * 4096 / Vdd_Aver1S / 2);

  DPRINTF(("VTEMP=%dmV\r\n", Vtemp_Aver1S));
}
#endif
//计算1秒的灰尘采样平均值
void
CalDustVolTageAVG()
{
  char strVal[LCD_WIDTH] = "Sec_Value:      ";
  uint16_t sec_Aver;
  // 计算秒均值  浓度
  sec_Aver = Util_CalcAvg(Sec_Data, DATA_SIZE);
#ifdef  REF25_VDD
  // 但AD实际以 VDD作为参考，该参考会变化不稳定， 所以 用TL431作为基准电压2.5v， 计算出VDD的实际电压，
  //再以测量的灰尘电压换算为 5V为基准 4096 的 采样值
  //d25=v25/vdd*4096    ds=vs/vdd*4096  =>vs=ds/d25*v25=ds/d25*2500
  //Sref=vs/5000*4096
  //Sref=(2500/5000)*(ds/d25)*4096=ds*4096/d25/2
  sec_Aver = (uint16_t)((uint32_t) sec_Aver * 4096 / Vdd_Aver1S / 2); //以5v为标准的量化值（4096档）
#endif

  Min_Data[Min_ReadCount++] = sec_Aver;
  Update_Data[Update_ReadCount++] = sec_Aver;
  // 输出电压值到LCD
  if (CurrentUARTMod == UARTMOD_DEBUG && Total_ReadCount > 3)
    {
      PrintToLCD(0, strVal, (int) (sec_Aver * REFVOL / ADC_MAX_VALUE), 10);
    }

  DPRINTF(("Sec_Value: %dmV\r\n", (int)(sec_Aver * REFVOL / ADC_MAX_VALUE)));
}

uint16_t
GetCurrentSmoke(uint16_t tempv)
{
  uint16_t smokeRet;
  float smoke;
  if (tempv > CleanVoltage)
    {
      if (tempv <= Calibrate_B_AD)
        {
          if (tempv > VB1)
            smoke = (tempv - VB1) / (Sensitivity * Kn1);
          else
            smoke = 0;
        }
      else
        {
          if (tempv > VB1)
            smoke = (tempv - VB2) / (Sensitivity * Kn2);
          else
            smoke = 0;
        }

    }
  else
    smoke = 0;
  smokeRet = (uint16_t) smoke;
  return smokeRet;

}

//判断数据有否突发变化，便于快速更新数据显示
void
JudgeRedo()
{
  uint16_t CyclicAver1, i;      //最近n秒平均值 突变
  //最近3秒变化量大，则数据重新采样，以利于显示的快速更新
  CyclicAver1 = Cyclic_Aver((uint16_t*) Min_Data, 60, Min_ReadCount, 3,
      &Total_ReadCount);
  //    if(abs(CyclicAver1-CyclicAver)*10/CyclicAver>2)
  if (abs((int32_t) CyclicAver1 - (int32_t) CyclicAver) > 45)
    {
      Min_ReadCount = 0;
      Update_ReadCount = 0;
      Total_ReadCount = 0;
      TurnDownCleanVolConfirmTimes = 0;
      for (i = 0; i < UPDATE_CYCLIC_COUNT; i++)
        {
          Min_Data[i] = CyclicAver1;
          Update_Data[i] = CyclicAver1;
        }
    }
}

//每秒调用一次
//向下更新
void
CalTurnDown(void)
{
  if ((CurrentUARTMod == UARTMOD_CAL)
      || (CyclicAveraftercalibrate > CleanVoltage)
#ifdef          REF25_VDD
      //电源电压过低，不做更新无尘电压  //jxd
      || (VddRel_MV < MIN_UPDATE_VDD)  //4.8v

#endif
      )

    {
      TurnDownCleanVolConfirmTimes = 0;
    }

  
  // 如果持续次数超过 CLEAN_VOLTAGE_CONFIRM_TIMES 次 则更新CLEAN_VOLTAGE
  // 注意: 此处为每秒比较一次,而不是每5秒比较一次.
  if (++TurnDownCleanVolConfirmTimes >= CLEAN_DOWN_CONFIRM_TIMES)
    {
      if (CurrentUARTMod == UARTMOD_DATA_LINK)
        {
          /**/
          CurrentUARTMod = UARTMOD_DEBUG;
          DPRINTF(("CLEAN_VOLTAGE Updated to: %d\r\n", CyclicAveraftercalibrate));
          CurrentUARTMod = UARTMOD_DATA_LINK;
          /**/
        }
      CleanVoltage = CyclicAveraftercalibrate; //??????????????????????????????????????还是去  CyclicAver  CyclicAveraftercalibrate
      if (Calibrate_A_Concentration == 0)
        {
          Calibrate_A_AD = CleanVoltage;
          StoreCAndAD_A();
        }
      if (Calibrate_B_Concentration == 0)
        {
          Calibrate_B_AD = CleanVoltage;
          StoreCAndAD_B();
        }
      StoreCleanVoltage();
      Calc_Virtual_KandB();
      TurnDownCleanVolConfirmTimes = 0;
    }

}
// 输出模式为DATA_LINK 每秒输出一次数据
void
DisplaySmoke(void)
{
  char strSVal[LCD_WIDTH] = "        ug      ";
  //char strCVal[LCD_WIDTH] = "Dust:    ug     ";
  if (CurrentUARTMod == UARTMOD_DATA_LINK)
    {
      if (AutoOutPutCurrentSmoke)
        {
          /**/
          CurrentUARTMod = UARTMOD_DEBUG;
          printf("%dug\r\n", CurrentConcentration_Smoke);
          CurrentUARTMod = UARTMOD_DATA_LINK;

        }
    }
//                else if(CurrentUARTMod != UARTMOD_CAL)
//                {
//                    DPRINTF(("Smoke: %dug\t Dust: %dug\r\n", CurrentConcentration_Smoke, CurrentConcentration_Dust));
//                }
  //PrintToLCD(2, strSVal, CurrentConcentration_Smoke, 4);
  //PrintToLCD(3, strCVal, CurrentConcentration_Dust, 6);
#ifdef  ADDR_USE_SWITCH
    {
      extern void
      SetDustIndicateLED( uint16_t);
      if (CurrentUARTMod == UARTMOD_DATA_LINK)    //防止校准输出LED指示，以免短路地址跳线冲突
        SetDustIndicateLED(CurrentConcentration_Smoke);
    }
#endif
}
void
DustSensor_Process(void)
{
  if (Sec_ReadCount >= DATA_SIZE)
    {
#ifdef REF25_VDD
      CalVDDAVG();
#endif
#ifdef TEMP_DETECT
      CalTempVoltageAVG();
#endif
      //计算1秒的灰尘采样平均值
      CalDustVolTageAVG();
      Sec_ReadCount = 0;     //重新开始新一轮测量，10ms，  一个，将缓冲区指针置0
//实际已经采样的数据个数，250随便取的，采样未满60，取只取Total_ReadCount个数计算平均数，满60 择取60个或CYCLIC_COUNT（较小）的平均数
      if (Total_ReadCount < 250)
        Total_ReadCount++;
        {
          // 更新PWM输出
          SetOV_Enable(CurrentConcentration_Smoke);
          // 计算最近CYCLIC_COUNT=?秒的均值
          CyclicAver = Cyclic_Aver((uint16_t*) Min_Data, 60, Min_ReadCount,
              CYCLIC_COUNT, &Total_ReadCount);
          //最近3秒变化量大，则数据重新采样，以利于显示的快速更新
          JudgeRedo();
          CyclicAveraftercalibrate = CalTempCalibrate(); //其他温度下的测量电压映射到 25度的电压 ，在25度的曲线运算，所有存储的电压系数都是25度的
          CurrentConcentration_Smoke = GetCurrentSmoke(
              CyclicAveraftercalibrate);      //根据电压计算浓度值

            {
              /* 当前值比Flash中存储的大 */
              if (CyclicAveraftercalibrate > CleanVoltage)
                {
#if 0
                  /* 计算灰尘类型（烟/尘） */
                  for ( i = 0; i < 100; i++)
                    {
                      if (abs(Sec_Data[i] - sec_Aver) > sec_Aver * TYPE_DISTINGUISH_RANGE && Sec_Data[i] > sec_Aver)
                        {
                          dust_Data[greaterCount++] = Sec_Data[i];
                        }
                    }
                  //if(greaterCount > TYPE_DISTINGUISH_COUNT)
                    {
                      // Dust
                      uint32_t dustVal;
                      uint8_t i;
                      uint32_t dustValAver = 0;
                      dustVal = Calc_Dust_Aver(dust_Data, greaterCount, sec_Aver);
                      // 丢弃过大的数值
                      if (dustVal < CurrentConcentration_Smoke * DUST_VALUE_LIMIT)
                        {
                          Dust_N_ReadCount %= CYCLIC_TIME;
                          Dust_N_Data[Dust_N_ReadCount] = dustVal;
                          Dust_N_ReadCount++;
                        }
                      for (i = 0; i < CYCLIC_TIME; i++)
                        {
                          dustValAver += Dust_N_Data[i];
                        }
                      dustValAver /= CYCLIC_TIME;
                      DustFiveSecAver = dustValAver;
                      CurrentConcentration_Dust = (int)(dustValAver / Dust_Calc_N + CurrentConcentration_Smoke);
                    }

#ifdef	DEBUG_DUST
                  DPRINTF(("Value over 15%%: %d\r\n", greaterCount));
#endif
#endif
                }
              else
                {
                  //发光老化或电源电压低了
                  //每秒调用一次
                  //向下更新
                  CalTurnDown();
                }

            }
          // 输出模式为DATA_LINK 每秒输出一次数据
          DisplaySmoke();
        }
#define	DUSTIN_STATUS	P04

      // 获取机器工作状态
      // 发送格式4字节：FA55
      // 返回值格式为： 04 + 状态（关/开: 55/aa） + 03
      if (Update_ReadCount == UPDATE_CYCLIC_COUNT - 1)
        {

          if (DustPortOrUartMode == 0) //jxd利用端口判定状态
            {
              if (DUSTIN_STATUS == 1)
                Readed_Machine_State = 1;
              else
                Readed_Machine_State = 0;
            }
          else //利用串口判定状态
            {
              ReadStatus();
            }
        }

      // 计算分钟均值
      if (Min_ReadCount >= 60)
        {
          uint16_t min_Aver;
          min_Aver = Util_CalcAvg((uint16_t *) Min_Data, 60);
          Min_ReadCount = 0;
          //Todo: DisPlay Min_Aver
          if (CurrentUARTMod == UARTMOD_DEBUG)
            Display_Min_Aver(min_Aver);
        }

      // 向上更新无尘电压 镜头有灰尘，散射严重,每分钟检查一次
      if (Update_ReadCount >= UPDATE_CYCLIC_COUNT)
        {
//每分钟干活
          uint16_t min_Aver;
          min_Aver = Util_CalcAvg((uint16_t *) Update_Data,
              UPDATE_CYCLIC_COUNT);
          Update_ReadCount = 0;
          TryUpdateClean(min_Aver);
        }

      Start_PWMG_INT();
    }

  /* 处理串口命令 */
  if (CurrentUARTMod == UARTMOD_DEBUG || CurrentUARTMod == UARTMOD_CAL)
    ProcessSerialCommand();
  else if (CurrentUARTMod == UARTMOD_DATA_LINK)
    ProcessInterfaceCommand();
}

