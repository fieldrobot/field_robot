/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "comms.h"
#include "protocol.h"
#include "hallinterrupts.h"
#include "crc32.h"
#include <stdbool.h>
#include <string.h>	//robo

void SystemClock_Config(void);

extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
//LCD_PCF8574_HandleTypeDef lcd;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

int cmd1, cmd1_ADC;  // normalized input values. -1000 to 1000
int cmd2, cmd2_ADC;
int cmd3;

#ifdef SPEED_IS_KMH // ROBO
  int cmd2Goal;	// goal speed for SPEED_IS_KMH
#endif

bool ADCcontrolActive = false;
bool SoftWatchdogActive= false;

typedef struct{
   int16_t steer;
   int16_t speed;
   uint32_t crc;
} Serialcommand;
volatile Serialcommand command;

//ROBO begin
typedef struct{
   int16_t iSpeedL;		// 100* km/h
   int16_t iSpeedR;		// 100* km/h
   uint16_t iHallSkippedL;
   uint16_t iHallSkippedR;
   uint16_t iTemp;		// °C
   uint16_t iVolt;		// 100* V
   int16_t iAmpL;		// 100* A
   int16_t iAmpR;		// 100* A
   uint32_t crc;
} SerialFeedback;
 SerialFeedback oFeedback;  // volatile

#ifdef DEBUG_SERIAL_USART3
#define UART_DMA_CHANNEL DMA1_Channel2
#endif
#ifdef DEBUG_SERIAL_USART2
#define UART_DMA_CHANNEL DMA1_Channel7
#endif
//ROBO end


int disablepoweroff = 0;
int powerofftimer = 0;

uint8_t button1, button2, button1_ADC, button2_ADC;

int steer; // global variable for steering. -1000 to 1000
int speed; // global variable for speed. -1000 to 1000

extern volatile int pwml;  // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000
extern volatile int weakl; // global variable for field weakening left. -1000 to 1000
extern volatile int weakr; // global variable for field weakening right. -1000 to 1000

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
uint16_t buzzerLen = 0;

extern uint8_t enable; // global variable for motor enable

extern volatile uint32_t timeout; // global variable for timeout
extern float batteryVoltage; // global variable for battery voltage
extern float currentL;	// defined and updated in bldc.c
extern float currentR;	// defined and updated in bldc.c


uint32_t inactivity_timeout_counter;

extern uint8_t nunchuck_data[6];
#ifdef CONTROL_PPM
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif

int milli_vel_error_sum = 0;


void poweroff() {
  enable = 0;    // disable Motors
  if (ABS(speed) < 20) {
    buzzerPattern = 0;
    for (int i = 0; i < 8; i++) {
      buzzerFreq = i;

#ifdef SOFTWATCHDOG_TIMEOUT
      for(int j = 0; j < 50; j++) {
        __HAL_TIM_SET_COUNTER(&htim3, 0); // Kick the Watchdog
        HAL_Delay(1);
      }
    }
#else
      HAL_Delay(100);
    }
#endif

    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0); // shutdown  power
    while(1) {}
  } else {
    powerofftimer = 1000;
  }
}

//ROBO begin
#if defined(CONTROL_SERIAL_NAIVE_USART2) || defined(CONTROL_SERIAL_NAIVE_USART3)
bool checkCRC2(volatile Serialcommand* command) 
{
	uint8_t a[15];
	memcpy((void*)a,(void*)command,8);
	memcpy((void*)(a+8),(void*)command,7);
	
	for (uint8_t iOffset=0; iOffset< 8; iOffset++)
	{
		Serialcommand* pCmd = (Serialcommand*) (a+iOffset);
		uint32_t crc = 0;
		crc32((const void *)pCmd, 4, &crc); // 4 2x uint16_t = 4 bytes
		if(pCmd->crc == crc) 
		{
			memcpy((void*)command,(void*)pCmd,8);
			setScopeChannel(0, 1);
			if (	(abs(pCmd->steer)>1000) || (abs(pCmd->speed)>1000)	)
			{
				setScopeChannel(1,255);
				return false;
			}
			setScopeChannel(1,(int) iOffset);
			return true;
		}
	}
	setScopeChannel(0, 0);
	setScopeChannel(1, (int)command->crc);
	return false;
}
#endif
//ROBO end

int main(void) {
  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();


  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  #ifdef SERIAL_USART2_IT
  USART2_IT_init();
  #endif
  #ifdef SERIAL_USART3_IT
  USART3_IT_init();
  #endif

  for (int i = 8; i >= 0; i--) {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  int lastSpeedL = 0, lastSpeedR = 0;
  int speedL = 0, speedR = 0;

  #ifdef SPEED_IS_KMH // ROBO
    cmd2Goal = 0;	// goal speed for SPEED_IS_KMH
  #endif


  #ifdef HALL_INTERRUPTS
    // enables interrupt reading of hall sensors for dead reconing wheel position.
    HallInterruptinit();
  #endif

  #ifdef CONTROL_PPM
    PPM_Init();
  #endif

  #ifdef CONTROL_NUNCHUCK
    I2C_Init();
    Nunchuck_Init();
  #endif

  #if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_NAIVE_USART2)
    UART2_Init();
  #endif

  #if defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_NAIVE_USART3)
    UART3_Init();
  #endif

  #ifdef CONTROL_SERIAL_NAIVE_USART2
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)&command, sizeof(command));
  #endif

  #ifdef CONTROL_SERIAL_NAIVE_USART3
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)&command, sizeof(command));
  #endif

  #ifdef DEBUG_I2C_LCD
    I2C_Init();
    HAL_Delay(50);
    lcd.pcf8574.PCF_I2C_ADDRESS = 0x27;
      lcd.pcf8574.PCF_I2C_TIMEOUT = 5;
      lcd.pcf8574.i2c = hi2c2;
      lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_2;
      lcd.type = TYPE0;

      if(LCD_Init(&lcd)!=LCD_OK){
          // error occured
          //TODO while(1);
      }

    LCD_ClearDisplay(&lcd);
    HAL_Delay(5);
    LCD_SetLocation(&lcd, 0, 0);
    LCD_WriteString(&lcd, "Hover V2.0");
    LCD_SetLocation(&lcd, 0, 1);
    LCD_WriteString(&lcd, "Initializing...");
  #endif

  float board_temp_adc_filtered = (float)adc_buffer.temp;
  float board_temp_deg_c;

#if defined(SERIAL_USART2_IT)
  serial_usart_buffer_flush(&usart2_it_RXbuffer);
#endif

#if defined(SERIAL_USART3_IT)
  serial_usart_buffer_flush(&usart3_it_RXbuffer);
#endif

  enable = 1;  // enable motors
#ifdef SOFTWATCHDOG_TIMEOUT
  MX_TIM3_Softwatchdog_Init(); // Start the WAtchdog
  SoftWatchdogActive= true;
#endif

	uint16_t iLog = 0;
  while(1) {
  	iLog++;
      HAL_Delay(DELAY_IN_MAIN_LOOP); //delay in ms

    // TODO: Method to select which input is used for Protocol when both are active
    #if defined(SERIAL_USART2_IT) && defined(CONTROL_SERIAL_PROTOCOL)
      if(!enable_immediate) timeout++;
      while (serial_usart_buffer_count(&usart2_it_RXbuffer) > 0) {
        SERIAL_USART_IT_BUFFERTYPE inputc = serial_usart_buffer_pop(&usart2_it_RXbuffer);
        protocol_byte( (unsigned char) inputc );
      }
      cmd1 = PwmSteerCmd.steer;
      cmd2 = PwmSteerCmd.base_pwm;
    #elif defined(SERIAL_USART3_IT) && defined(CONTROL_SERIAL_PROTOCOL)
      if(!enable_immediate) timeout++;
      while (serial_usart_buffer_count(&usart3_it_RXbuffer) > 0) {
        SERIAL_USART_IT_BUFFERTYPE inputc = serial_usart_buffer_pop(&usart3_it_RXbuffer);
        protocol_byte( (unsigned char) inputc );
      }
      cmd1 = PwmSteerCmd.steer;
      cmd2 = PwmSteerCmd.base_pwm;
    #endif

    #ifdef CONTROL_NUNCHUCK
      Nunchuck_Read();
      cmd1 = CLAMP((nunchuck_data[0] - 127) * 8, -1000, 1000); // x - axis. Nunchuck joystick readings range 30 - 230
      cmd2 = CLAMP((nunchuck_data[1] - 128) * 8, -1000, 1000); // y - axis

      button1 = (uint8_t)nunchuck_data[5] & 1;
      button2 = (uint8_t)(nunchuck_data[5] >> 1) & 1;
    #endif

    #ifdef CONTROL_PPM
      cmd1 = CLAMP((ppm_captured_value[0] - 500) * 2, -1000, 1000);
      cmd2 = CLAMP((ppm_captured_value[1] - 500) * 2, -1000, 1000);
      button1 = ppm_captured_value[5] > 500;
      float scale = ppm_captured_value[2] / 1000.0f;
    #endif

    #ifdef CONTROL_ADC
      // ADC values range: 0-4095, see ADC-calibration in config.h

      #ifdef ADC_SWITCH_CHANNELS

        if(adc_buffer.l_rx2 < ADC2_ZERO) {
          cmd1_ADC = (CLAMP(adc_buffer.l_rx2, ADC2_MIN, ADC2_ZERO) - ADC2_ZERO) / ((ADC2_ZERO - ADC2_MIN) / ADC2_MULT_NEG); // ADC2 - Steer
        } else {
          cmd1_ADC = (CLAMP(adc_buffer.l_rx2, ADC2_ZERO, ADC2_MAX) - ADC2_ZERO) / ((ADC2_MAX - ADC2_ZERO) / ADC2_MULT_POS); // ADC2 - Steer
        }

        if(adc_buffer.l_tx2 < ADC1_ZERO) {
          cmd2_ADC = (CLAMP(adc_buffer.l_tx2, ADC1_MIN, ADC1_ZERO) - ADC1_ZERO) / ((ADC1_ZERO - ADC1_MIN) / ADC1_MULT_NEG); // ADC1 - Speed
        } else {
          cmd2_ADC = (CLAMP(adc_buffer.l_tx2, ADC1_ZERO, ADC1_MAX) - ADC1_ZERO) / ((ADC1_MAX - ADC1_ZERO) / ADC1_MULT_POS); // ADC1 - Speed
        }

        if((adc_buffer.l_tx2 < ADC_OFF_START) || (adc_buffer.l_tx2 > ADC_OFF_END) ) {
          ADCcontrolActive = true;
        } else {
          if(ADCcontrolActive) {
            cmd1 = 0;
            cmd2 = 0;
          }
          ADCcontrolActive = false;
        }

      #else


        if(adc_buffer.l_tx2 < ADC1_ZERO) {
          cmd1_ADC = (CLAMP(adc_buffer.l_tx2, ADC1_MIN, ADC1_ZERO) - ADC1_ZERO) / ((ADC1_ZERO - ADC1_MIN) / ADC1_MULT_NEG); // ADC1 - Steer
        } else {
          cmd1_ADC = (CLAMP(adc_buffer.l_tx2, ADC1_ZERO, ADC1_MAX) - ADC1_ZERO) / ((ADC1_MAX - ADC1_ZERO) / ADC1_MULT_POS); // ADC1 - Steer
        }

        if(adc_buffer.l_rx2 < ADC2_ZERO) {
          cmd2_ADC = (CLAMP(adc_buffer.l_rx2, ADC2_MIN, ADC2_ZERO) - ADC2_ZERO) / ((ADC2_ZERO - ADC2_MIN) / ADC2_MULT_NEG); // ADC2 - Speed
        } else {
          cmd2_ADC = (CLAMP(adc_buffer.l_rx2, ADC2_ZERO, ADC2_MAX) - ADC2_ZERO) / ((ADC2_MAX - ADC2_ZERO) / ADC2_MULT_POS); // ADC2 - Speed
        }


        if((adc_buffer.l_rx2 < ADC_OFF_START) || (adc_buffer.l_rx2 > ADC_OFF_END) ) {
          ADCcontrolActive = true;
        } else {
          if(ADCcontrolActive) {
            cmd1 = 0;
            cmd2 = 0;
          }
          ADCcontrolActive = false;
        }

      #endif

      #ifdef ADC_REVERSE_STEER
        cmd1_ADC = -cmd1_ADC;
      #endif

      // use ADCs as button inputs:
      button1_ADC = (uint8_t)(adc_buffer.l_tx2 > 2000);  // ADC1
      button2_ADC = (uint8_t)(adc_buffer.l_rx2 > 2000);  // ADC2
    #endif

    #if defined(CONTROL_SERIAL_NAIVE_USART2) || defined(CONTROL_SERIAL_NAIVE_USART3)
      if(checkCRC2(&command))	//ROBO
      {
        cmd1 = command.steer;
        cmd2 = command.speed;
      }
      else
      {
    	  cmd1 = 0;
    	  cmd2 = 0;
      }
	//cmd2 = -100;

      timeout = 0;
    #endif


    #if defined CONTROL_ADC
      if(ADCcontrolActive) {
        cmd1 = cmd1_ADC;
    	  cmd2 = cmd2_ADC;
        timeout = 0;
      }
    #endif

#ifdef SPEED_IS_KMH
	long iSpeed;
  if (abs(HallData[0].HallSpeed_mm_per_s) > abs(HallData[1].HallSpeed_mm_per_s))
  {
    #ifdef INVERT_L_DIRECTION
      iSpeed = -HallData[0].HallSpeed_mm_per_s;
    #else
      iSpeed = HallData[0].HallSpeed_mm_per_s;
    #endif
  } 
  else
  {
    #ifdef INVERT_R_DIRECTION
      iSpeed = -HallData[1].HallSpeed_mm_per_s;
    #else
      iSpeed = HallData[1].HallSpeed_mm_per_s;
    #endif
  } 

	long iSpeed_Goal = (cmd2 * 1000) / 36;  // mm_per_s

	if (	(abs(iSpeed_Goal) < 56)	&& (abs(cmd2Goal) < 50)	)	// iSpeed_Goal = 56 = 0.2 km/h
	{
	    cmd2 = cmd2Goal = 0;
	}	
#ifdef MAX_RECUPERATION
	else if ((currentL+currentR)/2 < -MAX_RECUPERATION)
	{
		cmd2Goal += 5;
		if (cmd2Goal > 1000)	cmd2Goal = 1000;
	}
#endif
	else if (iSpeed > (iSpeed_Goal + 56))	// 28 = 27.777 = 0.1 km/h
	{
		cmd2Goal -= CLAMP((iSpeed-iSpeed_Goal)/56,  1,3);
    if (  (iSpeed_Goal > 56)  && (cmd2Goal < 2)  ) cmd2Goal = 2;   // don't set backward speed when iSpeed_goal is set forwards
		else if (cmd2Goal < -1000)	cmd2Goal = -1000;
	}
	else if (iSpeed < (iSpeed_Goal -56))
	{
		//cmd2Goal += 3;
		cmd2Goal += CLAMP((iSpeed_Goal-iSpeed)/56,  1,3);
    if (  (iSpeed_Goal < -56)  && (cmd2Goal > -2)  ) cmd2Goal = -2;   // don't set forward speed when iSpeed_goal is set backwards
		else if (cmd2Goal > 1000)	cmd2Goal = 1000;

	}

  speed = cmd2Goal;
  steer = steer * (1.0 - FILTER) + cmd1 * FILTER;

#else
    // ####### LOW-PASS FILTER #######
    steer = steer * (1.0 - FILTER) + cmd1 * FILTER;
    speed = speed * (1.0 - FILTER) + cmd2 * FILTER;
#endif



    // ####### MIXER #######
    #ifdef SWITCH_WHEELS
      speedL = CLAMP(speed * SPEED_COEFFICIENT -  steer * STEER_COEFFICIENT, -1000, 1000);
      speedR = CLAMP(speed * SPEED_COEFFICIENT +  steer * STEER_COEFFICIENT, -1000, 1000);
    #else
      speedR = CLAMP(speed * SPEED_COEFFICIENT -  steer * STEER_COEFFICIENT, -1000, 1000);
      speedL = CLAMP(speed * SPEED_COEFFICIENT +  steer * STEER_COEFFICIENT, -1000, 1000);
    #endif

    #ifdef ADDITIONAL_CODE
      ADDITIONAL_CODE;
    #endif


    // ####### SET OUTPUTS #######
    if ((speedL < lastSpeedL + 50 && speedL > lastSpeedL - 50) && (speedR < lastSpeedR + 50 && speedR > lastSpeedR - 50) && timeout < TIMEOUT) {
    #ifdef INVERT_R_DIRECTION
      pwmr = speedR;
    #else
      pwmr = -speedR;
    #endif
    #ifdef INVERT_L_DIRECTION
      pwml = -speedL;
    #else
      pwml = speedL;
    #endif
    }




    lastSpeedL = speedL;
    lastSpeedR = speedR;


    //if (inactivity_timeout_counter % 25 == 0) 
    if (iLog*DELAY_IN_MAIN_LOOP > 200)	// log every 200 ms
    {
    	iLog = 0;
      // ####### CALC BOARD TEMPERATURE #######
      board_temp_adc_filtered = board_temp_adc_filtered * 0.99 + (float)adc_buffer.temp * 0.01;
      board_temp_deg_c = ((float)TEMP_CAL_HIGH_DEG_C - (float)TEMP_CAL_LOW_DEG_C) / ((float)TEMP_CAL_HIGH_ADC - (float)TEMP_CAL_LOW_ADC) * (board_temp_adc_filtered - (float)TEMP_CAL_LOW_ADC) + (float)TEMP_CAL_LOW_DEG_C;


//ROBO begin
#if defined DEBUG_SERIAL_FEEDBACK && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)

	    if(UART_DMA_CHANNEL->CNDTR == 0) 
	    {
			oFeedback.iSpeedL	= (int) (float)HallData[0].HallSpeed_mm_per_s * 0.36f;
			oFeedback.iSpeedR	= (int) (float)HallData[1].HallSpeed_mm_per_s * 0.36f;
			oFeedback.iHallSkippedL	= HallData[0].HallSkipped;
			oFeedback.iHallSkippedR	= HallData[1].HallSkipped;
			oFeedback.iTemp	= (int)	board_temp_deg_c;
			oFeedback.iVolt	= (int)	(batteryVoltage * 100.0f);
			oFeedback.iAmpL = (int) (currentL * 100.0f);
			oFeedback.iAmpR = (int)	(currentR * 100.0f);
			oFeedback.crc = 0;
			crc32((const void *)&oFeedback, sizeof(oFeedback)-4, &oFeedback.crc);

/*			oFeedback.iSpeedL	= 1;
			oFeedback.iSpeedR	= 2;
			oFeedback.iHallSkippedL	= 3;
			oFeedback.iHallSkippedR	= 4;
			oFeedback.iTemp	= (int)	5;
			oFeedback.iVolt	= (int)	6;
			oFeedback.iAmpL = (int)	7;
			oFeedback.iAmpR = (int)	8;
			oFeedback.crc = 0;
			crc32((const void *)&oFeedback, sizeof(oFeedback)-4, &oFeedback.crc);
*/


			UART_DMA_CHANNEL->CCR &= ~DMA_CCR_EN;
			UART_DMA_CHANNEL->CNDTR = sizeof(oFeedback);
			UART_DMA_CHANNEL->CMAR  = (uint32_t)&oFeedback;
			UART_DMA_CHANNEL->CCR |= DMA_CCR_EN;
	    }
#else
//ROBO end

      // ####### DEBUG SERIAL OUT #######
      #ifdef CONTROL_ADC
        setScopeChannel(0, (int)adc_buffer.l_tx2);  // 1: ADC1
        setScopeChannel(1, (int)adc_buffer.l_rx2);  // 2: ADC2
      #endif
      setScopeChannel(2, (int)speedR);  // 3: output speed: 0-1000
      setScopeChannel(3, (int)speedL);  // 4: output speed: 0-1000
      setScopeChannel(4, (int)adc_buffer.batt1);  // 5: for battery voltage calibration
      setScopeChannel(5, (int)(batteryVoltage * 100.0f));  // 6: for verifying battery voltage calibration
      setScopeChannel(6, (int)board_temp_adc_filtered);  // 7: for board temperature calibration
      setScopeChannel(7, (int)board_temp_deg_c);  // 8: for verifying board temperature calibration
      consoleScope();
#endif	//ROBO
    }


      // ####### POWEROFF BY POWER-BUTTON #######
      if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && weakr == 0 && weakl == 0) {
        enable = 0;
        int i = 0;
        while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {    // wait, till the power button is released. Otherwise cutting power does nothing
#ifdef SOFTWATCHDOG_TIMEOUT
          __HAL_TIM_SET_COUNTER(&htim3, 0); // Kick the Watchdog
          HAL_Delay(i++/2);                 // The watchdog will get you eventually..
#endif
        }
        poweroff();
      }


    // ####### BEEP AND EMERGENCY POWEROFF #######
    if (TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && ABS(speed) < 20) {  // poweroff before mainboard burns
      poweroff();
    } else if (batteryVoltage < ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && ABS(speed) < 20) {  // poweroff low bat 3
      poweroff();
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {  // beep if mainboard gets hot
      buzzerFreq = 4;
      buzzerPattern = 1;
    } else if (batteryVoltage < ((float)BAT_LOW_LVL1 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL1_ENABLE) {  // low bat 1: slow beep
      buzzerFreq = 5;
      buzzerPattern = 42;
    } else if (batteryVoltage < ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL2_ENABLE) {  // low bat 2: fast beep
      buzzerFreq = 5;
      buzzerPattern = 6;
    } else if (BEEPS_BACKWARD && speed < -50) {  // backward beep
      buzzerFreq = 5;
      buzzerPattern = 1;
    } else {  // do not beep
      if (buzzerLen > 0) {
        buzzerLen--;
      } else {
        buzzerFreq = 0;
        buzzerPattern = 0;
      }
    }


    // ####### INACTIVITY TIMEOUT #######
    if (ABS(speedL) > 50 || ABS(speedR) > 50) {
      inactivity_timeout_counter = 0;
    } else {
      inactivity_timeout_counter ++;
    }

    // inactivity 10s warning; 1s bleeping
    if ((inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 50 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) &&
        (buzzerFreq == 0)) {
      buzzerFreq = 3;
      buzzerPattern = 1;
      buzzerLen = 1000;
    }

    // inactivity 5s warning; 1s bleeping
    if ((inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 55 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) &&
        (buzzerFreq == 0)) {
      buzzerFreq = 2;
      buzzerPattern = 1;
      buzzerLen = 1000;
    }

    // power off after ~60s of inactivity
    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // rest of main loop needs maybe 1ms
          inactivity_timeout_counter = 0;
        poweroff();
    }


    if (powerofftimer > 0){
      powerofftimer --;

      // spit a msg every 2 seconds
      if (!(powerofftimer % (2000/DELAY_IN_MAIN_LOOP))){
        char tmp[30];
        sprintf(tmp, "power off in %ds\r\n", (powerofftimer*DELAY_IN_MAIN_LOOP)/1000 );
        consoleLog(tmp);
      }

      if (powerofftimer <= 10000/DELAY_IN_MAIN_LOOP){
        buzzerFreq = 3;
        buzzerPattern = 1;
        buzzerLen = 1000;
      }

      if (powerofftimer <= 5000/DELAY_IN_MAIN_LOOP){
        buzzerFreq = 2;
        buzzerPattern = 1;
        buzzerLen = 1000;
      }

      if (powerofftimer <= 0){
        powerofftimer = 0;
        poweroff();
      }
    }

#ifdef SOFTWATCHDOG_TIMEOUT
    __HAL_TIM_SET_COUNTER(&htim3, 0); // Kick the Watchdog
#endif
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Software Watchdog Actions
 * */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim3)
{
  while(SoftWatchdogActive) {

    // Stop Left Motor
    LEFT_TIM->LEFT_TIM_U = 0;
    LEFT_TIM->LEFT_TIM_V = 0;
    LEFT_TIM->LEFT_TIM_W = 0;
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;

    // Stop Right Motor
    RIGHT_TIM->RIGHT_TIM_U = 0;
    RIGHT_TIM->RIGHT_TIM_V = 0;
    RIGHT_TIM->RIGHT_TIM_W = 0;
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;

    // Just to be safe, set every variable which is somehow involved in motor control to safe values
    steer = 0;
    speed = 0;
    enable = 0;
    timeout = TIMEOUT + 1;
    pwml = 0;
    pwmr = 0;
    weakl = 0;
    weakr = 0;
    cmd1 = 0;
    cmd2 = 0;

    // shutdown power
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0); // shutdown  power
  }
  SoftWatchdogActive = true;
}