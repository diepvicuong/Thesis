/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "IntToStr6.h"
#include "pid.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
pid_t wheel_r, wheel_l;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void pwmLeft(float energy,int dir);
void controlSpeedLeft(int dir);
void pwmRight(float energy,int dir);
void controlSpeedRight(int dir);
void split(char str[15]);
uint8_t signVel(float vel);  

long int speedRight=0, speedLeft=0;
long int pre_speedRight = 0;
uint8_t receive[15];
uint8_t buff[32];

int flag_uart=0,flag_tim=0;
//=================================
float vel_right = 0,vel_left = 0;		// can not convert from string to float in format "000-xx"
float last_velright =0, last_velleft = 0;

float kp_r = 1.2, ki_r=0.1, kd_r=0.01;	// right_wheel v2
float kp_l = 1.0, ki_l=0.0, kd_l=0.01;		// left_wheel v2
long int temp =0 ,temp1 = 0;
long int enc_left=0;
int sign_r = 0, sign_l = 0;
uint8_t tim_cnt=0;
float debug_right, debug_left;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim11);

	pidReset(&wheel_l);
	setInputBoder(&wheel_l, -110, 110);
	setOutputBoder(&wheel_l, -500, 500);

	pidReset(&wheel_r);
	setInputBoder(&wheel_r, -110, 110);
	setOutputBoder(&wheel_r, -500, 500);
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_TC);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	flag_tim=0;
	HAL_UART_Receive_IT(&huart1,receive,15);
  while (1)
  {
//		pwmRight(200,1);
//		temp1 = TIM2->CNT;
//		pwmLeft(200,1);
//		temp = TIM5->CNT;	
		
		if(flag_uart==1)
		{
			flag_uart = 0;
			split(receive);
//  			memset(receive, 0, 15);
		}	
		if(flag_tim==1)		// interrupt per 0.05s
		{				
				flag_tim=0;		
				
				setPoint(&wheel_l, fabs(vel_left*28.65));
				if(vel_left !=0)
					sign_l = signVel(vel_left);					
				controlSpeedLeft(sign_l);
				
				setPoint(&wheel_r, fabs(vel_right*28.65));	
				if(vel_right != 0)
					sign_r = signVel(vel_right);
				controlSpeedRight(sign_r);
							
					enc_left 	= TIM5->CNT;
					temp1 = TIM2->CNT;
					buff[0]=' ';
					IntToStr6(enc_left, buff, 1);
					buff[6] = ' ';
					IntToStr6(temp1, buff, 7);
					buff[12] = '\r';
					buff[13] = '\n';
					HAL_UART_Transmit_IT(&huart6, buff, 14);
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim2, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM5 init function */
void MX_TIM5_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim5, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

}

/* TIM11 init function */
void MX_TIM11_Init(void)
{

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 420;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 9999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim11);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART6 init function */
void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart6);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance)
	{
		flag_uart=1;
		HAL_UART_Receive_IT(&huart1,receive,15);
		
	}
}		
void split(char str[15])
{
	 char *a1,*a2;
   const char s[2] = "/";
   char *token;
   int index=0;
   /* lay token dau tien */
   token = strtok(str, s);
   /* duyet qua cac token con lai */
   while( token != NULL ) 
   {
			if(index==0)
				a1=token;
			else if(index==1)
				a2=token;
		  index++;		 
      token = strtok(NULL, s);
   }
			vel_right  = atof(a1);
			vel_left   = atof(a2);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==htim11.Instance)
		
			flag_tim=1;
		
}

//============================================================
void controlSpeedLeft(int dir)
{
			static long int pulse=0,prePulse=0;
			static float  check_left = 0;
	
			pulse = TIM5->CNT;
			if(dir==0)
			{
					speedLeft = pulse - prePulse;
					if(speedLeft < 0) 
							speedLeft = 2000 + speedLeft;
			}
			else if(dir==1)
			{
					speedLeft = -(pulse - prePulse);
					 if(speedLeft<0) 
							speedLeft = 2000 + speedLeft;
			}

				setProcessVariable(&wheel_l, speedLeft);
			check_left += pidFunction2(&wheel_l, kp_l, ki_l, kd_l, 0.01); 	
			debug_left = check_left;
			if(check_left<0)
				{
					check_left = -check_left;;							
				}
			if(check_left < 140 && vel_left == 0)		
					{
							HAL_TIM_Encoder_Stop(&htim5, TIM_CHANNEL_1);
							HAL_TIM_Encoder_Stop(&htim5, TIM_CHANNEL_2);						
							HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
					}		

			if(vel_left != 0)
			{
					HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1);
					HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_2);
					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			}

			pwmLeft(check_left,dir);
			prePulse=pulse;
}	

void pwmLeft(float energy,int dir)
{
	//	energy = -energy;
			if(dir==1) energy = -energy;
			if(energy<0)
			{
				TIM3->CCR1 = -energy; 
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	
			}
			if(energy>=0)
			{
				TIM3->CCR1 = energy;
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);	
			}
}
//===========================================================================
void controlSpeedRight(int dir)
{
			static long int pulse=0,prePulse=0;
			static float check_right = 0;
			pulse=TIM2->CNT;
			if(dir==0)
			{
					speedRight = pulse - prePulse;
					if(speedRight<0) 
						speedRight = 2000 + speedRight;	
			}
			else if(dir==1)
			{
					speedRight = -(pulse - prePulse);
					if(speedRight<0) 
							speedRight = 2000 + speedRight;
			}
				setProcessVariable(&wheel_r ,speedRight);
			check_right += pidFunction2(&wheel_r ,kp_r, ki_r, kd_r, 0.01);
			debug_right = check_right;
			if(check_right<0)
				{
					check_right = -check_right;				
				}			
			if( check_right < 140 && vel_right==0)
			{
					HAL_TIM_Encoder_Stop(&htim2,TIM_CHANNEL_1);	
					HAL_TIM_Encoder_Stop(&htim2,TIM_CHANNEL_2);					
					HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			}				
			if(vel_right!=0)
			{
				HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			}
			pwmRight(check_right,dir);
			prePulse = pulse;

}
void pwmRight(float energy,int dir)
{
//			energy = -energy;
			if(dir==1) energy = -energy;
			if(energy<0)
			{
				TIM3->CCR2=-energy;
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);	
			}
			if(energy>=0)
			{
				TIM3->CCR2=energy;
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);	
			}
}
uint8_t signVel(float vel)
{
	if(vel>0) 
		return 0;
	else if(vel<0)
		return 1;
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
