/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Defines.h"
#include "i2c-lcd.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Input_hi_limit1 280
#define Input_hi_limit2 270

#define Input_lo_limit1 170
#define Input_lo_limit2 180

#define RLY1_SET		HAL_GPIO_WritePin(RLY1_GPIO_Port, RLY1_Pin, GPIO_PIN_SET)
#define RLY1_RESET  HAL_GPIO_WritePin(RLY1_GPIO_Port, RLY1_Pin, GPIO_PIN_RESET)

#define RLY2_SET		HAL_GPIO_WritePin(RLY2_GPIO_Port, RLY2_Pin, GPIO_PIN_SET)
#define RLY2_RESET  HAL_GPIO_WritePin(RLY2_GPIO_Port, RLY2_Pin, GPIO_PIN_RESET)

#define RLY3_SET		HAL_GPIO_WritePin(RLY3_GPIO_Port, RLY3_Pin, GPIO_PIN_SET)
#define RLY3_RESET  HAL_GPIO_WritePin(RLY3_GPIO_Port, RLY3_Pin, GPIO_PIN_RESET)

#define RLY4_SET		HAL_GPIO_WritePin(RLY4_GPIO_Port, RLY4_Pin, GPIO_PIN_SET)
#define RLY4_RESET  HAL_GPIO_WritePin(RLY4_GPIO_Port, RLY4_Pin, GPIO_PIN_RESET)

#define Normal_Ind_SET      HAL_GPIO_WritePin(Normal_Ind_GPIO_Port, Normal_Ind_Pin, GPIO_PIN_SET)
#define Normal_Ind_RESET    HAL_GPIO_WritePin(Normal_Ind_GPIO_Port, Normal_Ind_Pin, GPIO_PIN_RESET)

#define Buck_Ind_SET        HAL_GPIO_WritePin(Buck_Ind_GPIO_Port, Buck_Ind_Pin, GPIO_PIN_SET)
#define Buck_Ind_RESET      HAL_GPIO_WritePin(Buck_Ind_GPIO_Port, Buck_Ind_Pin, GPIO_PIN_RESET)

#define Boost_Ind_SET       HAL_GPIO_WritePin(Boost_Ind_GPIO_Port, Boost_Ind_Pin, GPIO_PIN_SET)
#define Boost_Ind_RESET     HAL_GPIO_WritePin(Boost_Ind_GPIO_Port, Boost_Ind_Pin, GPIO_PIN_RESET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
flags flg;
mains_status mainsstatus;
uint8_t ms_2_cnt, two_ms_count, ten_ms_cnt, fifty_ms_count, hundred_ms_count,sec_count,min_count,hr_count;
uint16_t mains_input,stab_output;
uint8_t mains_dly_cnt, mains_dly_on_cnt;
uint8_t intr_count, fast_mains_ok_cnt;
uint8_t Avr_normal_dly_cnt,Avr_buck_dly_cnt,Avr_boost_dly_cnt,RLY3_off_delay,Avr_normal_1_dly_cnt;
char Temp_bufr[6];
uint8_t four_digit,ind;
uint32_t ADC_BUFR[2];
uint32_t UPS_REC[2];
char DataBuffer[21];
char temp_b1[6];
char temp_b2[6];
uint8_t scale[16]={3,4,1,1,1,1,1,1,1,1,1,1,1,1,1};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void check_mains_stages(void);
void check_mains(void);
void check_fast_mains(void);
void revu_analog(void);
void Display_refresh(void);
void ltoa(long num,char *string);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		uint8_t k;
		for(uint8_t chn_num = 0; chn_num < 2; chn_num++)
		{
				//UPS_REC[i] = ADC_BUFR[i];
			k = scale[chn_num];
		if (k)
       UPS_REC[chn_num] = ADC_BUFR[chn_num] >> scale[chn_num];
    else
       UPS_REC[chn_num] = ADC_BUFR[chn_num];
       ADC_BUFR[chn_num] = 0;
		}
		mains_input = UPS_REC[0];
		stab_output = UPS_REC[1];
}
void check_mains_stages(void)
{
		if(boost_flag)
    {
			Boost_Ind_RESET;//ON
			Buck_Ind_SET;
			Normal_Ind_SET;
			RLY2_SET;
			RLY3_SET;
			RLY4_RESET;//ON
    }
		if(Normal_flag)
		{
			Boost_Ind_SET;
			Buck_Ind_SET;
			Normal_Ind_RESET;//ON
			RLY2_SET;
			RLY3_RESET;//ON
			RLY4_SET;
		}
		if(Normal_1_flag)
		{
			Boost_Ind_SET;
			Buck_Ind_SET;
			Normal_Ind_RESET;//ON
			RLY2_RESET;//ON
			RLY3_SET;
			RLY4_SET;
		}
		if (Buck_flag)
		{
			Boost_Ind_SET;
			Buck_Ind_RESET;//ON
			Normal_Ind_RESET;
			RLY2_SET;
			RLY3_SET;
			RLY4_SET;	
		}
}
void check_mains(void)
{
		if((mains_input>Input_hi_limit1)||(mains_input<Input_lo_limit1))   //mains fail
    {
			mains_ok_flag = 0;
			mains_dly_cnt++;
			if (mains_dly_cnt > 5)
			{
				mains_dly_cnt = 0;
				RLY1_RESET;    
        Mains_Fail1 = 1;
        mains_dly_on_cnt = 0;
			}
		}
		else if((mains_input < Input_hi_limit2)&&(mains_input > Input_lo_limit2))  //mains ok
    {
        mains_ok_flag = 1;
        mains_dly_on_cnt++;
			  if(mains_dly_on_cnt >= 5)
			  {
					mains_dly_on_cnt = 0;
					mains_dly_cnt = 0;
					Mains_Fail  = 0;
					mains_abnormal = 0;
					HAL_Delay(2);
					RLY1_RESET;
       }
		}
		if((!Mains_Fail))
    {
			 if((mains_input > 180) && (mains_input < 200))//180,208
			 {
				 Avr_buck_dly_cnt++;
				 if(Avr_buck_dly_cnt == 5)
				 {
					 Avr_buck_dly_cnt = 0;
					 boost_flag = 0;
					 Normal_flag = 0;
					 Buck_flag = 1;
           Normal_1_flag = 0;
				 }
			 }
			 if((mains_input > 202) && (mains_input < 220))//218,240
			 {
					Avr_normal_dly_cnt++;
					if(Avr_normal_dly_cnt == 5)
					{
             Avr_normal_dly_cnt = 0;
             Normal_flag = 0;
             Buck_flag = 0;
             boost_flag = 0;
             Normal_1_flag = 1;
					}
			 }
			 if((mains_input > 222) && (mains_input < 240))
			 {
				  Avr_normal_1_dly_cnt++;
				  if(Avr_normal_1_dly_cnt == 5)
				  {
             Avr_normal_1_dly_cnt = 0;
             Normal_1_flag = 0;
             Normal_flag = 1;
             Buck_flag = 0;
             boost_flag = 0;
				  }
	     }
			 if((mains_input > 242) &&  (mains_input < 270))//249,270
			 {
	        Avr_boost_dly_cnt++;
	        if(Avr_boost_dly_cnt==5)
	        {
             Avr_boost_dly_cnt=0;
             boost_flag =1;
             Buck_flag =0;
             Normal_flag =0;
             Normal_1_flag =0;
	        }
	     }
		}
		if(Mains_Fail1)
    {
				if(++RLY3_off_delay >= 35)
        {
            Mains_Fail = 1;
            RLY3_off_delay = 0;
            Mains_Fail1 = 0;
        }
    }
}
void check_fast_mains(void)
{
    if(!HAL_GPIO_ReadPin(Mains_Fail_ip_GPIO_Port, Mains_Fail_ip_Pin))
    {
        if(!Main_Fail_ip_Temp_Flag)
                Main_Fail_ip_Temp_Flag=1;
        else
        {
            IPAB = 1;
            fast_mains_fail = 1;
            fast_mains_ok_cnt = 0;
            Main_Fail_ip_Temp_Flag = 0;
        }
    }
    else
    {
         fast_mains_ok_cnt++;
         if(fast_mains_ok_cnt >= 10)
         {
             IPAB = 0;
             fast_mains_fail = 0;
             Main_Fail_ip_Temp_Flag = 0;
         }
    }
}
void revu_analog(void)
{
	
}
void Display_refresh(void)
{
	lcd_send_cmd (0x80);
	lcd_send_string("Mains i/p  Stab o/p ");
	memset(DataBuffer,' ',20);
	ltoa(mains_input,temp_b1);
	ltoa(stab_output,temp_b2);
	
	DataBuffer[3] = temp_b1[0];
	DataBuffer[4] = temp_b1[1];
	DataBuffer[5] = temp_b1[2];
	DataBuffer[6] = temp_b1[3];
	DataBuffer[7] = 'V';
	
	DataBuffer[13] = temp_b1[0];
	DataBuffer[14] = temp_b1[1];
	DataBuffer[15] = temp_b1[2];
	DataBuffer[16] = temp_b1[3];
	DataBuffer[17] = 'V';
	
	lcd_send_cmd (0xC0);
	lcd_send_string(DataBuffer);
	
	lcd_send_cmd (0x94);
	
	lcd_send_cmd (0xD4);
	
}
void ltoa(long num,char *string)
{
      four_digit = 1;
      ind = 0;

      while(num)
      {
          Temp_bufr[ind++] = ((num%10) + (0x30));
          num=num/10;
      }

      if(ind>=0 && ind<=3)
      {
          if((ind!=3) || (ind==3 && four_digit))
             *string++='0';

          if(ind==0)
          {
           *string++='0';
           *string++='0';

           if(four_digit)
                  *string++='0';

          }
          else if(ind==1)
          {
           *string++='0';

           if(four_digit)
                  *string++='0';
          }
          else if(ind==2)
          {
           if(four_digit)
                  *string++='0';
          }
      }

      while(ind)
	   	 *string++=Temp_bufr[--ind];

      *string=0x0;

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	lcd_init ();
	lcd_clear ();
	lcd_send_cmd (0x80);
	lcd_send_string(" Saumya controllers ");
	lcd_send_cmd (0xC0);
	lcd_send_string("   Mb: 9604861056   ");
	if(HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start_DMA(&hadc1, ADC_BUFR, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			if(ten_ms_flag)
	    {					
				 if(++ten_ms_cnt >= 10)
				 {
					  ten_ms_cnt=0;
					  hundred_ms_flag = 1;
				 }
		     ten_ms_flag = 0;
				 revu_analog();
				 //check_mains();
         //check_mains_stages();
	    }
			if(fifty_ms_flag)
			{
				 fifty_ms_flag = 0;
				 hundred_ms_count++;
				 check_mains();
         check_mains_stages();
			}
			if(hundred_ms_flag)
			{
				 if(hundred_ms_count >= 10)
				 {
					 hundred_ms_count = 0;
					 one_sec = 1;
				 }
				 hundred_ms_flag = 0; 
			}
			if(one_sec)
			{
				 one_sec = 0;
			}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 18000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Normal_Ind_Pin|Buck_Ind_Pin|Boost_Ind_Pin|RLY3_Pin 
                          |RLY4_Pin|RLY1_Pin|RLY2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Normal_Ind_Pin Buck_Ind_Pin Boost_Ind_Pin RLY3_Pin 
                           RLY4_Pin RLY1_Pin RLY2_Pin */
  GPIO_InitStruct.Pin = Normal_Ind_Pin|Buck_Ind_Pin|Boost_Ind_Pin|RLY3_Pin 
                          |RLY4_Pin|RLY1_Pin|RLY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Mains_Fail_ip_Pin */
  GPIO_InitStruct.Pin = Mains_Fail_ip_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Mains_Fail_ip_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
