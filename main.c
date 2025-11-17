/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body Final Project SP2025
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "seg7.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char ramp = 0;
char RED_BRT = 0;
char GREEN_BRT = 0;
char BLUE_BRT = 0;
char RED_STEP = 1;
char GREEN_STEP = 2;
char BLUE_STEP = 3;
char DIM_Enable = 0;
char Music_ON = 0;
int TONE = 0;
int COUNT = 0;
int INDEX = 0;
int Note = 0;
int Save_Note = 0;
int Vibrato_Depth = 1;
int Vibrato_Rate = 40;
int Vibrato_Count = 0;
char Animate_On = 0;
char Message_Length = 0;
char *Message_Pointer;
char *Save_Pointer;
int Delay_msec = 0;
int Delay_counter = 0;
char time_field = 0;  // 0=seconds, 1=minutes, 2=hours
char alarm_field = 0;  // 0=seconds, 1=minutes, 2=hours (for alarm setting)
char last_pc10_state = 1;  // Track button state for edge detection

/* HELLO ECE-330L */
char Message[] =
		{SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,
		CHAR_H,CHAR_E,CHAR_L,CHAR_L,CHAR_O,SPACE,CHAR_E,CHAR_C,CHAR_E,DASH,CHAR_3,CHAR_3,CHAR_0,CHAR_L,
		SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE};

/* Declare array for Song */
Music Song[100];
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
  MX_TIM7_Init();
  //MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /********************************************************************
   * PWR->CR |= ???;  //Enable Real Time Clock (RTC) Register Access  *
   * RCC->BDCR |= ???;  //Set clock source for RTC                    *
   * RCC->BDCR |= ???; //Enable RTC									  *
   ********************************************************************/

  PWR->CR |= (1 << 8);  // Enable Real Time Clock Register Access

  RCC->BDCR |= (1 << 9);  // Set RTCSEL[1:0] = 10
  RCC->BDCR &= ~(1 << 8); // Clear bit 8 for RTCSEL = 10
  RCC->BDCR |= (1 << 15); // Enable RTC clock

  RTC->WPR = 0xCA;  // Write first key
  RTC->WPR = 0x53;  // Write second key

  RTC->ISR |= (1 << 7);  // Set INIT bit to enter initialization mode

  while (!(RTC->ISR & (1 << 6)));  // Wait for INITF flag

  RTC->PRER = 0x102;        // Set synchronous prescaler to 258
  RTC->PRER |= 0x007F0000;  // Set asynchronous prescaler to 127

  RTC->TR = 0x00120000;  // HT=1, HU=2, MNT=0, MNU=0, ST=0, SU=0
  
  RTC->DR = 0x00012501;  // YT=2, YU=5, WDU=1 (Monday), MT=0, MU=1, DT=0, DU=1

  RTC->ISR &= ~(1 << 7);  // Clear INIT bit to exit initialization mode

  RTC->CR &= ~(1 << 8);  // Clear ALRAE to disable Alarm A

  while (!(RTC->ISR & (1 << 0)));  // Wait for ALRAWF flag

  RTC->ALRMAR = 0x00120030;  // HT=1, HU=2, MNT=0, MNU=0, ST=3, SU=0
  RTC->ALRMAR |= (1 << 31);  // MSK4 = 1
  
  RTC->CR |= (1 << 12);  // Enable Alarm A interrupt
  RTC->CR |= (1 << 8);   // Enable Alarm A

  GPIOD->MODER = 0x55555555; // set all Port D pins to outputs
  GPIOA->MODER |= 0x000000FF; // Port A mode register - make A0 to A3 analog pins
  GPIOE->MODER |= 0x55555555; // Port E mode register - make E0 to E15 outputs
  GPIOC->MODER |= 0x0; // Port C mode register - all inputs
  GPIOE->ODR = 0xFFFF; // Set digit select pins high

  RCC->APB2ENR |= 1<<8;  // Turn on ADC1 clock by forcing bit 8 to 1 while keeping other bits unchanged
  ADC1->SMPR2 |= 1; // 15 clock cycles per sample
  ADC1->CR2 |= 1;        // Turn on ADC1 by forcing bit 0 to 1 while keeping other bits unchanged

  /*****************************************************************************************************
  These commands are handled as part of the MX_TIM7_Init() function and don't need to be enabled
  RCC->AHB1ENR |= 1<<5; // Enable clock for timer 7
  __enable_irq(); // Enable interrupts
  NVIC_EnableIRQ(TIM7_IRQn); // Enable Timer 7 Interrupt in the NVIC controller
  *******************************************************************************************************/

  TIM7->PSC = 199; //250Khz timer clock prescaler value, 250Khz = 50Mhz / 200
  TIM7->ARR = 1; // Count to 1 then generate interrupt, 125Khz interrupt rate to increment byte counter for 78Hz PWM
  TIM7->DIER |= 1; // Enable timer 7 interrupt
  TIM7->CR1 |= 1; // Enable timer counting

  Song[0].note = A4;
  Song[0].size = quarter;
  Song[0].tempo = 1400;
  Song[0].space = 10;
  Song[0].end = 0;

  Save_Note = Song[0].note;  // Needed for vibrato effect
  INDEX = 0;
  Music_ON = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Message_Pointer = &Message[0];
  Save_Pointer = &Message[0];
  Message_Length = sizeof(Message)/sizeof(Message[0]);
  Delay_msec = 200;
  Animate_On = 0;


  while (1)
  {
    /* USER CODE BEGIN 3 */
    
    // Check if PC0 is LOW (time setting mode)
    if (!(GPIOC->IDR & (1 << 0)))
    {
      // Check PC10 button for field switching (with debounce)
      char pc10_state = (GPIOC->IDR & (1 << 10)) ? 1 : 0;
      if (pc10_state == 0 && last_pc10_state == 1)  // Falling edge (button pressed)
      {
        time_field = (time_field + 1) % 3;  // Cycle through 0, 1, 2
        HAL_Delay(200);  // Debounce delay
      }
      last_pc10_state = pc10_state;
      
      ADC1->SQR3 = 1;  // Select ADC channel 1 (PA1)
      ADC1->CR2 |= (1 << 30);  // Start conversion
      while (!(ADC1->SR & (1 << 1)));  // Wait for conversion complete
      uint32_t adc_value = ADC1->DR;  // Read ADC value
      
      uint8_t value;
      uint32_t value_bcd;
      uint32_t mask;
      uint32_t shift;
      
      if (time_field == 0)  // Seconds
      {
        value = (adc_value * 60) / 4096;  // 0-59
        mask = 0x7F;  // Bits 6:0
        shift = 0;
      }
      else if (time_field == 1)  // Minutes
      {
        value = (adc_value * 60) / 4096;  // 0-59
        mask = 0x7F00;  // Bits 14:8
        shift = 8;
      }
      else  // Hours (time_field == 2)
      {
        value = (adc_value * 24) / 4096;  // 0-23
        mask = 0x3F0000;  // Bits 21:16
        shift = 16;
      }
      
      // Convert value to BCD format
      uint8_t tens = value / 10;
      uint8_t ones = value % 10;
      value_bcd = ((tens << 4) | ones) << shift;
      
      RTC->WPR = 0xCA;  // Unlock write protection
      RTC->WPR = 0x53;
      RTC->ISR |= (1 << 7);  // Enter init mode
      while (!(RTC->ISR & (1 << 6)));  // Wait for INITF
      
      // Update the selected field in RTC->TR
      RTC->TR = (RTC->TR & ~mask) | value_bcd;
      
      // Exit initialization mode
      RTC->ISR &= ~(1 << 7);
      
      // Display the current time
      Seven_Segment(RTC->TR);
    }
    // Check if PC1 is LOW (alarm setting mode)
    else if (!(GPIOC->IDR & (1 << 1)))
    {
      // Check PC10 button for field switching (with debounce)
      char pc10_state = (GPIOC->IDR & (1 << 10)) ? 1 : 0;
      if (pc10_state == 0 && last_pc10_state == 1)  // Falling edge (button pressed)
      {
        alarm_field = (alarm_field + 1) % 3;  // Cycle through 0, 1, 2
        HAL_Delay(200);  // Debounce delay
      }
      last_pc10_state = pc10_state;
      
      ADC1->SQR3 = 1;  // Select ADC channel 1 (PA1)
      ADC1->CR2 |= (1 << 30);  // Start conversion
      while (!(ADC1->SR & (1 << 1)));  // Wait for conversion complete
      uint32_t adc_value = ADC1->DR;  // Read ADC value
      
      uint8_t value;
      uint32_t value_bcd;
      uint32_t mask;
      uint32_t shift;
      
      if (alarm_field == 0)  // Seconds
      {
        value = (adc_value * 60) / 4096;  // 0-59
        mask = 0x7F;  // Bits 6:0
        shift = 0;
      }
      else if (alarm_field == 1)  // Minutes
      {
        value = (adc_value * 60) / 4096;  // 0-59
        mask = 0x7F00;  // Bits 14:8
        shift = 8;
      }
      else  // Hours (alarm_field == 2)
      {
        value = (adc_value * 24) / 4096;  // 0-23
        mask = 0x3F0000;  // Bits 21:16
        shift = 16;
      }
      
      uint8_t tens = value / 10;
      uint8_t ones = value % 10;
      value_bcd = ((tens << 4) | ones) << shift;
      
      RTC->CR &= ~(1 << 8);  // Clear ALRAE
      while (!(RTC->ISR & (1 << 0)));  // Wait for ALRAWF flag
      
      RTC->ALRMAR = (RTC->ALRMAR & ~mask) | value_bcd;
      
      RTC->CR |= (1 << 8);  // Set ALRAE
      
      Seven_Segment(RTC->ALRMAR);
    }
    else
    {
      Seven_Segment(RTC->TR);
    }
    
    HAL_Delay(100);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}






static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
