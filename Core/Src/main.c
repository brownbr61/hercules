/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define NUM_MOTORS 2

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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim6_up;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM6_Init(void);
void setDutyCycles();
void PI_update();
/* USER CODE BEGIN PFP */
struct LEDs {
  uint8_t red;
  uint8_t orange;
  uint8_t green;
  uint8_t blue;
  void (*set)(struct LEDs*);
} leds;

volatile int16_t target_tick_diff = 0;

struct Motor {
  enum DIR{F,R} dir;                      // Direction of movement (<0 = reverse; >0 = forward)
  uint8_t dirPin[2];

    struct PI_data {
    int16_t target_ticks;               // Aka desired speed target
    int16_t tick_rate;                 // Aka motor speed
    int16_t error_integral;            // Integrated error signal
    int16_t error_distance;            // Distance error signal
    int16_t total_ticks;                 // Aka motor speed
    uint16_t duty_cycle;
  } pi_data;

  void (*spin)(struct Motor*, uint16_t, int);
  void (*stop)(struct Motor*);
  void (*setCycle)(struct Motor*, uint8_t);
};

struct Motors {
  struct Motor l;
  struct Motor r;
} motors = { 
    {.dir = F, .dirPin = {4, 5}, .pi_data = {0,0,0,0,0,0}}, 
    {.dir = F, .dirPin = {6, 7}, .pi_data = {0,0,0,0,0,0}}
  };

// void initMotors(void);

void setLEDs(struct LEDs* this) {
  GPIOC->ODR &= ((this->green << 9) | (this->orange << 8) | (this->blue << 7) | (this->red << 6));
  GPIOC->ODR |= ((this->green << 9) | (this->orange << 8) | (this->blue << 7) | (this->red << 6));
}

void initLEDs(struct LEDs* this) {
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER &= ~(0xFF000);
  GPIOC->MODER |= 0x55000;
  this->red = 0;
  this->orange = 0;
  this->green = 0;
  this->blue = 0;
  this->set = &setLEDs;
}

void transmitValue(uint8_t val) {
  // Wait until our tx reg is ready
  volatile int wait = 1;
  while (wait)
    if (USART3->ISR & 0x80)
      wait = 0;
  // Write to tx reg
  USART3->TDR = val;
}

void transmit2bytes(uint16_t val) {
  transmitValue((uint8_t)(val >> 8));
  transmitValue((uint8_t)val);
}

void transmitChar(char c) {
    // Wait until our tx reg is ready
    volatile int wait = 1;
    while (wait)
        if (USART3->ISR & 0x80)
            wait = 0;
    // Write to tx reg
    USART3->TDR = c;
}

void transmitString(char* s) {
    while (*s != 0) {
        transmitChar(*s++);
    }
}

void initUart() {
  // Set PC10 and PC11 to alt fx mode
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER &= ~(0xF00000);    // Clear
  GPIOC->MODER |= 0xA00000;   		 // 1010

  // Set PC10 to AF1 for USART3_TX/RX
  GPIOC->AFR[1] &= ~(0xFF00);   	 // Clear
  GPIOC->AFR[1] |= 0x1100;   		 // Set 11 and 10 to 000

  // Enable system clock for USART3
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

  // Set Baud rate
  uint64_t t_baud = 9600;
  uint64_t f_clk = SystemCoreClock;
  uint64_t brr = f_clk/t_baud;
  USART3->BRR &= ~(0xFFFFFFFF);   	 // Clear
  USART3->BRR |= brr;

  // Enable tx
  USART3->CR1 |= (1 << 3);

  // Enable rx
  USART3->CR1 |= (1 << 2);

  // Enable rx not-empty interrupt
  USART3->CR1 |= (1 << 5);

  // Enable USART
  USART3->CR1 |= (1 << 0);

  // Enable USART3 interrupt on NVIC
  NVIC_EnableIRQ(USART3_4_IRQn);
  NVIC_SetPriority(USART3_4_IRQn, 1);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM6_Init();
  initLEDs(&leds);
  initUart();
  
  int i = 0;
  while (1)
  {
    i = i % 100;
    if (i == 0) 
    {
      leds.red = !leds.red;
      leds.set(&leds);
    }
    PI_update();
    setDutyCycles();
    
    i++;
    HAL_Delay(5);
  }
}

struct Tick_Counts {
  int16_t l;
  int16_t r;
} ticks;

/**
  * @brief This function handles EXTI line 2 and 3 interrupts.
  */
void EXTI2_3_IRQHandler(void)
{

  if (EXTI->PR & (1 << 2))
  {
    leds.green = !leds.green;
    ticks.l += (motors.l.dir == F) - (motors.l.dir == R);
  }
  if (EXTI->PR & (1 << 3))
  {
    leds.orange = !leds.orange;
    ticks.r += (motors.r.dir == F) - (motors.r.dir == R);
  }
  leds.set(&leds);

  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

/**
  * @brief This function handles TIM6 global and DAC channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  leds.blue = !leds.blue;
  leds.set(&leds);

  motors.l.pi_data.tick_rate = ticks.l;
  ticks.l = 0;
  motors.r.pi_data.tick_rate = ticks.r;
  ticks.r = 0;

  motors.l.pi_data.total_ticks += motors.l.pi_data.tick_rate;
  motors.r.pi_data.total_ticks += motors.r.pi_data.tick_rate;

  TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

// Calculates error for single motor and resets PWM signal
/* Needs:
  - 
  */
void PI_update() {

  int16_t meanTicks = (motors.l.pi_data.total_ticks + motors.r.pi_data.total_ticks) >> 1;

  struct Motor* myMotor = &motors.l;

  int16_t error;
  int16_t angular_error;
  int16_t duty_cycle;

  int16_t Kp = 1;
  int16_t Ki = 1;
  int16_t Ka = 1;

  angular_error = motors.l.pi_data.total_ticks - motors.r.pi_data.total_ticks - target_tick_diff;

  for (int i = 0; i < NUM_MOTORS; i++) {
    // error calculation
    error = myMotor->pi_data.target_ticks - myMotor->pi_data.tick_rate;

    // error integral calculation
    myMotor->pi_data.error_integral += error;
    myMotor->pi_data.error_integral = (myMotor->pi_data.error_integral > 3200) ? 3200 : myMotor->pi_data.error_integral;
    myMotor->pi_data.error_integral = (myMotor->pi_data.error_integral < 0) ? 0 : myMotor->pi_data.error_integral; // do I want negative values?

    // calculate output
    duty_cycle = (((error) << 1) + angular_error + myMotor->pi_data.error_integral) >> 5;
    myMotor->pi_data.duty_cycle = (duty_cycle > 100) ? 100 : duty_cycle;
    myMotor++;
  }
}

// Set the duty cycle of the PWM, accepts (0-100)
void setDutyCycles() {
  struct Motor* myMotor = &motors.l;
  uint16_t duty = motors.l.pi_data.duty_cycle;

  for (int i = 0; i < NUM_MOTORS; i++) {
    duty = myMotor->pi_data.duty_cycle;
    if(duty <= 100) {
        switch (i) {
            case 0:
                TIM2->CCR1 = duty;
                break;
            case 1:
                TIM2->CCR2 = duty;
                break;
        }
    }
    myMotor++;
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

  // // Select PSC and ARR values that give an appropriate interrupt rate
  TIM6->PSC = 11;
  TIM6->ARR = 30000;

  TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
  TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

  NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
  NVIC_SetPriority(TIM6_DAC_IRQn, 2);

}

// void initMotors() {
//     int pwm_in_pins[] = {0, 1};
//     int mtr_A_dir_pins[] = {4, 5};
//     int mtr_B_dir_pins[] = {6, 7};

//     for (int i = 0; i < NUM_MOTORS; i++) {
//         struct Motor *motor = &(motors[i]);
//         motor->side = i; // Range is 1-2
//         motor->direction = 1;
//         motor->target_ticks = 100;
//         motor->num_ticks = 0;
//         motor->error = 0;
//         motor->error_integral = 0;

//         // motor->setCycle = &pwm_setDutyCycle;
//         // motor->spin = &spinMotor;
//         // motor->stop = &stopMotor;

//         motor->pwmGpio = GPIOA;
//         motor->pwm_in_pin = pwm_in_pins[i];

//         motor->dirGpio = GPIOB;
//         switch (i) {
//             case 0:
//                 motor->dir_pin_A = mtr_A_dir_pins[0];
//                 motor->dir_pin_B = mtr_A_dir_pins[1];
//                 break;
//             case 1:
//                 motor->dir_pin_A = mtr_B_dir_pins[0];
//                 motor->dir_pin_B = mtr_B_dir_pins[1];
//                 break;
//         }
//     }
// }

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */

  // Set up pin PA0-1 for H-bridge PWM output (TIMER 2 CH1-2)
  GPIOA->MODER |= (1 << 1);
  GPIOA->MODER &= ~(1 << 0);
  GPIOA->MODER |= (1 << 3);
  GPIOA->MODER &= ~(1 << 2);

  // Set PA0-1 to AF1
  GPIOA->AFR[0] &= 0xFFFFFF00; // clear PA4 bits,
  GPIOA->AFR[0] |= (1 << 1);
  GPIOA->AFR[0] |= (1 << 5);
  

  // Set up PWM timer
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->CR1 = 0;                          // Clears; defaults to edge-aligned upcounting
  TIM2->CCMR1 = 0;                        // (prevents having to manually clear bits)
  TIM2->CCER = 0;

  // Set output-compare CH1-4 to PWM1 mode and enable CCR1 preload buffer
  TIM2->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); //| TIM_CCMR1_OC1PE); // Enable channel 1
  TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1); //| TIM_CCMR1_OC2PE); // Enable channel 2

  TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);        // Enable capture-compare channel 1-2
  TIM2->PSC = 1;                         // Run timer on 24Mhz
  TIM2->ARR = 2400;                      // PWM at 20kHz

  TIM2->CCR1 = 2000;                        // Start PWMs at 0% duty cycle
  TIM2->CCR2 = 2000;

  TIM2->CR1 |= TIM_CR1_CEN;              // Enable timer
  // GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

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
