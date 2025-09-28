/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
uint8_t step = 1;
uint16_t duty_cycle = 1800; // 50% 占空比 (1800/3600)
uint16_t speed_delay = 50;   // 换相延迟 (ms), 控制转速
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void BLDC_Commutate(uint8_t step, uint16_t duty);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_MOE_ENABLE(&htim1); // 使能TIM1主输出
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    BLDC_Commutate(step, duty_cycle);
    HAL_Delay(speed_delay);
    step++;
    if(step > 6)
    {
      step = 1;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void BLDC_Commutate(uint8_t step, uint16_t duty)
{
    TIM_OC_InitTypeDef sConfigOC_PWM = {0};
    TIM_OC_InitTypeDef sConfigOC_LOW = {0};

    // 配置PWM模式
    sConfigOC_PWM.OCMode = TIM_OCMODE_PWM1;
    sConfigOC_PWM.Pulse = duty;
    sConfigOC_PWM.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC_PWM.OCFastMode = TIM_OCFAST_DISABLE;

    // 配置强制低电平模式
    sConfigOC_LOW.OCMode = TIM_OCMODE_FORCED_INACTIVE;
    sConfigOC_LOW.Pulse = 0;
    sConfigOC_LOW.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC_LOW.OCFastMode = TIM_OCFAST_DISABLE;

    // 停止所有通道，进入高阻态
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);

    switch(step)
    {
        case 1: // U->V, W Hi-Z
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_PWM, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC_LOW, TIM_CHANNEL_2);
            HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
            break;
        case 2: // W->V, U Hi-Z
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_PWM, TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC_LOW, TIM_CHANNEL_2);
            HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
            break;
        case 3: // W->U, V Hi-Z
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_PWM, TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC_LOW, TIM_CHANNEL_1);
            HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
            break;
        case 4: // V->U, W Hi-Z
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_PWM, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC_LOW, TIM_CHANNEL_1);
            HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
            break;
        case 5: // V->W, U Hi-Z
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_PWM, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC_LOW, TIM_CHANNEL_3);
            HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);
            break;
        case 6: // U->W, V Hi-Z
            HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_PWM, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC_LOW, TIM_CHANNEL_3);
            HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);
            break;
    }
}
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
