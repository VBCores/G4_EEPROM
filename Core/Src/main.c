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
#include "i2c.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  const char wmsg[] = "Some data"; //Данные которые хотим записать в EEPROM
	  char rmsg[sizeof(wmsg)]; //Массив в который будем записывать прочитанные данные из EEPROM
	  uint8_t str[100]; //Массив для красивого форматирования для отправки через Serial (не обязательно)

	  uint16_t memAddr = 0x0000; //Адрес в памяти EEPROM

	  if (at24_isConnected()) //Проверяем есть ли связь EEPROM
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1); //Если есть зажигаем LED 1
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0); //Если нет LED 1 не горит
	  }

	  at24_write(memAddr, wmsg, sizeof(wmsg), 100); // Записываем данные из wmsg в EEPROM

	  if(at24_read(memAddr, rmsg, sizeof(wmsg), 100)) //Читаем данные из EEPROM в rmsg
		{
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); //Мигаем LED 2, если все ок
		  sprintf(str, "Data: %s \r\n", rmsg); //Собираем красивую строку для Serial
		  HAL_UART_Transmit_IT(&huart2, str, sizeof(wmsg)+8); //Отправляем по Serial
		  HAL_Delay(1000);
		}
	  else
	  {
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); //Мигаем LED 1, если НЕ ок
		  HAL_Delay(1000);
	  }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
void CTRL_Reg_Set(){
	uint16_t TX = 0x0000;
	TX += (0x03 << 10); // 850 ns dead time
	TX += (0x03 << 8); // Gain of 40
	TX += (1 << 3); // 1/4 stepn
	TX += 0x01 ; // Enable motor
	RegAccess(WRITE, 0x00, TX); // write CTRL Register (Address = 0x00)
	return;
}

void TORQUE_Reg_Set(){
	uint16_t TX = 0x0000;
	TX += (0x01 << 8); // sample time = 100 us
	TX += 0x01; // Torque = 0x3F
//	TX = 0b0001000100000001;
	RegAccess(WRITE, 0x01, TX); // write TORQUE Register (Address = 0x01)
	return;
}

void STATUS_Reg_Set(){
	RegAccess(WRITE, 0x07, 0x0000); // write STATUS Register (Address = 0x00)
}



uint16_t RegAccess(uint8_t operation, uint8_t address, uint16_t value)
{
  uint16_t parcel = value;

  parcel += (address << 12); // register address
  parcel += (operation << 15); // read-write operation choice

  uint16_t received = 0;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
 // HAL_Delay(1);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&parcel, (uint8_t*)&received, 1, 1000);
 // HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
 // HAL_Delay(1);
  received &= ~0xF000; // clear upper 4 bits, leave lower 12 bits

  return received;
*/
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
