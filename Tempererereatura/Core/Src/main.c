/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "bmp2_config.h"
#include "bmp2_defs.h"
#include "bmp2.h"
#include "LCD.h"


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
float current_temp_f;
char current_temp_ch_UART[29];
char current_temp_ch_LCD[29];
float set_temp_f = 25.0;

double akt_temp = 0.0f;
float akt_temp_f= 0.0f;

char set_temp_ch_UART[24];
char set_temp_ch_LCD[24];
int32_t pressure;

uint32_t enc_uint;
static uint32_t prev_enc_uint = 65535 / 2;
int32_t enc_diff_int;
int suma_enkodera;

char enc_ch[64];

float pwm_duty_f;
uint16_t pwm_duty_u = 500;

char get_UART[10];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Controller{
	float Kp;
	float Ki;
	float Kd;
	float Tp;
	float prev_error;
	float prev_u_I;

};


float calculate_PID(struct Controller *PID, float set_temp, float meas_temp){
	float u = 0;
	float error;
	float u_P, u_I , u_D;

	error = set_temp - meas_temp;

	// Proportional
	u_P = PID->Kp * error;

	// Integral
	u_I = PID->Ki * PID->Tp / 2.0 * (error + PID->prev_error) + PID->prev_u_I;
	PID->prev_u_I = u_I;

	// Derivative
	u_D = (error - PID->prev_error) / PID->Tp;

	PID->prev_error = error;

	// Sum of P, I and D components
	u = u_P + u_I + u_D;

	return u;
}

struct Controller PID1;



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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  // TIM1 used used for controlling MOSFET with PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // TIM3 used for sampling temperature and PID controller
  HAL_TIM_Base_Start_IT(&htim3);

  // TIM4 used for encoder
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // BMP280 (temperature sensor) init
  BMP2_Init(&bmp2dev);

  // Prevents from bugging set_temp_f when encoder counter value goes through 0
  htim4.Instance->CNT = 65535 / 2;

  LCD_init();
  LCD_write_command(LCD_CLEAR_INSTRUCTION);
  LCD_write_command(LCD_HOME_INSTRUCTION);
  HAL_Delay(2000);

  // Initialize PID Controller parameters and init data
  PID1.Kp = 60;
  PID1.Ki = 0.1;
  PID1.Kd =30;
  PID1.Tp = 1;
  PID1.prev_error = 0;
  PID1.prev_u_I = 0;

  // Get setpoint value from user
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)get_UART, 10);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // ENCODER
	  //enc_uint = __HAL_TIM_GET_COUNTER(&htim4);	// enc_uint = htim4.Instance->CNT;
	  enc_uint = htim4.Instance->CNT;
	  enc_diff_int = enc_uint - prev_enc_uint;
	  if(enc_diff_int >= 2 || enc_diff_int <= -2){
		  enc_diff_int /= 2;
		  set_temp_f += enc_diff_int;//*0.5;
		  if(set_temp_f > 65) set_temp_f = 65;
		  if(set_temp_f < 20) set_temp_f = 20;
	  }
	  prev_enc_uint = enc_uint;

	  // LCD
	  snprintf(current_temp_ch_LCD, LCD_MAXIMUM_LINE_LENGTH, "Temp:  %.2f", current_temp_f);
	  LCD_write_text(current_temp_ch_LCD);
	  LCD_write_data(LCD_CHAR_DEGREE);
	  LCD_write_char('C');
	  snprintf(set_temp_ch_LCD, LCD_MAXIMUM_LINE_LENGTH, "Set T: %.2f", set_temp_f);
	  LCD_goto_line(1);
	  LCD_write_text(set_temp_ch_LCD);
	  LCD_write_data(LCD_CHAR_DEGREE);
	  LCD_write_char('C');
	  HAL_Delay(100);
	  LCD_write_text("                ");
	  LCD_write_command(LCD_HOME_INSTRUCTION);
	  akt_temp=BMP2_ReadTemperature_degC(&bmp2dev);
	  // Reset data from UART
	  memset(get_UART, 0, 10);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/* USER CODE BEGIN 4 */
// TIMERS callback handling
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		static unsigned int cnt = 0;
		cnt++;
		// TEMPERATURE
		BMP2_ReadData(&bmp2dev, &pressure, &current_temp_f);
		akt_temp=BMP2_ReadTemperature_degC(&bmp2dev);
		akt_temp_f=akt_temp;
		//current_temp_f=20;
		sprintf(current_temp_ch_UART, "Current temperature: %.2f\n\r", akt_temp_f);
		HAL_UART_Transmit(&huart3, (uint8_t *)current_temp_ch_UART, sizeof(current_temp_ch_UART)-1, 1000);

		sprintf((char*)set_temp_ch_UART, "Set temperature: %.2f\n\r", set_temp_f);
		HAL_UART_Transmit(&huart3, (uint8_t*)set_temp_ch_UART, strlen(set_temp_ch_UART), 1000);

		pwm_duty_f = (htim1.Init.Period * calculate_PID(&PID1, set_temp_f, akt_temp));

		// Saturation
		if(pwm_duty_f < 0.0) pwm_duty_u = 0;
		else if(pwm_duty_f > htim1.Init.Period) pwm_duty_u = htim1.Init.Period;
		else pwm_duty_u = (uint16_t) pwm_duty_f;

//		pwm_duty_u = htim1.Init.Period;		// 100% PWM duty for creating model
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty_u);

	}

}

// UART callback handling
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART3){
		float tmp = atof(get_UART);
		if(tmp < 20) set_temp_f = 20;
		else if(tmp > 65) set_temp_f = 65;
		else set_temp_f = tmp;

		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)get_UART, 10);
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
