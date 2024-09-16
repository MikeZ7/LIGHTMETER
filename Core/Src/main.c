/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "crc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "veml7700.h"
#include "camera_params.h"

//#include "ssd1306_conf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VEML6030_I2C_ADDRESS    0x10
//Bronica ETRS
//#define ISO_TABLE_SIZE 6
#define APERTURE_F_TABLE_SIZE 7
#define APERTURE_TIME_TABLE_SIZE 7
#define SETTINGS_NUMBER 2


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

APERTURE_RECTANGLE_POS_t aperture_coordinates = {5, 60, 45, 60};
EXPOSURE_RECTANGLE_POS_t exposure_coordinates = {65, 125, 45, 60};


volatile uint8_t state = 0;
volatile uint32_t counter = 0;

double aperture_f = 2.8;
uint16_t exposure_time = 8;
uint8_t EV = 0;

uint8_t exposure_time_list_index = 1;
uint8_t aperture_f_list_index = 0;

//uint16_t exposure_times_list[APERTURE_TIME_TABLE_SIZE] = {8, 15, 30, 60, 125, 250, 500};
//double apertures_f_list[APERTURE_F_TABLE_SIZE] = {2.8, 4.0, 5.6, 8.0, 11.0, 16.0, 22.0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_initial_veml_params(veml6030 veml){
	veml6030_init(&veml, &hi2c1, VEML6030_I2C_ADDRESS);
	veml6030_power_on(&veml);
	veml6030_set_als_integration_time(&veml, REG_ALS_CONF_IT25);
	veml6030_set_als_gain(&veml, REG_ALS_CONF_GAIN_1_8);
}

void display_initial_description(void){
	ssd1306_SetCursor(0, 30);
	ssd1306_WriteString("Aperture:", Font_6x8, White);
	ssd1306_SetCursor(60, 30);
	ssd1306_WriteString("Exposure:", Font_6x8, White);
	ssd1306_UpdateScreen();
}

uint16_t the_closest_integer_from_list(uint16_t value, uint16_t *arr, size_t size) {
	uint16_t closest = *arr;  // Initialize with the first element
    uint16_t min_diff = abs(value - *arr);  // Compute the absolute difference

    for (size_t i = 1; i < size; i++) {
    	uint16_t diff = abs(value - *(arr+i));  // Calculate the difference for the current element

        if (diff < min_diff) {
            min_diff = diff;
            closest = *(arr+i);  // Update the closest value
        }
    }
    return closest;
}


double the_closest_double_from_list(double value, double *arr, size_t size) {
	double closest = *arr;  // Initialize with the first element
    double min_diff = fabs(value - *arr);  // Compute the absolute difference

    for (size_t i = 1; i < size; i++) {
        double diff = fabs(value - *(arr+i));  // Calculate the difference for the current element

        if (diff < min_diff) {
            min_diff = diff;
            closest = *(arr+i);  // Update the closest value
        }
    }
    return closest;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint16_t als = 0;
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_ALL);
	ssd1306_Init();
	display_initial_description();
	veml6030 veml;
	set_initial_veml_params(veml);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		als = veml6030_read_als(&veml);
		uint32_t lux_als = (uint32_t)RESOLUTION*als;
		uint32_t lux_als_correction =  (uint32_t)((6.0135e-13*lux_als*lux_als*lux_als*lux_als)-(9.3924e-9*lux_als*lux_als*lux_als)+(8.1488e-5*lux_als*lux_als)+1.0023*lux_als);
		EV = (uint8_t)(log10(lux_als_correction/2.5)/0.3);

		if (state == 0){
			exposure_time = (uint16_t)(1.0/((aperture_f*aperture_f)/(pow(2, EV))));
			size_t time_list_size = APERTURE_TIME_TABLE_SIZE;
			exposure_time = the_closest_integer_from_list(exposure_time, exposure_times_list, time_list_size);
		}
		if (state == 1){
			aperture_f = sqrt(pow(2,EV)*(1.0/exposure_time));
			size_t f_list_size = APERTURE_F_TABLE_SIZE;
			aperture_f = the_closest_double_from_list(aperture_f, apertures_f_list, f_list_size);
		}
		if (state == 2){
			exposure_time = (uint16_t)(1.0/((aperture_f*aperture_f)/(pow(2, EV))));
			size_t time_list_size = APERTURE_TIME_TABLE_SIZE;
			exposure_time = the_closest_integer_from_list(exposure_time, exposure_times_list, time_list_size);
		}

		char lux_screen[11];
		char EV_screen[6];

		sprintf(lux_screen,"LUX:%06d", lux_als_correction);
		sprintf(EV_screen,"EV:%02d", EV);
		ssd1306_SetCursor(0, 10);
		ssd1306_WriteString(EV_screen, Font_6x8, White);
		ssd1306_SetCursor(60, 10);
		ssd1306_WriteString(lux_screen, Font_6x8, White);

		char aperture_screen[8];
		char time_screen[9];
		sprintf(aperture_screen,"f/%.1f", aperture_f);
		sprintf(time_screen,"1/%3d [s]", exposure_time);
		ssd1306_SetCursor(10, 50);
		ssd1306_WriteString(aperture_screen, Font_6x8, White);
		ssd1306_SetCursor(70, 50);
		ssd1306_WriteString(time_screen, Font_6x8, White);
		ssd1306_UpdateScreen();

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_6){
		state = (state+1)%SETTINGS_NUMBER;
		if(state == 0){
			ssd1306_DrawRectangle(exposure_coordinates.x1, exposure_coordinates.y1, exposure_coordinates.x2, exposure_coordinates.y2, Black);
			ssd1306_DrawRectangle(aperture_coordinates.x1, aperture_coordinates.y1, aperture_coordinates.x2, aperture_coordinates.y2, White);
		}
		else{
			ssd1306_DrawRectangle(aperture_coordinates.x1, aperture_coordinates.y1, aperture_coordinates.x2, aperture_coordinates.y2, Black);
			ssd1306_DrawRectangle(exposure_coordinates.x1, exposure_coordinates.y1, exposure_coordinates.x2, exposure_coordinates.y2, White);
		}
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim2){

		switch (HAL_TIM_GetActiveChannel(&htim2)){
		case HAL_TIM_ACTIVE_CHANNEL_1:
			counter = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
			break;

		case HAL_TIM_ACTIVE_CHANNEL_2:
			counter = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
			break;
		default:
			break;
		}
		if(state==0){
			//change aperture -> recalculate aperture time
			aperture_f_list_index = counter%APERTURE_F_TABLE_SIZE;
			aperture_f = apertures_f_list[aperture_f_list_index];
			exposure_time = (uint16_t)(1.0/((aperture_f*aperture_f)/(pow(2, EV))));
			exposure_time = the_closest_integer_from_list(exposure_time, exposure_times_list, APERTURE_TIME_TABLE_SIZE);


		}
		else if(state==1){
			//change aperture time -> recalculate aperture
			exposure_time_list_index = counter%APERTURE_TIME_TABLE_SIZE;
			exposure_time = exposure_times_list[exposure_time_list_index];
			aperture_f = sqrt(pow(2,EV)*(1/exposure_time));
			aperture_f = the_closest_double_from_list(aperture_f, apertures_f_list, APERTURE_F_TABLE_SIZE);
		}
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
