/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "kinematic.h"
#include "motor.h"
#include "AMT21.h"
#include "stdio.h"
#include "PID.h"
#include "math.h"
#include "KalmanFilter.h"
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
AMT21 encoders[5];
Stepper_Motor steppers[5];
Servo_Motor servo_motor;
PIDController position_pid_controller[5];
PIDController velocity_pid_controller[5];

KalmanFilter kalman_filter[5];
float kalman_R = 0.00125;
float kalman_Q = 2000;

int32_t joint_state[5] = { 0 };
int32_t prev_joint_state[5] = { 0 };
int32_t joint_speed[5] = { 0 };
int32_t set_point[5] = { 0 };

uint8_t desired_value_mode = 1; // if (step = 0, sin = 1)
float desired_amplitude = 500;
float desired_position_cal = 0;
float desired_velocity_cal = 0;
float desired_position[5] = {0};
float desired_velocity[5] = {0};

/*
 * for step generator
 */
uint16_t tim_cnt = 0;
uint16_t step_interval = 500;
/*
 * for sin wave generator
 */
float AngleInput = 0;
float changeRate = 0.0125;
/*
 * for cascade
 */
float cascade_out = 0;
int enc_cnt = 0;
uint8_t j_num = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/*
 * printf
 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC3_Init();
	MX_SPI3_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM12_Init();
	MX_TIM13_Init();
	MX_TIM14_Init();
	MX_TIM15_Init();
	MX_TIM16_Init();
	MX_TIM17_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM24_Init();
	MX_TIM23_Init();
	/* USER CODE BEGIN 2 */
//	HAL_TIM_Base_Start_IT(&htim24);
	uint32_t timestamp = 0;
	int32_t encoder_unwrap_value[5] = {0};
	/*
	 * Servo Initialise
	 */
//	HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, 1);
//	servo_initialise(&servo_motor, &htim17, TIM_CHANNEL_1);
	/*
	 * Encoder Initialise
	 */
	AMT21_initialise(&encoders[0], &huart2, 0x2C, USART2_DE_GPIO_Port,
	USART2_DE_Pin);
	AMT21_initialise(&encoders[1], &huart2, 0x70, USART2_DE_GPIO_Port,
	USART2_DE_Pin);
	AMT21_initialise(&encoders[2], &huart2, 0x54, USART2_DE_GPIO_Port,
	USART2_DE_Pin);
	AMT21_initialise(&encoders[3], &huart2, 0xE8, USART2_DE_GPIO_Port,
	USART2_DE_Pin);
	AMT21_initialise(&encoders[4], &huart2, 0xB4, USART2_DE_GPIO_Port,
	USART2_DE_Pin);
	/*
	 * Stepper Initialise
	 */
	stepper_initialise(&steppers[0], &htim1, TIM_CHANNEL_1, DIR1_GPIO_Port,
	DIR1_Pin, 0);
	stepper_initialise(&steppers[1], &htim2, TIM_CHANNEL_1, DIR2_GPIO_Port,
	DIR2_Pin ,1);
	stepper_initialise(&steppers[2], &htim3, TIM_CHANNEL_1, DIR3_GPIO_Port,
	DIR3_Pin ,1);
	stepper_initialise(&steppers[3], &htim4, TIM_CHANNEL_1, DIR4_GPIO_Port,
	DIR4_Pin ,0);
	stepper_initialise(&steppers[4], &htim15, TIM_CHANNEL_1, DIR5_GPIO_Port,
	DIR5_Pin ,0);
	/*
	 * Kalman Filter Initialise
	 */
	KalmanFilter_Init(&kalman_filter[0], 0, 0, 1, 0, 0, 1, kalman_R, kalman_Q);
	KalmanFilter_Init(&kalman_filter[1], 0, 0, 1, 0, 0, 1, kalman_R, kalman_Q);
	KalmanFilter_Init(&kalman_filter[2], 0, 0, 1, 0, 0, 1, kalman_R, kalman_Q);
	KalmanFilter_Init(&kalman_filter[3], 0, 0, 1, 0, 0, 1, kalman_R, kalman_Q);
	KalmanFilter_Init(&kalman_filter[4], 0, 0, 1, 0, 0, 1, kalman_R, kalman_Q);
	/*
	 * Position Pid Initialise
	 */
	PIDController_initialise(&position_pid_controller[0], 1, 0, 0);
	PIDController_initialise(&position_pid_controller[1], 2, 0, 0);
	PIDController_initialise(&position_pid_controller[2], 3, 0, 0);
	PIDController_initialise(&position_pid_controller[3], 1, 0, 0);
	PIDController_initialise(&position_pid_controller[4], 2, 0, 0);
	/*
	 * Velocity Pid Initialise
	 */
	PIDController_initialise(&velocity_pid_controller[0], 0, 0, 0);
	PIDController_initialise(&velocity_pid_controller[1], 0, 0, 0);
	PIDController_initialise(&velocity_pid_controller[2], 0, 0, 0);
	PIDController_initialise(&velocity_pid_controller[3], 0, 0, 0);
	PIDController_initialise(&velocity_pid_controller[4], 0, 0, 0);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (HAL_GetTick() - timestamp >= 1) {
			timestamp = HAL_GetTick();
//			if (j_num == 0){
//				AngleInput += changeRate;
//				if (AngleInput > 2 * M_PI) {
//					AngleInput -= 2 * M_PI;
//				} else if (AngleInput < 0) {
//					AngleInput += 2 * M_PI;
//				}
//				desired_position_cal = sin(AngleInput) * desired_amplitude;
//			}
			HAL_StatusTypeDef rep = HAL_ERROR;
			encoder_unwrap_value[j_num] = 0;
			while (1) {
				AMT21_read_value(&(encoders[j_num]));
				rep = AMT21_check_value(&(encoders[j_num]));
				if (rep == HAL_OK) {
					encoder_unwrap_value[j_num] = AMT21_unwrap(
							(int32_t) encoders[j_num].position,
							(int32_t) encoders[j_num].prev_position);
					encoders[j_num].prev_position = encoders[j_num].position;
					break;
				}
			}
			joint_state[j_num] = joint_state[j_num] + encoder_unwrap_value[j_num];
			KalmanFilter_Update(&kalman_filter[j_num], joint_state[j_num]);
			/*
			 * Speed Controller
			 */
//			PIDController_update(&velocity_pid_controller[j_num],
//					desired_velocity[j_num], kalman_filter[j_num].x2);
//			stepper_set_speed(&steppers[j_num],
//					velocity_pid_controller[j_num].out);
			/*
			 * Position Controller
			 */
//			PIDController_update(&position_pid_controller[j_num], desired_position_cal, kalman_filter[j_num].x1);
			PIDController_update(&position_pid_controller[j_num], desired_position[j_num], joint_state[j_num]);
			stepper_set_speed(&steppers[j_num], position_pid_controller[j_num].out);
			/*
			 * Cascade Control
			 */
//			cascade_out = Cascade_PIDController_update(&position_pid_controller[CURRENT_NUMBER], &velocity_pid_controller[CURRENT_NUMBER], &kalman_filter[CURRENT_NUMBER], desired_position, desired_velocity);
//			stepper_set_speed(&steppers[CURRENT_NUMBER], cascade_out);
			j_num++;
//			if(j_num ==)
			if (j_num == 3){
				j_num = 4;
			}
			if (j_num == 5){
				j_num = 0;
			}
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}
	/** Macro to configure the PLL clock source
	 */
	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 2;
	RCC_OscInitStruct.PLL.PLLN = 44;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInitStruct.PLL2.PLL2M = 2;
	PeriphClkInitStruct.PLL2.PLL2N = 15;
	PeriphClkInitStruct.PLL2.PLL2P = 2;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 2950;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 1);

	return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim24);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

