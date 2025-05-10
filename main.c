/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file : main.c
 * @brief : Main program body
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dxl_2_0.h"
#include "Horizontal_Servo.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define on 1
#define off 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DMA_BUF_SIZE 8         // DMA 버퍼 크기 (필요에 따라 조정 가능)

/* UART 핸들 */
extern UART_HandleTypeDef huart1; // UART1 핸들
void process_input(void);
void uart_rx_dma1_handler(void);
void UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/* 수신 버퍼 및 플래그 */
uint8_t tx_dma_buf[DMA_BUF_SIZE];
uint8_t pc_rx_buf[DMA_BUF_SIZE];        // UART 수신 DMA 버퍼
volatile uint8_t input_char;             // 입력 문자 저장 변수
volatile uint8_t pc_new_data_received;   // 새 데이터 수신 플래그
bool torque_status = false;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t sit_angle[5][3] = {
		{0,0,0},{518, 2850, 340},{2533, -495, 752},{2577, 71, 752}, {2041, 3150, 370}
};
uint32_t stand_angle[5][3] = {
		{0, 0, 0},{825, 1665, 512}, {2246, 690, 512}, {2290, 1256, 512}, {2328, 1965, 590}
};
uint32_t sit_ride[5][3] = {
		{0, 0, 0},{223, 2899, 345}, {2843, -500, 630}, {2854, 131, 630}, {1761, 3168, 345}
};
uint32_t angle_speed[4] = {40, 40, 40, 40};
uint32_t angle_speed_Hip[4] = {20, 20, 20, 20};
uint16_t torque_limit[4] = {1023, 1023, 1023, 1023};
uint16_t angle_speed_1[4] = {0xfefe, 0xfefe, 0xfefe, 0xfefe};
int wheel_data[4] = {0,0,0,0};
uint16_t ss1[1], ss2[1], ss3[1], ss4[1];
char before_input = 0;

data_set_all data_set;
bool uart_tx_busy = false;


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
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_USART6_UART_Init();
	MX_TIM5_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	dxl_torque_set(ALL, on, on);//정상동작
	init_servo_set();
	send_sync_write_2(ALL, DXL_2_Goal_Velocity, 4, angle_speed, angle_speed);//정상동작
	send_sync_write_1(ALL, DXL_1_Goal_Torque_Limit, 2, angle_speed_1, angle_speed_1);

	HAL_UART_Receive_DMA(&huart1, pc_rx_buf, DMA_BUF_SIZE);



	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		uart_rx_dma1_handler();
		process_input();
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */
void process_input() {

	if (pc_new_data_received) {
		switch(input_char) {
		case 'x':{
			align_Wheels();
			ss1[0] = 0;
			ss2[0] = 0;
			ss3[0] = 0;
			ss4[0] = 0;
			UART_Transmit_DMA(&huart1, (uint8_t*)"Go backward\r\n", 13);

			for(int j = 0; j<10; j++){

				ss1[0] += 50;
				ss2[0] -= 50;
				ss3[0] += 50;
				ss4[0] -= 50;

				send_sync_write_1(FR_WHEEL, DXL_1_MOVING_SPEED, 2, ss1, ss1);
				HAL_Delay(20);
				send_sync_write_1(FL_WHEEL, DXL_1_MOVING_SPEED, 2, ss2, ss2);
				HAL_Delay(20);
				send_sync_write_1(RR_WHEEL, DXL_1_MOVING_SPEED, 2, ss3, ss3);
				HAL_Delay(20);
				send_sync_write_1(RL_WHEEL, DXL_1_MOVING_SPEED, 2, ss4, ss4);
				HAL_Delay(20);
			}
		} break;

		case 'w':{
			align_Wheels();
			ss1[0] = 0;
			ss2[0] = 0;
			ss3[0] = 0;
			ss4[0] = 0;
			UART_Transmit_DMA(&huart1, (uint8_t*)"Go Straight\r\n", 13);
			for(int j = 0; j<10; j++){

				ss1[0] -= 50;
				ss2[0] += 50;
				ss3[0] -= 50;
				ss4[0] += 50;

				send_sync_write_1(FR_WHEEL, DXL_1_MOVING_SPEED, 2, ss1, ss1);
				HAL_Delay(20);
				send_sync_write_1(FL_WHEEL, DXL_1_MOVING_SPEED, 2, ss2, ss2);
				HAL_Delay(20);
				send_sync_write_1(RR_WHEEL, DXL_1_MOVING_SPEED, 2, ss3, ss3);
				HAL_Delay(20);
				send_sync_write_1(RL_WHEEL, DXL_1_MOVING_SPEED, 2, ss4, ss4);
				HAL_Delay(20);
			}
		} break;


		case 'q':{
			turn_left();
			ss1[0] = 0;
			ss2[0] = 0;
			ss3[0] = 0;
			ss4[0] = 0;
			UART_Transmit_DMA(&huart1, (uint8_t*)"Go Left\r\n", 9);
			for(int j = 0; j<10; j++){

				ss1[0] -= 50;//right front
				ss2[0] += 40; //left front
				ss3[0] -= 50;//right rear
				ss4[0] += 40; //left rear

				send_sync_write_1(FR_WHEEL, DXL_1_MOVING_SPEED, 2, ss1, ss1);
				HAL_Delay(20);
				send_sync_write_1(FL_WHEEL, DXL_1_MOVING_SPEED, 2, ss2, ss2);
				HAL_Delay(20);
				send_sync_write_1(RR_WHEEL, DXL_1_MOVING_SPEED, 2, ss3, ss3);
				HAL_Delay(20);
				send_sync_write_1(RL_WHEEL, DXL_1_MOVING_SPEED, 2, ss4, ss4);
				HAL_Delay(20);
			}
		} break;

		case 'e':{
			turn_right();
			ss1[0] = 0;
			ss2[0] = 0;
			ss3[0] = 0;
			ss4[0] = 0;
			UART_Transmit_DMA(&huart1, (uint8_t*)"Go Right\r\n", 10);
			for(int j = 0; j<10; j++){

				ss1[0] -= 40;
				ss2[0] += 50;
				ss3[0] -= 40;
				ss4[0] += 50;

				send_sync_write_1(FR_WHEEL, DXL_1_MOVING_SPEED, 2, ss1, ss1);
				HAL_Delay(20);
				send_sync_write_1(FL_WHEEL, DXL_1_MOVING_SPEED, 2, ss2, ss2);
				HAL_Delay(20);
				send_sync_write_1(RR_WHEEL, DXL_1_MOVING_SPEED, 2, ss3, ss3);
				HAL_Delay(20);
				send_sync_write_1(RL_WHEEL, DXL_1_MOVING_SPEED, 2, ss4, ss4);
				HAL_Delay(20);
			}
		} break;

		case 'd':{
			zero_turn();
			ss1[0] = 0;
			ss2[0] = 0;
			ss3[0] = 0;
			ss4[0] = 0;
			UART_Transmit_DMA(&huart1, (uint8_t*)"Turn Right\r\n", 12);
			for(int j = 0; j<10; j++){

				ss1[0] -= 20;
				ss2[0] += 20;
				ss3[0] += 20;
				ss4[0] -= 20;

				send_sync_write_1(FR_WHEEL, DXL_1_MOVING_SPEED, 2, ss1, ss1);
				HAL_Delay(20);
				send_sync_write_1(FL_WHEEL, DXL_1_MOVING_SPEED, 2, ss2, ss2);
				HAL_Delay(20);
				send_sync_write_1(RR_WHEEL, DXL_1_MOVING_SPEED, 2, ss3, ss3);
				HAL_Delay(20);
				send_sync_write_1(RL_WHEEL, DXL_1_MOVING_SPEED, 2, ss4, ss4);
				HAL_Delay(20);
			}
		} break;

		case 'a':{
			zero_turn();
			ss1[0] = 0;
			ss2[0] = 0;
			ss3[0] = 0;
			ss4[0] = 0;
			UART_Transmit_DMA(&huart1, (uint8_t*)"Turn Left\r\n", 11);
			for(int j = 0; j<10; j++){

				ss1[0] -= 20;
				ss2[0] += 20;
				ss3[0] += 20;
				ss4[0] -= 20;

				send_sync_write_1(FR_WHEEL, DXL_1_MOVING_SPEED, 2, ss1, ss1);
				HAL_Delay(20);
				send_sync_write_1(FL_WHEEL, DXL_1_MOVING_SPEED, 2, ss2, ss2);
				HAL_Delay(20);
				send_sync_write_1(RR_WHEEL, DXL_1_MOVING_SPEED, 2, ss3, ss3);
				HAL_Delay(20);
				send_sync_write_1(RL_WHEEL, DXL_1_MOVING_SPEED, 2, ss4, ss4);
				HAL_Delay(20);
			}
		} break;



		case 'z':{
			turn_left();
			ss1[0] = 0;
			ss2[0] = 0;
			ss3[0] = 0;
			ss4[0] = 0;
			UART_Transmit_DMA(&huart1, (uint8_t*)"Back Left\r\n", 11);
			for(int j = 0; j<10; j++){

				ss1[0] += 50;
				ss2[0] -= 40;
				ss3[0] += 50;
				ss4[0] -= 40;

				send_sync_write_1(FR_WHEEL, DXL_1_MOVING_SPEED, 2, ss1, ss1);
				HAL_Delay(20);
				send_sync_write_1(FL_WHEEL, DXL_1_MOVING_SPEED, 2, ss2, ss2);
				HAL_Delay(20);
				send_sync_write_1(RR_WHEEL, DXL_1_MOVING_SPEED, 2, ss3, ss3);
				HAL_Delay(20);
				send_sync_write_1(RL_WHEEL, DXL_1_MOVING_SPEED, 2, ss4, ss4);
				HAL_Delay(20);
			}
		} break;

		case 'c':{
			turn_right();
			ss1[0] = 0;
			ss2[0] = 0;
			ss3[0] = 0;
			ss4[0] = 0;
			UART_Transmit_DMA(&huart1, (uint8_t*)"Back Right\r\n", 12);
			for(int j = 0; j<10; j++){

				ss1[0] += 40;
				ss2[0] -= 50;
				ss3[0] += 40;
				ss4[0] -= 50;

				send_sync_write_1(FR_WHEEL, DXL_1_MOVING_SPEED, 2, ss1, ss1);
				HAL_Delay(20);
				send_sync_write_1(FL_WHEEL, DXL_1_MOVING_SPEED, 2, ss2, ss2);
				HAL_Delay(20);
				send_sync_write_1(RR_WHEEL, DXL_1_MOVING_SPEED, 2, ss3, ss3);
				HAL_Delay(20);
				send_sync_write_1(RL_WHEEL, DXL_1_MOVING_SPEED, 2, ss4, ss4);
				HAL_Delay(20);
			}
		} break;


		case 's':{
			UART_Transmit_DMA(&huart1, (uint8_t*)"Stop Move\r\n", 11);
			uint16_t tos1[1], tos2[1], tos3[1], tos4[1];

			tos1[0] = 0; tos2[0] = 0; tos3[0] = 0; tos4[0] = 0;
			send_sync_write_1(FR_WHEEL, DXL_1_MOVING_SPEED, 2, tos1, tos1);
			HAL_Delay(20);
			send_sync_write_1(FL_WHEEL, DXL_1_MOVING_SPEED, 2, tos2, tos2);
			HAL_Delay(20);
			send_sync_write_1(RR_WHEEL, DXL_1_MOVING_SPEED, 2, tos3, tos3);
			HAL_Delay(20);
			send_sync_write_1(RL_WHEEL, DXL_1_MOVING_SPEED, 2, tos4, tos4);
			HAL_Delay(20);



		} break;

		case 'b':{
			UART_Transmit_DMA(&huart1, (uint8_t*)"Folding Hip...\r\n", 16);
			uint32_t tos1[1] = {223};
			uint32_t tos2[1] = {2843};
			uint32_t tos3[1] = {2854};
			uint32_t tos4[1] = {1761};
			uint32_t tos1_1[1] = {2899};
			uint32_t tos2_2[1] = {-500};
			uint32_t tos3_3[1] = {131};
			uint32_t tos4_4[1] = {3168};
			send_sync_write_2(FR_LEG, DXL_2_Goal_Position, 4, tos1, tos1_1);
			HAL_Delay(20);
			send_sync_write_2(FL_LEG, DXL_2_Goal_Position, 4, tos2, tos2_2);
			HAL_Delay(20);
			send_sync_write_2(RR_LEG, DXL_2_Goal_Position, 4, tos3, tos3_3);
			HAL_Delay(20);
			send_sync_write_2(RL_LEG, DXL_2_Goal_Position, 4, tos4, tos4_4);
			HAL_Delay(20);

		} break;

		case 'v':{
			UART_Transmit_DMA(&huart1, (uint8_t*)"Folding Ankle...\r\n", 18);
			uint16_t tos1[1] = {345};
			uint16_t tos2[1] = {675};
			uint16_t tos3[1] = {660};
			uint16_t tos4[1] = {405};
			send_sync_write_1(FR_ANKLE, DXL_1_Goal_Position, 2, tos1, tos1);
			HAL_Delay(20);
			send_sync_write_1(FL_ANKLE, DXL_1_Goal_Position, 2, tos2, tos2);
			HAL_Delay(20);
			send_sync_write_1(RR_ANKLE, DXL_1_Goal_Position, 2, tos3, tos3);
			HAL_Delay(20);
			send_sync_write_1(RL_ANKLE, DXL_1_Goal_Position, 2, tos4, tos4);
			HAL_Delay(20);

		} break;


		case 'h':{
			UART_Transmit_DMA(&huart1, (uint8_t*)"Standding Hip...\r\n", 18);
			uint32_t tos1[1] = {825};
			uint32_t tos2[1] = {2246};
			uint32_t tos3[1] = {2290};
			uint32_t tos4[1] = {2328};
			uint32_t tos1_1[1] = {1665};
			uint32_t tos2_2[1] = {690};
			uint32_t tos3_3[1] = {1256};
			uint32_t tos4_4[1] = {1965};
			send_sync_write_2(FR_LEG, DXL_2_Goal_Position, 4, tos1, tos1_1);
			HAL_Delay(20);
			send_sync_write_2(FL_LEG, DXL_2_Goal_Position, 4, tos2, tos2_2);
			HAL_Delay(20);
			send_sync_write_2(RR_LEG, DXL_2_Goal_Position, 4, tos3, tos3_3);
			HAL_Delay(20);
			send_sync_write_2(RL_LEG, DXL_2_Goal_Position, 4, tos4, tos4_4);
			HAL_Delay(20);

		} break;

		case 'g':{
			UART_Transmit_DMA(&huart1, (uint8_t*)"Standing Ankle...\r\n", 19);
			uint16_t tos1[1] = {512};
			uint16_t tos2[1] = {512};
			uint16_t tos3[1] = {512};
			uint16_t tos4[1] = {590};
			send_sync_write_1(FR_ANKLE, DXL_1_Goal_Position, 2, tos1, tos1);
			HAL_Delay(20);
			send_sync_write_1(FL_ANKLE, DXL_1_Goal_Position, 2, tos2, tos2);
			HAL_Delay(20);
			send_sync_write_1(RR_ANKLE, DXL_1_Goal_Position, 2, tos3, tos3);
			HAL_Delay(20);
			send_sync_write_1(RL_ANKLE, DXL_1_Goal_Position, 2, tos4, tos4);
			HAL_Delay(20);

		} break;


		case 't' :{
			uint32_t t_s_2[1];
			uint16_t t_s_1[1];
			send_sync_write_2(ALL, DXL_2_Goal_Velocity, 4, angle_speed_Hip, angle_speed);
			HAL_Delay(20);
			send_sync_write_1(ALL, DXL_1_Goal_Torque_Limit, 2, torque_limit, torque_limit);
			HAL_Delay(20);
			send_sync_write_1(ALL_ANKLE, 28, 2, angle_speed_1, angle_speed_1);
			HAL_Delay(20);
			if(torque_status == 0)
			{
				UART_Transmit_DMA(&huart1, (uint8_t*)"***Torque On***\r\n", 17);
				torque_status = 1;
				t_s_2[0] = 1;
				t_s_1[0] = 1;
				for(int i = 1; i <= 4; i++)
				{
					send_sync_write_1(i, DXL_1_Torque_Enable, 1, t_s_1, t_s_1);
					HAL_Delay(20);
					send_sync_write_2(i, DXL_2_Torque_Enable, 1, t_s_2, t_s_2);
					HAL_Delay(20);
				}
			}
			else{
				UART_Transmit_DMA(&huart1, (uint8_t*)"***Torque Off***\r\n", 18);
				torque_status = 0;
				t_s_2[0] = 0;
				t_s_1[0] = 0;

				for(int i = 1; i <= 4; i++)
				{
					send_sync_write_1(i, DXL_1_Torque_Enable, 1, t_s_1, t_s_1);
					HAL_Delay(20);
					send_sync_write_2(i, DXL_2_Torque_Enable, 1, t_s_2, t_s_2);
					HAL_Delay(20);
				}
			}
		} break;

		case ']':{
			UART_Transmit_DMA(&huart1, (uint8_t*)"***Angle_Speed_reset***\r\n", 25);
			send_sync_write_2(ALL, DXL_2_Goal_Velocity, 4, angle_speed, angle_speed);
			HAL_Delay(20);
			send_sync_write_1(ALL, DXL_1_Goal_Torque_Limit, 2, angle_speed_1, angle_speed_1);
			HAL_Delay(20);

		} break;

		default:
			UART_Transmit_DMA(&huart1, (uint8_t*)"?", 1);
			HAL_Delay(20);
			break;
		}
		pc_new_data_received = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		pc_new_data_received = 1;
		HAL_UART_Receive_DMA(huart, pc_rx_buf, 1);
	}
}

void UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	if (!uart_tx_busy) {
		uart_tx_busy = 1;
		memcpy(tx_dma_buf, pData, Size);
		HAL_UART_Transmit_DMA(huart, tx_dma_buf, Size);
	}
}


void uart_rx_dma1_handler() {
	static uint16_t old_pos = 0;
	uint16_t pos = DMA_BUF_SIZE - huart1.hdmarx->Instance->NDTR;

	if (pos != old_pos) {
		if (pos > old_pos) {
			for (uint16_t i = old_pos; i < pos; i++) {
				input_char = pc_rx_buf[i];
				pc_new_data_received = 1;
			}
		} else {
			for (uint16_t i = old_pos; i < DMA_BUF_SIZE; i++) {
				input_char = pc_rx_buf[i];
				pc_new_data_received = 1;
			}
			for (uint16_t i = 0; i < pos; i++) {
				input_char = pc_rx_buf[i];
				pc_new_data_received = 1;
			}
		}
		old_pos = pos;
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		uart_tx_busy = false;
		// USART1 처리
	}
	else if(huart->Instance == USART2)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		uart_tx_busy = false;
	}
	else if(huart->Instance == USART3)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		uart_tx_busy = false;
	}
	else if(huart->Instance == UART4) // USART4를 UART4로 수정
	{
		uart_tx_busy = false;
		// UART4 처리
	}
	else if(huart->Instance == UART5) // USART5를 UART5로 수정
	{
		uart_tx_busy = false;
		// UART5 처리
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
