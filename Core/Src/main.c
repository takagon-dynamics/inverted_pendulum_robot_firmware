/* USER CODE BEGIN Header */


/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARRAY_LEN 200
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define CONTROL_MODE 1 /*WHEELSPEED_PI_CONTROL 0
 	 	 	 	 	 	   *INVERTED_PENDULUM_PID_CONTROL 1
 	 	 	 	 	 	   */
#define PI 3.1415926535

#define WHEEL_DIA 0.07	/*[m]*/
#define TREAD 0.195 /*[m]*/

#define GEAR_RATIO 70
#define CPR 64 			/*Count per rotation*/

const float move_per_pulse = (WHEEL_DIA * PI) / (CPR * GEAR_RATIO); /*[m]*/
const float sampling_time = 0.02; /*0.02[s]*/

const int PWM_MAX_VALUE = 999;

double wheel_speed[2] = {0.0, 0.0};

/* SPEED PID CONTROL DEFINE */
double i_left = 0.0; //i制御用変数
double i_right = 0.0;
double left_wheel_dir = 0.0;
double right_wheel_dir = 0.0;
double kp_left = 8000, kp_right = 7500;  //P制御ゲイン
double ki_left = 20, ki_right = 20;  //I制御ゲイン
double ref_wheel_speed[2] = {0.0, 0.0}; //左右車輪目標速度

/* INVERTED PENDULUM PID CONTROL DEFINE */
//double K[4] = {0.0,0.0,0.0,0.0};
double K[4];
double angle = 0.0;
double old_angle = 0.0;
double angle_velocity = 0.0;

double left_distance = 0.0;
double right_distance = 0.0;
double distance = 0.0;
double velocity = 0.0;

double steering = 0.0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
rcl_publisher_t publish_enc_cnt;
rcl_publisher_t publish_wheel_speed;
rcl_publisher_t publisher_string;
rcl_publisher_t publisher_imu;
std_msgs__msg__UInt16 pub_enc_cnt_msg;
std_msgs__msg__Float32 pub_wheel_speed_msg;
std_msgs__msg__String pub_str_msg;

std_msgs__msg__Float32MultiArray  kgain_msg;

sensor_msgs__msg__Imu pub_imu_msg;
geometry_msgs__msg__Twist twist_msg;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		pub_enc_cnt_msg.data = TIM3 -> CNT;
		RCSOFTCHECK(rcl_publish(&publish_enc_cnt, &pub_enc_cnt_msg, NULL));

		pub_wheel_speed_msg.data = wheel_speed[0];
		RCSOFTCHECK(rcl_publish(&publish_wheel_speed, &pub_wheel_speed_msg, NULL));
	}
}

void subscription_str_callback(const void * msgin)
{
  std_msgs__msg__String * msg = (std_msgs__msg__String *)msgin;
  pub_str_msg = *msg;
  char str[100];
  strcpy(str, msg->data.data);
  sprintf(pub_str_msg.data.data, "F446RE heard: %s", str);
  pub_str_msg.data.size = strlen(pub_str_msg.data.data);
  rcl_publish(&publisher_string, &pub_str_msg, NULL);
  debug_led();
}

void subscription_imu_callback(const void * msgin)
{
  sensor_msgs__msg__Imu * msg = (sensor_msgs__msg__Imu *)msgin;
  pub_imu_msg = *msg;
  rcl_publish(&publisher_imu, &pub_imu_msg, NULL);
  debug_led();
}

void subscription_Kgain_callback(const void * msgin)
{
	std_msgs__msg__Float32MultiArray * msg = (std_msgs__msg__Float32MultiArray *)msgin;
	kgain_msg = *msg;
}

void subscription_twist_callback(const void * msgin)
{
  geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *)msgin;
  twist_msg = *msg;
}

void debug_led()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //LED turned on
  HAL_Delay(200); //Wait for 200[ms]
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //LED turned off
  HAL_Delay(200);
}

double low_pass_filter(double val, float pre_val, float gamma){
	return gamma * pre_val + (1.0 - gamma) * val;
}

int get_left_encoder(void){
	int16_t count = 0;
	uint16_t enc_buff = TIM3 -> CNT;

	TIM3 -> CNT = 0;

	if(enc_buff > 32767){
		count = (int16_t)enc_buff * -1; //-1倍はCWがプラスになるよう調整用
	}else{
		count = (int16_t)enc_buff * -1; //-1倍はCCWがマイナスになるよう調整用
	}

	return count;
}

int get_right_encoder(void){
	int16_t count = 0;
	uint16_t enc_buff = TIM1 -> CNT;

	TIM1 -> CNT = 0;

	if(enc_buff > 32767){
		count = (int16_t)enc_buff; //-1倍はCWがプラスになるよう調整用
	}else{
		count = (int16_t)enc_buff; //-1倍はCCWがマイナスになるよう調整用
	}

	return count;
}



void twist_to_wheelspeed(double linear_x, double angular_z, double wheelspeed[2])
{
	/*	説明：ロボットの並進速度と旋回角速度から左右の車輪の速度を計算
	 *	引数：
	 *		(linear_x) ロボットの並進速度
	 *		(linear_z) ロボットの旋回角速度
	 *	戻り値：
	 *		(wheelspeed[]) 左右の車輪の速度
	 */

	wheelspeed[0] = linear_x - (TREAD/2) * angular_z;
	wheelspeed[1] = linear_x + (TREAD/2) * angular_z;
}

void LEFTMOTOR_SetPwm(double wheeldir)
{
	/*
	 * 	説明：左モーターに印加するPWMを設定
	 * 	引数：
	 * 		(wheeldir) -PWM_MAX_VALUE~PWM_MAX_VALUEの値
	 * 		正の値を設定すると正転(CW),負の値を設定すると逆転(CCW)
	 * */
	if(wheeldir > 0){
		if(wheeldir>PWM_MAX_VALUE) wheeldir = PWM_MAX_VALUE;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (int)(fabs(wheeldir)));
	}else{
		if(wheeldir<-PWM_MAX_VALUE) wheeldir = PWM_MAX_VALUE;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (int)(fabs(wheeldir)));
	}
}

void RIGHTMOTOR_SetPwm(double wheeldir)
{
	/*
	 * 	説明：右モーターに印加するPWMを設定
	 * 	引数：
	 * 		(wheeldir) -PWM_MAX_VALUE~PWM_MAX_VALUEの値
	 * 		正の値を設定すると正転(CW),負の値を設定すると逆転(CCW)
	 * */
	if(wheeldir > 0){
		if(wheeldir>PWM_MAX_VALUE) wheeldir = PWM_MAX_VALUE;  //PWM入力制限
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (int)(fabs(wheeldir)));
	}else{
		if(wheeldir<-PWM_MAX_VALUE) wheeldir = PWM_MAX_VALUE; //PWM入力制限
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (int)(fabs(wheeldir)));
	}
}

void getEulerAngle(double* roll, double* pitch, double* yaw){

	double w = pub_imu_msg.orientation.w;
	double x = pub_imu_msg.orientation.x;
	double y = pub_imu_msg.orientation.y;
	double z = pub_imu_msg.orientation.z;

	//roll
	double sinr_cosp = 2.0 * (w*x + y*z);
	double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
	*roll = atan2(sinr_cosp, cosr_cosp);

	//pitch
	double sinp = 2.0 * (w*y - z*x);
	if(fabs(sinp) >= 1){
		*pitch = copysign(M_PI / 2, sinp);
	}else{
		*pitch = asin(sinp);
	}

	//yaw
	double siny_cosp = 2.0 * (w*z + x*y);
	double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
	*yaw = atan2(siny_cosp,cosy_cosp);
}

char scnt[200];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//タイマー割り込みコールバック関数
	if(htim == &htim7){ //タイヤの速度計算(Timer7 0.02[ms] 割り込み処理)
		if(CONTROL_MODE == 0){ //wheel speed pid control
			HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);

			twist_to_wheelspeed(twist_msg.linear.x, twist_msg.angular.z, ref_wheel_speed);

			double left_wheel_speed_tmp = (move_per_pulse * get_left_encoder()) / sampling_time;
			double right_wheel_speed_tmp = (move_per_pulse * get_right_encoder()) / sampling_time;

			wheel_speed[0] = low_pass_filter(wheel_speed[0], left_wheel_speed_tmp, 0.5);
			wheel_speed[1] = low_pass_filter(wheel_speed[1], right_wheel_speed_tmp, 0.5);

			double delta_left_wheel_speed = -ref_wheel_speed[0] + wheel_speed[0];
			double delta_right_wheel_speed = ref_wheel_speed[1] - wheel_speed[1];

			i_left += delta_left_wheel_speed * sampling_time;
			i_right += delta_right_wheel_speed * sampling_time;

			left_wheel_dir = kp_left * delta_left_wheel_speed  + ki_left * i_left;
			right_wheel_dir = kp_right * delta_right_wheel_speed + ki_right * i_right;

//			if(left_wheel_dir >= PWM_MAX_VALUE){ //Anti-Windup-Control
//				i_left = 0;
//			}
//
//			if(right_wheel_dir >= PWM_MAX_VALUE){ //Anti-Windup-Control
//				i_right = 0;
//			}
			LEFTMOTOR_SetPwm(left_wheel_dir);
			RIGHTMOTOR_SetPwm(right_wheel_dir);

		}else if(CONTROL_MODE == 1){ //inverted pendulum pid control
			K[0] = kgain_msg.data.data[0];
			K[1] = kgain_msg.data.data[1];
			K[2] = kgain_msg.data.data[2];
			K[3] = kgain_msg.data.data[3];

			double left_wheel_pos = move_per_pulse * get_left_encoder();
			double right_wheel_pos = move_per_pulse * get_right_encoder();

			left_distance += left_wheel_pos;
			right_distance += right_wheel_pos;

			double left_wheel_speed = left_wheel_pos / sampling_time;
			double right_wheel_speed = right_wheel_pos / sampling_time;

			double roll = 0, pitch = 0, yaw = 0;
			getEulerAngle(&roll, &pitch, &yaw);

			angle = pitch;
			angle_velocity = (angle - old_angle) / sampling_time;
			old_angle = angle;

			distance = (left_distance + right_distance) / 2;
			velocity = (left_wheel_speed + right_wheel_speed) / 2;

			left_wheel_dir = (angle_velocity * K[0]) + (angle * K[1]) + (velocity * K[2]) + (distance * K[3]) + steering;
			right_wheel_dir = (angle_velocity * K[0]) + (angle * K[1]) + (velocity * K[2]) + (distance * K[3]) - steering;

			LEFTMOTOR_SetPwm(left_wheel_dir);
			RIGHTMOTOR_SetPwm(right_wheel_dir);
		}
	}
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 HAL_Delay( 100 );
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 200-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8400-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 200-1;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);


void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  // micro-ROS configuration
  char test_array[ARRAY_LEN];
  memset(test_array,'z',ARRAY_LEN);

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    printf("Error on default allocators (line %d)\n", __LINE__);
  }

  rcl_subscription_t subscriber_string, subscriber_imu, subscriber_twist, subscriber_kgain;
  std_msgs__msg__String sub_str_msg;
  sensor_msgs__msg__Imu sub_imu_msg;
  geometry_msgs__msg__Twist sub_twist_msg;
  std_msgs__msg__Float32MultiArray sub_kgain_msg;

  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "f446re_node", "", &support));

  // create timer,
  	rcl_timer_t timer;
  	const unsigned int timer_timeout = 100;
  	RCCHECK(rclc_timer_init_default(
  		&timer,
  		&support,
  		RCL_MS_TO_NS(timer_timeout),
  		timer_callback));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publish_enc_cnt,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
    "/f446re_enc_cnt_publisher"));

  RCCHECK(rclc_publisher_init_best_effort(
    &publish_wheel_speed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/f446re_wheel_speed_publisher"));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_string,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/f446re_string_publisher"));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/f446re_imu_publisher"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber_string,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/f446re_string_subscriber"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data_raw"));
	
	RCCHECK(rclc_subscription_init_default(
    &subscriber_twist,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

	RCCHECK(rclc_subscription_init_default(
	&subscriber_kgain,
	&node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	"/k_gain"));



  // create executor
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_string, &sub_str_msg, &subscription_str_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_imu, &sub_imu_msg, &subscription_imu_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_twist, &sub_twist_msg, &subscription_twist_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_kgain, &sub_kgain_msg, &subscription_Kgain_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // initialize message memory
  pub_str_msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
  pub_str_msg.data.size = 0;
  pub_str_msg.data.capacity = ARRAY_LEN;

  pub_imu_msg.header.frame_id.capacity = 100;
  pub_imu_msg.header.frame_id.data =(char * ) malloc(100 * sizeof(char));
  pub_imu_msg.header.frame_id.size = 0;

  sub_str_msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
  sub_str_msg.data.size = 0;
  sub_str_msg.data.capacity = ARRAY_LEN;

  sub_imu_msg.header.frame_id.capacity = 100;
  sub_imu_msg.header.frame_id.data =(char * ) malloc(100 * sizeof(char));
  sub_imu_msg.header.frame_id.size = 0;

  sub_kgain_msg.data.capacity = 100;
  sub_kgain_msg.data.data = (float * ) malloc(100 * sizeof(float));
  sub_kgain_msg.data.size = 0;

  // execute subscriber
  rclc_executor_spin(&executor);

  // cleaning Up
  RCCHECK(rcl_publisher_fini(&publisher_string, &node));
  RCCHECK(rcl_publisher_fini(&publisher_imu, &node));
  RCCHECK(rcl_subscription_fini(&subscriber_string, &node));
  RCCHECK(rcl_subscription_fini(&subscriber_imu, &node));
  RCCHECK(rcl_subscription_fini(&subscriber_twist, &node));
  RCCHECK(rcl_node_fini(&node));
  /* USER CODE END 5 */
}

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
