/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "i2c.h"
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
#define PI acos(-1)
#define PWMA   TIM1->CCR1  //PA8
#define Balance_Kp  450
#define Balance_Kd 1
#define Velocity_Kp 80
#define Velocity_Ki 1
#define ZHONGZHI 0
#define Gyro_Banlance 77
#define MAX_Encoder_Integral 8000 //10000
#define PWMB   TIM1->CCR4  //PA11
#define PWM_Amplitude 6900 //max 7200 limit 6900
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t test_16;
uint16_t battery_adc;
int16_t battery_volt;
float Encoder_Integral;
float Target_Velocity;
uint8_t Flag_Qian, Flag_Hou, Flag_Left, Flag_Right;
int encoder_left, encoder_right;
int Temperature;
float Angle_Balance, Gyro_Balance, Gyro_Turn;
float Acceleration_Z,Target_Velocity,Turn_Amplitude;
int Encoder_Left,Encoder_Right;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId nonblockHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void set_Pwm(int32_t motor1, int32_t motor2);
int pid_Balance_Velocity(float angle,float gyro, int encoder_left, int encoder_right);
void Get_Angle(uint8_t way);
int Read_Encoder(uint8_t timx);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask_nonblock(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of nonblock */
  osThreadDef(nonblock, StartTask_nonblock, osPriorityNormal, 0, 128);
  nonblockHandle = osThreadCreate(osThread(nonblock), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);      //10ms overtime
		if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
		{
				battery_adc=HAL_ADC_GetValue(&hadc1);
				battery_volt = battery_adc*3.3*11*100/4096;
		}

		
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(800);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask_nonblock */
/**
* @brief Function implementing the nonblock thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_nonblock */
void StartTask_nonblock(void const * argument)
{
  /* USER CODE BEGIN StartTask_nonblock */
  int32_t Balance_Pwm, Moto1, Moto2;
  /* Infinite loop */
  for(;;)
  {
    Encoder_Left = Read_Encoder(2); 
		Encoder_Right = Read_Encoder(4); 
		
		Get_Angle(2);
    
    Balance_Pwm = pid_Balance_Velocity(Angle_Balance, Gyro_Balance,Encoder_Left, Encoder_Right);

    // Turn_Pwm = turn(Encoder_Left, Encoder_Right, Gyro_Turn);
    Moto1 = Balance_Pwm ;			 
    Moto2 = Balance_Pwm ;			 
    // Xianfu_Pwm();											
    set_Pwm(Moto1, Moto2);				   
    osDelay(5);
  }
  /* USER CODE END StartTask_nonblock */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void set_Pwm(int32_t motor1, int32_t motor2)
{
	if (motor1 > 0){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);     //AIN1
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);   //AIN2
  }
	else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //AIN1
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   //AIN2
  }
  motor1 = (motor1 < -PWM_Amplitude) ? -PWM_Amplitude : motor1;
  motor1 = (motor1 > PWM_Amplitude) ? PWM_Amplitude : motor1;
	PWMA = abs(motor1);
	if (motor2 > 0)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);    //BIN2
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);   //BIN1 
  }
	else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);   //BIN2
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   //BIN1    
  }
  motor2 = (motor2 < -PWM_Amplitude) ? -PWM_Amplitude : motor2;
  motor2 = (motor2 > PWM_Amplitude) ? PWM_Amplitude : motor2; 
	PWMB = abs(motor2);
}


// 平衡PID 速度PID
int pid_Balance_Velocity(float angle,float gyro, int encoder_left, int encoder_right)
{
  int balance;
	static float Velocity, Encoder, Movement;
	Target_Velocity = 110;

	// 平衡
  balance = Balance_Kp * (angle - ZHONGZHI) + Balance_Kd * (gyro + Gyro_Banlance); //===计算平衡控制的电机PWM  PD控制 

	if (1 == Flag_Qian)
		Movement = -Target_Velocity; //===前进标志位置1
	else if (1 == Flag_Hou)
		Movement = Target_Velocity; 
	else
		Movement = 0;

	Encoder = 0.8 * Encoder + 0.2 * (encoder_left + encoder_right);		

	Encoder_Integral = Encoder_Integral + Encoder - Movement;						

	if (Encoder_Integral > MAX_Encoder_Integral)
		Encoder_Integral = MAX_Encoder_Integral;    
	if (Encoder_Integral < -MAX_Encoder_Integral)
		Encoder_Integral = -MAX_Encoder_Integral;	  

	Velocity = Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki; //===速度控制

	return balance + Velocity;
}


void Get_Angle(uint8_t way)
{
	float Accel_Y, Accel_Angle, Accel_Z, Gyro_X, Gyro_Z;
	Temperature = Read_Temperature(); //===读取MPU6050内置温度传感器数据，近似表示主板温度
	if (way == 1)					  //===DMP的读取在数据采集中断读取，严格遵循时序要
	{
		Read_DMP();				   //===读取加�?�度、角速度、�?�角
		Angle_Balance = -Roll;	 //===更新平衡倾角
		Gyro_Balance = -gyro[0];   //===更新平衡角�?�度
		Gyro_Turn = gyro[2];	   //===更新转向角�?�度
		Acceleration_Z = accel[2]; //===更新Z轴加速度�??
	}
	else
	{
		Gyro_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_L);	//读取Y轴陀螺仪
		Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);	//读取Z轴陀螺仪
		Accel_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度�??
		Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度�??
		if (Gyro_X > 32768)
			Gyro_X -= 65536; //数据类型转换  也可通过short强制类型转换
		if (Gyro_Z > 32768)
			Gyro_Z -= 65536; //数据类型转换
		if (Accel_Y > 32768)
			Accel_Y -= 65536; //数据类型转换
		if (Accel_Z > 32768)
			Accel_Z -= 65536;							  //数据类型转换
		Gyro_Balance = Gyro_X;							  
		Accel_Angle = atan2(Accel_Y, Accel_Z) * 180 / PI; //计算倾角
		Gyro_X = Gyro_X / 16.4;							  //量程转换
		if (way == 2)
			Kalman_Filter(Accel_Angle, Gyro_X);
		else if (way == 3)
			Yijielvbo(Accel_Angle, Gyro_X); //互补滤波
		Angle_Balance = angle;				//更新平衡倾角
		Gyro_Turn = Gyro_Z;					
		Acceleration_Z = Accel_Z;		
	}
}

int Read_Encoder(uint8_t timx)
{
  int Encoder_TIM;    
  switch(timx)
  {
    case 2:  Encoder_TIM = (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
    case 4:  Encoder_TIM = (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
    default:  Encoder_TIM = 0;
  }
  return Encoder_TIM;
}



/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
