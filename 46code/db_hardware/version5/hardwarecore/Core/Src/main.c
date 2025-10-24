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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
//#include "wirless.h"
#include "encoder.h"
#include "pid.h"
#include "mpu6500.h"
#include "pid1.h"
#include "encoder1.h"
extern DMA_HandleTypeDef hdma_usart3_rx;
extern LaserPointTypeDef ax_ls_point[250];


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
uint8_t receiveData[50];
uint32_t systick = 0;
	int encoder_A_l = 0,encoder_B_l = 0;//编码器（上一次）
	int encoder_A_n = 0,encoder_B_n = 0;//编码器（这一次）
	
	
	//imu变量（反馈量）
	Kalman_t kalmanX = {0.001f, 0.003f, 0.03f, 0.0f, 0.0f, {{0.0f, 0.0f}, {0.0f, 0.0f}}};
	Kalman_t kalmanY = {0.001f, 0.003f, 0.03f, 0.0f, 0.0f, {{0.0f, 0.0f}, {0.0f, 0.0f}}};
	Attitude_t attitude = {0.0f, 0.0f, 0.0f};
	float ax, ay, az, gx, gy, gz;//原始数据
	float yaw_last;
	int roll_int, pitch_int, yaw_int;//解算数据
	
	//pid变量
	extern PID_controller  yaw_v_pid;
											/* 速度  YAW角速度*/		
	float Mechanical_zero[2]={0 ,   0};
	float PID_yaw_v[3]=          {  0,       0  ,     0 };//P、I、D 偏航角速度环，转向环内环
	int count1, count2;
	int duty;//输出量
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart == &huart3){
	HAL_UART_Transmit_DMA(&huart3, receiveData, Size);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,receiveData,sizeof(receiveData));
	 __HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);

	//HAL_UART_Transmit_DMA(&huart3, receiveData, 100);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,receiveData,sizeof(receiveData));
	 __HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);

		
	// 启动编码器模式
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // 启动电机A编码器
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // 启动电机B编码器
  
  // 启动PWM输出
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // 电机A控制引脚1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // 电机A控制引脚2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  // 电机B控制引脚1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  // 电机B控制引脚2
  
  // 启动ADC
  HAL_ADC_Start(&hadc1);
  

	uint8_t time_str[27];
	int i = 0;


	AX_LASER_Start();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  //进行转向操作
 // motorTurnLeft(200);
  
  
  
  // 变量定义区
  int motor1_speed = 400;
  int motor2_speed = 400;
  
  uint8_t key_last = 1;
  // PID相关变量
  PID_t pidA;
  PID_t pidB;
  float target_speedA = 20.0f; // 初始目标速度
  float target_speedB = 20.0f; // 初始目标速度
  PID_Init(&pidA, 0.73f, 0.007f, 0.0f, 1000.0f, 0.0f); // 这里的参数请根据实际调试
  PID_Init(&pidB, 0.75f, 0.007f, 0.0f, 1000.0f, 0.0f); // 这里的参数请根据实际调试
  
  
  
  //1111
//   set_motor_speed(1, motor1_speed);
//   set_motor_speed(2, motor2_speed);
   //yanhsi
   //HAL_Delay(100);
   
   
   //motorTurnRight(1300);
	 static int32_t last_count_a = 0;
static int32_t last_count_b = 0;
  #define ENCODER_PULSE_PER_TURN 390.0 // 输出轴一圈脉冲数（13*30）
#define ENCODER_INTERVAL_MS 50       // 速度采样周期，单位ms
	
	 
	 
	 
	 
	 
	 
   
  while (1)
  {
	  //motorStraight(400);
	  if(receiveData[0]==2)
	  {
		motorStraight(400,24.0f);
		 // 读取编码器计数值
		  int32_t now1 = __HAL_TIM_GET_COUNTER(&htim2);
		int32_t diff1 = now1 - last_count_a;
    last_count_a = now1;
    // 处理定时器溢出
    if (diff1 > 0x7FFFFFFF) diff1 -= 0xFFFFFFFF;
    else if (diff1 < (int32_t)0x80000000) diff1 += 0xFFFFFFFF;
    double turns = (double)diff1 / ENCODER_PULSE_PER_TURN;
     double speedA =turns / (ENCODER_INTERVAL_MS / 1000.0);
			
int32_t now2 = __HAL_TIM_GET_COUNTER(&htim4);
    int32_t diff2 = now2 - last_count_b;
    last_count_b = now2;
    if (diff2 > 0x7FFFFFFF) diff2 -= 0xFFFFFFFF;
    else if (diff2 < (int32_t)0x80000000) diff2 += 0xFFFFFFFF;
    double turns2 = (double)diff2 / ENCODER_PULSE_PER_TURN;
    double speedB= turns2 / (ENCODER_INTERVAL_MS / 1000.0);
			char buf[128];
			snprintf(buf, sizeof(buf), "%ld,%ld,%.3f,%.3f\n", 
				(long)motor1_speed, 
				(long)motor2_speed, 
				speedA*10,
				speedB*10);
			HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 100); 
		  
		  
		  
		  
		  
		  
		  
		  
	  }else 
	  if(receiveData[0]==8)
	  {
		motorReverse(400,24.0f);
	  }else
	  if(receiveData[0]==4)
	  {
		motorTurnLeft();
	  }else
	  if(receiveData[0]==6)
	  {
		motorTurnRight();
	  }else
	  if(receiveData[0]==5)
	  {
		motorAB_stop();
	  }
	  if(receiveData[0]==7)
	  {
		for(i = 0; i < 250; i++)
		{

			sprintf(time_str, "angle:%d  distance:%d ", ax_ls_point[i].angle, ax_ls_point[i].distance); 
//			Wirless_SendString(&huart3, time_str);
			HAL_UART_Transmit(&huart3, time_str, sizeof(time_str), 100);
		}
	  }
	  
	  if(receiveData[0]==1)
	  {
	  
	  
//		MPU6500_Read_Accel(&hi2c1, &ax, &ay, &az);
//		MPU6500_Read_Gyro(&hi2c1, &gx, &gy, &gz);


			char buff[50] ;
		  for(i = 0; i < 100; i++ ){
		
		  sprintf(buff, "Roll: %d.%02d, Pitch: %d.%02d, Yaw: %d.%02d, gz:%.2f\r\n",
               roll_int / 100, abs(roll_int % 100),
               pitch_int / 100, abs(pitch_int % 100),
               yaw_int / 100, abs(yaw_int % 100),gz);//格式化输出字符串
		//sprintf(buff, "Roll");//格式化输出字符串
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)buff, strlen(buff));
		
		HAL_Delay(100);
	  
		  }
	  }
	      
	  if(receiveData[0]==3)
	  {
	  fakemove();
	  for(i = 0; i < 250; i++)
		{

			sprintf(time_str, "angle:%d  distance:%d ", ax_ls_point[i].angle, ax_ls_point[i].distance); 
//			Wirless_SendString(&huart3, time_str);
			HAL_UART_Transmit(&huart3, time_str, sizeof(time_str), 100);
		}
	  }
	  
	  
	  
	  
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
		//极度危险!!!!!!!!!
	  
	  
		
		
//		
//    
//   
//    // 读取ADC值计算输入电压
//    uint32_t adc_value = 0;
//    HAL_ADC_PollForConversion(&hadc1, 100);
//    adc_value = HAL_ADC_GetValue(&hadc1);
//    float input_voltage = adc_value * (3.3f * 11.0f / 4096.0f);
//    // 按键检测（PC0，低电平为按下）
//    uint8_t key_now = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
//	  
//	  //作简单防抖和目标速度递增
//    if (key_last == 1 && key_now == 0) {
//      target_speedB += 5.0f;
//      if (target_speedB > 30.0f) target_speedB = 30.0f;
//		
//		//修改占空比pwmA
//		motor2_speed += 100;
//		
//    }
//    key_last = key_now;
//    // PID调节
//	
////	set_motor_speed(1, pwmA);
////    set_motor_speed(2, 0);
//	
//	 // 读取编码器计数值
//    int32_t encoderA_value = EncoderA_GetCount();  // 电机A编码器计数
//    int32_t encoderB_value = EncoderB_GetCount();  // 电机B编码器计数
//    // 读取编码器速度（圈/秒）
//    double speedA = EncoderA_GetSpeed();
//    double speedB = EncoderB_GetSpeed();
//	
//	
//	
//	
//	 set_motor_speed(1,400);  //此处有一定问题
//	 set_motor_speed(2,400);  //此处有一定问题
//	
//	
//	float pid_out1 = PID_Incremental_Calc(&pidA, target_speedA, (float)speedA);
//	float pid_out2 = PID_Incremental_Calc(&pidB, target_speedB, (float)speedB);
//	
//	
//	//测试用
//	float error = target_speedB - speedB ; 
//	
//    motor1_speed += (int)pid_out1;
//	
//	
//    if (pid_out1 > 1000) 
//	{pid_out1 = 1000;}
//    if (pid_out1 < 0) 
//	{pid_out1 = 0;}
//	
//	motor2_speed += (int)pid_out2;
//	
//	if (pid_out2 > 1000) 
//	{pid_out2 = 1000;}
//    if (pid_out2 < 0) 
//	{pid_out2 = 0;}
//	
//	
//   set_motor_speed(1, motor1_speed);
//    set_motor_speed(2, motor2_speed);
//	
//	
//	
//	
//	
//    HAL_Delay(100);
//    extern UART_HandleTypeDef huart3;
//    char buf[128];
//    snprintf(buf, sizeof(buf), "%ld,%ld,%.3f,%.3f\n", motor1_speed, motor2_speed, speedA, speedB);
//    Wirless_SendString(&huart3, buf);
//    


	
	//发送imu数据
		














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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
if (htim == &htim6) {
			  
			  count1++;//分时器
			
			  //imu数据获取
        static float dt = 0.001f;
        MPU6500_Read_Accel(&hi2c1, &ax, &ay, &az);
        MPU6500_Read_Gyro(&hi2c1, &gx, &gy, &gz);
				//对原始三轴角速度处理：处理零偏和抖动问题
				gz = gz +0.76;
				if(gz <= 0.5 && gz >= -0.5) gz = 0;

        Attitude_Update(ax, ay, az, gx, gy, gz, yaw_last, dt);

        roll_int = (int)(attitude.roll * 100); // 转换为整数 (e.g., 12.34 -> 1234)
        pitch_int = (int)(attitude.pitch * 100);
        yaw_int = (int)(attitude.yaw * 100);
			
				//偏航角速度转向环
				if(count1 == 10)
				{
					duty=(int)(increment__yaw_v(&yaw_v_pid,gz));//目标量是俯仰角速度，反馈量是俯仰角速度，输出量是电机占空比
					if(duty>=8400)duty=8400;
					if(duty<=-8400)duty=-8400;
					count1=0;
				}
			
			}
  /* USER CODE END Callback 1 */
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
