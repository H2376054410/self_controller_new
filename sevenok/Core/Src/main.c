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
#include "dma.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buff1[256]={0};
uint8_t buff2[256]={0};
uint8_t buff3[256]={0};
uint8_t buff4[256]={0};
uint8_t buff5[256]={0};
uint8_t buff6[256]={0};
uint8_t top1=0;
uint8_t top2=0;
uint8_t top3=0;
uint8_t top4=0;
uint8_t top5=0;
uint8_t top6=0;
float angle1=0;
float angle2=0;
float angle3=0;
float angle4=0;
float angle5=0;
float angle6=0;
uint8_t sendbuff[39]={0};
extern const unsigned char CRC8_INIT;
extern uint16_t CRC_INIT; 
extern unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
extern uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
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
  MX_UART5_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
	int data_length = 30;
	int cmd_id = 0x0302;
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,buff1,256);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,buff2,256);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,buff3,256);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4,buff4,256);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5,buff5,256);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6,buff6,256);
  HAL_TIM_Base_Start_IT(&htim2);
//  sendbuff[0]=0xA5;
//  sendbuff[1]=(int16_t)(data_length);
//  sendbuff[2]=(int16_t)(data_length)>>8;
//  sendbuff[3]=0x5C;
//  sendbuff[4]=Get_CRC8_Check_Sum(sendbuff,4,CRC8_INIT);
//  sendbuff[5]=(int16_t)(cmd_id);
//  sendbuff[6]=(int16_t)(cmd_id)>>8;

  sendbuff[0]=0xA5;
  sendbuff[1]=(int16_t)(data_length);;
  sendbuff[2]=(int16_t)(data_length)>>8;
  sendbuff[3]=0x5C;
  sendbuff[4]=Get_CRC8_Check_Sum(sendbuff,4,CRC8_INIT);
  sendbuff[5]=(int16_t)(cmd_id);
  sendbuff[6]=(int16_t)(cmd_id)>>8;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

uint32_t count=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint16_t temp=0;
  if(htim==&htim2)
  {
    count++;
		sendbuff[7]=1;
    sendbuff[8]=((uint8_t*)&angle4)[0];
    sendbuff[9]=((uint8_t*)&angle4)[1];
    sendbuff[10]=((uint8_t*)&angle4)[2];
    sendbuff[11]=((uint8_t*)&angle4)[3];

    sendbuff[12]=((uint8_t*)&angle2)[0];
    sendbuff[13]=((uint8_t*)&angle2)[1];
    sendbuff[14]=((uint8_t*)&angle2)[2];
    sendbuff[15]=((uint8_t*)&angle2)[3];

    sendbuff[16]=((uint8_t*)&angle3)[0];
    sendbuff[17]=((uint8_t*)&angle3)[1];
    sendbuff[18]=((uint8_t*)&angle3)[2];
    sendff[19]=((uint8_t*)&angle3)[3];

    sendbuff[20]=((uint8_t*)&angle1)[0];
    sendbuff[21]=((uint8_t*)&angle1)[1];
    sendbuff[22]=((uint8_t*)&angle1)[2];
    sendbuff[23]=((uint8_t*)&angle1)[3];

    sendbuff[24]=((uint8_t*)&angle5)[0];
    sendbuff[25]=((uint8_t*)&angle5)[1];
    sendbuff[26]=((uint8_t*)&angle5)[2];
    sendbuff[27]=((uint8_t*)&angle5)[3];

    sendbuff[28]=((uint8_t*)&angle6)[0];
    sendbuff[29]=((uint8_t*)&angle6)[1];
    sendbuff[30]=((uint8_t*)&angle6)[2];
    sendbuff[31]=((uint8_t*)&angle6)[3];

    sendbuff[32] = 0;
    sendbuff[33] = 0;
    sendbuff[34] = 0;
    sendbuff[35] = 0;
    sendbuff[36] = 0;
    temp=Get_CRC16_Check_Sum(sendbuff,37,CRC_INIT);
    sendbuff[38]=temp>>8;
    sendbuff[37]=temp;

    HAL_UART_Transmit_DMA(&huart7,sendbuff,39);
  }
}
float angle_tranform(uint8_t *buff)
{
  uint32_t temp=0;
  for(int i=0;i<=3;i++)
    ((uint8_t*)(&temp))[i]=buff[3-i];
  return temp/262144.0f*360.0f;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart==&huart1)
  {
    if(top1<(uint8_t)(top1+9)&&Size-top1==9)
    {
      if(buff1[top1]==0x00&&buff1[top1+1]==0x03&&buff1[top1+2]==0x04)
        angle1=angle_tranform(&buff1[(uint8_t)(top1+3)]);
    }
    top1=Size;
  }
  else if(huart==&huart2)
  {
    if(top2<(uint8_t)(top2+9)&&Size-top2==9)
    {
      if(buff2[top2]==0x00&&buff2[top2+1]==0x03&&buff2[top2+2]==0x04)
        angle2=angle_tranform(&buff2[(uint8_t)(top2+3)]);
    }
    top2=Size;
  }
  else if(huart==&huart3)
  {
    if(top3<(uint8_t)(top3+9)&&Size-top3==9)
    {
      if(buff3[top3]==0x00&&buff3[top3+1]==0x03&&buff3[top3+2]==0x04)
        angle3=angle_tranform(&buff3[(uint8_t)(top3+3)]);
    }
    top3=Size;
  }
  else if(huart==&huart4)
  {
    if(top4<(uint8_t)(top4+9)&&Size-top4==9)
    {
      if(buff4[top4]==0x00&&buff4[top4+1]==0x03&&buff4[top4+2]==0x04)
        angle4=angle_tranform(&buff4[(uint8_t)(top4+3)]);
    }
    top4=Size;
  }
  else if(huart==&huart5)
  {
    if(top5<(uint8_t)(top5+9)&&Size-top5==9)
    {
      if(buff5[top5]==0x00&&buff5[top5+1]==0x03&&buff5[top5+2]==0x04)
        angle5=angle_tranform(&buff5[(uint8_t)(top5+3)]);
    }
    top5=Size;
  }
  else if(huart==&huart6)
  {
    if(top6<(uint8_t)(top6+9)&&Size-top6==9)
    {
      if(buff6[top6]==0x00&&buff6[top6+1]==0x03&&buff6[top6+2]==0x04)
        angle6=angle_tranform(&buff6[(uint8_t)(top6+3)]);
    }
    top6=Size;
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
