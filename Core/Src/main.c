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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mb.h"
#include "mt_port.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 13

#define I2C_ADDRESS_1115    0x4A //1115
#define I2C_ADDRESS_1115_1  0x4B //1115
#define I2C_ADDRESS_1110    0x48 //1110
#define I2C_ADDRESS_1110_1  0x49 //1110

//Define 1-shot read or continuous
#define ONESHOT_ON  0b10000001
#define ONESHOT_OFF 0

//Define Analog Inputs
#define AI0 0b01000000 //  AINP = AIN0 and AINN = GND
#define AI1 0b01010000 //  AINP = AIN1 and AINN = GND
#define AI2 0b01100000 //  AINP = AIN2 and AINN = GND
#define AI3 0b01110000 //  AINP = AIN3 and AINN = GND

//Define gain setting
#define GAIN1 0b00000000  //Full Scale Voltage is: 6.144 + or -
#define GAIN2 0b00000010  //FS 4.096
#define GAIN3 0b00000100  //FS 2.048
#define GAIN4 0b00000110  //FS 1.024
#define GAIN5 0b00001000  //FS  .512
#define GAIN6 0b00001010  //FS  .256
#define GAIN7 0b00001100  //FS  .256
#define GAIN8 0b00001110  //FS  .256

#define I2C_ID_ADDRESS                                           0xD0
#define I2C_TIMEOUT                                              50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS] ;

uint8_t i2c_1115[2] = {I2C_ADDRESS_1115, I2C_ADDRESS_1115_1};
uint8_t i2c_1110[2] = {I2C_ADDRESS_1110, I2C_ADDRESS_1110_1};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t lock_nesting_count = 0;
void __critical_enter(void)
{
    __disable_irq();
    ++lock_nesting_count;
}
void __critical_exit(void)
{
    /* Unlock interrupts only when we are exiting the outermost nested call. */
    --lock_nesting_count;
    if (lock_nesting_count == 0) {
        __enable_irq();
    }
}

typedef enum  {
  SEND_CONFIG = 0,
  WAIT_MEAS, 
  SWITCH_REG,
} state_t;

uint8_t ai_arr[4] = {AI0, AI1, AI2, AI3}; 
static uint8_t ai = 0;


void readI2CRegs()
{
    static state_t state = SEND_CONFIG;
    uint8_t writeBuf[3] = {1, ONESHOT_ON + GAIN1, 0b11100000} ;
    writeBuf[1] = ONESHOT_ON + GAIN1 + ai_arr[ai]; 
    uint8_t writeBuf_110[2] = {1, 0b10000000} ;

    switch (state)
    {
    case SEND_CONFIG:

      for (int i = 0; i < 2; i++) {
        if (ai == 0) {
          HAL_I2C_Master_Transmit(&hi2c2, (i2c_1110[i] << 1), writeBuf_110, 2,  I2C_TIMEOUT);
          HAL_Delay(1);
        }

        #ifndef WITHOUT_115
        if (HAL_I2C_Master_Transmit(&hi2c2, (i2c_1115[i] << 1), writeBuf, 3,  I2C_TIMEOUT) == HAL_OK) 
          usRegInputBuf[REG_INPUT_NREGS - 3] |= 1 << i;
        else
          usRegInputBuf[REG_INPUT_NREGS - 3] &= ~(1 << i);
        HAL_Delay(1);
        #endif
      }

      state++;
      break;

    case WAIT_MEAS:
      if (ai < 4 ) {
        #ifndef WITHOUT_115
        for (int i = 0; i < 2; i++) {
          if (usRegInputBuf[REG_INPUT_NREGS - 3] & (1 << i)) 
            do{
              HAL_I2C_Master_Receive(&hi2c2, (i2c_1115[i] << 1), writeBuf, 2,  I2C_TIMEOUT);
            } while ((writeBuf[0] & 0x80) == 0);
        }

        writeBuf[0] = 0;
        for (int i = 0; i < 2; i++)
          if (usRegInputBuf[REG_INPUT_NREGS - 3] & (1 << i)) 
            HAL_I2C_Master_Transmit(&hi2c2, (i2c_1115[i] << 1), writeBuf, 1,  I2C_TIMEOUT);

        for (int i=0; i < 2; i++)
          if (usRegInputBuf[REG_INPUT_NREGS - 3] & (1 << i)) 
            HAL_I2C_Master_Receive(&hi2c2, (i2c_1115[i] << 1),(uint8_t *) ( usRegInputBuf + i*4 + ai), 2,  I2C_TIMEOUT);
        #endif
        state = SEND_CONFIG;
        ai ++; 
      } else {
        ai = 0;
        for (uint8_t i =0; i < 2; i++) {
          if (HAL_I2C_Master_Receive(&hi2c2, (i2c_1110[i] << 1), (uint8_t *)(usRegInputBuf + 8 + i), 2,  I2C_TIMEOUT) == HAL_OK)
            usRegInputBuf[REG_INPUT_NREGS - 3] |= (1 << (i + 2));
          else
            usRegInputBuf[REG_INPUT_NREGS - 3] &= ~(1 << (i +2));
        }
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  uint8_t MSG[35] = {'\0'};
  char* MSG_1115_ERR="1115 transmit err\r\n";
  char* MSG_1115_OK="1115 transmit OK\r\n";
  char* MSG_1110_ERR="1110 transmit err\r\n";
  char* MSG_1110_OK="1110 transmit OK\r\n";

  uint8_t X = 0;
  
  MT_PORT_SetTimerModule(&htim14);
  MT_PORT_SetUartModule(&huart1);
  eMBErrorCode eStatus;
  eStatus = eMBInit(MB_RTU, 1, 0, 115200, MB_PAR_NONE);
  eStatus = eMBEnable();
  // memset (&usRegInputBuf,0, sizeof(usRegInputBuf));
  if (eStatus != MB_ENOERR)
  {
  // Error handling
    
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    readI2CRegs();
    usRegInputBuf[REG_INPUT_NREGS - 2] =  HAL_GetTick() / 1000;
    usRegInputBuf[REG_INPUT_NREGS - 1] =  HAL_GetTick();
    HAL_IWDG_Refresh(&hiwdg);
    eMBPoll();

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00300617;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 1024;
  hiwdg.Init.Reload = 1024;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 399;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TEST_Pin */
  GPIO_InitStruct.Pin = TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
  eMBErrorCode eStatus = MB_ENOERR;
  int iRegIndex;
  if ((usAddress >= REG_INPUT_START) &&
      (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
  {
    iRegIndex = (int)(usAddress - usRegInputStart);
    while(usNRegs > 0)
    {
        *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
        *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
        iRegIndex++;
        usNRegs--;
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}
/*----------------------------------------------------------------------------*/
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                             eMBRegisterMode eMode)
{
  return MB_ENOREG;
}
/*----------------------------------------------------------------------------*/
eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
                           eMBRegisterMode eMode)
{
  return MB_ENOREG;
}
/*----------------------------------------------------------------------------*/
eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
  return MB_ENOREG;
}
/*----------------------------------------------------------------------------*/
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
