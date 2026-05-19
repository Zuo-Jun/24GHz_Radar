/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "Radar_protocol.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SP3485E.h"
#include "Radar_frame.h"
#include "Radar_protocol.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define PERIPH_POWER_ON()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
// #define PERIPH_POWER_OFF()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)

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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  RS485_Init();
  //  RS485_SendStr("$RADAR,BOOT:OK\r\n");

  RadarProtocol_Init(1U); /* 设备ID=1，单设备时固定为1 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* ======================================================
     *  ADF4153A（PLL）初始化
     *
     *  起始频率24.000GHz：
     *    RF = 24000MHz / 16 = 1500MHz
     *    N = 1500MHz / 25MHz = 60.0
     *    INT=60, FRAC=0
     *  prescaler=0（4/5模式）：1500MHz < 2GHz，INT=60 ≥ 31 ✓
     * ====================================================== */
    // ADF4153A_Config_t pll_cfg = {
    //     .fastlock_en    = 0,
    //     .INT            = 60,
    //     .FRAC           = 0,
    //     .resync_en      = 0,
    //     .muxout         = MUXOUT_DLOCK_DIGITAL,
    //     .prescaler      = RADAR_PRESCALER,   /* 0=4/5，来自radar_config.h */
    //     .R              = RADAR_R,           /* 1 */
    //     .MOD            = RADAR_MOD,         /* 125 */
    //     .resync         = 0,
    //     .refin_doubler  = RADAR_REFIN_DBL,   /* 0 */
    //     .cp_current     = 7,
    //     .pd_polarity    = 1,
    //     .ldp            = 0,
    //     .noise_spur     = NOISE_SPUR_LOWEST_NOISE,
    // };

    // BGT24MTR11_Config_t bgt_cfg = {
    // .gs               = 0,              /* 正常LNA增益 */
    // .tx_disable       = 0,              /* 允许发射 */
    // .tx_pa_level      = 7,              /* PA最大功率(+11dBm) */
    // .tx_buf_high_pwr  = 1,              /* TX Buffer高功率 */
    // .lo_buf_high_pwr  = 1,              /* LO Buffer高功率 */
    // .dis_div16        = 0,              /* 必须为1，否则ADF失锁 */
    // .dis_div64k       = 1,              /* Q2输出不需要可禁用 */
    // .amux             = BGT_AMUX_VTEMP, /* ANA引脚输出温度信号 */
    // };
    
    // ADF4153A_Init(&pll_cfg);

    // if (!ADF4153A_WaitLock(500)) {
    //     RS485_SendStr("$RADAR,ERR:PLL_LOCK_FAIL\r\n");
    //     Error_Handler();
    //     return -1;
    // }
    // RS485_SendStr("$RADAR,PLL:LOCKED\r\n");
    // /* ANA引脚是否配置，GPIO配置为输入该如何解读数据 */
    // BGT24MTR11_Init(&bgt_cfg);

    // /* 预计算chirp查找表（只做一次） */
    // Chirp_Precompute();

    /* ======================================================
     *  主循环：每帧约50ms
     *    阶段1：水位  1次上扫 → Range-FFT → 水面距离（约2ms）
     *    阶段2：水速 16对三角波 → 相位差分 → 水速（约34ms）
     *  帧结束后通过RS-485上报结果
     * ====================================================== */
    // RadarFrame_t frame;

  while (1)
  {
    // Radar_MeasureFrame(&frame);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    RadarFrame_t frame = {
        .water_level_valid = 1,
        .water_level_m = 1.234f,
        .water_velocity_valid = 1,
        .water_velocity_mps = 0.567f,
    };

    /* 当前先手动补硬件状态位，后续可从硬件状态寄存器读取 */
    uint16_t extra_status = RADAR_STATUS_PLL_LOCK_OK |
                            RADAR_STATUS_ADC_OK;

    /* 只发二进制帧，不等ACK */
    // RadarProtocol_SendData(&frame, extra_status);
    /* 发二进制帧，等待ACK，失败重试 */
    RadarProtocol_SendDataWaitAck(&frame, extra_status);

    /* 如果想用串口助手看ASCII，可临时改成： */
    // RadarProtocol_SendDebug(&frame); 

    HAL_Delay(1000);
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
#ifdef USE_FULL_ASSERT
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
