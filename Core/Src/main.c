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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_cortex.h"
#include "stm32l4xx_hal_pwr.h"
#include "stm32l4xx_hal_spi.h"
#include "stm32l4xx_hal_uart.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h> 
#include <math.h>
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
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//[change point 1] add button, for waking up from sleep mode
volatile int button_pressed_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// --- Definition of Registers ---
#define WHO_AM_I_REG   0x0F
#define CTRL1_XL       0x10
#define CTRL2_G        0x11
#define OUTX_L_G       0x22
#define OUTX_L_XL      0x28

// --- Constant ---
#define SENSITIVITY_4G      0.000061f 

// --- SPI Read/Write ---
void IMU_Write(uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); // CS low
    uint8_t addr = reg & 0x7F;

    HAL_StatusTypeDef st;
    st = HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
    if (st != HAL_OK) printf("IMU_Write: addr tx failed (HAL_Status=%d)\r\n", (int)st);
    
    st = HAL_SPI_Transmit(&hspi2, &data, 1, 100);
    if (st != HAL_OK) printf("IMU_Write: data tx failed (HAL_Status=%d)\r\n", (int)st);
    
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);   // CS high
}

void IMU_Read(uint8_t reg, uint8_t *data, uint16_t len) {
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); // CS low
  uint8_t addr = reg | 0x80; // read + auto-inc
  
  HAL_StatusTypeDef st;
  st = HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
  if (st != HAL_OK) printf("IMU_Read: addr tx failed (HAL_Status=%d)\r\n", (int)st);
  
  for (uint16_t i = 0; i < len; i++) {
    uint8_t tx = 0xFF;
    st = HAL_SPI_TransmitReceive(&hspi2, &tx, &data[i], 1, 100);
    if (st != HAL_OK) {
      printf("IMU_Read: rx[%u] failed (HAL_Status=%d)\r\n", (unsigned)i, (int)st);
      data[i] = 0x00;
    }
  }
  
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);   // CS high
}

// --- Initialization and Read ---
void IMU_Init_SPI(void) {
  HAL_Delay(100);
  uint8_t who_am_i = 0;
  
  // Read ID
  IMU_Read(WHO_AM_I_REG, &who_am_i, 1);
  printf("WHO_AM_I: 0x%02X\r\n", who_am_i);

  if (who_am_i == 0x70 || who_am_i == 0x71 || who_am_i == 0x6A) {
    printf(">>> Sensor Found! Initializing...\r\n");

    // Reset (SW_RESET) & Address Auto Increment (IF_INC)
    IMU_Write(0x12, 0x05); 
    HAL_Delay(50); // wait for reset

    // Make sure for IF_INC and close BDU (protect against partial reads)
    IMU_Write(0x12, 0x04);

    // Set configuration for accelerometer: range ±4g, rate 104Hz
    IMU_Write(CTRL1_XL, 0x44); 
    
    // check if CTRL1_XL really be 0x44
    uint8_t check_val = 0;
    IMU_Read(CTRL1_XL, &check_val, 1);
    
    if(check_val == 0x44) {
        printf(">>> Init SUCCESS! CTRL1_XL is 0x44. Sensor Active.\r\n");
    } else {
        printf("!!! Init FAILED! CTRL1_XL read back: 0x%02X (Expected 0x44) !!!\r\n", check_val);
        printf("Possible cause: SPI Write Error or CS pin issue.\r\n");
    }

  } else {
    printf("Warning: Unknown Device ID 0x%02X.\r\n", who_am_i);
  }
}


// Try alternate SPI CPOL/CPHA combinations if WHO_AM_I was unexpected
void IMU_Probe_SPI_Modes(uint8_t *detected_id) {
  uint8_t id = *detected_id;
  if (id == 0x6A) return; // ok

  struct {
    uint32_t cpol;
    uint32_t cpha;
    const char *name;
  } modes[] = {
    {SPI_POLARITY_LOW,  SPI_PHASE_1EDGE, "Mode 0"},
    {SPI_POLARITY_LOW,  SPI_PHASE_2EDGE, "Mode 1"},
    {SPI_POLARITY_HIGH, SPI_PHASE_1EDGE, "Mode 2"},
    {SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, "Mode 3"},
  };

  for (int m = 0; m < 4; m++) {
    hspi2.Init.CLKPolarity = modes[m].cpol;
    hspi2.Init.CLKPhase = modes[m].cpha;
    
    if (HAL_SPI_DeInit(&hspi2) != HAL_OK) {
      printf("IMU_Probe: HAL_SPI_DeInit failed for %s\r\n", modes[m].name);
    }
    
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
      printf("IMU_Probe: HAL_SPI_Init failed for %s\r\n", modes[m].name);
      continue;
    }
   
    HAL_Delay(10);
    uint8_t who = 0;
    IMU_Read(WHO_AM_I_REG, &who, 1);
    
    printf("Probe %s -> WHO_AM_I=0x%02X\r\n", modes[m].name, who);
    if (who == 0x6A) {
      *detected_id = who;
      printf("IMU found with %s\r\n", modes[m].name);
      return;
    }
  }
}

// Helper: read raw accel registers (6 bytes)
void IMU_ReadRawAccel(uint8_t raw[6]) {
  IMU_Read(OUTX_L_XL, raw, 6);
}

// === Algorithm 1: calculate vector magnitude ===
float Get_Vector(void) {
    uint8_t raw[6];
    IMU_Read(OUTX_L_XL, raw, 6);
    int16_t x = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t y = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t z = (int16_t)(raw[5] << 8 | raw[4]);
    float xg = x * SENSITIVITY_4G;
    float yg = y * SENSITIVITY_4G;
    float zg = z * SENSITIVITY_4G;
    return sqrtf(xg*xg + yg*yg + zg*zg);
}


// === Algorithm 2: calculate angle ===
float Get_Angle(void) {
    uint8_t raw[6];
    // Read acceleration data (SPI read 6 bytes)
    IMU_Read(OUTX_L_XL, raw, 6);
    
    // Convert raw data
    // angle calculation temporarily not using X axis
    int16_t y = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t z = (int16_t)(raw[5] << 8 | raw[4]);
    
    // Convert to physical units (g)
    float yg = y * SENSITIVITY_4G;
    float zg = z * SENSITIVITY_4G;
    
    // Calculate angle (using atan2)
    float angle_rad = atan2f(yg, zg);
    float angle_deg = angle_rad * 57.296f; // 57.296f = (180 / PI)
    
    return fabsf(angle_deg); // Take absolute value
} 

void Profiling_Init(void){
  // Use DWT (Data Watchpoint and Trace)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  
  DWT->CYCCNT = 0; // reset cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // start period
}

void Profiling_Start(void){
  DWT->CYCCNT = 0;
}

uint32_t Profiling_Stop(void){
  return DWT->CYCCNT;
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  // Hardware self-test: UART + LED blink
  printf("\r\n--- Fall Detection Project Start ---\r\n");
  printf("SELFTEST: Blinking LED2 for 3 times\r\n");
  for (int k = 0; k < 3; k++) {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_Delay(200);
  }

  IMU_Init_SPI();

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint8_t init_done = 0;
  if(init_done == 0) {
      IMU_Init_SPI();
      HAL_Delay(500);
      init_done = 1;
  }

  Profiling_Init(); 
  printf("System Clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq()); // Print system clock frequency

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* USER CODE BEGIN 3 */
    
    // ------------------------------------------
    // 1. ACTIVE MODE (data acquisition & processing)
    // ------------------------------------------
    printf("Active Mode...\r\n");
    /* Method 2: improved angle-based detection
       - low-pass filter the angle to remove jitter
       - detect a rapid angle spike (large delta), then check for sustained large tilt
         combined with low acceleration (lying state) within a short window
       - latch alarm (LED2 + message) until cleared with user button (B1)
    */
    for(int i = 0; i < 300; i++){
      uint8_t raw[6];
      IMU_ReadRawAccel(raw);

      int16_t x_raw = (int16_t)(raw[1] << 8 | raw[0]);
      int16_t y_raw = (int16_t)(raw[3] << 8 | raw[2]);
      int16_t z_raw = (int16_t)(raw[5] << 8 | raw[4]);

      float xg = x_raw * SENSITIVITY_4G;
      float yg = y_raw * SENSITIVITY_4G;
      float zg = z_raw * SENSITIVITY_4G;
      
      Profiling_Start();
      float vec = sqrtf(xg*xg + yg*yg + zg*zg);
      uint32_t cycles_vec = Profiling_Stop();

      Profiling_Start();
      float ang = fabsf(atan2f(sqrtf(yg*yg + zg*zg), fabsf(xg)) * 57.296f);
      uint32_t cycles_ang = Profiling_Stop();

      if(i % 100 == 0) {
        
        printf("Perf: Vector=%lu cyc | Angle=%lu cyc\r\n", cycles_vec, cycles_ang);
        
        printf("Z:%d | V:%d.%02d | A:%d\r\n", 
                   (int)z_raw, 
                   (int)vec, (int)((vec - (int)vec)*100),
                   (int)ang);



        // --- Energy Calculation---
        
        // get system frequency
        uint32_t freq_hz = HAL_RCC_GetSysClockFreq();
        float freq_mhz = (float)freq_hz / 1000000.0f;

        // Calculate power
        float power_active_mW = 0.5f * freq_mhz; 
        float power_sleep_mW = 0.015f; 

        // Calculate Time
        float time_vec_s = (float)cycles_vec / (float)freq_hz;
        float time_ang_s = (float)cycles_ang / (float)freq_hz;
        float time_sleep_s = 5.0f; 

        // Calculate Energy
        float energy_vec_uJ = power_active_mW * time_vec_s * 1000.0f * 1000.0f; 
        float energy_ang_uJ = power_active_mW * time_ang_s * 1000.0f * 1000.0f;
        float energy_sleep_uJ = power_sleep_mW * time_sleep_s * 1000.0f * 1000.0f;
        float total_energy = energy_ang_uJ + energy_sleep_uJ;

        printf("\r\n--- Energy Report ---\r\n");
        
        printf("Freq: %d.%d MHz | Active Pwr: %d.%02d mW\r\n", 
               (int)freq_mhz, (int)((freq_mhz - (int)freq_mhz)*10),
               (int)power_active_mW, (int)((power_active_mW - (int)power_active_mW)*100));

        printf("Algo 1 (Vector): %lu cyc -> %d.%04d uJ\r\n", 
               cycles_vec, 
               (int)energy_vec_uJ, (int)((energy_vec_uJ - (int)energy_vec_uJ)*10000));
               
        printf("Algo 2 (Angle) : %lu cyc -> %d.%04d uJ\r\n", 
               cycles_ang, 
               (int)energy_ang_uJ, (int)((energy_ang_uJ - (int)energy_ang_uJ)*10000));
               
        printf("Sleep Mode (5s): %d.%02d uJ\r\n", 
               (int)energy_sleep_uJ, (int)((energy_sleep_uJ - (int)energy_sleep_uJ)*100));
               
        printf("Total Energy/Period: %d.%02d uJ\r\n", 
               (int)total_energy, (int)((total_energy - (int)total_energy)*100));
               
        printf("---------------------\r\n");
      }

      // Thereshold check
      if(vec > 1.5f || ang > 60.0f){
         printf("!!! FALL DETECTED !!! (V:%d.%02d)\r\n", (int)vec, (int)((vec - (int)vec)*100));

         printf("X:%d | Y:%d | Z:%d | V:%.2f\r\n", x_raw, y_raw, z_raw, vec);
          
         HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
         HAL_Delay(1000); 
         HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
         
         //break;
      }

      HAL_Delay(10);
    }
    
    // ------------------------------------------
    // 2. SLEEP MODE
    // ------------------------------------------

    printf("Entering Sleep Mode (5s auto-wakeup)...\r\n");
    
    
    uint32_t sleep_start = HAL_GetTick();
    
    while ((HAL_GetTick() - sleep_start) < 5000) 
    {
      HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
    
    printf("--- Auto Woke Up! ---\r\n");

  
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == B1_Pin){
        button_pressed_flag = 1;
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
