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
#include <string.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>  // Tambahkan untuk fungsi abs()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Communication structures
typedef struct {
    uint8_t header;
    int16_t pwm_motor;
    int16_t pwm_servo;
    uint8_t checksum;
} ESP_Command_t;

typedef struct {
    uint8_t header;
    int16_t distance;
    int16_t encoder;
    uint8_t checksum;
} Sensor_Data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HC_Trigger_Pin GPIO_PIN_0
#define HC_Trigger_GPIO_Port GPIOA
#define HC_Echo_EXT1_Pin GPIO_PIN_1
#define HC_Echo_EXT1_GPIO_Port GPIOA
#define Motor_IN1_Pin GPIO_PIN_4
#define Motor_IN1_GPIO_Port GPIOA
#define Motor_IN2_Pin GPIO_PIN_5
#define Motor_IN2_GPIO_Port GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Global variables
ESP_Command_t rx_cmd;
Sensor_Data_t tx_data;
uint8_t uart_rx_buffer[sizeof(ESP_Command_t)];

// HC-SR04 variables (volatile for interrupt)
volatile uint32_t echo_start_time = 0;
volatile uint32_t echo_end_time = 0;
volatile uint32_t echo_duration = 0;
volatile uint8_t echo_capture_complete = 0;
volatile uint8_t echo_measuring = 0;

uint16_t obstacle_distance = 0;
int32_t encoder_value = 0;
int16_t current_motor_pwm = 0;
int16_t current_servo_angle = 90;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
// Function prototypes
void Set_Motor_PWM(int16_t pwm);
void Set_Servo_Angle(int16_t angle);
void Trigger_Distance_Measurement(void);
void Read_Distance_Sensor(void);
void Read_Encoder(void);
void Send_Sensor_Data(void);
uint8_t Calculate_Checksum(uint8_t* data, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Override _write for printf
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// UART Receive Complete Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Validate received data
        if (uart_rx_buffer[0] == 0xA5) {
            uint8_t checksum = Calculate_Checksum(uart_rx_buffer, sizeof(ESP_Command_t)-1);
            if (checksum == uart_rx_buffer[sizeof(ESP_Command_t)-1]) {
                memcpy(&rx_cmd, uart_rx_buffer, sizeof(ESP_Command_t));

                // Execute commands from ESP32
                Set_Motor_PWM(rx_cmd.pwm_motor);
                Set_Servo_Angle(rx_cmd.pwm_servo);

                printf("CMD Received - Motor: %d, Servo: %d\r\n",
                       rx_cmd.pwm_motor, rx_cmd.pwm_servo);
            }
        }

        // Restart UART reception
        HAL_UART_Receive_IT(&huart2, uart_rx_buffer, sizeof(ESP_Command_t));
    }
}

// External Interrupt Callback for HC-SR04 Echo
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == HC_Echo_EXT1_Pin) {
        if (HAL_GPIO_ReadPin(HC_Echo_EXT1_GPIO_Port, HC_Echo_EXT1_Pin) == GPIO_PIN_SET) {
            // Rising edge - start measurement
            echo_start_time = HAL_GetTick();
            echo_measuring = 1;
        } else {
            // Falling edge - end measurement
            if (echo_measuring) {
                echo_end_time = HAL_GetTick();
                echo_duration = echo_end_time - echo_start_time;
                echo_capture_complete = 1;
                echo_measuring = 0;
            }
        }
    }
}

// Set motor PWM and direction
void Set_Motor_PWM(int16_t pwm) {
    // Constrain PWM values (0 to 1000)
    uint16_t pwm_abs = (pwm < 0) ? 0 : (pwm > 1000) ? 1000 : pwm;

    if (pwm_abs > 0) {
        // Motor FORWARD only (single motor application)
        HAL_GPIO_WritePin(Motor_IN1_GPIO_Port, Motor_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_IN2_GPIO_Port, Motor_IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_abs);
    } else {
        // Motor STOP
        HAL_GPIO_WritePin(Motor_IN1_GPIO_Port, Motor_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_IN2_GPIO_Port, Motor_IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }

    current_motor_pwm = pwm_abs;
}

// Set servo angle (0-180 degrees)
void Set_Servo_Angle(int16_t angle) {
    // Constrain angle (0-180 degrees)
    angle = (angle < 0) ? 0 : (angle > 180) ? 180 : angle;

    // Convert angle to PWM pulse width (500-2500 microseconds)
    uint16_t pulse_width = 500 + (angle * 2000 / 180);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_width);

    current_servo_angle = angle;
}

// Trigger HC-SR04 measurement
void Trigger_Distance_Measurement(void) {
    // Ensure we're not already measuring
    if (!echo_measuring) {
        HAL_GPIO_WritePin(HC_Trigger_GPIO_Port, HC_Trigger_Pin, GPIO_PIN_SET);
        HAL_Delay(1);  // 10us minimum, but HAL_Delay is in ms
        HAL_GPIO_WritePin(HC_Trigger_GPIO_Port, HC_Trigger_Pin, GPIO_PIN_RESET);
    }
}

// Read and process distance from HC-SR04
void Read_Distance_Sensor(void) {
    if (echo_capture_complete) {
        // Convert duration to distance (cm)
        // Speed of sound: 340 m/s = 0.034 cm/us
        // Using HAL_GetTick() in milliseconds: 1 ms = 34 cm round trip = 17 cm one way
        obstacle_distance = (echo_duration * 17);

        // Limit maximum distance to 400cm (HC-SR04 max range)
        if (obstacle_distance > 400) {
            obstacle_distance = 400;
        }

        echo_capture_complete = 0;
    }
}

// Read encoder value - PERBAIKAN: gunakan abs() dengan cast ke int
void Read_Encoder(void) {
    encoder_value = (int16_t)TIM3->CNT;

    // Reset counter if approaching overflow
    if (abs((int)encoder_value) > 30000) {  // Cast ke int untuk menghindari warning
        TIM3->CNT = 0;
    }
}

// Send sensor data to ESP32
void Send_Sensor_Data(void) {
    tx_data.header = 0x5A;
    tx_data.distance = obstacle_distance;
    tx_data.encoder = (int16_t)encoder_value;  // Pastikan tipe data match
    tx_data.checksum = Calculate_Checksum((uint8_t*)&tx_data, sizeof(Sensor_Data_t)-1);

    HAL_UART_Transmit(&huart2, (uint8_t*)&tx_data, sizeof(Sensor_Data_t), 10);
}

// Calculate checksum for data validation
uint8_t Calculate_Checksum(uint8_t* data, uint8_t len) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // Start peripherals
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    // PWM Motor
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Encoder
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);    // PWM Servo

  // Initialize motor to STOP state
  HAL_GPIO_WritePin(Motor_IN1_GPIO_Port, Motor_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Motor_IN2_GPIO_Port, Motor_IN2_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

  // Initialize servo to center position (90 degrees)
  Set_Servo_Angle(90);

  // Start UART reception
  HAL_UART_Receive_IT(&huart2, uart_rx_buffer, sizeof(ESP_Command_t));

  printf("=== STM32F401 Robot Controller ===\r\n");
  printf("Single Motor + HC-SR04 + Encoder\r\n");
  printf("Waiting for commands from ESP32...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_sensor_read = 0;
  uint32_t last_trigger = 0;
  uint32_t last_debug = 0;

  while (1)
  {
    uint32_t current_time = HAL_GetTick();

    // 1. Trigger distance measurement every 100ms
    if (current_time - last_trigger >= 100) {
        Trigger_Distance_Measurement();
        last_trigger = current_time;
    }

    // 2. Read sensors and send data every 50ms
    if (current_time - last_sensor_read >= 50) {
        Read_Distance_Sensor();
        Read_Encoder();
        Send_Sensor_Data();
        last_sensor_read = current_time;
    }

    // 3. Debug output every 1 second - PERBAIKAN: gunakan %ld untuk int32_t
    if (current_time - last_debug >= 1000) {
        printf("Status - Distance: %dcm, Encoder: %ld, Motor PWM: %d, Servo: %d\r\n",
               obstacle_distance,
               (long)encoder_value,  // Cast ke long dan gunakan %ld
               current_motor_pwm,
               current_servo_angle);
        last_debug = current_time;
    }

    HAL_Delay(1);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 839;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HC_Trigger_Pin|Motor_IN1_Pin|Motor_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HC_Trigger_Pin Motor_IN1_Pin Motor_IN2_Pin */
  GPIO_InitStruct.Pin = HC_Trigger_Pin|Motor_IN1_Pin|Motor_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HC_Echo_EXT1_Pin */
  GPIO_InitStruct.Pin = HC_Echo_EXT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(HC_Echo_EXT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
