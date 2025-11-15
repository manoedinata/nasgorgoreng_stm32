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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t steering_pwm;
  uint16_t sensor_servo_pwm;
  int16_t motor_pwm;
} ControlData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFFER_SIZE 64
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_NEUTRAL 1500

// GPIO Pin defines
#define MOTOR_IN1_Pin GPIO_PIN_1
#define MOTOR_IN1_GPIO_Port GPIOA
#define MOTOR_IN2_Pin GPIO_PIN_4
#define MOTOR_IN2_GPIO_Port GPIOA
#define HC_SR04_TRIG_Pin GPIO_PIN_5
#define HC_SR04_TRIG_GPIO_Port GPIOA
#define HC_SR04_ECHO_Pin GPIO_PIN_6
#define HC_SR04_ECHO_GPIO_Port GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// HC-SR04 variables
float distance_cm = 0.0;
uint8_t measurement_done = 0;

// Control data
ControlData_t control_data;
char uart_rx_buffer[UART_BUFFER_SIZE];
uint8_t uart_rx_index = 0;

// Flag to signal new command from UART
volatile uint8_t g_uart_command_ready = 0;

// Timing variables
uint32_t last_distance_send = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// HC-SR04 Functions
void HC_SR04_StartMeasurement(void);
uint32_t HC_SR04_GetPulseDuration(void);
float HC_SR04_GetDistance(void);
void HC_SR04_Delay_us(uint16_t us);

// PWM Control Functions
void SetSteeringPWM(uint16_t pwm_value);
void SetSensorServoPWM(uint16_t pwm_value);
void SetMotorSpeed(int16_t speed);

// UART Functions
void ParseUARTCommand(void);
void SendDistanceData(void);

// Utility Functions
uint32_t Get_Micros(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HC_SR04_Delay_us(uint16_t us) {
  // Catat waktu 'mulai' saat ini dari timer yang sedang berjalan
  uint32_t start_time = __HAL_TIM_GET_COUNTER(&htim3);

  // Tunggu dalam loop sampai selisih waktunya >= us
  // Pengurangan unsigned int (uint32_t) akan secara otomatis
  // menangani overflow timer dengan benar
  while ((__HAL_TIM_GET_COUNTER(&htim3) - start_time) < us);
}

// Fungsi untuk mendapatkan waktu dalam mikrosecond
uint32_t Get_Micros(void) {
  return __HAL_TIM_GET_COUNTER(&htim3);
}

// Fungsi untuk memulai pengukuran HC-SR04
void HC_SR04_StartMeasurement(void) {
  // Reset trigger pin
  HAL_GPIO_WritePin(HC_SR04_TRIG_GPIO_Port, HC_SR04_TRIG_Pin, GPIO_PIN_RESET);
  HC_SR04_Delay_us(2);

  // Kirim trigger pulse 10us
  HAL_GPIO_WritePin(HC_SR04_TRIG_GPIO_Port, HC_SR04_TRIG_Pin, GPIO_PIN_SET);
  HC_SR04_Delay_us(10);
  HAL_GPIO_WritePin(HC_SR04_TRIG_GPIO_Port, HC_SR04_TRIG_Pin, GPIO_PIN_RESET);
}

// Fungsi untuk membaca pulse duration dari HC-SR04
uint32_t HC_SR04_GetPulseDuration(void) {
    uint32_t timeout = 1000000; // Increased timeout
    uint32_t start_time = 0, end_time = 0;

    // Wait for ECHO to be LOW (ensure clean start)
    timeout = 100000;
    while (HAL_GPIO_ReadPin(HC_SR04_ECHO_GPIO_Port, HC_SR04_ECHO_Pin) == GPIO_PIN_SET) {
        if (timeout-- == 0) {
//            printf("Timeout: ECHO stuck HIGH\r\n");
            return 0;
        }
    }

    // Wait for rising edge (start of pulse)
    timeout = 100000;
    while (HAL_GPIO_ReadPin(HC_SR04_ECHO_GPIO_Port, HC_SR04_ECHO_Pin) == GPIO_PIN_RESET) {
        if (timeout-- == 0) {
//            printf("Timeout: No rising edge\r\n");
            return 0;
        }
    }
    start_time = Get_Micros();

    // Wait for falling edge (end of pulse)
    timeout = 100000;
    while (HAL_GPIO_ReadPin(HC_SR04_ECHO_GPIO_Port, HC_SR04_ECHO_Pin) == GPIO_PIN_SET) {
        if (timeout-- == 0) {
//            printf("Timeout: No falling edge\r\n");
            return 0;
        }
    }
    end_time = Get_Micros();

    // Calculate duration (handle timer overflow)
    uint32_t pulse_duration;
    if (end_time >= start_time) {
        pulse_duration = end_time - start_time;
    } else {
        pulse_duration = (0xFFFF - start_time) + end_time;
    }

    // Validate pulse duration (58us to 38ms for 1cm to 6.5m)
    if (pulse_duration < 58 || pulse_duration > 38000) {
//        printf("Invalid pulse: %lu us\r\n", pulse_duration);
        return 0;
    }

    return pulse_duration;
}

// Fungsi untuk mendapatkan jarak dalam cm
float HC_SR04_GetDistance(void) {
  HC_SR04_StartMeasurement();
  uint32_t pulse_duration = HC_SR04_GetPulseDuration();

  if (pulse_duration == 0) {
//    return 0.0; // Timeout atau error
	  return distance_cm; // return current distance if error
  }

  // Konversi ke cm (34000 cm/s, dibagi 2 untuk pulang-pergi)
  float distance = (pulse_duration * 0.034) / 2.0;

  // Filter nilai valid (2cm - 400cm)
  if (distance > 400.0 || distance < 2.0) {
//    return 0.0;
	  return distance;
  }

  return distance;
}

// Fungsi untuk set PWM Steering Servo
void SetSteeringPWM(uint16_t pwm_value) {
  if (pwm_value < PWM_MIN) pwm_value = PWM_MIN;
  if (pwm_value > PWM_MAX) pwm_value = PWM_MAX;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
}

// Fungsi untuk set PWM Sensor Servo
void SetSensorServoPWM(uint16_t pwm_value) {
  if (pwm_value < PWM_MIN) pwm_value = PWM_MIN;
  if (pwm_value > PWM_MAX) pwm_value = PWM_MAX;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value);
}

void SetMotorSpeed(int16_t speed)
{
  // Limit speed to -1000 to +1000 range
  if (speed > 1000) speed = 1000;
  if (speed < -1000) speed = -1000;

  // Safety: Stop motors first
  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
  HAL_Delay(1); // Short delay for safety

  // Set direction based on sign
  if (speed > 0) {
    // Forward
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
  }
  else if (speed < 0) {
    // Backward
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -speed);
  }
  else {
    // Stop
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  }
}

// Parse command dari ESP32
void ParseUARTCommand(void) {
  char* token = strtok(uart_rx_buffer, ",");

  while (token != NULL) {
    if (strncmp(token, "S:", 2) == 0) {
      control_data.steering_pwm = atoi(token + 2);
      SetSteeringPWM(control_data.steering_pwm);
    }
    else if (strncmp(token, "T:", 2) == 0) {
      control_data.sensor_servo_pwm = atoi(token + 2);
      SetSensorServoPWM(control_data.sensor_servo_pwm);
    }
    else if (strncmp(token, "M:", 2) == 0) {
      control_data.motor_pwm = atoi(token + 2);
      SetMotorSpeed(control_data.motor_pwm);
    }

    token = strtok(NULL, ",");
  }

}

// Kirim data jarak ke ESP32
void SendDistanceData(void) {
//  char buffer[32];
//  int len = sprintf(buffer, "DIST:%.2f\r\n", distance_cm);
//  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);
	printf("DIST:%.2f\r\n", distance_cm);
}

// UART Receive Complete Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {

    // If we are already processing a command, ignore this byte
    if (g_uart_command_ready == 1) {
      HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
      return;
    }

    if (uart_rx_buffer[uart_rx_index] == '\n') {
      uart_rx_buffer[uart_rx_index] = '\0'; // Null terminate
      g_uart_command_ready = 1; // Set the flag!
      uart_rx_index = 0;
    } else {
      uart_rx_index++;
      if (uart_rx_index >= UART_BUFFER_SIZE) {
        uart_rx_index = 0; // Overflow
      }
    }

    // Re-arm the interrupt ONLY if we are not processing
    if (g_uart_command_ready == 0) {
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
    }
  }
}

// Redirect printf untuk debug
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
  return len;
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Start PWM untuk servo
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  // Start PWM untuk motor
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  // Start TIM3 untuk measurement (microseconds)
  HAL_TIM_Base_Start(&htim3);

  // Inisialisasi data kontrol
  control_data.steering_pwm = PWM_NEUTRAL;
  control_data.sensor_servo_pwm = PWM_NEUTRAL;
  control_data.motor_pwm = 0;

  // Set servo ke posisi netral
  SetSteeringPWM(PWM_NEUTRAL);
  SetSensorServoPWM(PWM_NEUTRAL);
  SetMotorSpeed(0);

  // Inisialisasi HC-SR04
  HAL_GPIO_WritePin(HC_SR04_TRIG_GPIO_Port, HC_SR04_TRIG_Pin, GPIO_PIN_RESET);

  // Start UART receive interrupt
  HAL_UART_Receive_IT(&huart2, (uint8_t*)uart_rx_buffer, 1);

//  printf("STM32 Line Follower Slave Ready\r\n");
//  printf("HC-SR04 Initialized with TIM3 measurement\r\n");
//  printf("PWM Servo and Motor Ready\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      // --- 1. Check for new UART command ---
      if (g_uart_command_ready) {
        g_uart_command_ready = 0; // Clear the flag immediately

        ParseUARTCommand(); // Do the heavy work HERE

        memset(uart_rx_buffer, 0, UART_BUFFER_SIZE); // Clear buffer

        // Re-arm interrupt to receive the next byte
        HAL_UART_Receive_IT(&huart2, (uint8_t*)uart_rx_buffer, 1);
      }

      // --- 2. Baca jarak HC-SR04 ---
      distance_cm = HC_SR04_GetDistance();

      // --- 3. Kirim data jarak setiap 100ms ---
      if (HAL_GetTick() - last_distance_send >= 100) {
        SendDistanceData();
        last_distance_send = HAL_GetTick();
      }

      HAL_Delay(10); // Small delay to reduce CPU load
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
