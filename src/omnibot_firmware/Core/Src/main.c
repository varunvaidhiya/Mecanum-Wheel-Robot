#include "main.h"
#include "stm32f4xx_hal.h"

// Private variables
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2, htim3, htim4, htim5;
TIM_HandleTypeDef htim1, htim8, htim9, htim10;

// Wheel velocities (rad/s)
float wheel_velocities[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// Encoder values (ticks)
int32_t encoder_values[4] = {0, 0, 0, 0};

// Private function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

int main(void)
{
  // Reset of all peripherals, Initializes the Flash interface and the Systick
  HAL_Init();
  
  // Configure the system clock
  SystemClock_Config();
  
  // Initialize all configured peripherals
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM_Init();
  
  // Start PWM generation
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  
  // Start encoder interfaces
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim9, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim10, TIM_CHANNEL_ALL);
  
  // Main loop
  while (1)
  {
    // Read encoder values
    encoder_values[0] = (int32_t)TIM1->CNT;
    encoder_values[1] = (int32_t)TIM8->CNT;
    encoder_values[2] = (int32_t)TIM9->CNT;
    encoder_values[3] = (int32_t)TIM10->CNT;
    
    // Send encoder values to Raspberry Pi
    char buffer[64];
    sprintf(buffer, "<ENCODERS,%ld,%ld,%ld,%ld>\n", 
            encoder_values[0], encoder_values[1], 
            encoder_values[2], encoder_values[3]);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    // Small delay
    HAL_Delay(10);
  }
}

// System Clock Configuration
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
  // Configure the main internal regulator output voltage
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  // Initializes the RCC Oscillators according to the specified parameters
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Initializes the CPU, AHB and APB buses clocks
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

// Error handler
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
} 