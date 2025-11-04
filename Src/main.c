/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "SEGGER_RTT.h"

/* Private define ------------------------------------------------------------*/
#define NUM_CALIBRATION_POINTS (sizeof(temp_calibration_table) / sizeof(TempCalibrationPoint))

// ADC Calibration Constants
#define ADC_MAX_VALUE             4095      /* 12-bit ADC maximum value (2^12 - 1) */
#define VREFINT_MV                1200      /* Internal Vref in mV (e.g., 1.2V = 1200mV) */
#define ADC_AVERAGE_COUNT         16        /* Number of samples for averaging */

// ADC Channel Indexing (Ensure these match the order of configuration in APP_AdcInit)
#define ADC_INDEX_T12             0         /* ADC_CHANNEL_0: T12 Temperature Sensor */
#define ADC_INDEX_RK              1         /* ADC_CHANNEL_2: RK (Resistance Knob) */
#define ADC_INDEX_TEMPSENSOR      2         /* ADC_CHANNEL_TEMPSENSOR */
#define ADC_INDEX_VREFINT         3         /* ADC_CHANNEL_VREFINT */
#define ADC_CHANNEL_COUNT         4         /* Total number of channels to be scanned */

// PWM/Timer Configuration
#define PWM_PERIOD_COUNTS         2000      /* TIM Period (ARR value: 2000-1) */
#define PWM_MAX_DUTY              (PWM_PERIOD_COUNTS - 1) /* Max duty cycle for 100% */
#define PWM_MIN_DUTY              0         /* Min duty cycle for 0% */
#define TIMER_PRESCALER           8         /* TIM Prescaler (8-1) */

// ------------------- Temperature Control Configuration -------------------
#define MAX_TEMPERATURE_LIMIT     400       /* Max safety limit in Celsius */
#define KNOB_ADC_MIN              20        /* Minimum ADC value from the knob */
#define KNOB_ADC_MAX              2000      /* Maximum ADC value from the knob */
#define KNOB_TEMP_MIN             200       /* Temperature corresponding to KNOB_ADC_MAX */
#define KNOB_TEMP_MAX             400       /* Temperature corresponding to KNOB_ADC_MIN */

// ------------------- PID Controller Configuration -------------------
#define PID_SAMPLE_TIME_MS        50        /* Control loop interval in milliseconds */
#define PID_SAMPLE_TIME_S         0.05f     /* Control loop interval in seconds */
#define PID_TUNING_THRESHOLD_C    20        /* Threshold for switching between PID parameter sets (in Celsius) */

// Conservative PID Gains (for fine control when near the setpoint)
#define PID_KP_CONSERVATIVE  5.f
#define PID_KI_CONSERVATIVE  1.f
#define PID_KD_CONSERVATIVE  0.2f

// Aggressive PID Gains (for fast heating when far from the setpoint)
#define PID_KP_AGGRESSIVE         10.0f
#define PID_KI_AGGRESSIVE         1.f
#define PID_KD_AGGRESSIVE         0.2f

#define SLEEP_TIMEOUT_MS  60000

typedef struct {
    uint16_t adc_val;
    uint16_t temp_c;
} TempCalibrationPoint;

typedef struct {
    float Kp, Ki, Kd;
    int32_t out_min, out_max;
    float integral, prev_measurement;
    float sample_time;
} PID_Controller;

typedef enum
{
  IRON_STATE_WORKING = 0,
  IRON_STATE_SLEEP
} IronState_t;

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef   AdcHandle;
TIM_HandleTypeDef   TimHandle;
PID_Controller      pid;

const TempCalibrationPoint temp_calibration_table[] = {
    {0,    25},
    {1212, 216},
    {1696, 308},
    {2181, 390},
};
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
void APP_AdcPoll(void);
static void APP_AdcInit(void);
static void APP_TimInit(void);
static void APP_GpioInit(void);
static void APP_FlashSetOptionBytes(void);
static void APP_ConfigureExti(void);
static void APP_EnterSleepMode(void);
static void APP_ExitSleepMode(void);
uint16_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
uint16_t calculateTemp(uint16_t RawTemp);
// PID Controller Functions
void PID_Init(PID_Controller* pid_controller);
uint16_t PID_Compute(PID_Controller* pid_controller, int16_t setpoint, int16_t measurement);

uint32_t adc_value[4];
uint8_t SW = 0;
volatile IronState_t current_state = IRON_STATE_WORKING;
uint16_t currentTemperature, targetTemperature, pwm_output;
uint32_t last_activity_time = 0; // for sleep
uint32_t last_control_tick = 0; //for main loop

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();

  /* Configure the system clock */
  APP_SystemClockConfig(); 
  
  last_activity_time = HAL_GetTick();
  
  /* Initialize peripherals */
  APP_AdcInit();
  APP_TimInit();
  APP_GpioInit();
  APP_ConfigureExti();
  
  SEGGER_RTT_printf(0, "hello world!\n");
  
  /* Initialize PID controller */
  PID_Init(&pid);
  
  // Disable PF2 reset
  if(READ_BIT(FLASH->OPTR, FLASH_OPTR_NRST_MODE) == OB_RESET_MODE_RESET)
  {
    APP_FlashSetOptionBytes();
  }
  
  while (1)
  {
    uint32_t current_tick = HAL_GetTick();
    
    if ((current_tick - last_activity_time) >= SLEEP_TIMEOUT_MS)
    {
      APP_EnterSleepMode();
    }

    if ((current_tick - last_control_tick) >= PID_SAMPLE_TIME_MS)
    {
      last_control_tick = current_tick;
      // 1. Read all sensor values from ADC
      APP_AdcPoll();
      uint16_t VCC_mv = (ADC_MAX_VALUE * VREFINT_MV) / adc_value[ADC_INDEX_VREFINT];
			uint16_t T12_mv = adc_value[ADC_INDEX_T12] * VCC_mv / ADC_MAX_VALUE;
			
      // 2. Process sensor values into meaningful units (temperatures in Celsius)
      if (current_state == IRON_STATE_WORKING)
      {
        targetTemperature = map(adc_value[ADC_INDEX_RK], KNOB_ADC_MIN, KNOB_ADC_MAX, KNOB_TEMP_MAX, KNOB_TEMP_MIN);
      }
      else
      {
        targetTemperature = 30; // sleep target temperature
      }
      currentTemperature = calculateTemp(T12_mv);
      // Calculate the absolute temperature error
      int16_t error = targetTemperature - currentTemperature;
      // If the temperature difference is large, use aggressive PID gains for faster heating.
      // Otherwise, switch to conservative gains for better stability near the setpoint.
      if (error > PID_TUNING_THRESHOLD_C)
      {
          pid.Kp = PID_KP_AGGRESSIVE;
          pid.Ki = PID_KI_AGGRESSIVE;
          pid.Kd = PID_KD_AGGRESSIVE;
      }
      else
      {
          pid.Kp = PID_KP_CONSERVATIVE;
          pid.Ki = PID_KI_CONSERVATIVE;
          pid.Kd = PID_KD_CONSERVATIVE;
      }
      
      // 3. Compute the new heater output using the PID controller
      pwm_output = PID_Compute(&pid, targetTemperature, currentTemperature);

      // 4. Update the heater PWM duty cycle
      __HAL_TIM_SET_COMPARE(&TimHandle, TIM_CHANNEL_2, pwm_output);

      // Debug
      SEGGER_RTT_printf(0, "%d %d %d %d\n", targetTemperature, currentTemperature, pwm_output, (uint16_t)pid.Kp);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (currentTemperature >= targetTemperature));

      // 5. Wait for the next control cycle to maintain a constant sample time
      HAL_Delay(PID_SAMPLE_TIME_MS);
    }
  }
}

/**
  * @brief  Initializes the PID controller structure.
  * @param  pid_controller Pointer to the PID_Controller structure.
  * @retval None
  */
void PID_Init(PID_Controller* pid_controller)
{
    pid_controller->Kp = PID_KP_CONSERVATIVE;
    pid_controller->Ki = PID_KI_CONSERVATIVE;
    pid_controller->Kd = PID_KD_CONSERVATIVE;

    pid_controller->out_min = PWM_MIN_DUTY;
    pid_controller->out_max = PWM_MAX_DUTY;

    pid_controller->integral = 0.0f;
    pid_controller->prev_measurement = 0.0f;
    
    pid_controller->sample_time = PID_SAMPLE_TIME_S;
}

/**
  * @brief  Computes the PID output value.
  * @param  pid_controller Pointer to the PID_Controller structure.
  * @param  setpoint The desired target value (e.g., target temperature).
  * @param  measurement The current measured value (e.g., current temperature).
  * @retval The calculated output value, clamped to the defined limits (e.g., PWM duty cycle).
  */
uint16_t PID_Compute(PID_Controller* pid_controller, int16_t setpoint, int16_t measurement)
{
    // Calculate the error
    float error = (float)setpoint - (float)measurement;

    // Proportional term
    float p_out = pid_controller->Kp * error;

    // Integral term (with anti-windup handled below)
    pid_controller->integral += error * pid_controller->sample_time;
    float i_out = pid_controller->Ki * pid_controller->integral;

    // Derivative term
    float derivative = (pid_controller->prev_measurement - (float)measurement) / pid_controller->sample_time;
    float d_out = pid_controller->Kd * derivative;

    // Calculate total output
    float output = p_out + i_out + d_out;

    // --- Anti-windup and Output Clamping ---
    // Clamp the output to its defined limits to prevent excessive values.
    if (output > pid_controller->out_max)
    {
        // If output is maxed out, prevent the integral term from growing infinitely (windup).
        // We adjust the integral term to "undo" the part that caused the saturation.
        pid_controller->integral -= (output - pid_controller->out_max) / pid_controller->Ki;
        output = pid_controller->out_max;
    }
    else if (output < pid_controller->out_min)
    {
        // Same for the lower limit.
        pid_controller->integral -= (output - pid_controller->out_min) / pid_controller->Ki;
        output = pid_controller->out_min;
    }

    // Store the current error for the next iteration's derivative calculation
    pid_controller->prev_measurement = (float)measurement;

    // Return the final constrained output
    return (uint16_t)output;
}

/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Oscillator Configuration */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI; /* Select oscillators HSE,HSI,LSI */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                          /* Enable HSI */
#if defined(RCC_HSIDIV_SUPPORT)
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                          /* HSI not divided */
#endif
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_8MHz;  /* Configure HSI clock as 8MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                         /* Disable HSE */
  /*RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;*/
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                         /* Disable LSI */

  /* Configure oscillators */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Clock source configuration */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* Select clock types HCLK, SYSCLK, PCLK1 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; /* Select HSI as the system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     /* AHB clock not divide */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      /* APB clock not divided */
  /* Configure clock source */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  ADC Initialisation
  * @param  None
  * @retval None
  */
static void APP_AdcInit(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();
  __HAL_RCC_ADC_CLK_ENABLE();

  AdcHandle.Instance = ADC1;
  /* ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)                 
  {
    APP_ErrorHandler();
  }

  AdcHandle.Instance = ADC1;
  AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;              /* Set ADC clock */
  AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;                        /* 12-bit resolution for converted data */
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                        /* Right-alignment for converted data */
  AdcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;              /* Scan sequence direction */
  AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;                     /* Single sampling  */
  AdcHandle.Init.LowPowerAutoWait = ENABLE;                              /* Enable wait for conversion mode */
  AdcHandle.Init.ContinuousConvMode = DISABLE;                           /* Single conversion mode */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                        /* Disable discontinuous mode */
  AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;                  /* Software triggering */
  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;   /* No external trigger edge */
  AdcHandle.Init.DMAContinuousRequests = DISABLE;                        /* isable DMA */
  AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                     /* Overrun handling: overwrite previous value  */
  AdcHandle.Init.SamplingTimeCommon=ADC_SAMPLETIME_239CYCLES_5;          /* Set sampling time */
  /* Initialize ADC */
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)                                
  {
    APP_ErrorHandler();
  }

  sConfig.Channel = ADC_CHANNEL_0;                                       /* ADC channel selection */
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;                                /* Set the rank for the ADC channel order */ 
  /* Configure ADC channels */  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)            
  {
    APP_ErrorHandler();
  }

  sConfig.Channel = ADC_CHANNEL_2;                                       /* ADC channel selection */
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;                                /* Set the rank for the ADC channel order */
  /* Configure ADC channels */  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)           
  {
    APP_ErrorHandler();
  }

  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;                              /* ADC channel selection */
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;                                /* Set the rank for the ADC channel order */
  /* Configure ADC channels */  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)           
  {
    APP_ErrorHandler();
  }

  sConfig.Channel = ADC_CHANNEL_VREFINT;                                 /* ADC channel selection */
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;                                /* Set the rank for the ADC channel order */
  /* Configure ADC channels */  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)           
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  TIM3 Initialisation
  * @param  None
  * @retval None
  */
static void APP_TimInit(void)
{
  TIM_OC_InitTypeDef sConfig = {0};
  GPIO_InitTypeDef  GPIO_InitStruct = {0};
  
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();                             /* Enable GPIO clock */
  
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;                   /* Push-pull output */
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;                     /* Pull-up */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;             /* GPIO speed */
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);                   /* Initialize GPIO */
  
  TimHandle.Instance = TIM3;                                           /* Select TIM3 */
  TimHandle.Init.Period            = PWM_PERIOD_COUNTS - 1;            /* Auto-reload value */
  TimHandle.Init.Prescaler         = TIMER_PRESCALER - 1;              /* Prescaler */
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;           /* No clock division */
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;               /* Up counting */
  TimHandle.Init.RepetitionCounter = 0;                                /* No repetition counting */
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;   /* Auto-reload register not buffered */
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)                         /* Initialize TIM3 */
  {
    APP_ErrorHandler();
  }

  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig.Pulse = 0;
  if (HAL_TIM_OC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_2);
}

/**
  * @brief  Initialize GPIO
  * @param  None
  * @retval None
  */
static void APP_GpioInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();                          /* Enable GPIO clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();                          /* Enable GPIO clock */

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;            /* Push-pull output */
  GPIO_InitStruct.Pull = GPIO_PULLUP;                    /* Pull-up */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;          /* GPIO speed */
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                /* Initialize GPIO */
  
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;            /* Push-pull output */
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);                /* Initialize GPIO */
}

/**
  * @brief  Configure EXTI
  * @param  None
  * @retval None
  */
static void APP_ConfigureExti(void)
{
  /* Configure GPIO pin */
  GPIO_InitTypeDef  GPIO_InitStruct ={0};

  __HAL_RCC_GPIOB_CLK_ENABLE();                  /* Enable GPIOB clock */

  GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;  /* GPIO mode set to falling edge interrupt */
  GPIO_InitStruct.Pull  = GPIO_PULLUP;           /* Pull-up */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* High-speed  */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* Enable EXTI interrupt */
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  
  /* Configure interrupt priority */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
}

static void APP_FlashSetOptionBytes(void)
{
  FLASH_OBProgramInitTypeDef OBInitCfg;

  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();

  OBInitCfg.OptionType = OPTIONBYTE_USER;
  OBInitCfg.USERType = OB_USER_BOR_EN | OB_USER_BOR_LEV | OB_USER_IWDG_SW | OB_USER_WWDG_SW | OB_USER_NRST_MODE | OB_USER_nBOOT1;
  OBInitCfg.USERConfig = OB_BOR_DISABLE | OB_BOR_LEVEL_3p1_3p2 | OB_IWDG_SW | OB_WWDG_SW | OB_RESET_MODE_GPIO | OB_BOOT1_SYSTEM;
  HAL_FLASH_OBProgram(&OBInitCfg);

  HAL_FLASH_Lock();
  HAL_FLASH_OB_Lock();
  HAL_FLASH_OB_Launch();
}

void APP_AdcPoll(void)
{
  uint32_t adc_sum[ADC_CHANNEL_COUNT] = {0};
  
  // Temporarily set PWM to 0 during sampling to reduce noise
  __HAL_TIM_SET_COMPARE(&TimHandle, TIM_CHANNEL_2, 0);
  HAL_Delay(5);

  // --- Start 16x Averaging Loop ---
  for (uint8_t avg_count = 0; avg_count < ADC_AVERAGE_COUNT; avg_count++)
  {
    /* Start ADC */
    HAL_ADC_Start(&AdcHandle);
    
    /* Waiting for ADC conversion */
    for (uint8_t ch = 0; ch < ADC_CHANNEL_COUNT; ch++)
    {
      HAL_ADC_PollForConversion(&AdcHandle, 10000); 
      /* Get ADC value */
      adc_sum[ch] += HAL_ADC_GetValue(&AdcHandle);   
    }
  }

  // --- Calculate Average and Store ---
  for (uint8_t ch = 0; ch < ADC_CHANNEL_COUNT; ch++)
  {
    // Calculate the average (integer division)
    adc_value[ch] = adc_sum[ch] / ADC_AVERAGE_COUNT; 
  }
}

/**
  * @brief  Enter sleep mode.
  * @param  None
  * @retval None
  */
static void APP_EnterSleepMode(void)
{
  if (current_state == IRON_STATE_WORKING)
  {
    current_state = IRON_STATE_SLEEP;
    
    /* TODO: 1. (Turn off the heating element, e.g., PWM stop) */
    // Example: HAL_GPIO_WritePin(HEATER_PORT, HEATER_PIN, GPIO_PIN_RESET);
    
    /* TODO: 2. (Update display, e.g., show "SLEEP" or "Zzz") */
    // Example: Display_SetText("SLEEP");  
  }
}

/**
  * @brief  Exit sleep mode.
  * @param  None
  * @retval None
  */
static void APP_ExitSleepMode(void)
{
  if (current_state == IRON_STATE_SLEEP)
  {
    current_state = IRON_STATE_WORKING;
    
		pid.integral = 0.0f;
		
    /* TODO: 1. (Restore the heating element state/start PWM) */
    // Example: HAL_GPIO_WritePin(HEATER_PORT, HEATER_PIN, GPIO_PIN_SET);
    
    /* TODO: 2. (Restore display to normal temperature/working mode) */
    // Example: Display_SetText("WORK");
    
    // printf("Exiting Sleep Mode...\r\n");
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0) 
  {
    SW = 0;
    last_activity_time = HAL_GetTick();
    if (current_state == IRON_STATE_SLEEP)
    {
      APP_ExitSleepMode();
    }
  }
}

uint16_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  if (in_max == in_min) {
    return out_min;
  }
  
  int32_t input_low = (in_min < in_max) ? in_min : in_max;
  int32_t input_high = (in_min < in_max) ? in_max : in_min;

  if (x < input_low) {
    x = input_low;
  } else if (x > input_high) {
    x = input_high;
  }

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t calculateTemp(uint16_t RawTemp)
{
    if (RawTemp <= temp_calibration_table[1].adc_val) {
        return map(RawTemp,
                   temp_calibration_table[0].adc_val,
                   temp_calibration_table[1].adc_val,
                   temp_calibration_table[0].temp_c,
                   temp_calibration_table[1].temp_c);
    }

    for (size_t i = 1; i < NUM_CALIBRATION_POINTS - 1; ++i) {
        if (RawTemp <= temp_calibration_table[i+1].adc_val) {
            return map(RawTemp,
                       temp_calibration_table[i].adc_val,
                       temp_calibration_table[i+1].adc_val,
                       temp_calibration_table[i].temp_c,
                       temp_calibration_table[i+1].temp_c);
        }
    }

    return map(RawTemp,
               temp_calibration_table[NUM_CALIBRATION_POINTS-2].adc_val,
               temp_calibration_table[NUM_CALIBRATION_POINTS-1].adc_val,
               temp_calibration_table[NUM_CALIBRATION_POINTS-2].temp_c,
               temp_calibration_table[NUM_CALIBRATION_POINTS-1].temp_c);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  /* Infinite loop */
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
    HAL_Delay(250);
  }
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
  /* User can add his own implementation to report the file name and line number,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line)  */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
