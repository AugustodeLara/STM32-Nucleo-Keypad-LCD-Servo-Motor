/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define R1_PORT GPIOB
#define R1_PIN GPIO_PIN_0

#define R2_PORT GPIOB
#define R2_PIN GPIO_PIN_1

#define R3_PORT GPIOB
#define R3_PIN GPIO_PIN_2

#define R4_PORT GPIOB
#define R4_PIN GPIO_PIN_3

#define C1_PORT GPIOB
#define C1_PIN GPIO_PIN_6

#define C2_PORT GPIOB
#define C2_PIN GPIO_PIN_7

#define C3_PORT GPIOB
#define C3_PIN GPIO_PIN_8

#define C4_PORT GPIOB
#define C4_PIN GPIO_PIN_9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);

/*****************************************************************************************************************************/

void LCD_init(void);

void LCD_command(uint8_t value);

void LCD_data(uint8_t value);

void LCD_string(uint8_t * str);

void Keys_Detect(int user);

int Keys_DetectNewUser(void);

/* Global Variable */

uint16_t data_pins[8];

uint16_t access[8];

int newUser[8];

uint16_t controller = 1;

uint16_t enter = 1;

uint16_t newUserControl = 0;

uint8_t tentativas = 0;

int validUser1 = 0;
int validUser2 = 0;
int validUser3 = 0;

int blockKeypad = 0;

uint16_t accessMock[8] = {
  4,
  0,
  8,
  6,
  1,
  2,
  3,
  4
};

uint16_t accessMock2[8] = {};

uint16_t accessMock3[8] = {};

int contadorIdNewUser = 0;

int contador = 1;

int contadorNewUser = 1;

uint8_t messageUserOrPass[30] = "Access ";
uint8_t messageUserPassHave[30] = "Access";

uint8_t messageNewUserNumber[30] = "Create User";
uint8_t messageNewPassNumber[30] = "Create Pass";

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t key;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
  MX_TIM2_Init();
  MX_TIM16_Init();

  data_pins[0] = D0_Pin;
  data_pins[1] = D1_Pin;
  data_pins[2] = D2_Pin;
  data_pins[3] = D3_Pin;
  data_pins[4] = D4_Pin;
  data_pins[5] = D5_Pin;
  data_pins[6] = D6_Pin;
  data_pins[7] = D7_Pin;

  /* Initialize LCD */

  LCD_init();
  contadorNewUser = 1;
  validUser1 = 0;
  validUser2 = 0;
  validUser3 = 0;

  HAL_Delay(1000);

  LCD_string((uint8_t * )
    "T2 -- LAB.PROCESS");

  LCD_command(0x38); // Set 8 bit mode
  LCD_command(0XC0); // Moving the cursor to second line
  LCD_string((uint8_t * )
    "AUGUSTO");

  showWelcome();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start( & htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //checkPass();

    if (tentativas == 2) {
      LCD_command(0x01);
      LCD_string((uint8_t * )
        "BLOQUEADO");
      LCD_command(0x38); // Set 8 bit mode
      LCD_command(0XC0); // Moving the cursor to second line
      LCD_string((uint8_t * )
        "********************");
      blockKeypad = 1;
      HAL_Delay(6000);
      blockKeypad = 0;
      LCD_command(0x01);
      tentativas = 0;
      showWelcome();

    } else {
      if (newUserControl == 1) {
        if (blockKeypad == 0) {
          Keys_Detect(1); //1 is type new user

        }

      } else {

        if (blockKeypad == 0) {
          Keys_Detect(0); //0 is type user
        }
      }

      //check controller cancela
      if (controller == 0) {
        LCD_command(0x01);
        LCD_string((uint8_t * )
          "ABRIR CANCELA");
        LCD_command(0x38); // Set 8 bit mode
        LCD_command(0XC0); // Moving the cursor to second line
        LCD_string((uint8_t * )
          "Aguarda 5 segs");
        //bloqueia Keypad
        blockKeypad = 1;
        //run servo
        moveServoSobe();
        HAL_Delay(5000);
        moveServoDesce();
        //libera Keypad
        blockKeypad = 0;
        LCD_command(0x01);
        controller = 1;
        showWelcome();
      }

    }

  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {
    0
  };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {
    0
  };
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {
    0
  };

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig( & RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig( & RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM16;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig( & PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {
    0
  };
  TIM_MasterConfigTypeDef sMasterConfig = {
    0
  };

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init( & htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource( & htim2, & sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization( & htim2, & sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {
    0
  };
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {
    0
  };

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 72 - 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init( & htim16) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init( & htim16) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel( & htim16, & sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime( & htim16, & sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit( & htim16);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {
    0
  };

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D0_Pin | D1_Pin | D4_Pin | D5_Pin |
    D6_Pin | D7_Pin | RS_Pin | RW_Pin |
    EN_Pin | D2_Pin | D3_Pin | GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R1_Pin | R2_Pin | R3_Pin | R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D0_Pin D1_Pin D4_Pin D5_Pin
                           D6_Pin D7_Pin RS_Pin RW_Pin
                           EN_Pin D2_Pin D3_Pin PA15 */
  GPIO_InitStruct.Pin = D0_Pin | D1_Pin | D4_Pin | D5_Pin |
    D6_Pin | D7_Pin | RS_Pin | RW_Pin |
    EN_Pin | D2_Pin | D3_Pin | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, & GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R1_Pin | R2_Pin | R3_Pin | R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, & GPIO_InitStruct);

  /*Configure GPIO pins : C1_Pin C2_Pin C3_Pin C4_Pin */
  GPIO_InitStruct.Pin = C1_Pin | C2_Pin | C3_Pin | C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, & GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*****************************************************************************************************************************/
void Keys_Detect(int user) {

  /* Setting Row 1 ad High and others LOW */
  HAL_GPIO_WritePin(GPIOB, R1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, R2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, R3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, R4_Pin, GPIO_PIN_RESET);

  if (HAL_GPIO_ReadPin(GPIOB, C1_Pin) == GPIO_PIN_SET) {
    /* 1 */
    if (user == 0) {
      displayMapDigit(contador, messageUserOrPass, "UTL ");
      displayAnimation(contador);
      HAL_Delay(200);
      access[contador] = 1;
      contador = contador + 1;
    } else {
      displayMapDigit(contadorNewUser, messageUserOrPass, "CONF ");
      displayAnimation(contadorNewUser);
      HAL_Delay(200);
      newUser[contadorNewUser] = 1;
      contadorNewUser++;
    }

    while (HAL_GPIO_ReadPin(GPIOB, C1_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C2_Pin) == GPIO_PIN_SET) {
    /* 2 */
    if (user == 0) {
      displayMapDigit(contador, messageUserOrPass, "UTL ");
      displayAnimation(contador);
      HAL_Delay(200);
      access[contador] = 2;
      contador++;
    } else {
      displayMapDigit(contadorNewUser, messageUserOrPass, "CONF ");
      displayAnimation(contadorNewUser);
      HAL_Delay(200);
      newUser[contadorNewUser] = 2;
      contadorNewUser++;
    }

    while (HAL_GPIO_ReadPin(GPIOB, C2_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C3_Pin) == GPIO_PIN_SET) {
    /* 3 */
    if (user == 0) {

      displayMapDigit(contador, messageUserOrPass, "UTL ");
      displayAnimation(contador);
      HAL_Delay(200);
      access[contador] = 3;
      contador++;
    } else {
      displayMapDigit(contadorNewUser, messageUserOrPass, "CONF ");
      displayAnimation(contadorNewUser);
      HAL_Delay(200);
      newUser[contadorNewUser] = 3;
      contadorNewUser++;
    }
    while (HAL_GPIO_ReadPin(GPIOB, C3_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C4_Pin) == GPIO_PIN_SET) {
    /* A */
    LCD_command(0x01);
    LCD_string((uint8_t * )
      "ACESSANDO");
    LCD_command(0x38); // Set 8 bit mode
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "*******************");
    HAL_Delay(800);
    enter = 0;
    SavePassNewUser();
    checkPass();

    while (HAL_GPIO_ReadPin(GPIOB, C4_Pin));
  }

  /* Setting Row 2 ad High and others LOW */
  HAL_GPIO_WritePin(GPIOB, R1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, R2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, R3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, R4_Pin, GPIO_PIN_RESET);

  if (HAL_GPIO_ReadPin(GPIOB, C1_Pin) == GPIO_PIN_SET) {
    /* 4 */
    if (user == 0) {
      displayMapDigit(contador, messageUserOrPass, "UTL ");
      displayAnimation(contador);
      HAL_Delay(200);
      access[contador] = 4;
      contador++;
    } else {
      displayMapDigit(contadorNewUser, messageUserOrPass, "CONF ");
      displayAnimation(contadorNewUser);
      HAL_Delay(200);

      newUser[contadorNewUser] = 4;
      contadorNewUser++;
    }
    while (HAL_GPIO_ReadPin(GPIOB, C1_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C2_Pin) == GPIO_PIN_SET) {
    /* 5 */
    if (user == 0) {
      displayMapDigit(contador, messageUserOrPass, "UTL ");
      displayAnimation(contador);
      HAL_Delay(200);
      access[contador] = 5;
      contador++;
    } else {
      displayMapDigit(contadorNewUser, messageUserOrPass, "CONF ");
      displayAnimation(contadorNewUser);
      HAL_Delay(200);
      newUser[contadorNewUser] = 5;
      contadorNewUser++;
    }
    while (HAL_GPIO_ReadPin(GPIOB, C2_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C3_Pin) == GPIO_PIN_SET) {
    /* 6 */
    if (user == 0) {
      displayMapDigit(contador, messageUserOrPass, "UTL ");
      displayAnimation(contador);
      HAL_Delay(200);
      access[contador] = 6;
      contador++;
    } else {
      displayMapDigit(contadorNewUser, messageUserOrPass, "CONF ");
      displayAnimation(contadorNewUser);
      HAL_Delay(200);
      newUser[contadorNewUser] = 6;
      contadorNewUser++;
    }
    while (HAL_GPIO_ReadPin(GPIOB, C3_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C4_Pin) == GPIO_PIN_SET) {
    /* B */
    LCD_command(0x01);
    LCD_string((uint8_t * )
      "Tecla detectada");

    LCD_command(0x38); // Set 8 bit mode
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "Key B");
    newUserControl = 1;
    modeConfig();
    while (HAL_GPIO_ReadPin(GPIOB, C4_Pin));
  }

  /* Setting Row 3 ad High and others LOW */
  HAL_GPIO_WritePin(GPIOB, R1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, R2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, R3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, R4_Pin, GPIO_PIN_RESET);

  if (HAL_GPIO_ReadPin(GPIOB, C1_Pin) == GPIO_PIN_SET) {
    /* 7 */
    if (user == 0) {
      displayMapDigit(contador, messageUserOrPass, "UTL ");
      displayAnimation(contador);
      HAL_Delay(200);
      access[contador] = 7;
      contador++;
    } else {
      displayMapDigit(contadorNewUser, messageUserOrPass, "CONF ");
      displayAnimation(contadorNewUser);
      HAL_Delay(200);
      newUser[contadorNewUser] = 7;
      contadorNewUser++;
    }
    while (HAL_GPIO_ReadPin(GPIOB, C1_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C2_Pin) == GPIO_PIN_SET) {
    /* 8 */
    if (user == 0) {
      displayMapDigit(contador, messageUserOrPass, "UTL ");
      displayAnimation(contador);
      HAL_Delay(200);
      access[contador] = 8;
      contador++;
    } else {
      displayMapDigit(contadorNewUser, messageUserOrPass, "CONF ");
      displayAnimation(contadorNewUser);
      HAL_Delay(200);
      newUser[contadorNewUser] = 8;
      contadorNewUser++;
    }
    while (HAL_GPIO_ReadPin(GPIOB, C2_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C3_Pin) == GPIO_PIN_SET) {
    /* 9 */
    if (user == 0) {
      displayMapDigit(contador, messageUserOrPass, "UTL ");
      displayAnimation(contador);
      HAL_Delay(200);
      access[contador] = 9;
      contador++;
    } else {
      displayMapDigit(contadorNewUser, messageUserOrPass, "CONF ");
      displayAnimation(contadorNewUser);
      HAL_Delay(200);
      contadorNewUser++;
    }
    while (HAL_GPIO_ReadPin(GPIOB, C3_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C4_Pin) == GPIO_PIN_SET) {
    /* # */
    LCD_command(0x01);
    LCD_string((uint8_t * )
      "Tecla detectada");

    LCD_command(0x38); // Set 8 bit mode
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "Key C");
    while (HAL_GPIO_ReadPin(GPIOB, C4_Pin));
  }

  /* Setting Row 4 ad High and others LOW */
  HAL_GPIO_WritePin(GPIOB, R1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, R2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, R3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, R4_Pin, GPIO_PIN_SET);

  if (HAL_GPIO_ReadPin(GPIOB, C1_Pin) == GPIO_PIN_SET) {
    /* * */
    LCD_command(0x01);
    LCD_string((uint8_t * )
      "Tecla detectada");

    LCD_command(0x38); // Set 8 bit mode
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "Key *");
    while (HAL_GPIO_ReadPin(GPIOB, C1_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C2_Pin) == GPIO_PIN_SET) {
    /* 0 */
    if (user == 0) {
      displayMapDigit(contador, messageUserOrPass, "UTL ");
      displayAnimation(contador);
      HAL_Delay(200);
      access[contador] = 0;
      contador++;
    } else {
      displayMapDigit(contadorNewUser, messageUserOrPass, "CONF ");
      displayAnimation(contadorNewUser);
      HAL_Delay(200);
      newUser[contadorNewUser] = 0;
      contadorNewUser++;
    }
    while (HAL_GPIO_ReadPin(GPIOB, C2_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C3_Pin) == GPIO_PIN_SET) {
    /* # */
    LCD_command(0x01);
    LCD_string((uint8_t * )
      "Tecla detectada");

    LCD_command(0x38); // Set 8 bit mode
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "Key #");
    while (HAL_GPIO_ReadPin(GPIOB, C3_Pin));
  }

  if (HAL_GPIO_ReadPin(GPIOB, C4_Pin) == GPIO_PIN_SET) {
    /* D */
    LCD_command(0x01);
    LCD_string((uint8_t * )
      "RESET SYSTEM");

    LCD_command(0x38); // Set 8 bit mode
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "*********");

    newUserControl = 0;
    cleanUsers();
    showWelcome();

    while (HAL_GPIO_ReadPin(GPIOB, C4_Pin));

  }

}

void displayAnimation(int opcao) {
  switch (opcao) {
  case 1:
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "X: ");
    break;
  case 2:
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "XX: ");
    break;
  case 3:
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "XXX: ");
    break;
  case 4:
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "XXXX: ");
    break;
  case 5:
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "XXXXX: ");
    break;
  case 6:
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "XXXXXX: ");
    break;
  case 7:
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "XXXXXXX: ");
    break;
  case 8:
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "XXXXXXXX: ");
    break;
  default:
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "APERTE ENTER: ");

  }

}

void displayMapDigit(int opcao, uint8_t * message, uint8_t * type) {

  switch (opcao) {
  case 1:
    LCD_command(0x01);
    //strcat(message, " User");
    LCD_string(message);
    LCD_string(type);
    LCD_string((uint8_t * )
      "usr");
    LCD_command(0x38); // Set 8 bit mode
    break;
  case 2:
    LCD_command(0x01);
    LCD_string(message);
    LCD_string(type);
    LCD_string((uint8_t * )
      "usr");
    LCD_command(0x38); // Set 8 bit mode
    break;
  case 3:
    LCD_command(0x01);
    LCD_string(message);
    LCD_string(type);
    LCD_string((uint8_t * )
      "usr");
    LCD_command(0x38); // Set 8 bit mode
    break;
  case 4:
    LCD_command(0x01);
    LCD_string(message);
    LCD_string(type);
    LCD_string((uint8_t * )
      "usr");
    LCD_command(0x38); // Set 8 bit mode
    break;
  case 5:
    LCD_command(0x01);
    LCD_string(message);
    LCD_string(type);
    LCD_string((uint8_t * )
      "pass");
    LCD_command(0x38); // Set 8 bit mode
    break;
  case 6:
    LCD_command(0x01);
    LCD_string(message);
    LCD_string(type);
    LCD_string((uint8_t * )
      "pass");
    LCD_command(0x38); // Set 8 bit mode
    break;
  case 7:
    LCD_command(0x01);
    LCD_string(message);
    LCD_string(type);
    LCD_string((uint8_t * )
      "pass");
    LCD_command(0x38); // Set 8 bit mode
    break;
  case 8:
    LCD_command(0x01);
    LCD_string(message);
    LCD_string(type);
    LCD_string((uint8_t * )
      "pass");
    LCD_command(0x38); // Set 8 bit mode


    HAL_Delay(500);
            LCD_command(0x01); // Moving the cursor to second line
            LCD_string((uint8_t * )
                  "APERTE ENTER: ");


    break;
  case 9:
    LCD_command(0x01);
    LCD_string(message);
    LCD_string(type);
    LCD_string((uint8_t * )
      "pass");
    LCD_command(0x38); // Set 8 bit mode
    break;
  default:
    LCD_command(0x01);
    LCD_string((uint8_t * )
      "TECLAS EXCEDIDAS");
    LCD_command(0x38); // Set 8 bit mode

  }

}

//Refactory da função

void modeConfig(void) {

  LCD_command(0x01);
  LCD_string((uint8_t * )
    "MODO CONFIGURACAO");
  LCD_command(0x38); // Set 8 bit mode
  LCD_command(0XC0); // Moving the cursor to second line
  LCD_string((uint8_t * )
    "***********************");
  HAL_Delay(1000);
  LCD_command(0x01);
  showWelcome();

}

//this function clean variable global hardcode
void cleanUsers(void) {

  memset(accessMock2, 0, 8);
  memset(accessMock3, 0, 8);
  memset(newUser, 0, 8);
  memset(access, 0, 8);
  contadorIdNewUser = 0;
  contador = 1;
  contadorNewUser = 0;
  validUser1 = 0;
  validUser2 = 0;
  validUser3 = 0;
  blockKeypad = 0;
  controller = 1;
  enter = 1;
  newUserControl = 0;
  tentativas = 0;
}

void checkPass(void) {
  if ((contador == 9 || contadorNewUser == 9) && enter == 0) {
    for (int i = 0; i < 8; i++) {
      if (accessMock[i] == access[i + 1]) {
        validUser1++;
      }
      if (accessMock2[i] == access[i + 1]) {
        validUser2++;
      }
      if (accessMock3[i] == access[i + 1]) {
        validUser3++;
      }
    }
    if (validUser1 == 8 || validUser2 == 8 || validUser3 == 8) {
      LCD_command(0x01);
      LCD_string((uint8_t * )
        "Acesso Liberado");
      HAL_Delay(1000);
      LCD_command(0x38); // Set 8 bit mode
      LCD_command(0XC0); // Moving the cursor to second line
      LCD_string((uint8_t * )
        "*************");
      controller = 0;
      contador = 1;
      contadorNewUser = 1;
      validUser1 = 0;
      validUser2 = 0;
      validUser3 = 0;
    } else {
      LCD_command(0x01);
      LCD_string((uint8_t * )
        "Sem acesso");
      HAL_Delay(1000);
      contador = 1;
      contadorNewUser = 1;
      validUser1 = 0;
      validUser2 = 0;
      validUser3 = 0;
      tentativas++;
      showWelcome();
    }
  }
}

void SavePassNewUser(void) {
  if (contadorNewUser == 9 && enter == 0) {
    for (int i = 0; i < 8; i++) {
      if (contadorIdNewUser == 0) {
        accessMock2[i] = newUser[i+1];
      }
      if (contadorIdNewUser == 1) {
        accessMock3[i] = newUser[i+1];
      }
    }
    contadorIdNewUser++;
    LCD_command(0x01);
    LCD_string((uint8_t * )
      "SAVE USER");
    LCD_command(0x38); // Set 8 bit mode
    LCD_command(0XC0); // Moving the cursor to second line
    LCD_string((uint8_t * )
      "*************");
    contadorNewUser = 1;
    newUserControl = 0;
    HAL_Delay(2000);
    showWelcome();

  }
}

void showWelcome(void) {
  LCD_command(0x01);
  LCD_string((uint8_t * )
    "DIGITE ACESSO");
  LCD_command(0x38); // Set 8 bit mode
  LCD_command(0XC0); // Moving the cursor to second line
  LCD_string((uint8_t * )
    "****************");
}

void moveServoSobe(void) {
  for (int i = 0; i < 12; i++) {
    GPIOA -> BSRR = (1 << 15);
    HAL_Delay(2);
    GPIOA -> BRR = (1 << 15);
    HAL_Delay(200);
  }
}

void moveServoDesce(void) {
  for (int i = 0; i < 4; i++) {
    GPIOA -> BSRR = (1 << 15);
    HAL_Delay(1);
    GPIOA -> BRR = (1 << 15);
    HAL_Delay(100);
  }
}

/*****************************************************************************************************************************/
/* Initialization of LCD in 8-bit mode */

void LCD_init(void) {
  HAL_Delay(100); // Give LCD time to intialize
  LCD_command(0x38); // Setting 8-bit mode
  LCD_command(0x38);
  LCD_command(0x38);

  LCD_command(0x0C); // Display ON and Cursor OFF

  LCD_command(0x06); // Auto Increment Cursor

  LCD_command(0x01); // Clear Display

  LCD_command(0x80); // Cursor at home position
}

/*****************************************************************************************************************************/
/* Command mode operations */

void LCD_command(uint8_t value) {
  HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);

  // witing to data to D0, D1, D2, D3, D4, D5, D6, D7

  for (int i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(GPIOA, data_pins[i], ((value >> i) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  // Pulse the Enable pin

  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1); // enable pulse must be >450ns
  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1); // commands need > 37us to settle

}
/*****************************************************************************************************************************/
/* Data mode operations */

void LCD_data(uint8_t value) {
  HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);

  for (int i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(GPIOA, data_pins[i], ((value >> i) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  // Pulse the Enable pin

  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1); // enable pulse must be >450ns
  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1); // commands need > 37us to settle
}
/*****************************************************************************************************************************/

/* Printing the string */

void LCD_string(uint8_t * str) {
  int i = 0;

  while (str[i] != '\0') {
    LCD_data(str[i]);
    i++;
  }

}

/*****************************************************************************************************************************/

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {}
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
void assert_failed(uint8_t * file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
