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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim4;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DC_MOTOR1            0
#define MIN_SPEED            0
#define MAX_SPEED            0XFFFF

#define HCSR04_SENSOR1       0
#define THRESHOLD_DISTANCE   50.0

#define BUFFER_SIZE          256

#define PASSWORD             "AB1"

#define GPS_CODE             "G"
#define ALARM_ON             'A'
#define ALARM_OFF            'S'
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t TRIG_Ticks = 0;

uint8_t RX_Data = 0;

char GGA[100];
char RMC[100];

GPSSTRUCT gpsData;

int flagGGA = 0, flagRMC = 0;
char lcdBuffer [50];

int VCCTimeout = 5000; // GGA or RMC will not be received if the VCC is not sufficient

char sentenceBuffer[BUFFER_SIZE];
uint8_t sentenceIndex = 0;
bool Is_Num = false;
uint8_t braillePattern = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Sys_Init(void);
void Display_Handler(void);
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
  Sys_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  Check_Password();  //Stuck here until entering correct password

  UART_SendString(&huart1, "UART is ready for sending & receiving...\r\n");
  HAL_Delay(50);

  Ringbuf_init();
  HAL_Delay (100);

  /*Local variables begin*/
  float Distance1 = 0.0;
  /*Local variables end*/

  UART_Receiving_IT_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	Distance1 = HCSR04_Read(HCSR04_SENSOR1);

	Ultraonic_Response(Distance1);

	PIR_Response();

	Send_Braille();
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
static void Sys_Init(void)
{
	HCSR04_Init(HCSR04_SENSOR1, &htim4);
    DC_MOTOR_Init(DC_MOTOR1);
    DC_MOTOR_Start(DC_MOTOR1, DIR_CCW, MIN_SPEED);
    Buzzer_Init();
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	HCSR04_TMR_IC_ISR(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	HCSR04_TMR_OVF_ISR(htim);
}

void SysTick_CallBack(void)
{
	TRIG_Ticks++;

    if(TRIG_Ticks >= 10) // Each 10msec
    {
    	HCSR04_Trigger(HCSR04_SENSOR1);
    	TRIG_Ticks = 0;
    }
}


void PIR_Response(void){
	if (HAL_GPIO_ReadPin(PIR_GPIO_Port, PIR_Pin) == GPIO_PIN_SET) {
		DC_MOTOR_Start(DC_MOTOR1, DIR_CCW, MAX_SPEED);
	} else{
		DC_MOTOR_Stop(DC_MOTOR1);
	}
}

void Ultraonic_Response(float distance1){
	if(distance1 < THRESHOLD_DISTANCE){
		DC_MOTOR_Start(DC_MOTOR1, DIR_CCW, MAX_SPEED);
		Buzzer_ON();
	} else{
		DC_MOTOR_Stop(DC_MOTOR1);
		Buzzer_OFF();
	}
}


void Read_Buttons(void) {
    if (HAL_GPIO_ReadPin(But1_GPIO_Port, But1_Pin) == GPIO_PIN_SET) {
    	braillePattern |= (1 << 5);
    	while (HAL_GPIO_ReadPin(But1_GPIO_Port, But1_Pin) == GPIO_PIN_SET);
    }
    if (HAL_GPIO_ReadPin(But2_GPIO_Port, But2_Pin) == GPIO_PIN_SET) {
    	braillePattern |= (1 << 4);
    	while (HAL_GPIO_ReadPin(But1_GPIO_Port, But2_Pin) == GPIO_PIN_SET);
    }
    if (HAL_GPIO_ReadPin(But3_GPIO_Port, But3_Pin) == GPIO_PIN_SET) {
    	braillePattern |= (1 << 3);
    	while (HAL_GPIO_ReadPin(But1_GPIO_Port, But3_Pin) == GPIO_PIN_SET);
    }
    if (HAL_GPIO_ReadPin(But4_GPIO_Port, But4_Pin) == GPIO_PIN_SET) {
    	braillePattern |= (1 << 2);
    	while (HAL_GPIO_ReadPin(But1_GPIO_Port, But4_Pin) == GPIO_PIN_SET);
    }
    if (HAL_GPIO_ReadPin(But5_GPIO_Port, But5_Pin) == GPIO_PIN_SET) {
    	braillePattern |= (1 << 1);
    	while (HAL_GPIO_ReadPin(But1_GPIO_Port, But5_Pin) == GPIO_PIN_SET);
    }
    if (HAL_GPIO_ReadPin(But6_GPIO_Port, But6_Pin) == GPIO_PIN_SET) {
    	braillePattern |= (1 << 0);
    	while (HAL_GPIO_ReadPin(But1_GPIO_Port, But6_Pin) == GPIO_PIN_SET);
    }
}

char Translate_Braille(uint8_t braillePattern, bool *isNumber) {
	if (braillePattern == 0b001111){
		*isNumber = true;
		return '\0';
	}

	if (*isNumber){
		*isNumber = false;
		switch (braillePattern) {
			case 0b100000: return '1';
			case 0b110000: return '2';
			case 0b100100: return '3';
			case 0b100110: return '4';
			case 0b100010: return '5';
			case 0b110100: return '6';
			case 0b110110: return '7';
			case 0b110010: return '8';
			case 0b010100: return '9';
			case 0b010110: return '0';
			default: return '?';
		}
	} else{
		switch (braillePattern) {
			case 0b000000: return ' ';
			case 0b100000: return 'A';
			case 0b110000: return 'B';
			case 0b100100: return 'C';
			case 0b100110: return 'D';
			case 0b100010: return 'E';
			case 0b110100: return 'F';
			case 0b110110: return 'G';
			case 0b110010: return 'H';
			case 0b010100: return 'I';
			case 0b010110: return 'J';
			case 0b101000: return 'K';
			case 0b111000: return 'L';
			case 0b101100: return 'M';
			case 0b101110: return 'N';
			case 0b101010: return 'O';
			case 0b111100: return 'P';
			case 0b111110: return 'Q';
			case 0b111010: return 'R';
			case 0b011100: return 'S';
			case 0b011110: return 'T';
			case 0b101001: return 'U';
			case 0b111001: return 'V';
			case 0b010111: return 'W';
			case 0b101101: return 'X';
			case 0b101111: return 'Y';
			case 0b101011: return 'Z';
			case 0b010011: return '.';
			case 0b010000: return ',';
			default: return '?';
		}
	}
}

void Store_Character(void) {
	char currentChar = Translate_Braille(braillePattern, &Is_Num);
    if ((currentChar != '\0') && (sentenceIndex < sizeof(sentenceBuffer) - 1)) {
        sentenceBuffer[sentenceIndex++] = currentChar;
        sentenceBuffer[sentenceIndex] = '\0';  // Null-terminate the string
    }
    braillePattern = 0;
}

void Send_Sentence(void) {
    // Implement sending the sentence via UART
	if((strcmp(sentenceBuffer, GPS_CODE)) == 0){
		Send_GPS_Data();
	} else{
		HAL_UART_Transmit(&huart1, (uint8_t *)sentenceBuffer, sentenceIndex, HAL_MAX_DELAY);
	}

    sentenceIndex = 0;  // Reset the buffer index for the next sentence
    sentenceBuffer[0] = '\0';  // Clear the buffer
}

void Send_Braille(void){
	Read_Buttons();
	// Check if store char button is pressed (PB4)
	if (HAL_GPIO_ReadPin(Store_But_GPIO_Port, Store_But_Pin) == GPIO_PIN_SET) {
		Store_Character();
		//braillePattern = 0;  // Reset the pattern for next input
		while (HAL_GPIO_ReadPin(Store_But_GPIO_Port, Store_But_Pin) == GPIO_PIN_SET);  // Wait for button release
	}

	// Check if send button is pressed (PB2)
	if (HAL_GPIO_ReadPin(Send_But_GPIO_Port, Send_But_Pin) == GPIO_PIN_SET) {
		Send_Sentence();
		while (HAL_GPIO_ReadPin(Send_But_GPIO_Port, Send_But_Pin) == GPIO_PIN_SET);  // Wait for button release
	}
}


void Check_Password(void){
	while(1){
		Read_Buttons();
		// Check if store char button is pressed (PB4)
		if (HAL_GPIO_ReadPin(Store_But_GPIO_Port, Store_But_Pin) == GPIO_PIN_SET) {
			Store_Character();
			//braillePattern = 0;  // Reset the pattern for next input
			while (HAL_GPIO_ReadPin(Store_But_GPIO_Port, Store_But_Pin) == GPIO_PIN_SET);  // Wait for button release
		}

		// Check if send button is pressed (PB2)
		if (HAL_GPIO_ReadPin(Send_But_GPIO_Port, Send_But_Pin) == GPIO_PIN_SET) {
			if(strcmp(sentenceBuffer, PASSWORD) == 0){
				UART_SendString(&huart1, "Welcome... \r\n");
				sentenceIndex = 0;  // Reset the buffer index for the next sentence
				sentenceBuffer[0] = '\0';  // Clear the buffer
				break;
			} else{
				UART_SendString(&huart1, "Wrong password..!! \r\n");
				UART_SendString(&huart1, "Please try again \r\n");
				sentenceIndex = 0;  // Reset the buffer index for the next sentence
				sentenceBuffer[0] = '\0';  // Clear the buffer
			}

			while (HAL_GPIO_ReadPin(Send_But_GPIO_Port, Send_But_Pin) == GPIO_PIN_SET);  // Wait for button release
		}
	}
}


void Send_GPS_Data(void){
	uint32_t Start_Time = HAL_GetTick();
	while((HAL_GetTick() - Start_Time) < 200){
		if (Wait_for("GGA") == 1) {

			VCCTimeout = 5000; // Reset the VCC Timeout indicating the GGA is being received
			Copy_upto("*", GGA);
			if (decodeGGA(GGA, &gpsData.ggastruct) == 0)
				flagGGA = 2;  // 2 indicates the data is valid
			else
				flagGGA = 1;  // 1 indicates the data is invalid
		}

		if (Wait_for("RMC") == 1) {

			VCCTimeout = 5000; // Reset the VCC Timeout indicating the RMC is being received
			Copy_upto("*", RMC);
			if (decodeRMC(RMC, &gpsData.rmcstruct) == 0)
				flagRMC = 2;  // 2 indicates the data is valid
			else
				flagRMC = 1;  // 1 indicates the data is invalid
		}

		if ((flagGGA == 2) | (flagRMC == 2)) {
			sprintf(lcdBuffer, "%02d:%02d:%02d, %02d%02d%02d",
					gpsData.ggastruct.tim.hour, gpsData.ggastruct.tim.min,
					gpsData.ggastruct.tim.sec, gpsData.rmcstruct.date.Day,
					gpsData.rmcstruct.date.Mon, gpsData.rmcstruct.date.Yr);
			UART_SendString(&huart2, lcdBuffer);
			UART_SendString(&huart2, "\r\n");
			memset(lcdBuffer, '\0', 50);
			sprintf(lcdBuffer, "%.2f%c, %.2f%c  ",
					gpsData.ggastruct.lcation.latitude,
					gpsData.ggastruct.lcation.NS,
					gpsData.ggastruct.lcation.longitude,
					gpsData.ggastruct.lcation.EW);
			UART_SendString(&huart2, lcdBuffer);
		}

		else if ((flagGGA == 1) | (flagRMC == 1)) {
			UART_SendString(&huart2, "NO FIX YET \r\n");
			UART_SendString(&huart2, "Please wait \r\n");
		}

		if (VCCTimeout <= 0) {
			VCCTimeout = 5000;  // Reset the timeout

			//reset flags
			flagGGA = flagRMC = 0;

			// You are here means the VCC is less, or maybe there is some connection issue
			// Check the VCC, also you can try connecting to the external 5V
			UART_SendString(&huart2, "VCC Issue \r\n");
			UART_SendString(&huart2, "Check Connection \r\n");
		}
	}
}


void UART_Receiving_IT_Init(void){
	 if (HAL_UART_Receive_IT(&huart1, &RX_Data, 1) != HAL_OK) {
		  Error_Handler();
	  }
}

//Function to send string over UART(Blocking mode)
void UART_SendString(UART_HandleTypeDef *huart, char *string){
	uint16_t length = 0;
	while(string[length] != '\0'){
		length++;
	}

	HAL_UART_Transmit(huart, (uint8_t *)string, length, HAL_MAX_DELAY);
}

//Function to send float over UART(Blocking mode)
void UART_SendFloat(UART_HandleTypeDef *huart, float num){
    char buffer[20];               // Buffer to hold the string representation of the float
    sprintf(buffer, "%.2f", num);  // Convert float to string with 2 decimal places
    UART_SendString(huart, buffer);       // Send the string via UART
}

//Auto Called when receiving a new data over UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1) {
		HAL_UART_Receive_IT(&huart1, &RX_Data, 1);
		if (ALARM_ON == RX_Data){
			DC_MOTOR_Start(DC_MOTOR1, DIR_CCW, MAX_SPEED);
			Buzzer_ON();

			DELAY_MS(100);
			DC_MOTOR_Stop(DC_MOTOR1);
			Buzzer_OFF();
		}
    }

	if (huart->Instance == USART2) {
		//HAL_UART_Receive_IT(&huart2, &RX_Data, 1);
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
