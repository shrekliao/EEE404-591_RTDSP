/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// image size 96 x 128 = 12288
#define BUFFER_SIZE 8 // reduced from 12288 to 8 for debugging

// input buffer
uint8_t rcvd_data[BUFFER_SIZE];

// output buffer
// for debugging of thresholding, quantization, and gray level transformations
uint8_t trmt_data[BUFFER_SIZE] = {122, 155, 80, 254, 7, 40, 240, 50};

// for debugging of histogram equalization
//uint8_t trmt_data[BUFFER_SIZE] = {1, 1, 3, 5, 6, 7, 7, 7};

// segmentation/thresholding constants
#define THRESHOLD 80
#define THRESHOLD1 80
#define THRESHOLD2 240

// gray level transformation constants
#define A 120
#define B 50

#define Quantization_Shift 1

// histogram equalization constant and variables
#define NUM_GRAY_LEVEL 8 // reduced from 256 to 8 for histogram equalization debugging

uint32_t hist_cnt[NUM_GRAY_LEVEL] = {0};  // store counts
uint8_t mapped_levels[NUM_GRAY_LEVEL];


// 1. segmentation/thresholding functions
void global_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold);
void band_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold1, uint8_t threshold2);
void semi_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold);

void global_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold);
void band_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold1, uint8_t threshold2);
void semi_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold);

// 2. gray level quantization functions
void gray_level_quantization_c(uint8_t *x, uint32_t size, uint8_t shift_factor);
void gray_level_quantization_hybrid(uint8_t *x, uint32_t size, uint8_t shift_factor);

// 3. gray level transformation functions
void gray_level_transformation1_c(uint8_t *x, uint32_t size);
void gray_level_transformation2_c(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);
void gray_level_transformation3_c(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);

void gray_level_transformation1_hybrid(uint8_t *x, uint32_t size);
void gray_level_transformation2_hybrid(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);
void gray_level_transformation3_hybrid(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);

// 4. histogram equalization functions
void calculate_histogram_c(uint8_t *x, uint32_t *hist, uint32_t size);
void map_levels_c(uint32_t *hist, uint8_t *mapping_table, uint32_t size, uint16_t levels);
void transform_image_c(uint8_t *x, uint8_t *mapping_table, uint32_t size);

void calculate_histogram_hybrid(uint8_t *x, uint32_t *hist, uint32_t size);
void map_levels_hybrid(uint32_t *hist, uint8_t *mapping_table, uint32_t size, uint16_t levels);
void transform_image_hybrid(uint8_t *x, uint8_t *mapping_table, uint32_t size);


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
  /* USER CODE BEGIN 2 */

  // test your image processing functions here
  //1.
  //global_thresholding_c(trmt_data, BUFFER_SIZE, THRESHOLD);
  //global_thresholding_hybrid(trmt_data, BUFFER_SIZE, THRESHOLD);
  //band_thresholding_c(trmt_data, BUFFER_SIZE, THRESHOLD1, THRESHOLD2);
  //band_thresholding_hybrid(trmt_data, BUFFER_SIZE, THRESHOLD1, THRESHOLD2);
  //semi_thresholding_c(trmt_data, BUFFER_SIZE, THRESHOLD);
  //semi_thresholding_hybrid(trmt_data, BUFFER_SIZE, THRESHOLD);
  //2.
  //gray_level_quantization_c(trmt_data, BUFFER_SIZE,Quantization_Shift);
  //gray_level_quantization_hybrid(trmt_data, BUFFER_SIZE,1);
  //3.
  //gray_level_transformation1_c(trmt_data, BUFFER_SIZE);
  //gray_level_transformation2_c(trmt_data, BUFFER_SIZE, A, B);
  //gray_level_transformation3_c(trmt_data, BUFFER_SIZE, A, B);
  //gray_level_transformation1_hybrid(trmt_data, BUFFER_SIZE);
  gray_level_transformation2_hybrid(trmt_data, BUFFER_SIZE, A, B);
  //gray_level_transformation3_hybrid(trmt_data, BUFFER_SIZE, A, B);
  //4.
  //calculate_histogram_c(trmt_data, hist_cnt, BUFFER_SIZE);
  //map_levels_c(hist_cnt, mapped_levels, BUFFER_SIZE, NUM_GRAY_LEVEL);
  //transform_image_c(trmt_data, mapped_levels, BUFFER_SIZE);
  //calculate_histogram_hybrid(trmt_data, hist_cnt, BUFFER_SIZE);
  //map_levels_hybrid(hist_cnt, mapped_levels, BUFFER_SIZE, NUM_GRAY_LEVEL);
  //transform_image_hybrid(trmt_data, mapped_levels, BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //HAL_UART_Receive_IT(&huart2,rcvd_data,BUFFER_SIZE); //main.c
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // for debugging
	// copy received image to the transmit buffer
	// if no image processing function is selected below,
	// the original image received will be sent back (loop back)
	memcpy(trmt_data, rcvd_data, BUFFER_SIZE);
	// clear the histogram array for each new image received
    memset(hist_cnt, 0, NUM_GRAY_LEVEL);

    // test your image processing functions here
    //global_thresholding_hybrid(trmt_data, BUFFER_SIZE, THRESHOLD);
    //band_thresholding_hybrid(trmt_data, BUFFER_SIZE, THRESHOLD1, THRESHOLD2);


	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // for debugging

	// transmit processed image
	HAL_UART_Transmit(huart,trmt_data,BUFFER_SIZE,HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // for debugging
}

void global_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold)
{
    int loopCounter = 0;
    const unsigned char constant1 = 255;
    const unsigned char constant2 = 0;

    while (loopCounter < size) {
        unsigned char element = x[loopCounter];

        if (element >= threshold) {
            x[loopCounter] = constant1;
        }
        else {
            x[loopCounter] = constant2;
        }

        loopCounter++;
    }
}

__attribute__ ((naked)) void global_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold)
{
	__asm volatile (
	// x -> r0, size -> r1, threshold -> r2
			"PUSH {r4, r5, r6, lr}\n\t" // save return address //why r3 no need push?

			// loop over the array
			"MOV r3, #0\n\t" // loop counter r3
			"MOV r4, #255\n\t" // constant used
			"MOV r5, #0\n\t" // constant used"
			"loop_gth: CMP r3, r1\n\t" // terminate the loop when r3 >= r1
			"BGE exit_gth\n\t"
			"LDRB r6, [r0]\n\t" // load array element to r6
			"CMP r6, r2\n\t" // compare r6 with threshold r2
			"BGE else_gth\n\t" //if >=80,return else_gth
			"STRB r5, [r0], #1\n\t" // store r5 to [r0] memory location, post increment [r0] by 1
			"B endif_gth\n\t" //return to end if

			"else_gth: STRB r4, [r0], #1\n\t" // else: store r4 to its memory location, post increment r0 by 1

			"endif_gth: ADD r3, r3, #1\n\t" // end if: increment loop counter

			"B loop_gth\n\t"
			"exit_gth: POP {r4, r5, r6, pc}\n\t" // return from function

    );

}

void band_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold1, uint8_t threshold2)
{
    int loopCounter = 0;
    const unsigned char constant1 = 255;
    const unsigned char constant2 = 0;

    while (loopCounter < size) {
        unsigned char element = x[loopCounter];

        if (element >= threshold1 && element <= threshold2) {
            x[loopCounter] = constant1;
        }
        else {
            x[loopCounter] = constant2;
        }

        loopCounter++;
    }
}

__attribute__ ((naked)) void band_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold1, uint8_t threshold2)
{
	__asm volatile (
	// x -> r0, size -> r1, threshold1 -> r2, threshold2 -> r3
	// counter -> r4, white -> r5, black -> r6, array address element -> r7
			"PUSH {r4, r5, r6, r7, lr}\n\t" // save return address
			//set parameters
			"MOV r4, #0\n\t" // loop counter r4 start from 0
			"MOV r5, #0\n\t" // constant (black): th1(80)> x or x > th2(240)
			"MOV r6, #255\n\t" // constant (white):else
			//loop over the array
			"loop_bth: CMP r4, r1\n\t" // terminate the loop when counter r4 >= size r1
				"BGE exit_bth\n\t"
				"LDRB r7, [r0]\n\t" // load array element (Load Byte) [r0] to r7

				"CMP r7, r2\n\t" // compare with threshold1(80) r2
				"BLT else_bth\n\t" //if x<80, go to else_bth
				"CMP r7, r3\n\t" // compare with threshold2(240) r3
				"BGT else_bth\n\t" //if x>240, go to else_bth
				"STRB r6, [r0], #1\n\t" // else: store r6 (255) to its memory location, post increment r0 by 1
				"B endif_bth\n\t"
			//else
			"else_bth: STRB r5, [r0], #1\n\t" //[r0]=0 (black)
				"B endif_bth\n\t"
			//endif
			"endif_bth: ADD r4, r4, #1\n\t" // increment loop counter
			"B loop_bth\n\t"
			//exit
			"exit_bth: POP {r4, r5, r6, r7, pc}\n\t" // return from function
    );
}

void semi_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold)
{
    int loopCounter = 0;
    const unsigned char constant2 = 0;

    while (loopCounter < size) {
        unsigned char element = x[loopCounter];

        if (element >= threshold) { //80
            x[loopCounter] = x[loopCounter];
        }
        else {
            x[loopCounter] = constant2;
        }

        loopCounter++;
    }
}

__attribute__ ((naked)) void semi_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold)
{
	__asm volatile (
	// x -> r0, size -> r1, threshold -> r2
	// counter -> r3, black -> r4, array address element -> r5 //r3 no need push?
			"PUSH {r4, r5, lr}\n\t" // save return address

			// loop over the array
			"MOV r3, #0\n\t" // loop counter r3
			"MOV r4, #0\n\t" // constant black 0

			"loop_sth: CMP r3, r1\n\t" // terminate the loop when r3 >= r1
			"BGE exit_sth\n\t"
			"LDRB r5, [r0]\n\t" // load array element to r5
			"CMP r5, r2\n\t" // compare r5 with threshold r2
			"BLT else_sth\n\t" //if < 80, go to else_sth
			"STRB r5, [r0], #1\n\t" // store r5 (itself) to [r0] memory location, post increment [r0] by 1
			"B endif_sth\n\t" //return to end if

			"else_sth: STRB r4, [r0], #1\n\t" // else: store r4 (black) to its memory location, post increment r0 by 1
			"B endif_sth\n\t" //return to end if

			"endif_sth: ADD r3, r3, #1\n\t" // end if: increment loop counter
			"B loop_sth\n\t"

			"exit_sth: POP {r4, r5, pc}\n\t" // return from function

    );
}

// shift factor is 1, 2, 3, 4 for 128, 64, 32, 16 gray levels
void gray_level_quantization_c(uint8_t *x, uint32_t size, uint8_t shift_factor)
{
    int loopCounter = 0;

    while (loopCounter < size) {

        x[loopCounter] = x[loopCounter] >> shift_factor;
        loopCounter++;
    }
}


__attribute__ ((naked)) void gray_level_quantization_hybrid(uint8_t *x, uint32_t size, uint8_t shift_factor)
{
	__asm volatile (
	// address of x -> r0, size -> r1, shift_factor  -> r2, counter -> r3, x value -> r4
			"PUSH {r3, r4, lr}\n\t" // do I need to push r3?
			// loop over the array
			"MOV r3, #0\n\t" // loop counter r3

			"loop_glq: CMP r3, r1\n\t" // terminate the loop when r2 >= r1
			"BGE exit_glq\n\t" //BGE endloop_glt1
			"LDRB r4, [r0]\n\t" // load array element to r3
			"LSR r4, r2\n\t" // right shift r2
			//"LSL r4, r2\n\t" // left shift r2 back to get 8bits in total
			"STRB r4, [r0], #1\n\t"

			"ADD r3, r3, #1\n\t" // end if: increment loop counter
			"B loop_glq\n\t"
			"exit_glq: POP {r3, r4, pc}"//endloop

    );
}


void gray_level_transformation1_c(uint8_t *x, uint32_t size)
{
    int loopCounter = 0;
    while (loopCounter < size) {
        x[loopCounter] = 255 - x[loopCounter];
        loopCounter++;
    }
}

__attribute__ ((naked)) void gray_level_transformation1_hybrid (uint8_t *x, uint32_t size)
{
	__asm volatile (
	// x -> r0, size -> r1, counter -> r2, array address element -> r3
			"PUSH {r2, r3, lr}\n\t" // all assign variables
			// loop over the array
			"MOV r2, #0\n\t" // loop counter r2

			"loop_glt1: CMP r2, r1\n\t" // terminate the loop when r2 >= r1
			"BGE exit_glt1\n\t" //BGE endloop_glt1
			"LDRB r3, [r0]\n\t" // load array element to r3
			"RSB r3, r3, #255\n\t" // REVERSE SUB: r3 = 255 - A
			"STRB r3, [r0], #1\n\t"

			"ADD r2, r2, #1\n\t" // end if: increment loop counter
			"B loop_glt1\n\t"
			"exit_glt1: POP {r2, r3, pc}"//endloop

    );
}


void gray_level_transformation2_c(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0)
{
    int loopCounter = 0;
    //A=120, B=50
    double constA = a0; //120.00
    double constB = b0; //50.00
    double A_B = constA/constB;
    double N_A_B = (255.00-constA)/(255.00-constB);

    while (loopCounter < size) {
    	double p0 = x[loopCounter];

        if (p0 <= constB) {
            x[loopCounter] = floor(A_B * p0);
        }
        else {
            x[loopCounter] = floor (N_A_B * (p0 - constB) + constA);
        }

        loopCounter++;
    }
}

__attribute__ ((naked)) void gray_level_transformation2_hybrid(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0)
{
	// make sure you "MUL" first, then "UDIV", otherwise you will get zero
	__asm volatile (
	// x -> r0, size -> r1, a0 -> r2 (120), b0 -> r3 (50)
	// counter -> r4, 255-A -> r5, 255-B -> r6, array address element -> r7, TEMP. -> r8
			"PUSH {r4, r5, r6, r7, r8, r9, lr}\n\t" // save return address

			// loop over the array
			"MOV r4, #0\n\t" // loop counter r4
			"MOV r8, #255\n\t" // load r8 with 255
			"SUB r5, r8, r2\n\t" // r5 = r8 - A
			"SUB r6, r8, r3\n\t" // r6 = r8 - B

			"loop_glt2: CMP r4, r1\n\t" // terminate the loop when r4 >= r1
			"BGE exit_glt2\n\t"
			"LDRB r7, [r0]\n\t" // load array element to r7
			"CMP r7, r3\n\t" // compare r7 with B
			"BLE elseif_glt2\n\t" //if <= B, go to elseif_glt3
			"SUB r8, r7, r3\n\t" //r8 = [r0]-B
			"MUL r8, r5, r8\n\t" //r8 = (255-A) * (P-B)
			"UDIV r8, r8, r6\n\t" //r8 = (255-A)*(P-B) / (255-B)
			"ADD r8, r8, r2\n\t" // r8 = r8 + A
			"STRB r8, [r0], #1\n\t"
			"B endif_glt2\n\t" //return to end if

			"elseif_glt2: MUL r8, r2, r7\n\t" //r8 = A * [r0]
			"UDIV r8, r8, r3\n\t"
			"STRB r8, [r0], #1\n\t"
			"B endif_glt2\n\t" //return to end if


			"endif_glt2: ADD r4, r4, #1\n\t" // end if: increment loop counter
			"B loop_glt2\n\t"

			"exit_glt2: POP {r4, r5, r6, r7, r8, pc}\n\t" // return from function

    );

}

void gray_level_transformation3_c(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0)
{
    int loopCounter = 0;
    //A=120, B=50
    double constA = a0; //120.00
    double constB = b0; //50.00
    double A_B = constA/constB;
    double N_A_B = (255.00 - 2*constA)/(255.00 - 2*constB);

    while (loopCounter < size) {
    	double p0 = x[loopCounter];

        if (p0 <= constB) {
            x[loopCounter] = floor(A_B * p0);
        }
        else if (p0 <= 255.00-constB){
            x[loopCounter] = floor(N_A_B * (p0 - constB) + constA);
        }
        else {
            x[loopCounter] = floor(A_B * (p0-(255.00-constB)) + (255.00-constA));
        }
        loopCounter++;
    }

}

__attribute__ ((naked)) void gray_level_transformation3_hybrid(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0)
{
	// make sure you "MUL" first, then "UDIV", otherwise you will get zero
	__asm volatile (
	// x -> r0, size -> r1, a0 -> r2 (120), b0 -> r3 (50)
	// counter -> r4, 255-A -> r5, 255-B -> r6, array address element -> r7, TEMP. -> r8, r9
			"PUSH {r4, r5, r6, r7, r8, r9, lr}\n\t" // save return address

			// loop over the array
			"MOV r4, #0\n\t" // loop counter r4
			"MOV r8, #255\n\t" // load r8 with 255
			"SUB r5, r8, r2\n\t" // r5 = r8 - A
			"SUB r6, r8, r3\n\t" // r6 = r8 - B

			"loop_glt3: CMP r4, r1\n\t" // terminate the loop when r4 >= r1
			"BGE exit_glt3\n\t"
			"LDRB r7, [r0]\n\t" // load array element to r7
			"CMP r7, r3\n\t" // compare r5 with B
			"BLE elseif_glt3\n\t" //if <= B, go to elseif_glt3
			"CMP r7, r6\n\t" // compare r7 with 255-B
			"BLE else_glt3\n\t" //if <= 255-B, go to else_glt3
			"SUB r8, r7, r6\n\t" //r8 = [r0]-(255-B)
			"MUL r8, r2, r8\n\t" //r8 = A * (P-(255-B))
			"UDIV r8, r8, r3\n\t" //r8 = A*(P-(255-B)) / B
			"ADD r8, r8, r5\n\t" // r8 = r8 + (255-A)
			"STRB r8, [r0], #1\n\t" // store r5 (itself) to [r0] memory location, post increment [r0] by 1
			"B endif_glt3\n\t" //return to end if

			"elseif_glt3: MUL r8, r2, r7\n\t" //r8 = A * [r0]
			"UDIV r8, r8, r3\n\t"
			"STRB r8, [r0], #1\n\t"
			"B endif_glt3\n\t" //return to end if

			"else_glt3: SUB r8, r5, r2\n\t" //r8 = (255-A)-A
			"SUB r9, r7, r3\n\t" //r9 = [r0]-B
			"MUL r8, r9, r8\n\t" //r8 = (255-2A) * (P-B)
			"SUB r9, r6, r3\n\t" //r9 = (255-B)-B
			"UDIV r8, r8, r9\n\t" //r8 = (255-2A)*(P-B) / (255-2B)
			"ADD r8, r8, r2\n\t" // r8 = r8 + A
			"STRB r8, [r0], #1\n\t"
			"B endif_glt3\n\t" //return to end if

			"endif_glt3: ADD r4, r4, #1\n\t" // end if: increment loop counter
			"B loop_glt3\n\t"

			"exit_glt3: POP {r4, r5, r6, r7, r8, r9, pc}\n\t" // return from function

    );
}

/*
"calculate_histogram" is a function that counts the frequency of the occurrence of each
grayscale level in an image.
*/
void calculate_histogram_c(uint8_t *x, uint32_t *hist, uint32_t size)
{
    for (int i = 0; i < size; i++) {
        hist[x[i]]++; //counting the number of occurrences of each gray level
    }
}
//how to debug one line by a line?
__attribute__ ((naked)) void calculate_histogram_hybrid(uint8_t *x, uint32_t *hist, uint32_t size)
{
	__asm volatile ( //what's the initial address of r0 and r1, will they be too close to each other?
	// memory address of x -> r0, memory address of hist_cnt -> r1, size -> r2,counter --> r3
	// x element x[i] -> r4, hist_cnt[x[i]] element --> r5
			"PUSH {r4, r5, lr}\n\t" // all assign variables
			// loop over the array
			"MOV r3, #0\n\t" // initial loop counter r3 = 0
			//"MOV r5, #0\n\t" // initial hist_cnt r5 = 0

			"loop_his1: CMP r3, r2\n\t" // Compare counter with size
			"BGE exit_his1\n\t" // If i >= size, exit the loop
			//"LDRB r4, [r0, r3]\n\t" //r4 <- memory address[r0]+r3 = x[i]
			"LDRB r4, [r0], #1\n\t" //r4 <- memory address[r0], then r0+1
			"LDRB r5, [r1, r4, LSL#2]\n\t" //r5 <- memory[r1+r4*4]= hist[x[i]]'s count (*4 is for shifting 4 memory address each x)
			"ADD r5, r5, #1\n\t" // Increment hist_cnt[x[i]]
			"STRB r5, [r1, r4, LSL#2]\n\t"  // Store the updated value of hist_cnt[x[i]] to the same memory address

			"ADD r3, r3, #1\n\t" // end if: increment loop counter
			"B loop_his1\n\t"
			"exit_his1: POP {r4, r5, pc}\n\t"// endloop

    );
}

/*
"map_levels" is a function that generates a mapping table (mapped_levels) to
equalize the histogram.
*/
void map_levels_c(uint32_t *hist, uint8_t *mapping_table, uint32_t size, uint16_t levels)
{
    int sum = 0;
    for (int i = 0; i <= levels; i++) { //levels = MAX_PIXEL_VALUE
        sum += hist[i]; //accumulated histogram count (cdf)
        mapping_table[i] = (int)(((levels-1) * sum) / size);
    }
}
//how to know which memory address should add4, which should add 1: bits number
__attribute__ ((naked)) void map_levels_hybrid(uint32_t *hist, uint8_t *mapping_table, uint32_t size, uint16_t levels)
{
	// make sure you "MUL" first, then "UDIV", otherwise you will get zero
	// you need to accumulate the histogram counts, multiply by (levels-1), then divide by size
	__asm volatile (
	// memory address for hist_cnt -> r0, memory address for mapping_table -> r1, size -> r2, levels -> r3,
	// counter -> r4, sum -> r5, hist_cnt accumulated value -> r6, level-1 -> r7
			"PUSH {r4, r5, r6, r7, lr}\n\t" // all assign variables
			// loop over the array
			"MOV r4, #0\n\t" // initial counter = 0
			"MOV r5, #0\n\t" // initial sum = 0

			"loop_his2: CMP r4, r3\n\t" // Compare counter with levels
			"BGE exit_his2\n\t" // If i >= size, exit the loop
			//"LDRB r6, [r0, r4, LSL#2]\n\t" // Load hist[i] (memory[r0+r4*4]) into r6, r0 unchange
			"LDRB r6, [r0], #4\n\t" // Load hist[i] (memory[r0]) into r6, then r0 +4
			"ADD r5, r5, r6\n\t" // Add hist[i] to sum
			"SUB r7, r3, #1\n\t" //Load levels-1 into r7
			"MUL r6, r7, r5\n\t" //Multiply (levels-1)*sum
			"UDIV r6, r6, r2\n\t" // Divide by size to get new value
			"STRB r6, [r1], #1\n\t"  // Store the new value in mapping_table[i]

			"ADD r4, r4, #1\n\t" // end if: increment loop counter
			"B loop_his2\n\t"

			"exit_his2: POP {r4, r5, r6, r7, pc}\n\t"// endloop

    );
}


/*
"transform_image" is a function that transforms the brightness levels of an image array
according to the mapping_table using histogram equalization
*/

void transform_image_c(uint8_t *x, uint8_t *mapping_table, uint32_t size)
{
    for (int i = 0; i < size; i++) {
        x[i] = mapping_table[x[i]]; //img_equ[i] = mapped_levels[img_org[i]]
    }
}
//* means the address
__attribute__ ((naked)) void transform_image_hybrid(uint8_t *x, uint8_t *mapping_table, uint32_t size)
{
	__asm volatile (
	// memory address of x -> r0, memory address of mapping_table -> r1, size -> r2,counter --> r3
	// x element x[i] -> r4, mapping_table[x[i]] element --> r5
			"PUSH {r4, r5, lr}\n\t" // all assign variables
			// loop over the array
			"MOV r3, #0\n\t" // initial loop counter r3 = 0

			"loop_his3: CMP r3, r2\n\t" // Compare counter with size
			"BGE exit_his3\n\t" // If i >= size, exit the loop
			"LDRB r4, [r0]\n\t" //r4 <- memory address[r0]
			"LDRB r5, [r1, r4]\n\t" //r5 <- memory[r1+r4]= mapping_table[x[i]]
			"STRB r5, [r0], #1\n\t"  // Store the updated value of x[i], then r0+1

			"ADD r3, r3, #1\n\t" // end if: increment loop counter
			"B loop_his3\n\t"
			"exit_his3: POP {r4, r5, pc}\n\t"// endloop

    );
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
