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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include "arm_math.h"
#include "wav_player.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	ALL_KEYS, //Checks For ALL Specified Keys in the mask
	ANY_KEYS  //Checks For ANY DTMF Key Code
}Code_Sequence_Mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_BUFFER_SIZE 15u*2048
#define BUFFER_SIZE 2048
#define BUFFER_HALFSIZE 1024
#define FFT_LENGTH 2048
#define N_TAPS 32
#define N_BLOCK 32
#define SAMPLING_RATE 20000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t dac_buffer[DAC_BUFFER_SIZE];
uint16_t adc_buffer[BUFFER_SIZE];
float32_t input_signal[BUFFER_SIZE];
float32_t output_fft[FFT_LENGTH];
float32_t output_fft_mag[FFT_LENGTH/2];
float32_t output_fft_freq[FFT_LENGTH/2];
float32_t state[N_TAPS + N_BLOCK - 1];

uint16_t FLAG = 0;

static float32_t coeff[N_TAPS] = {
  -0.004321596631323414,
  -0.005152814576266107,
  -0.007114956251177061,
  -0.008402242541365729,
  -0.0082785116637233,
  -0.0059695981473415925,
  -0.0007953660993506074,
  0.007666395307462849,
  0.019477490844025627,
  0.03423210036270327,
  0.05106299019968595,
  0.06868925654691155,
  0.08555952663530968,
  0.10004618866353766,
  0.11066200257921187,
  0.11627631449955723,
  0.11627631449955723,
  0.11066200257921187,
  0.10004618866353766,
  0.08555952663530968,
  0.06868925654691155,
  0.05106299019968595,
  0.03423210036270327,
  0.019477490844025627,
  0.007666395307462849,
  -0.0007953660993506074,
  -0.0059695981473415925,
  -0.0082785116637233,
  -0.008402242541365729,
  -0.007114956251177061,
  -0.005152814576266107,
  -0.004321596631323414
};

arm_rfft_fast_instance_f32 fft_handler;
arm_fir_instance_f32 fir_filter;

PLAY_FILE audio_file;
P_STATE player_state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void Process_Signal();
void Get_DTMF_Code(int32_t*);
void Led_Audio_Control(int32_t mode);
void Evaluate_Code_Sequence(int32_t, int32_t*, int32_t*, Code_Sequence_Mode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Print Player Output Statuses Via UART
extern void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}
//Calculates the Output FFT of the Audio Signal from ADC
void Process_Signal(){

	//Calculate the output FFT Frequencies at 20kHz Sampling frequency
	for(int i = 0; i < FFT_LENGTH/2; i++){
		output_fft_freq[i] = (float)(i)/FFT_LENGTH * SAMPLING_RATE;
	}

//	float32_t filtered_signal[FFT_LENGTH];

	//Finite Impulse Response Filter applied to the input signal (passes signals with a freq band of 600Hz to 3.4kHz)
//	for(int i = 0; i < FFT_LENGTH/N_BLOCK; i++){
//		arm_fir_f32(&fir_filter, &input_signal[i*N_BLOCK], &filtered_signal[i*N_BLOCK], N_BLOCK);
//	}

	//Calculate the real output fft (output_fft consists of interleaved real and imaginary components)
	arm_rfft_fast_f32(&fft_handler, input_signal, output_fft, 0);
	//Calculate the magnitudes of the output_fft (first 2 output_fft values are purely real for the first and last output fft values)
	arm_cmplx_mag_f32(output_fft+2, output_fft_mag+1, FFT_LENGTH/2);

	//Store the first 2 purely real fft values into their respective position in the output_fft_mag array
	output_fft_mag[0] = output_fft[0];
	output_fft_mag[FFT_LENGTH/2-1] = output_fft[1];
}

//Analyzes The FFT of Audio Signal to Retrieve Already Present DTMF Code
void Get_DTMF_Code(int32_t* dtmf_code_out){
	/*
	 *                                      How It Works:
	 * Things To Note:
	 * 		1. The Output FFT's Frequency range has a resolution of 10 approximately.
	 * 			- (Equation A) resolution = (1 / FFT_LENGTH) * SAMPLING_FREQUENCY
	 *
	 * 		2. The index of a desired frequency/Magnitude can be found (using the equation above) by;
	 * 			- (Equation B) i = ( (DESIRED_FREQUENCY * FFT_LENGTH) / SAMPLING_FREQUENCY ) + 1
	 * 				- i = index
	 *
	 *		3. Rearranging the the above equation we can also find the frequency of a desired index value by:
	 *			- (Equation C)DESIRED_FREQUENCY_EQV = ((i * SAMPLING_FREQUENCY) / FFT_LENGTH) - resolution.
	 *				- i = index
	 *
	 *
	 * Algorithm:
	 *		- Find the Output FFT Magnitudes at the frequencies of interest locations.
	 *			1. Use Equation B above to find the index of the desired frequency. (DO NOT ROUND)
	 *			2. Use Equation C to find the equivalent desired freq when the resolution is taken to account.
	 *			4. Plug in the frequency gotten in step 2 to Equation B to find the index associated with the freq calculated in step 3.(ROUND THE RESULT)
	 *			5. Use the index found in step 4 together with the Output FFT Magnitude array to find the magnitude. (Matches with the magnitude of the desired freq)

	 *
	 *		- Add up the frequency magnitudes according to the corresponding DTMF Codes. Example;
	 *			- Magnitude of freq 697Hz + Magnitude of freq 1209Hz = DTMF CODE 1
	 *
	 *		- Find the DTMF Code that has the largest Magnitude and give that as the answer.
	 *
	 *
	 * */

	//DTMF Frequencies, Row frequencies stored first then Column frequencies
	int dtmf_freq[] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};

	//DTMF Frequency's Magnitudes Represented as Rows and Cols Array
	float dtmf_freq_mag_rows[] = {0,0,0,0}; // in order 697Hz, 770Hz, 852Hz, 941Hz respectively
	float dtmf_freq_mag_cols[] = {0,0,0,0}; // in order 1209Hz, 1336Hz, 1477Hz, 1633Hz respectively

	//Stored in order 1,4,7,*,2,5,8,0,3,6,9,#,A,B,C,D respectively
	float32_t dtmf_mag[16];

	//Threshold for detecting true dtmf_code
	int32_t threshold = 100000;

	//Some temp Variables
	float32_t index = 0;
	int tmp_index = 0, max_mag_index = 0;

	//Setup the dtmf output code to negative in-case a code wasn't found
	*dtmf_code_out = -1;

	//Loop over each DTMF Frequency and calculate its Corresponding FFT Magnitude
	for(int i = 0; i < 8; i++){
		index = (int)(((dtmf_freq[i] * FFT_LENGTH) / SAMPLING_RATE) + 1);	//Find the index corresponding with dtmf_freq[i] (step 1.)

		if(output_fft_mag[(int)index] < output_fft_mag[(int)(index+0.5)])	//Compare magnitudes between the calculated index and the nxt index value to find the peak mag for the dtmf_freq[i]
			index = (int)(index+0.5);

		if(i < 4){	//Store the Row frequency Magnitudes
			dtmf_freq_mag_rows[i] = output_fft_mag[(int)index];
		}else{
			dtmf_freq_mag_cols[i-4] = output_fft_mag[(int)index];	//Store the Columns frequency Magnitudes

			//Calculate the DTMF MAG, 4 mag values are calculated on each run column wise.
			dtmf_mag[tmp_index]   = dtmf_freq_mag_rows[0] + dtmf_freq_mag_cols[i-4];
			dtmf_mag[tmp_index+1] = dtmf_freq_mag_rows[1] + dtmf_freq_mag_cols[i-4];
			dtmf_mag[tmp_index+2] = dtmf_freq_mag_rows[2] + dtmf_freq_mag_cols[i-4];
			dtmf_mag[tmp_index+3] = dtmf_freq_mag_rows[3] + dtmf_freq_mag_cols[i-4];


			//Find the Max Value out of the 4 mag values
			if(dtmf_mag[max_mag_index] < dtmf_mag[tmp_index+1])
				max_mag_index = tmp_index+1;
			if(dtmf_mag[max_mag_index] < dtmf_mag[tmp_index+2])
				max_mag_index = tmp_index+2;
			if(dtmf_mag[max_mag_index] < dtmf_mag[tmp_index+3])
				max_mag_index = tmp_index+3;
			if(dtmf_mag[max_mag_index] < dtmf_mag[tmp_index])
				max_mag_index = tmp_index;

			//Update the output dtmf code
			if(dtmf_mag[max_mag_index] > threshold)
				*dtmf_code_out = max_mag_index;
			tmp_index += 4;
		}
	}
}
/*
 * Evaluates The Occurence of a predefined Code Sequence.
 * Parameters;
	 * code   - Single digit input code received.
	 * length - code sequence length.
	 * mask   - mask that defines the accepted pass-key's positions. (0-15 bits used for the keys)
 * */
void Evaluate_Code_Sequence(int32_t code, int32_t* length, int32_t* mask, Code_Sequence_Mode mode){
	if(code < 0 || code > 15){
		Led_Audio_Control(0);
		return;
	}

	if(mode == ANY_KEYS){ //Accepts Any DTMF Code For a Success Condition
		Led_Audio_Control(1);
		return;
	}


	*mask = *mask ^ (1 << code);
	*length -= 1;

	if(*length < 1)
		Led_Audio_Control( *mask == 0 );	//Accepts one or more Specified DTMF Code(s) For a Success Condition
}
/* Controls the Led animation,
 * mode <= 0 Means Failure thus, repeated blinking Animation
 * mode >= 1 Means Success thus, led stays on for 2 seconds.
 * */
void Led_Audio_Control(int32_t mode){
	if(mode <= 0){
		for(int i = 0; i < 10; i++){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(100);
		}
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, dac_buffer, DAC_BUFFER_SIZE, DAC_ALIGN_12B_R);
		init_player("booing.wav", dac_buffer, DAC_BUFFER_SIZE, &audio_file, &player_state);
		return;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, dac_buffer, DAC_BUFFER_SIZE, DAC_ALIGN_12B_R);
	init_player("applause.wav", dac_buffer, DAC_BUFFER_SIZE, &audio_file, &player_state);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_DAC1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);
  arm_rfft_fast_init_f32(&fft_handler, FFT_LENGTH);
  arm_fir_init_f32(&fir_filter, N_TAPS, &coeff[0], &state[0], N_BLOCK);

  int32_t pass_code_mask = 1 << 1 | 1 << 4 | 1 << 12 | 1 << 15; //= 4 2 A D
  int32_t sequence_length = 4;

//  int32_t pass_code_mask = 1 << 1 | 1 << 4 | 1 << 8 | 1 << 12;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  player_error_handler(&audio_file, &player_state);
	  start_player_t16(&audio_file, &player_state);

	  if(FLAG){

	  		int32_t code = -1;

	  		Process_Signal();

	  		/* DTMF OUTPUT CODE KEY
	  		 * 0 - 3   = 1,4,7,* respectively
	  		 * 4 - 7   = 2,5,8,0 respectively
	  		 * 8 - 11  = 3,6,9,# respectively
	  		 * 12 - 15 = A,B,C,D respectively
	  		 * else = Code Not Found
	  		*/
	  		Get_DTMF_Code(&code); //7,9,c,d have problems getting detected
	  		Evaluate_Code_Sequence(code, &sequence_length, &pass_code_mask, ALL_KEYS);

	  		if(sequence_length <= 0){
	  			pass_code_mask = 1 << 1 | 1 << 4 | 1 << 12 | 1 << 15;
	  			sequence_length = 4;
	  		}

			FLAG = 0;
	  }
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	HAL_ADC_Start_DMA(&hadc1, adc_buffer, BUFFER_SIZE);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	for(int i = 0; i < BUFFER_HALFSIZE; i++){
		input_signal[i] = adc_buffer[i];
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	HAL_ADC_Stop_DMA(&hadc1);
	for(int i = BUFFER_HALFSIZE; i < BUFFER_SIZE; i++){
		input_signal[i] = adc_buffer[i];
	}
	FLAG = 1;
//	process_signal(&adc_buffer[0], &output_fft_freq[0]);
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac){

}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac){
	if((player_state & (CLOSE_FILE | PLAY_ERROR)) > 0 || player_state == 0){
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		for(int i = 0; i < DAC_BUFFER_SIZE; i++){ //ZERO OUT THE BUFFER FROM ANY REMAINING AUDIO
			dac_buffer[i] = 0;
		}
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
