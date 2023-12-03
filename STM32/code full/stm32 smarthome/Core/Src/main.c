#define PUMP_ON  		           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define PUMP_OFF                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define BUZZER_ON 						  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define BUZZER_OFF             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define LED_ON                 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define LED_OFF                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define BNT_UP                 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)
#define BNT_DOWN                 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)
#define BNT_SELECT                 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)
#define ADDRESS_HUM (uint32_t)0x0800FC00
#define ADDRESS_GAS (uint32_t)0x0800FC04


#include "main.h"
#include "DHT.h"
#include "CLCD_I2C.h"
#include "stdio.h"
typedef struct 
{
	uint8_t up;
	uint8_t down;
	uint8_t select;
	uint8_t mode;
	uint8_t cursor;
	uint8_t led;
	uint8_t pump;
	uint8_t flash;
	uint8_t warning;
}STATUS;
STATUS status;

typedef struct
{
	uint8_t p0;
	uint8_t p1;
	uint8_t p2;
	uint8_t dht;
	uint8_t old_dht;
	uint8_t led;
	uint8_t pump;
	uint8_t cursor;
	uint8_t set_temp_up;
	uint8_t set_temp_down;
	uint8_t set_gas_up;
	uint8_t set_gas_down;
	uint8_t gas;
	uint8_t old_gas;
	uint8_t warning;
	uint8_t old_warning;
}COUNT;
COUNT count;

uint8_t count_time;
uint32_t time;

uint8_t enable_select;
uint8_t temp_dht;
uint8_t hum_dht;

uint8_t set_hum;
uint8_t hum_land;

uint16_t readAdc[1];
uint16_t adc;
uint16_t sum_adc;

float vol;
uint8_t gas_value;


ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);

void page0(void);
void page1(void);
void page2(void);
void read_dht(void);
void led(void);
void pump(void);
void read_dht_gas(void);
void auto_mode(void);
void warning(void);

///////////////////////////////////
////// xoa bo nho flash 
void Flash_Erase(uint32_t address)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks = 1;
	EraseInitStruct.NbPages = 1;
	EraseInitStruct.PageAddress = address;
	uint32_t pageerr = 0;
	HAL_FLASHEx_Erase (&EraseInitStruct, &pageerr);
	HAL_FLASH_Lock();
}
//////////////////////////////////
////////////// ghi 2 byte vao bo nho flash ///////////////
void Flash_Write_Int(uint32_t address, int16_t value)
{
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, value);
    HAL_FLASH_Lock();
}
/////// doc 2 byte tu bo nho flash///////////////////////
int16_t Flash_Read_Init(uint32_t address)
{
  return *(__IO int16_t *)(address);
}
///////////////////////////////////


DHT_DataTypedef DHT11;
CLCD_I2C_Name LCD1;

int main(void)

 {

  HAL_Init();

  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
	
	CLCD_I2C_Init(&LCD1,&hi2c1,0x4e,16,2);
	CLCD_I2C_Clear(&LCD1);
	set_hum = Flash_Read_Init(ADDRESS_HUM);
	//set_gas = Flash_Read_Init(ADDRESS_GAS);

  while (1)
  {
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)readAdc, 1);

		sum_adc = readAdc[0];
		
		sum_adc = sum_adc/100;
		
		
		vol = ((3.3* sum_adc)/4095.0);
		
		
		read_dht();
		
		if(status.flash == 1)
		{
			Flash_Erase(ADDRESS_HUM);
			HAL_FLASH_Unlock();
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ADDRESS_HUM, (uint16_t)set_hum);
			//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ADDRESS_GAS, (uint16_t)set_gas);
			HAL_FLASH_Lock();		
			status.flash = 0;
		}
		
		
		switch (status.mode)
		{
			case 0 :
				page0();
			break;
				
			case 1 :
				page1();
			break;
			
			case 2 :
				page2();
			break;
			
		}
  }
}
 void warning(void)
 {
	 if(status.warning == 1)
	 {
		if(HAL_GetTick() - time > 1500)
		{
			count.warning ++;
			time = HAL_GetTick();
		}
		if(count.old_warning != count.warning)
		{
			count.old_warning = count.warning;
		}
	} 
 }
 /************ tu dong *********/
void auto_mode(void)
{
	/*if(gas > set_gas)
	{
    status.warning ++;
	}
	else
	{
		status.warning = 0;
		BUZZER_OFF;
	}
	if(status.warning == 50)
	{
		BUZZER_ON;
	}
	
	if(set_temp < temp_dht)
	{
		FAN_ON;
	}
	else FAN_OFF;*/
	
	if(hum_dht < set_hum)
	{
		PUMP_ON;
	}
	else PUMP_OFF;
	
	
}
 /*********** doc cam bien dht  *****/
void read_dht(void)
{
	if(BNT_DOWN == 1 && BNT_UP == 1)
		{
			if(HAL_GetTick() - time > 700)
			{
				count.dht ++;
				time = HAL_GetTick();
			}
			if(count.old_dht != count.dht)
			{
				DHT_GetData(&DHT11);
				temp_dht = DHT11.Temperature;
				hum_dht = DHT11.Humidity;
				hum_land = (100*vol)/3.3;
				count.old_dht = count.dht;
			}
		}
}
/********** che do tu dong ********/
void page0()
{
	auto_mode();
	CLCD_I2C_SetCursor(&LCD1, 0,0);
	CLCD_I2C_WriteString(&LCD1, "AUTO");
	CLCD_I2C_SetCursor(&LCD1, 8,0);
	CLCD_I2C_WriteString(&LCD1, "Temp :");
	CLCD_I2C_SetCursor(&LCD1, 14,0);
	char temp[2];
	sprintf(temp,"%d",temp_dht);
	CLCD_I2C_WriteString(&LCD1, temp);
	CLCD_I2C_SetCursor(&LCD1, 0,1);
	CLCD_I2C_WriteString(&LCD1, "Hum :");
	char hum[2];
	sprintf(hum,"%d",hum_dht);
	CLCD_I2C_WriteString(&LCD1, hum);
	CLCD_I2C_SetCursor(&LCD1, 10,1);
	CLCD_I2C_WriteString(&LCD1, "H_L:");
	if(hum_land < 10)
	{
		CLCD_I2C_WriteString(&LCD1, "0");
		char hum_value[2];
		sprintf(hum_value,"%d", hum_land);
		CLCD_I2C_WriteString(&LCD1, hum_value);	
	}
	else
	{
		char hum_value[2];
		sprintf(hum_value,"%d", hum_land);
		CLCD_I2C_WriteString(&LCD1, hum_value);		
	}
	
	//led();
	/********** chuyen page 1 ********/
	if(BNT_SELECT == 1) enable_select = 0;
	if(BNT_SELECT == 0 && enable_select == 0)
	{
		if(HAL_GetTick() - time > 1)
		{
			count.p0 ++;
			if(count.p0 > 2) count.p0 = 3;
			time = HAL_GetTick();	
		}
		if(count.p0 == 2)
		{
			status.mode = 1;
			count.p1 = 0;
			enable_select = 1;
			CLCD_I2C_Clear(&LCD1);
			PUMP_OFF;
			BUZZER_OFF;
		}
	}
}
/**************** che do thu cong *************/
void page1()
{
	CLCD_I2C_SetCursor(&LCD1, 0,0);
	CLCD_I2C_WriteString(&LCD1, "MANUAL");
	CLCD_I2C_SetCursor(&LCD1, 8,0);
	CLCD_I2C_WriteString(&LCD1, "Temp :");
	char temp[2];
	sprintf(temp,"%d",temp_dht);
	CLCD_I2C_WriteString(&LCD1, temp);
	CLCD_I2C_SetCursor(&LCD1, 0,1);
	CLCD_I2C_WriteString(&LCD1, "Hum :");
	char hum[2];
	sprintf(hum,"%d",hum_dht);
	CLCD_I2C_WriteString(&LCD1, hum);
	
	CLCD_I2C_SetCursor(&LCD1, 10,1);
	CLCD_I2C_WriteString(&LCD1, "Gas:");
	if(hum_land < 10)
	{
		CLCD_I2C_WriteString(&LCD1, "0");
		char hum_value[2];
		sprintf(hum_value,"%d", hum_land);
		CLCD_I2C_WriteString(&LCD1, hum_value);	
	}
	else
	{
		char hum_value[2];
		sprintf(hum_value,"%d", hum_land);
		CLCD_I2C_WriteString(&LCD1, hum_value);		
	}
	//led();
	pump();
	
	
	/************ chuyen page 2 ******/
	if(BNT_SELECT == 1) enable_select = 2;
	if(BNT_SELECT == 0 && enable_select == 2)
	{
		if(HAL_GetTick() - time > 1)
		{
			count.p1 ++;
			if(count.p1 > 2) count.p1 = 3;
			time = HAL_GetTick();	
		}
		if(count.p1 == 2)
		{
			status.mode = 2;
			count.p2 = 0;
			enable_select = 3;
			PUMP_OFF;
			LED_OFF;
			CLCD_I2C_Clear(&LCD1);
		}
	}
}
/********** che do cai dat ************/
void page2()
{
	CLCD_I2C_SetCursor(&LCD1, 5,0);
	CLCD_I2C_WriteString(&LCD1, "SETTING");
	
	CLCD_I2C_SetCursor(&LCD1, 1,1);
	CLCD_I2C_WriteString(&LCD1, "Hum:");
	char setHum[2];
	sprintf(setHum,"%d", set_hum);
	CLCD_I2C_WriteString(&LCD1, setHum);
	
		
		if(BNT_UP == 0 && BNT_DOWN == 1)
		{
			if(HAL_GetTick() - time > 1)
			{
				count.set_temp_up ++;
				if(count.set_temp_up > 1) count.set_temp_up = 2;
				time = HAL_GetTick();	
			}
			if(count.set_temp_up == 1)
			{
				set_hum ++;
				if(set_hum > 99) set_hum = 0;
			}
		}
		else count.set_temp_up = 0;
		if(BNT_DOWN == 0 && BNT_UP == 1)
		{
			if(HAL_GetTick() - time > 1)
			{
				count.set_temp_down ++;
				if(count.set_temp_down > 1) count.set_temp_down = 2;
				time = HAL_GetTick();	
			}
			if(count.set_temp_down == 1)
			{
				set_hum --;
				if(set_hum < 0) set_hum = 99;
			}
		}
		else count.set_temp_down = 0;
		

		
	
	/*if(BNT_DOWN == 0 && BNT_UP == 0)
	{
		if(HAL_GetTick() - time > 1)
		{
			count.cursor ++;
			if(count.cursor > 2) count.cursor = 3;
			time = HAL_GetTick();	
		}
		if(count.cursor == 2)
		{
			status.cursor = !status.cursor ;
		}	
	}
	else count.cursor = 0;*/
	
	
	/******* chuyen page 0 *********/
	if(BNT_SELECT == 1) enable_select = 4;
	if(BNT_SELECT == 0 && enable_select == 4)
	{
		if(HAL_GetTick() - time > 1)
		{
			count.p2 ++;
			if(count.p2 > 2) count.p2 = 3;
			time = HAL_GetTick();	
		}
		if(count.p2 == 2)
		{
			status.mode = 0;
			count.p0 = 0;
			enable_select = 5;
			CLCD_I2C_Clear(&LCD1);
			PUMP_OFF;
			status.flash = 1;
		}
	}
}
/************* dieu khien den **********/
void led(void)
{
	if(BNT_UP == 0)
	{
		if(HAL_GetTick() - time > 1)
		{
			count.led ++;
			if(count.led > 2) count.led = 3;
			time = HAL_GetTick();
		}
		if(count.led == 2)
		{
			status.led = !status.led;
		}
	}
	else count.led = 0;
	if(status.led == 0) LED_OFF;
	else LED_ON;	
}
/************ dieu khien quat ********/
void pump(void)
{
	if(BNT_DOWN == 0)
	{
		if(HAL_GetTick() - time > 1)
		{
			count.pump ++;
			if(count.pump > 2) count.pump = 3;
			time = HAL_GetTick();
		}
		if(count.pump == 2)
		{
			status.pump = !status.pump;
		}
	}
	else count.pump = 0;
	if(status.pump == 0) PUMP_OFF;
	else PUMP_ON;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
i  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
