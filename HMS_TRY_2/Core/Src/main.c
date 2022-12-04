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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BML_DEF.h"
#include "rcc.h"
#include "uart.h"
#include "adc.h"
#include "i2c.h"
#include "MLX90614.h"
#include "st7783.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
BML_StatusTypeDef ret;

#define SCL_PORT	PORTB
#define SCL_PIN		8

#define SDA_PORT	PORTB
#define SDA_PIN		9

#define graphRefreshRate_peak			2
#define graphRefreshRate_normal			5

uint8_t hmsParamVar = 0;
uint16_t adcVal = 0;
float temp = 0, ambient = 0;

const int graph_x0 = 30;
const int graph_y0 = (2 * (TFTHEIGHT / 3)) + 30;
const int graph_x1 = 30;
const int graph_y1 = graph_y0 + 50;
const int graph_x2 = graph_x0 + TFTWIDTH - 95;
const int graph_y2 = graph_y0;
const int graph_x3 = graph_x0 + TFTWIDTH - 95;
const int graph_y3 = graph_y0 + 50;
int graph_x = graph_x0;
int graph_y = 0;
int graph_y_prevVal = 0;
int graph_refresh_x = graph_x0;

uint8_t hnsStr[100] = { 0 };
uint8_t hnsStrData[100] = { 0 };
uint8_t hStr[10] = { 0 };
uint8_t sStr[10] = { 0 };
uint8_t hnsStrRecFlag = 0;
uint8_t idx = 0;
bool tempReadFlag = false;
bool dispTemp_flag = true, dispHnS_flag = true;
float hVal = 0;
int sVal = 0;
int16_t beatVal = 0;
volatile uint16_t hrVal = 0;

#define adcArrLen	200
uint16_t adcArr[adcArrLen] = { 0 };
uint16_t adcArrHead = 0, adcArrTail = 0;
uint16_t prevBeatVal = 0, nextBeatVal = 0;

uint32_t mSec = 100;
uint16_t sec = 0, min = 0, oneMin = 0;

void gpio_settingfn(void);
void gpio11_callback(void);
void gpio12_callback(void);
void HMS_bgdisplay(void);

uint16_t temp2color(int degree, int lo, int hi);
void colorgradient(int x, int y, int w, int h, int percent);
void LCD_DrawGraph(uint16_t value);

uint16_t readECG();
void readTemperature();
void displayTemperature();
void readHnS();
void displayHnS();

void LCD_ECGAnimation();
void ecgPeak();
void ecgLine();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osMessageQId myQueue01Handle;
osSemaphoreId myBinarySem01Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const *argument);
void StartTask02(void const *argument);
void StartTask03(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	/* USER CODE BEGIN 2 */
	uart_print_config(9600);
	USART_INIT(USART3, PORTC, PORTC, 10, 11, 9600);
	USART_IT_EN(USART3, 0, 1, 0);
	print("Running\n");

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of myBinarySem01 */
	osSemaphoreDef(myBinarySem01);
	myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* definition and creation of myQueue01 */
	osMessageQDef(myQueue01, 16, uint16_t);
	myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of myTask02 */
	osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 128);
	myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

	/* definition and creation of myTask03 */
	osThreadDef(myTask03, StartTask03, osPriorityIdle, 0, 256);
	myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void gpio_settingfn(void) {
	gpio_config(PORTA, 11, INPUT_MODE, LOW_SPEED, EN_PD, PHPL);	// Interrupt pin PA11
	gpio_config(PORTA, 12, INPUT_MODE, LOW_SPEED, EN_PD, PHPL);	// Interrupt pin PA12

	gpio_IT_config(PORTA, 11, RISING_EDGE);
	gpio_IT_config(PORTA, 12, RISING_EDGE);

	gpio_IT_EN(11, EXTI4_15_IRQn);
	gpio_IT_EN(12, EXTI4_15_IRQn);

	gpio_config(ADC14_PORT, ADC14_PIN, ANALOG_MODE, LOW_SPEED, DI_PUPD, PHPL);// Analog Pin

	gpio_config(SCL_PORT, SCL_PIN, ALT_MODE, HIGH_SPEED, EN_PU, OD);// I2C pins
	gpio_config(SDA_PORT, SDA_PIN, ALT_MODE, HIGH_SPEED, EN_PU, OD);

	gpio_altfn(SCL_PORT, SCL_PIN, I2C1_SCL_PB8);
	gpio_altfn(SDA_PORT, SDA_PIN, I2C1_SDA_PB9);
}

void gpio11_callback() {
	print("Gpio 11 IT\n");
}
void gpio12_callback() {
	print("Gpio 12 IT\n");
}

void HMS_bgdisplay(void) {

	//===================== | Border |===============================//
	for (uint16_t i = 5; i > 0; i--) {
		LCD_DrawRect(i, i, TFTWIDTH - 2 * i, TFTHEIGHT - 2 * i, WHITE);
	}

	LCD_DrawHBorder(0, TFTHEIGHT / 3, TFTWIDTH, WHITE, 5);
	LCD_DrawHBorder(0, 2 * (TFTHEIGHT / 3), TFTWIDTH, WHITE, 5);
	//===============================================================//

	//===================== | Box 1 |===============================//

	LCD_DrawButton1(" ", 20, 20, TFTWIDTH - 40, (TFTHEIGHT / 3) - 40, 10, CYAN,
	WHITE, 0, 2, 2, 2);

	LCD_SetTextSize(2);
	LCD_SetTextColor(WHITE, CYAN);
	LCD_SetCursor(((TFTWIDTH - 20) / 3) + 15, 25);
	LCD_Printf("Health");
	LCD_SetCursor(((TFTWIDTH - 20) / 3) - 10, 45);
	LCD_Printf("Monitoring");
	LCD_SetCursor(((TFTWIDTH - 20) / 3) + 15, 65);
	LCD_Printf("System");

	//===============================================================//

	//===================== | Box 2 |===============================//

	LCD_FillRoundRect(15, (TFTHEIGHT / 3) + 15, TFTWIDTH - 35, 25, 5, RED);
	LCD_SetTextColor(WHITE, RED);
	LCD_SetCursor(20, (TFTHEIGHT / 3) + 20);
	LCD_SetTextSize(2);
	LCD_Printf("H.Rate: ");

	LCD_FillRoundRect(15, (TFTHEIGHT / 3) + 45, TFTWIDTH - 35, 25, 5, GREEN);
	LCD_SetTextColor(WHITE, GREEN);
	LCD_SetCursor(20, (TFTHEIGHT / 3) + 50);
	LCD_SetTextSize(2);
	LCD_Printf("SpO2: ");

	LCD_FillRoundRect(15, (TFTHEIGHT / 3) + 75, TFTWIDTH - 35, 25, 5, BLUE);
	LCD_SetTextColor(WHITE, BLUE);
	LCD_SetCursor(20, (TFTHEIGHT / 3) + 80);
	LCD_SetTextSize(2);
	LCD_Printf("Temp: ");

	//===============================================================//

	//========================= | Box 3 |============================//

	LCD_FillRect(20, (2 * (TFTHEIGHT / 3)) + 20, TFTWIDTH - 35, 70, WHITE);

	//===============================================================//

}

uint16_t temp2color(int degree, int lo, int hi) {
	uint8_t r, g, b;
	r = map(degree, lo, hi, 255, 0);
	g = 0;
	b = map(degree, lo, hi, 0, 255);
	return LCD_Color565(r, g, b);
}

void colorgradient(int x, int y, int w, int h, int percent) {
//	LCD_DrawRect(x, y, w, h, BLACK);
	for (int row = 1; row < h - 1; row++) {
		LCD_DrawFastHLine(x + 1, y + row, w - 2, temp2color(row, 0, h));
	}
}

void LCD_DrawGraph(uint16_t value) {

	graph_y_prevVal = graph_y;
//	uint16_t mapVar = adcArr[adcArrTail];
//	adcArrTail = (adcArrTail + 1) % adcArrLen;
	uint16_t mapVar = value;
	uint8_t graphRefreshRate = 0;
	if (mapVar > 3000)
		mapVar = 3000;
	else if (mapVar < 1000)
		mapVar = 1000;
	graph_y = map(mapVar, 1000, 3000, 243, 293);

	LCD_DrawLine(graph_x, graph_y0 + graph_y_prevVal,
			graph_x + graphRefreshRate_peak, graph_y0 + graph_y, RED);
//	LCD_DrawLine(graph_x, graph_y0 + graph_y_prevVal - 1,
//				graph_x + graphRefreshRate_peak, graph_y0 + graph_y - 1, RED);

	int a = graph_x;
	if (a + graphRefreshRate_peak >= graph_x2) {
		LCD_FillRect(20, (2 * (TFTHEIGHT / 3)) + 20, TFTWIDTH - 80, 70, WHITE);
		graph_x = graph_x0;
	} else {
		graph_x += graphRefreshRate_peak;
	}

}

uint16_t readECG() {
	uint16_t adc_value = 0;
	ret = adc_read(&adc_value, 1, 10);
	if (ret == BML_OK) {
//		uint8_t adcStr[20];
//		sprintf(adcStr, "ECG Value: %d\n\r", adc_value);
//		USART_WRITE(USART2, adcStr, strlen(adcStr), 10);
//		print("ECG Value: %d\n\r", adcVal);
	} else {
		USART_WRITE(USART2, "Error: ADC read\n\r", 17, 100);
	}
	return adc_value;
}

void displayTemperature() {
	if (tempReadFlag) {
//		print("Temp in Celsius: = 	%0.2f\n", temp);
		if ((temp > 0) && (temp < 100)) {
			LCD_SetTextColor(WHITE, BLUE);
			LCD_SetCursor(130, (TFTHEIGHT / 3) + 80);
			LCD_SetTextSize(2);
			LCD_Printf("%0.2f'C", temp);
		}
	} else {
//		USART_WRITE(USART2, "Error: Celcius read\n\r", 21, 100);
		LCD_SetTextColor(WHITE, BLUE);
		LCD_SetCursor(130, (TFTHEIGHT / 3) + 80);
		LCD_SetTextSize(2);
		LCD_Printf("  ERROR");
	}
}

void readTemperature() {
	ret = mlx90614_getObject1(&temp);
	if (ret == BML_OK) {
		tempReadFlag = true;
	} else {
		tempReadFlag = false;
	}
}

void readHnS() {
	if (hnsStrRecFlag) {
//		USART_WRITE(USART2, hStr, ida, 100);
		uint8_t a = 0, b = 0, ida = 0, idb = 0;
		while (hnsStrData[a] != 'H') {
			a++;
		}
		while (hnsStrData[b] != 'S') {
			b++;
		}
		a++;
		b++;
		while (hnsStrData[a] != '@') {
			hStr[ida] = hnsStrData[a];
			ida++;
			a++;
		}
		hStr[ida] = '\0';
		while (hnsStrData[b] != '@') {
			sStr[idb] = hnsStrData[b];
//			USART_WRITE(USART2, sStr[idb], 1, 10);
			idb++;
			b++;
		}
		sStr[idb] = '\0';
		hnsStrRecFlag = 0;
//		USART_WRITE(USART2, hStr, ida, 100);
//		USART_WRITE(USART2, '\n', 1, 10);
//		USART_WRITE(USART2, sStr, idb, 100);
//		USART_WRITE(USART2, '\n', 1, 10);

	}

	hVal = atof(hStr);
	sVal = atoi(sStr);

}

void displayHnS() {
	LCD_SetTextColor(WHITE, RED);
	LCD_SetCursor(155, (TFTHEIGHT / 3) + 20);
	LCD_SetTextSize(2);
	if (hVal < 100 && hVal > 0) {
		LCD_Printf("%0.2f", hVal);
	} else if (hVal == 0) {
		LCD_Printf(" %0.2f", hVal);
	}

	LCD_SetTextColor(WHITE, GREEN);
	LCD_SetCursor(175, (TFTHEIGHT / 3) + 50);
	LCD_SetTextSize(2);
	if (sVal > 10 && sVal < 100) {
		LCD_Printf("%d%%", sVal);
	} else {
		LCD_Printf(" 0%%");
	}
}

void ecgPeak() {
	LCD_DrawLine(graph_x, graph_y0 + 25, graph_x + graphRefreshRate_peak,
			graph_y0 + 25, RED);
	LCD_DrawLine(graph_x, graph_y0 + 26, graph_x + graphRefreshRate_peak,
							graph_y0 + 26, RED);
	graph_x += graphRefreshRate_peak;
	LCD_DrawLine(graph_x, graph_y0 + 25, graph_x + graphRefreshRate_peak,
			graph_y0 + 25, RED);
	LCD_DrawLine(graph_x, graph_y0 + 26, graph_x + graphRefreshRate_peak,
							graph_y0 + 26, RED);
	graph_x += graphRefreshRate_peak;
	LCD_DrawLine(graph_x, graph_y0 + 25, graph_x + graphRefreshRate_peak,
			graph_y0 + 25, RED);
	LCD_DrawLine(graph_x, graph_y0 + 26, graph_x + graphRefreshRate_peak,
							graph_y0 + 26, RED);
	graph_x += graphRefreshRate_peak;

	LCD_DrawLine(graph_x, graph_y0 + 25, graph_x + graphRefreshRate_peak,
			graph_y0 + 40, RED);
	LCD_DrawLine(graph_x, graph_y0 + 26, graph_x + graphRefreshRate_peak,
				graph_y0 + 41, RED);
	graph_x += graphRefreshRate_peak;
	LCD_DrawLine(graph_x, graph_y0 + 40, graph_x + graphRefreshRate_peak,
			graph_y0, RED);
	LCD_DrawLine(graph_x, graph_y0 + 41, graph_x + graphRefreshRate_peak,
				graph_y0 + 1, RED);
	graph_x += graphRefreshRate_peak;
	LCD_DrawLine(graph_x, graph_y0, graph_x + graphRefreshRate_peak,
			graph_y0 + 25, RED);
	LCD_DrawLine(graph_x, graph_y0 + 1, graph_x + graphRefreshRate_peak,
				graph_y0 + 26, RED);
	graph_x += graphRefreshRate_peak;

	LCD_DrawLine(graph_x, graph_y0 + 25, graph_x + graphRefreshRate_peak,
			graph_y0 + 25, RED);
	LCD_DrawLine(graph_x, graph_y0 + 26, graph_x + graphRefreshRate_peak,
							graph_y0 + 26, RED);
	graph_x += graphRefreshRate_peak;
	LCD_DrawLine(graph_x, graph_y0 + 25, graph_x + graphRefreshRate_peak,
			graph_y0 + 25, RED);
	LCD_DrawLine(graph_x, graph_y0 + 26, graph_x + graphRefreshRate_peak,
							graph_y0 + 26, RED);
	graph_x += graphRefreshRate_peak;
	LCD_DrawLine(graph_x, graph_y0 + 25, graph_x + graphRefreshRate_peak,
			graph_y0 + 25, RED);
	LCD_DrawLine(graph_x, graph_y0 + 26, graph_x + graphRefreshRate_peak,
							graph_y0 + 26, RED);
}

void ecgLine() {
	for (uint8_t i = 0; i < 3; i++) {
		LCD_DrawLine(graph_x, graph_y0 + 25, graph_x + graphRefreshRate_peak,
				graph_y0 + 25, RED);
		LCD_DrawLine(graph_x, graph_y0 + 26, graph_x + graphRefreshRate_peak,
				graph_y0 + 26, RED);
		graph_x += graphRefreshRate_peak;
		LCD_DrawLine(graph_x, graph_y0 + 25, graph_x + graphRefreshRate_peak,
				graph_y0 + 25, RED);
		LCD_DrawLine(graph_x, graph_y0 + 26, graph_x + graphRefreshRate_peak,
				graph_y0 + 26, RED);
		graph_x += graphRefreshRate_peak;
		LCD_DrawLine(graph_x, graph_y0 + 25, graph_x + graphRefreshRate_peak,
				graph_y0 + 25, RED);
		LCD_DrawLine(graph_x, graph_y0 + 26, graph_x + graphRefreshRate_peak,
				graph_y0 + 26, RED);
	}
}

void LCD_ECGAnimation() {
	ecgLine();

	int a = graph_x;
	if (a + graphRefreshRate_peak >= graph_x2 - 25) {
		LCD_FillRect(20, (2 * (TFTHEIGHT / 3)) + 20, TFTWIDTH - 80, 70, WHITE);
		graph_x = graph_x0;
	} else {
		graph_x += graphRefreshRate_peak;
	}

	ecgPeak();
}


//=======================| Interrupt Handlers |=============================//

void EXTI4_15_IRQHandler() {

	if (gpio_IT_CHK(11)) {
		for (int i = 500000; i > 0; i--)
			;
		gpio_IT_CLR(11);
		gpio11_callback();
	}
	if (gpio_IT_CHK(12)) {
		for (int i = 500000; i > 0; i--)
			;
		gpio_IT_CLR(12);
		gpio12_callback();
	}
}

void USART3_4_IRQHandler() {
	char chr;
	uint8_t i = 0;
	if ((USART3->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
		chr = (uint8_t) (USART3->RDR);
//			USART_WRITE(USART2, chr, 1, 10);
	}
	if (chr == '\n') {
		strcpy(hnsStrData, hnsStr);
//		print(hnsStrData);
//		USART_WRITE(USART2, '\n', 1, 10);
		memset(hnsStr, 0, idx);
		idx = 0;
		hnsStrRecFlag = 1;
	} else {
		hnsStr[idx++] = chr;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */

	ret = adc_en(100);
	if (ret != BML_OK) {
		USART_WRITE(USART2, "Error: ADC EN\n\r", 10, 100);
		//		while (1)
		//			;
	}
	mlx90614_setUnit(MLX90614_UNIT_C);

	gpio_settingfn();

	ret = mlx90614_init();
	if (ret != BML_OK) {
		print("Error in Init MLX\n\r");
		//		while (1)
		//			;
	}

	adc_config(sample_time_2, CONT_off, SCANDIR_off, AUTOFF_off, WAIT_off,
	DISCEN_on, OVRMOD_off);

	//	===============| TFT |=====================//
	LCD_Begin();
	LCD_SetRotation(0);
	//	LCD_FillScreen(BLACK);
	colorgradient(0, 0, TFTWIDTH, TFTHEIGHT, 50);

	LCD_SetTextSize(3);

	HMS_bgdisplay();

	LCD_SetTextColor(BLACK, WHITE);
	LCD_SetCursor(190, 240);
	LCD_SetTextSize(2);
	LCD_Printf("HR");

	//===========================================//

	mSec = HAL_GetTick();

	for (;;) {
		readTemperature();
		readHnS();
		dispHnS_flag = true;
		dispTemp_flag = true;

	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void const *argument) {
	/* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	for (;;) {
//		adcVal = readECG();
		if (adcArrHead == 0) {
			prevBeatVal = 2000;
		} else {
			prevBeatVal = adcArr[adcArrHead - 1];
		}
		adcArr[adcArrHead] = readECG();
		nextBeatVal = adcArr[adcArrHead];

		uint8_t adcStr[20];
		sprintf(adcStr, "ECG Value: %d\n\r", adcArr[adcArrHead]);
		USART_WRITE(USART2, adcStr, strlen(adcStr), 10);

		beatVal = nextBeatVal - prevBeatVal;

		if (beatVal > 800 && beatVal < 2500) {
//			USART_WRITE(USART2, "Beat!\n", 6, 10);
			hrVal++;
		}

		if (HAL_GetTick() - mSec >= 1000) {
			sec++;
			if (sec >= 60) {
				sec = 0;
				min++;
			}
			mSec = HAL_GetTick();
		}

		adcArrHead = (adcArrHead + 1) % adcArrLen;
		osDelay(1);
	}
	/* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void const *argument) {
	/* USER CODE BEGIN StartTask03 */
	/* Infinite loop */
	for (;;) {
//		print("In task3\n");
		if (dispTemp_flag) {
			displayTemperature();
			dispTemp_flag = false;
		}
		if (dispHnS_flag) {
			displayHnS();
			dispHnS_flag = false;
		}

		if (min - oneMin == 1) {
			print("HR: %d\n", hrVal);

			if (hrVal >= 150 && hrVal <= 20) {
				hrVal = 0;
			}

			LCD_SetTextColor(BLACK, WHITE);
			LCD_SetCursor(190, 270);
			LCD_SetTextSize(2);
			if (hrVal < 10)
				LCD_Printf(" %d", hrVal);
			else if(hrVal < 100)
				LCD_Printf("%d", hrVal);
			hrVal = 0;
			oneMin = min;
		}

//		LCD_DrawGraph(adcArr[adcArrTail]);
//		adcArrTail = (adcArrTail + 1) % adcArrLen;
		LCD_ECGAnimation();
		osDelay(10);
	}
	/* USER CODE END StartTask03 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
