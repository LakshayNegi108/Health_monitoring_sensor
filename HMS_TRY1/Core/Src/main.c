#include "main.h"

#include "BML_DEF.h"
#include "rcc.h"
#include "gpio.h"
#include "uart.h"
#include "adc.h"
#include "i2c.h"
#include "MLX90614.h"
#include "st7783.h"

BML_StatusTypeDef ret;

#define SCL_PORT	PORTB
#define SCL_PIN		8

#define SDA_PORT	PORTB
#define SDA_PIN		9

#define graphRefreshRate	3

uint8_t hmsParamVar = 0;
uint16_t adcVal = 0;
float temp = 0, ambient = 0;

const int graph_x0 = 30;
const int graph_y0 = (2 * (TFTHEIGHT / 3)) + 30;
const int graph_x1 = 30;
const int graph_y1 = graph_y0 + 50;
const int graph_x2 = graph_x0 + TFTWIDTH - 55;
const int graph_y2 = graph_y0;
const int graph_x3 = graph_x0 + TFTWIDTH - 55;
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

void gpio_settingfn(void);
void gpio11_callback(void);
void gpio12_callback(void);
void HMS_bgdisplay(void);

uint16_t temp2color(int degree, int lo, int hi);
void colorgradient(int x, int y, int w, int h, int percent);
void LCD_DrawGraph(uint16_t value);

void readECG();
void readTemperature();
void readHnS();

int main(void) {

	RCC_CONFIG_48MHZ();
	uart_print_config(9600);
	USART_INIT(USART3, PORTC, PORTC, 10, 11, 9600);
	USART_IT_EN(USART3, 0, 1, 0);
	print("Working\n");

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

	adc_config(sample_time_7, CONT_off, SCANDIR_off, AUTOFF_off, WAIT_off,
	DISCEN_on, OVRMOD_off);

//	===============| TFT |=====================//
	LCD_Begin();
	LCD_SetRotation(0);
//	LCD_FillScreen(BLACK);
	colorgradient(0, 0, TFTWIDTH, TFTHEIGHT, 50);

	LCD_SetTextSize(3);
//	LCD_Printf("Working");
//	LCD_DrawPixel(20, 20, RED);
//	LCD_DrawPixel(20, TFTHEIGHT - 20, GREEN);
//	LCD_DrawPixel(TFTWIDTH - 20, 20, YELLOW);
//	LCD_DrawPixel(TFTWIDTH - 20, TFTHEIGHT - 20, BLUE);
//	LCD_DrawPixel(TFTWIDTH / 2, TFTHEIGHT / 2, WHITE);

//	HMS_bgdisplay();

	//===========================================//

	while (1) {

//		print("---\n");
		readECG();
//		readTemperature();
//		readHnS();

//		Delay(200);

//		LCD_DrawGraph(adcVal);
	}
	return 0;
}

//===================| Functions |=============================//

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
	uint16_t mapVar = adcVal;
	if (mapVar > 3000)
		mapVar = 3000;
	else if (mapVar < 1000)
		mapVar = 1000;
	graph_y = map(mapVar, 1000, 3000, 243, 293);

	for (int i = 0; i < graphRefreshRate; i++) {
		LCD_DrawFastVLine(graph_x + i + 1, graph_y0, graph_y1 - graph_y0,
		BLACK);
		LCD_DrawFastVLine(graph_x + i, graph_y0, graph_y1 - graph_y0, WHITE);
	}

//	LCD_DrawPixel(graph_x, graph_y0 + graph_y, RED);
//	LCD_DrawFastVLine(graph_x, graph_y0 + graph_y, graph_y1 - graph_y0 - graph_y, RED);
	LCD_DrawLine(graph_x, graph_y0 + graph_y_prevVal,
			graph_x + graphRefreshRate, graph_y0 + graph_y, RED);

	if (graph_x >= graph_x2) {
//		LCD_FillRect(20, (2 * (TFTHEIGHT / 3)) + 20, TFTWIDTH - 35, 70, WHITE);
		graph_x = graph_x0;
	} else {
		graph_x += graphRefreshRate;
	}

}

void readECG() {
	ret = adc_read(&adcVal, 1, 10);
	if (ret == BML_OK) {
		print("ECG Value: %d\n\r", adcVal);
	} else {
		USART_WRITE(USART2, "Error: ADC read\n\r", 17, 100);
	}
}

void readTemperature() {
	ret = mlx90614_getObject1(&temp);
	if (ret == BML_OK) {
//		print("Temp in Celsius: = 	%0.2f		", temp);
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

void readHnS() {

	float hVal = 0;
	int sVal = 0;

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
	}
	else{
		LCD_Printf(" 0%%");
	}

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
