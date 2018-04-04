/*-------------------------------------------------------------------------------------------
 ********************************************************************************************
 *-------------------------------------------------------------------------------------------
 *
 *				DATA LOGGER DE VARIABLES AMBIENTALES INTERNAS
 *							CIMEC CONICET - UTN FRP
 *								     2016
 *
 *						Polo, Franco		fjpolo@frp.utn.edu.ar
 *						Burgos, Sergio		sergioburgos@frp.utn.edu.ar
 *						Bre, Facundo		facubre@cimec.santafe-conicet.gov.ar
 *
 *	datalogger.c
 *
 *	Descripción:
 *
 *  Desarrollo del firmware de la placa base del data logger, constando de:
 *
 *  - Periféricos I2C:
 *  	a) HR y Tbs		HIH9131		0b0100111		0x27
 *  	b) Ev			TSL2563		0b0101001		0x29
 *  	c) Va			ADS			0b1001000		0x48
 *  	d) Tg			LM92		0b1001011		0x51
 *  	e) RTC			DS1703		0b1101000		0x68
 *
 *  - Periféricos OneWire@PD6
 *  	a) Ts01			MAX31850	ROM_Addr		0x3B184D8803DC4C8C
 *  	b) Ts02			MAX31850	ROM_Addr		0x3B0D4D8803DC4C3C
 *  	c) Ts03			MAX31850	ROM_Addr		0x3B4D4D8803DC4C49
 *  	d) Ts04			MAX31850	ROM_Addr		0x3B234D8803DC4C99
 *  	e) Ts05			MAX31850	ROM_Addr		0x3B374D8803DC4C1E
 *  	f) Ts06			MAX31850	ROM_Addr
 *
 *  - IHM
 *  	a) RESET		!RST
 *  	b) SW_SD		PC6
 *  	c) SW_ON		PC5
 *  	d) SW_1			PC7
 *  	e) WAKE			PF2
 *  	f) LEDON		PE0
 *  	g) LED1			PE1
 *  	h) LED2			PE2
 *
 *  - SD
 *  	a) SD_IN		PA6
 *  	b) SD_RX		PA4
 *  	c) SD_TX		PA5
 *  	d) SD_CLK		PA2
 *  	e) SD_FSS		PA3
 *
 *--------------------------------------------------------------------------------------------
 *********************************************************************************************
 *-------------------------------------------------------------------------------------------*/
/*****************************************************************************************************
 * Include
 ****************************************************************************************************/
// standard C
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
// inc
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
// driverlib
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
// datalogger
#include "datalogger/datalogger.h"
#include "datalogger/delay.h"
// utils
//#include "utils/uartstdio.h"

/*****************************************************************************************************
 * Function prototypes
 ****************************************************************************************************/
//void DataLoggingON(void);
//void DataLoggingOFF(void);

/*****************************************************************************************************
 * Defines
 ****************************************************************************************************/
// Loopback slave address
//#define SLAVE_ADDRESS 0x3C
// Tg
#define SLAVE_ADDRESS_TG 0x4B
#define TG_REG_READ 0x00
#define TG_REG_LOWPOW 0x01
// Ev
#define SLAVE_ADDRESS_EV 0x29
#define EV_REG_CONTROL 0x00
#define EV_WAKE 0x03
// RTC
#define SLAVE_ADDR_RTC 0x68
#define SEC 0x00
#define MIN 0x01
#define HRS 0x02
#define DAY 0x03
#define DATE 0x04
#define MONTH 0x05
#define YEAR 0x06
#define CNTRL 0x07
// Va ADC slave address
#define SLAVE_ADDR_VA 0x48
#define VA_REG_READ 0x00
// SWITCHES
#define SW_PORT 	GPIO_PORTC_BASE
#define SW_ON 		GPIO_PIN_5
#define SW_SD 		GPIO_PIN_6
#define SW_1 		GPIO_PIN_7
// LEDS
#define LED_PORT 	GPIO_PORTE_BASE
#define LED_ON 		GPIO_PIN_0
#define LED_1 		GPIO_PIN_1
#define LED_2 		GPIO_PIN_2
// Timer0
#define TOGGLE_FREQUENCY 1
// I2C3
#define GPIO_PD0_I2C3SCL        0x00030003
#define GPIO_PD1_I2C3SDA        0x00030403

/*****************************************************************************************************
 * Global variables
 ****************************************************************************************************/
//Datos a mantener durante la hibernacion
//0: sensor_flag
//1: hibernate_flag
//2: Tg y RTC
unsigned long ulNVData[3] = { 1, 0, 0};
//
//static unsigned long g_ulDataRx, MSB,LSB, Sign;
static unsigned long Tg_Raw, Tg_MSB,Tg_LSB, Tg_Sign;
unsigned char sec,min,hour,day,date,month,year;
double Tg=0;
//static stirng Dias[7] = {'Lunes', 'Martes', 'Miercoles', 'Jueves', 'Viernes', 'Sabado', 'Domingo'};
static uint32_t Ev_MSB,Ev_LSB;
static unsigned long Ev1,Ev2;
static double Ev;
static double PRECISION = 0.0001;
static int MAX_NUMBER_STRING_SIZE = 10;
char charEv[10];
char charTg[10];
char charTbs[10];
char charHR[10];
/*****************************************************************************************************
 * IntGPIOFHandler
 ****************************************************************************************************/
void IntGPIOFHandler (void){
	GPIOPinIntClear(GPIO_PORTF_BASE,GPIO_PIN_4);
	//Flag de que estoy en la interrupcion por GPIO
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x04);//
	SysCtlDelay(8000000);
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
	SysCtlDelay(4000000);
	ulNVData[2]=1;


}

/*****************************************************************************************************
 * dtoa
 ****************************************************************************************************/
char * dtoa(char *s, double n) {
	// handle special cases
	if (isnan(n)) {
		strcpy(s, "nan");
	} else if (isinf(n)) {
		strcpy(s, "inf");
	} else if (n == 0.0) {
		strcpy(s, "0");
	} else {
		int digit, m, m1;
		char *c = s;
		int neg = (n < 0);
		if (neg)
			n = -n;
		// calculate magnitude
		m = log10(n);
		int useExp = (m >= 14 || (neg && m >= 9) || m <= -9);
		if (neg)
			*(c++) = '-';
		// set up for scientific notation
		if (useExp) {
			if (m < 0)
				m -= 1.0;
			n = n / pow(10.0, m);
			m1 = m;
			m = 0;
		}
		if (m < 1.0) {
			m = 0;
		}
		// convert the number
		while (n > PRECISION || m >= 0) {
			double weight = pow(10.0, m);
			if (weight > 0 && !isinf(weight)) {
				digit = floor(n / weight);
				n -= (digit * weight);
				*(c++) = '0' + digit;
			}
			if (m == 0 && n > 0)
				*(c++) = '.';
			m--;
		}
		if (useExp) {
			// convert the exponent
			int i, j;
			*(c++) = 'e';
			if (m1 > 0) {
				*(c++) = '+';
			} else {
				*(c++) = '-';
				m1 = -m1;
			}
			m = 0;
			while (m1 > 0) {
				*(c++) = '0' + m1 % 10;
				m1 /= 10;
				m++;
			}
			c -= m;
			for (i = 0, j = m-1; i<j; i++, j--) {
				// swap without temporary
				c[i] ^= c[j];
				c[j] ^= c[i];
				c[i] ^= c[j];
			}
			c += m;
		}
		*(c) = '\0';
	}
	return s;
}


/*****************************************************************************************************
 * main
 *
 * Using DS1307 clock and calendar
 * Adding Tg
 * Adding Ev
 ****************************************************************************************************/
void main(void)
{
	//Init
	//Init();
	Initialize();
	// Set the clocking to run directly from the external crystal/oscillator.
	//SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);
	// Initialize I2C module 0
	InitI2C3();

	// TODO Set time and date
	//SetTimeDate(00,02,00,1,20,10,16);
	//
	while(1)
	{
		/*
		delayMS(1000);
		//
		// RTC
		//
		sec = GetClock(SEC);
		min = GetClock(MIN);
		hour = GetClock(HRS);
		date = GetClock(DATE);
		month = GetClock(MONTH);
		year = GetClock(YEAR);
		//SysCtlDelay(SysCtlClockGet()/10*3);
		//
		// Tg
		//
		I2CMasterSlaveAddrSet(I2C3_BASE,SLAVE_ADDRESS_TG, false);
		I2CMasterDataPut(I2C3_BASE, TG_REG_READ);
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		while(!I2CMasterBusy(I2C3_BASE));
		while(I2CMasterBusy(I2C3_BASE));
		I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDRESS_TG, true);
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
		while(!I2CMasterBusy(I2C3_BASE));
		while(I2CMasterBusy(I2C3_BASE));
		// Most significant bits
		//Tg_MSB = I2CMasterErr(I2C3_MASTER_BASE);
		Tg_MSB = I2CMasterDataGet(I2C3_BASE) << 8;
		// Less significant bits
		I2CMasterControl (I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		while(!I2CMasterBusy(I2C3_BASE));
		while(I2CMasterBusy(I2C3_BASE));
		Tg_LSB = I2CMasterDataGet(I2C3_BASE);
		// Low power mode on
		I2CMasterSlaveAddrSet(I2C3_BASE,SLAVE_ADDRESS_TG, false);
		I2CMasterDataPut(I2C3_BASE, TG_REG_LOWPOW);
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		while(!I2CMasterBusy(I2C3_BASE));
		//while(I2CMasterBusy(I2C3_BASE));
		// Final value
		Tg_Raw = (Tg_MSB + Tg_LSB) >> 3;
		Tg_Sign = Tg_Raw & 0x1000;
		Tg = Tg_Raw*0.0625;
		*/
		//
		// Ev
		//
		// A byte sent to the TSL256x with the most ignificant bit (MSB)
		// equal to 1 will be interpreted as a COMMAND byte
		//
		// The lower four bits of the COMMAND byte form the register
		// select address
		//
		/* ADDRESS RESISTER NAME REGISTER FUNCTION
		 *  −− COMMAND 			Specifies register address
		 *  0h CONTROL 			Control of basic functions
		 *  1h TIMING 			Integration time/gain control
		 *  2h THRESHLOWLOW 		Low byte of low interrupt threshold
		 *  3h THRESHLOWHIGH 	High byte of low interrupt threshold
		 *  4h THRESHHIGHLOW 	Low byte of high interrupt threshold
		 *  5h THRESHHIGHHIGH 	High byte of high interrupt threshold
		 *  6h INTERRUPT 		Interrupt control
		 *  7h −− 				Reserved
		 *  8h CRC 				Factory test — not a user register
		 *  9h −− 				Reserved
		 *  Ah ID 				Part number/ Rev ID
		 *  Bh −− 				Reserved
		 *  Ch DATA0LOW 			Low byte of ADC channel 0
		 *  Dh DATA0HIGH 		High byte of ADC channel 0
		 *  Eh DATA1LOW 			Low byte of ADC channel 1
		 *  Fh DATA1HIGH 		High byte of ADC channel 1
		 *  */
		I2CMasterSlaveAddrSet(I2C3_BASE,SLAVE_ADDRESS_EV, false);
		I2CMasterDataPut(I2C3_BASE, 0x81);
		I2CMasterDataPut(I2C3_BASE, 0x11);
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		while(!I2CMasterBusy(I2C3_BASE));
		while(I2CMasterBusy(I2C3_BASE));
		delayUS(110);
		//ADC0
		Ev_LSB = I2CReceive(SLAVE_ADDRESS_EV, 0x8C);
		Ev_MSB = I2CReceive(SLAVE_ADDRESS_EV, 0x8D);
		Ev1 = (Ev_MSB << 8) + Ev_LSB;
		//ADC1
		Ev_LSB = I2CReceive(SLAVE_ADDRESS_EV, 0x8E);
		Ev_MSB = I2CReceive(SLAVE_ADDRESS_EV, 0x8F);
		Ev2 = (Ev_MSB << 8) + Ev_LSB;
		// LowPow
		I2CMasterSlaveAddrSet(I2C3_BASE,SLAVE_ADDRESS_EV, false);
		I2CMasterDataPut(I2C3_BASE, 0x81);
		I2CMasterDataPut(I2C3_BASE, 0x00);
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		while(!I2CMasterBusy(I2C3_BASE));
		while(I2CMasterBusy(I2C3_BASE));
		// Final value
		if(Ev2/Ev1 < 0.125)
		{
			Ev = (0.0304*Ev1)-(0.0272*(Ev2/Ev1));
		}
		else if(Ev2/Ev1 < 0.250)
		{
			Ev = (0.0325*Ev1)-(0.0440*(Ev2/Ev1));
		}
		else if(Ev2/Ev1 < 0.375)
		{
			Ev = (0.351*Ev1)-(0.0544*(Ev2/Ev1));
		}
		else if(Ev2/Ev1 < 0.50)
		{
			Ev = (0.0382*Ev1)-(0.0624*(Ev2/Ev1));
		}
		else if(Ev2/Ev1 < 0.61)
		{
			Ev = (0.0224*Ev1) - (0.031*Ev2);
		}
		else if(Ev2/Ev1 < 0.80){
			Ev = (0.0128*Ev1) - (0.0153*Ev2);
		}
		else if(Ev2/Ev1 <= 1.30){
			Ev = (0.00146*Ev1) - (0.00112*Ev2);
		}
		else Ev = 0;
		Ev = Ev*16;
		delayMS(1);
		//
		double a = 55.35;
		dtoa(charTg, Tg);
		dtoa(charEv, Ev);
		delayMS(500);

	}
}
