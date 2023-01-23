/*-------------------------------------------------------------------------
					Technika Mikroprocesorowa 2 - laboratorium
					Lab 6 - Ćwiczenie 1: wyzwalanie programowe przetwornika A/C - tryb wyzwalania automatycznego
					autor: Mariusz Sokołowski
					wersja: 10.10.2022r.
----------------------------------------------------------------------------*/

/************************************************************************
*	Includes
************************************************************************/
#include "MKL05Z4.h"
#include "ADC.h"
#include "klaw.h"
#include "frdm_bsp.h"
#include "lcd1602.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uart0.h"
#include "led.h"

/***********************************************************************
*	Defines
************************************************************************/
#define CR	0x0D
/***********************************************************************/



/***********************************************************************
*	Static variables
************************************************************************/

static char command[] = "temp";
static float adc_volt_coeff = ((float)(((float)2.91) / 4096) );			// Współczynnik korekcji wyniku, w stosunku do napięcia referencyjnego przetwornika
static uint8_t wynik_ok=0;
static float	wynik;
static uint8_t	kal_error;
static char display[]={0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};
static volatile uint8_t size = sizeof(display)/sizeof(display[0]);
static uint8_t x;
static volatile uint8_t s2 = 0;
static uint16_t temp, tempo;
static uint8_t rx_FULL=0;
static uint8_t too_long=0;
static char rx_buf[16];
static uint8_t rx_buf_pos=0;

/***********************************************************************/

/***********************************************************************
*	IRQ HANDLERS
************************************************************************/
void PORTA_IRQHandler(void)
{
	uint32_t buf;
	buf=PORTA->ISFR & (S2_MASK | S3_MASK);

	switch(buf)
	{
		case S2_MASK:	DELAY(10)
									if(!(PTA->PDIR&S2_MASK))		// Minimalizacja drgan zestyków
									{
										if(!(PTA->PDIR&S2_MASK))	// Minimalizacja drgan zestyków (c.d.)
										{
											s2 =1 ;
										}
									}
									break;
		case S3_MASK:	DELAY(10)
									if(!(PTA->PDIR&S3_MASK))		// Minimalizacja drgan zestyków
									{
										if(!(PTA->PDIR&S3_MASK))	// Minimalizacja drgan zestyków (c.d.)
										{
											s2=2;
										}
									}
									break;
	}	
	PORTA->ISFR |=  S2_MASK | S3_MASK;	// Kasowanie wszystkich bitów ISF
	NVIC_ClearPendingIRQ(PORTA_IRQn);
}

void UART0_IRQHandler()
{
	if(UART0->S1 & UART0_S1_RDRF_MASK)
	{
		tempo=UART0->D;	// Odczyt wartości z bufora odbiornika i skasowanie flagi RDRF
		if(!rx_FULL)
		{
			if(tempo!=CR)
			{
				if(!too_long)	// Jeśli za długi ciąg, ignoruj resztę znaków
				{
					rx_buf[rx_buf_pos] = tempo;	// Kompletuj komendę
					rx_buf_pos++;
					s2=1;
					if(rx_buf_pos==16)
						too_long=1;		// Za długi ciąg
				}
			}
			else
			{
				if(!too_long)	// Jeśli za długi ciąg, porzuć tablicę
					rx_buf[rx_buf_pos] = 0;
				rx_FULL=1;
			}
		}
	NVIC_EnableIRQ(UART0_IRQn);
	}
}

void ADC0_IRQHandler()
{	
	temp = ADC0->R[0];	// Odczyt danej i skasowanie flagi COCO
	if(!wynik_ok)				// Sprawdź, czy wynik skonsumowany przez pętlę główną
	{
		wynik = temp;			// Wyślij nową daną do pętli głównej
		wynik_ok=1;
	}
}

/************************************************************************/


int main (void)
{
	Klaw_Init();
	Klaw_S1();
	UART0_Init();
	LCD1602_Init();		 // Inicjalizacja wyświetlacza LCD
	LCD1602_Backlight(TRUE);				
	LCD1602_SetCursor(0,0);
	LCD1602_Print("Stacja Pogody");				
	LCD1602_SetCursor(0,1);
	LCD1602_Print("Wcisnij S2");
	LED_Init();
	
	kal_error=ADC_Init();				// Inicjalizacja i kalibracja przetwornika A/C
	if(kal_error)
	{	
		while(1);									// Klaibracja się nie powiodła
	}
	
	ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(8);		// Pierwsze wyzwolenie przetwornika ADC0 w kanale 8 i odblokowanie przerwania
	
	
	while(1)
	{
		if(s2 == 1){
			if(wynik_ok || (strcmp(rx_buf, command) == 0))
			{
					LCD1602_ClearAll();
					wynik = wynik*adc_volt_coeff*100;		// Dostosowanie wyniku do zakresu napięciowego
					sprintf(display,"Temp=%.1f*C",wynik);
					if((wynik > 24.0) && (wynik < 25.0)){
						PTB->PCOR = (1<<9);
						PTB->PSOR = (1 << 10 | 1<< 8);
					}
					else if(wynik > 25.0){
						PTB->PCOR = (1<<8);
						PTB->PSOR = (1 << 9 | 1<< 10);
					}
					else if(wynik < 24.0){
						PTB->PCOR = (1<<10);
						PTB->PSOR = (1 << 9 | 1<< 8);
					}
					LCD1602_SetCursor(0,0);
					LCD1602_Print(display);
					for(int i = 0; i < size; i++){
						while(!(UART0->S1 & UART0_S1_TDRE_MASK));	// Czy anadajnik gotowy?
						UART0->D = display[i];  // Wyślij daną z powrotem do komputera
					}
					while(!(UART0->S1 & UART0_S1_TDRE_MASK));	// Czy anadajnik gotowy?
					UART0->D = 0x0A;  // Wyślij daną z powrotem do komputera
					while(!(UART0->S1 & UART0_S1_TDRE_MASK));	// Czy anadajnik gotowy?
					UART0->D = 0x0D;  // Wyślij daną z powrotem do komputera
					wynik_ok=0;
					s2 = 0;
			}
			
		}
		if(s2 == 2){
			if(wynik_ok){
				LCD1602_ClearAll();
				LCD1602_SetCursor(0,0);
				LCD1602_Print("Stacja Pogody");				
				LCD1602_SetCursor(0,1);
				LCD1602_Print("Wcisnij S2");
				wynik_ok=0;
				s2 = 0;
			}
		}
	}
}
