
/************************************************************************
*	Includes
************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "MKL05Z4.h"
#include "ADC.h"
#include "klaw.h"
#include "frdm_bsp.h"
#include "lcd1602.h"
#include "uart0.h"
#include "led.h"
/************************************************************************/

/***********************************************************************
*	Defines
************************************************************************/
#define CR					   	0x0D
#define LF  				   	0x0A
#define RX_BUFF_LEN 	 	16
#define LED_BLUE				( 1<<10 )
#define LED_RED					( 1<<8 )
#define LED_GREEN				( 1<<9 )
#define adc_volt_coeff 	((float)(((float)2.91) / 4096) )
/***********************************************************************/



/***********************************************************************
*	Static variables
************************************************************************/

static char command[] = "temp";
static uint8_t wynik_ok = 0;
static float	wynik;
static uint8_t	kal_error;
static volatile uint8_t s2 = 0;
static uint16_t temp, tempo;
static uint8_t rx_FULL=0;
static uint8_t too_long=0;
static char rx_buf[RX_BUFF_LEN];
static uint8_t rx_buf_pos=0;

/***********************************************************************/




/***********************************************************************
*	Static functions declarations
************************************************************************/
static void send(char* data, uint8_t* size);
static void setDiodes(float* var);


static inline void setGR(void);
static inline void setRED(void);
static inline void setGR(void);
static inline void setBLUE(void);
/************************************************************************/



/***********************************************************************
*	IRQ HANDLERS
************************************************************************/
void PORTA_IRQHandler(void)
{
	uint32_t buf;
	buf=PORTA->ISFR & (S2_MASK | S3_MASK);

	switch(buf)
	{
		case S2_MASK:	
			DELAY(10)
			if(!(PTA->PDIR&S2_MASK))		
			{
				if(!(PTA->PDIR&S2_MASK))
				{
					s2 =1 ;
				}
			}
			break;
		case S3_MASK:	
			DELAY(10)
			if(!(PTA->PDIR&S3_MASK))		
			{
				if(!(PTA->PDIR&S3_MASK))
				{
					s2=2;
				}
			}
			break;
	}	
	PORTA->ISFR |=  S2_MASK | S3_MASK;
	NVIC_ClearPendingIRQ(PORTA_IRQn);
}

void UART0_IRQHandler(void)
{
	if(UART0->S1 & UART0_S1_RDRF_MASK)
	{
		tempo=UART0->D;	
		if(!rx_FULL)
		{
			if(tempo!=CR)
			{
				if(!too_long)
				{
					rx_buf[rx_buf_pos] = tempo;	
					rx_buf_pos++;
					s2=1;
					if(rx_buf_pos==16)
						too_long=1;		
				}
			}
			else
			{
				if(!too_long)	
					rx_buf[rx_buf_pos] = 0;
				rx_FULL=1;
			}
		}
	NVIC_EnableIRQ(UART0_IRQn);
	}
}

void ADC0_IRQHandler(void)
{	
	temp = ADC0->R[0];	
	if(!wynik_ok)				
	{
		wynik = temp;			
		wynik_ok=1;
	}
}

/************************************************************************/


int main (void)
{
	char display[]={0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};
	uint8_t size = sizeof(display)/sizeof(display[0]);
	Klaw_Init();
	Klaw_S1();
	UART0_Init();
	LCD1602_Init();		 
	LCD1602_Backlight(TRUE);				
	LCD1602_SetCursor(0,0);
	LCD1602_Print("Stacja Pogody");				
	LCD1602_SetCursor(0,1);
	LCD1602_Print("Wcisnij S2");
	LED_Init();
	
	kal_error=ADC_Init();			
	if(kal_error)
	{	
		while(1);									
	}
	
	ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(8);		
	
	
	while(1)
	{
		if(s2 == 1){
			if(wynik_ok || (strcmp(rx_buf, command) == 0))
			{
				LCD1602_ClearAll();
				wynik = wynik*adc_volt_coeff*100;	
				sprintf(display,"Temp=%.1f*C",wynik);
				send(display, &size);
				setDiodes(&wynik);
				LCD1602_SetCursor(0,0);
				LCD1602_Print(display);
				
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



/***********************************************************************
*	Static functions definitions
************************************************************************/

/************************************************************************
params: data -> pointer to first element of an array which will be send
				size -> pointer to size of the array

returns: void
************************************************************************/
static void send(char* data, uint8_t* size)
{
	for(int i = 0; i < *size; i++){
		while(!(UART0->S1 & UART0_S1_TDRE_MASK));	
		UART0->D = data[i];
	}
	while(!(UART0->S1 & UART0_S1_TDRE_MASK));	
	UART0->D = LF;  
	while(!(UART0->S1 & UART0_S1_TDRE_MASK));
	UART0->D = CR;
}


/************************************************************************
params: var -> pointer to float value of ADC read

returns: void
************************************************************************/
static void setDiodes(float* var)
{
	if((*var > 24.0) && (*var < 25.0)){
		setGR();
	}
	else if(*var > 25.0){
		setRED();
	}
	else if(*var < 24.0){
		setBLUE();
	}
}


/************************************************************************
params: void

returns: void
************************************************************************/
static inline void setGR(void)
{
	PTB->PCOR = LED_GREEN;
	PTB->PSOR = (LED_BLUE | LED_RED);
}


/************************************************************************
params: void

returns: void
************************************************************************/
static inline void setRED(void)
{
	PTB->PCOR = (LED_RED);
	PTB->PSOR = (LED_GREEN | LED_BLUE);
}


/************************************************************************
params: void

returns: void
************************************************************************/
static inline void setBLUE(void)
{
	PTB->PCOR = (LED_BLUE);
	PTB->PSOR = (LED_GREEN | LED_RED);
}

/************************************************************************/
