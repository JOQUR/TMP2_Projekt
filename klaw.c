#include "klaw.h"

void Klaw_Init(void)
{
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;		// Wlaczenie portu A
	PORTA->PCR[S2] |= PORT_PCR_MUX(1);
	PORTA->PCR[S2] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
}
void Klaw_S1(void)
{
	PORTA -> PCR[S2] |= PORT_PCR_IRQC(0xa);		//0x8 - poziom "0"; 0x9 - zbocze narastające; 0xa - zbocze opadające; 0xb - obydwa zbocza
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}
