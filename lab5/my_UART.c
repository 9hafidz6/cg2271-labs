#include "MKL25Z4.h"                    // Device header

#define PTB0_Pin 0
#define PTB1_Pin 1
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

/* Delay routine */
static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__ASM("NOP");
		nof--;
	}
}

/* Init UART2 */
void initUART2(uint32_t baud_rate)
{
	uint32_t divisor, bus_clock;
	
	// enable clock to UART and Port A
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// connect UART to pins for PTE22, PTE23
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	// ensure tx and rx are disabled before configuration
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	// Set baud rate to 4800 baud
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock/(baud_rate*16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	// No parity, 8 bits, two stop bits, other settings
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	// Enable transmitter and receiver
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
}
	
/* UART2 Transmit Poll */
void UART2_Transmit_Poll(uint8_t data) {
	// wait until transmit data register is empty
	while (!(UART2->S1 & UART_S1_TDRE_MASK));
	UART2->D = data;
}

uint8_t UART2_Receive_Poll(void) {
	// wait until receive data register is full
	while (!(UART2->S1 & UART_S1_RDRF_MASK));
	return UART2->D;
}

/* init PWM */
void initPWM(void){
	// Enable the clock for port B
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// Configure mux settings to make pinb 0 GPIO 
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	
	// Configure mux settings to make pinb 1 GPIO
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	// Enable the clock for timer 1 (TPM1)
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

	// Turns off sound initially
	TPM1->MOD = 0;
	TPM1_C0V =  0;
	
	// Either MCGFLLCLK clock or MCGFLLCLK/2 is used
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	// Prescaler = 128, LPTPM counter will increment on every LPTPM counter clock
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Mode selected: Edge-aligned PWM for high-true pulses 
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK)|(TPM_CnSC_ELSA_MASK)|(TPM_CnSC_MSB_MASK)|(TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1)|(TPM_CnSC_MSB(1)));
}

void buzzC (void) {
	// Frequency: 262
	TPM1->MOD = 143;
	TPM1_C0V =  42;
}

void buzzD (void) {
	// Frequency: 294
	TPM1->MOD = 127;
	TPM1_C0V =  38;
}

void buzzE (void) {
	// Frequency: 330
	TPM1->MOD = 113;
	TPM1_C0V =  33;
}

void buzzF (void) {
	// Frequency: 349 
	TPM1->MOD = 107;
	TPM1_C0V =  32;
}

void buzzG (void) {
	// Frequency: 392
	TPM1->MOD = 95;
	TPM1_C0V =  28;
}

void buzzA (void) {
	// Frequency: 440
	TPM1->MOD = 85;
	TPM1_C0V =  42;
}

void buzzB (void) {
	// Frequency: 494
	TPM1->MOD = 75;
	TPM1_C0V =  36;
}

void buzzStop (void) {
	// Frequency: 0
	TPM1->MOD = 0;
	TPM1_C0V =  0;
}

/*
void delay(int counter) {
	// 2 second-ish
	for (int i = 0; i < counter; i ++) ;
}
*/

void playSequence(void) {
			//Let me play you the song of my people
		buzzC();
		delay(0xAFFFF);
		buzzStop();
		delay(0xAFFFF);
		buzzD();
		delay(0xAFFFF);
		buzzStop();
		delay(0xAFFFF);
		buzzE();
		delay(0xAFFFF);
		buzzStop();
		delay(0xAFFFF);
		buzzF();
		delay(0xAFFFF);	
		buzzStop();
		delay(0xAFFFF);		
		buzzG();
		delay(0xAFFFF);
		buzzStop();
		delay(0xAFFFF);		
		buzzA();
		delay(0xAFFFF);
		buzzStop();
		delay(0xAFFFF);		
		buzzB();
		delay(0xAFFFF);
		buzzStop();
		delay(0xAFFFF);
	}

	void playStar() {
		buzzC();
		delay(0xAFFFF);	
		buzzStop();
		delay(0xFFFFF);	
		buzzC();
		delay(0xAFFFF);	
		buzzStop();
		delay(0xFFFFF);
		buzzG();
		delay(0xAFFFF);	
		buzzStop();
		delay(0xFFFFF);			
		buzzG();
		delay(0xAFFFF);	
		buzzStop();
		delay(0xFFFFF);	
		buzzA();
		delay(0xAFFFF);	
		buzzStop();
		delay(0xFFFFF);
		buzzA();
		delay(0xFFFFF);	
		buzzStop();
		delay(0xFFFFF);	
		buzzG ();
		delay(0xFFFFF);	
		buzzStop();
		delay(0xFFFFF);	
	}
// main program
int main(void){
	uint8_t rx_data = 0x69;
	
	SystemCoreClockUpdate();
	initUART2(BAUD_RATE);
	
	while(1){
		//Rx and Tx
		UART2_Transmit_Poll(0x69);
		delay(0x80000);
	}
}
