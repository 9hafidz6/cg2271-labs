#include "MKL25Z4.h"                    // Device header

#define PTB0_Pin 0
#define PTB1_Pin 1

/* init PWM */
void initPWM(void){
	//enable the clock for port B
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// configure mux settings to make pinb 0 GPIO 
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	
	// configure mux settings to make pinb 1 GPIO
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	// To enable the clock for timer 1 (TPM1)
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

	//Turns off Sound initially
	TPM1->MOD = 0;
	TPM1_C0V =  0;
	
	// clears the SOPT2 to 0
	//selects the TPM clock source. 
	//Since 1 is selected, either MCGFLLCLK clock or MCGFLLCLK/2 is used.
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	// 
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	//
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

void delay (int counter) {
	// 2 second-ish
	for (int i = 0; i < counter; i ++) ;
}

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
delay(0xFFFFF);
delay(0xFFFFF);
	}
// main program
int main(void){
	//initialize PWM
	initPWM();

	while(1){
		//do something here
		//playStar();
		playSequence();
	}
}
