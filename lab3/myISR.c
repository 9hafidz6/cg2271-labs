#include "MKL25Z4.h"                    // Device header

#define SWITCH 6 // PortD Pin 6
#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK(x) (1 << (x))

unsigned int counter = 0;

unsigned int count = 0;
unsigned int time = 240000;

//enumeration color_t to hold the possible colors of LED (red,green,blue)
typedef enum{
	RED,
	GREEN,
	BLUE
} color_t;

//control LED color using parameter color
void led_control(color_t color){
	//turn on RED
	if(color == RED){
		PTB->PCOR = MASK(RED_LED); //on red
		PTB->PSOR = MASK(GREEN_LED); //off green
		PTD->PSOR = MASK(BLUE_LED); //off blue
	}
	//turn on GREEN
	else if(color == GREEN){
		PTB->PCOR = MASK(GREEN_LED); //on green
		PTB->PSOR = MASK(RED_LED); //off red
		PTD->PSOR = MASK(BLUE_LED); //off blue
	}
	//turn on BLUE
	else if(color == BLUE){
		PTD->PCOR = MASK(BLUE_LED); //on blue
		PTB->PSOR = MASK(RED_LED); //off red
		PTB->PSOR = MASK(GREEN_LED); //off green
	}
	else{
		//do nothing 
	}
}

void initLED(void)
{
// Enable Clock to PORTB and PORTD
SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
// Configure MUX settings to make all 3 pins GPIO
PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
// Set Data Direction Registers for PortB and PortD
PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
PTD->PDDR |= MASK(BLUE_LED);
}

void initSwitch(){
	//enable clock for port D
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	// Select GPIO and enable pull-up resistors and interrupts on falling edges of pin connected to switch
	PORTD->PCR[SWITCH] |= PORT_PCR_MUX(1) |
												PORT_PCR_PS_MASK |
												PORT_PCR_PE_MASK |
												PORT_PCR_IRQC(0x0a); 
	// Set PORTD D Switch bit to input 
	PTD->PDDR &= ~MASK(SWITCH);
	// Enable Interrupt
	NVIC_SetPriority(PORTD_IRQn,128);
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	NVIC_EnableIRQ(PORTD_IRQn);
}


//delay for about 1 second
void delay(){
	while(count < time){
		count++;
	}
	count = 0;
}

void PORTD_IRQHandler(){
	// Clear pending IRQ
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	
	// Updating variables 
	if(PORTD->ISFR & MASK(SWITCH)){
		counter++;
		counter = counter % 3;
	}
	delay();
	// CLear INT flag
	PORTD->ISFR = 0xffffffff;
	
}

/* MAIN function */

int main(void){
	initSwitch();
	initLED();
	__enable_irq();
	
	while(1){
		if(counter == 0){
			led_control(RED);
		}
		else if(counter == 1){
			led_control(GREEN);
		}
		else if(counter == 2){
			led_control(BLUE);
		}		
		else {
			led_control (BLUE);
		}
	}
}
