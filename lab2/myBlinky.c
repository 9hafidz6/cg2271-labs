#include "MKL25Z4.h"                    // Device header
#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK(x) (1 << (x))

unsigned int counter = 0;
unsigned int time = 2097152;

void InitGPIO(void)
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

//delay for about 1 second
void delay(){
	while(counter < time){
		counter++;
	}
	counter = 0;
}

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
		delay();
	}
	//turn on GREEN
	else if(color == GREEN){
		PTB->PCOR = MASK(GREEN_LED); //on green
		PTB->PSOR = MASK(RED_LED); //off red
		PTD->PSOR = MASK(BLUE_LED); //off blue
		delay();
	}
	//turn on BLUE
	else if(color == BLUE){
		PTD->PCOR = MASK(BLUE_LED); //on blue
		PTB->PSOR = MASK(RED_LED); //off red
		PTB->PSOR = MASK(GREEN_LED); //off green
		delay();
	}
	else{
		//do nothing 
	}
}

//Main function

int main(void){
	SystemCoreClockUpdate();
	InitGPIO();
	while(1){
		led_control(RED);
		led_control(GREEN);
		led_control(BLUE);		
	}
}
