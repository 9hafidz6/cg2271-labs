/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 ---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

/*----------------------------------------------------------------------------
 * Application main thread
 ---------------------------------------------------------------------------*/

#define SWITCH 6 // PortD Pin 6
#define RED_LED 18 //port b pin 18
#define GREEN_LED 19 //port b pin 19
#define BLUE_LED 1 //port D pin 1
#define MASK(x) (1 << x)

unsigned int counter = 0;
unsigned int count = 0;
unsigned int time = 240000;

osSemaphoreId_t mySem;
const osThreadAttr_t thread_attr = {.priority = osPriorityNormal1};

/* Delay routine */
static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__ASM("NOP");
		nof--;
	}
}

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
void delay1(){
	while(count < time){
		count++;
	}
	count = 0;
}
/*
void PORTD_IRQHandler(){
	// Clear pending IRQ
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	
	delay(0x80000);
	osSemaphoreRelease(mySem);
	
	// CLear INT flag
	PORTD->ISFR = 0xffffffff;
	
}
*/
void offRGB(void){
	//turn off all leds
	PTB->PSOR = MASK(RED_LED);
	PTB->PSOR = MASK(GREEN_LED);
	PTD->PSOR = MASK(BLUE_LED);	
}

//enumeration state_t to hold the state of led, on or off
typedef enum{
	led_on,
	led_off
} state_t;

//control LED state using parameter color
void led_control(int LED_COLOR, state_t state){
	//RED_LED
	if(LED_COLOR == RED_LED){
		if(state == led_on){
			PTB->PCOR = MASK(RED_LED);		}
		else if(state == led_off){
			offRGB();
		}
	}
	//GREEN_LED
	if(LED_COLOR == GREEN_LED){
		if(state == led_on){
			PTB->PCOR = MASK(GREEN_LED);
		}
		else if(state == led_off){
			offRGB();
		}
	}
	//BLUE_LED
	if(LED_COLOR == BLUE_LED){
		if(state == led_on){
			PTD->PCOR = MASK(BLUE_LED);	
		}
		else if(state == led_off){
			offRGB();
		}
	}
}

void led_green_thread (void *argument) {
  // ...
  for (;;) {
		osSemaphoreAcquire(mySem, osWaitForever);
		
		led_control(GREEN_LED,led_on);
		osDelay(1000);
		//delay(0x80000);
		led_control(GREEN_LED,led_off);
		osDelay(1000);
		//delay(0x80000);
		
		//osSemaphoreRelease(mySem);
	}
}

void led_red_thread (void *argument) {
  // ...
  for (;;) {
		osSemaphoreAcquire(mySem, osWaitForever);
		
		led_control(RED_LED,led_on);
		osDelay(1000);
		//delay(0x80000);
		led_control(RED_LED,led_off);
		osDelay(1000);
		//delay(0x80000);
		
		//osSemaphoreRelease(mySem);
	}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	//initSwitch();
	InitGPIO();
	offRGB();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	mySem = osSemaphoreNew(1,0,NULL);
  osThreadNew(led_red_thread, NULL, &thread_attr);    // Create application main thread
	osThreadNew(led_red_thread, NULL, NULL);    // Create application main thread
	//osThreadNew(led_green_thread, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}