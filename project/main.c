/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"                    // Device header

#define PTB0_Pin 0
#define PTB1_Pin 1
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

#define LED_RED 2 //0b00000010
#define LED_MASK(x) (x & 0x06)
#define BIT0_MASK(x) (x & 0x01)

#define RED_LED 18 //port b pin 18
#define GREEN_LED 19 //port b pin 19
#define BLUE_LED 1 //port b pin 1
#define MASK(x) (1 << x)
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 
  // ...
  for (;;) {}
}
/* Delay routine */
static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__ASM("NOP");
		nof--;
	}
}

void offRGB(void){
	//turn off all leds
	PTB->PSOR = MASK(RED_LED);
	PTB->PSOR = MASK(GREEN_LED);
	PTD->PSOR = MASK(BLUE_LED);	
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

//enumeration state_t to hold the state of led, on or off
typedef enum{
	led_on,
	led_off
} state_t;

//control LED state using parameter color
void led_control(int led_mapping[][2], int m, state_t state){
	//on red led
	if(state == led_on){
		PTB->PCOR = MASK(RED_LED);
		PTD->PSOR = MASK(BLUE_LED);
		PTB->PSOR = MASK(GREEN_LED);
	}
	//off red led
	else if(state == led_off){
		PTB->PSOR = MASK(RED_LED);
		PTD->PSOR = MASK(BLUE_LED);
		PTB->PSOR = MASK(GREEN_LED);		
	}
}

// main program
int main(void){
	uint8_t rx_data = 0x69;
	int led_mapping[3][2];
	
	SystemCoreClockUpdate();
	initUART2(BAUD_RATE);
	InitGPIO();
	offRGB();
	
	while(1){
		//Rx
		rx_data = UART2_Receive_Poll();
		
		if(LED_MASK(rx_data) == LED_RED){
			if(BIT0_MASK(rx_data)){
				led_control(led_mapping, 1, led_on);
			}
			else
				led_control(led_mapping, 1, led_off);
		}
	}
}

