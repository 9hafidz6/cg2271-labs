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
#include "MKL25Z4.h"                    // Device header

#define PTB0_Pin 0
#define PTB1_Pin 1
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

/*----------------------------------------------------------*/
//lab8 part 2
#define LED_GREEN 4
#define LED_RED 2
#define LED_MASK(x) (x & 0x06)
#define BIT0_MASK(x) (x & 0x01)
/*----------------------------------------------------------*/

#define RED_LED 18 //port b pin 18
#define GREEN_LED 19 //port b pin 19
#define BLUE_LED 1 //port b pin 1
#define MASK(x) (1 << x)

#define Q_SIZE (32)

unsigned int counter = 0;
unsigned int count = 0;
unsigned int time = 240000;

uint8_t rx_data = 0x69;

//osSemaphoreId_t myGREENSem;
//osSemaphoreId_t myREDSem;
//const osThreadAttr_t thread_attr = {.priority = osPriorityNormal1};

//osThreadId_t REDLED_id, GREENLED_id, BLUELED_id, control_id;

osEventFlagsId_t REDLED_id, GREENLED_id, BLUELED_id, control_id;

/* Delay routine */
static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__ASM("NOP");
		nof--;
	}
}

typedef struct{
	unsigned char Data[Q_SIZE];
	unsigned int Head; // points to oldest data element
	unsigned int Tail; // points to next free space
	unsigned int Size; // quantity of elements in queue
} Q_T;
Q_T tx_q, rx_q;

void Q_Init(Q_T * q) {
	unsigned int i;
	for (i=0; i<Q_SIZE; i++)
		q->Data[i] = 0; // to simplify our lives when debugging
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
}

int Q_Empty(Q_T * q) {
	return q->Size == 0;
}

int Q_Full(Q_T * q) {
	return q->Size == Q_SIZE;
}

int Q_Enqueue(Q_T * q, unsigned char d) {
	// What if queue is full?
	if (!Q_Full(q)) {
		q->Data[q->Tail++] = d;
		q->Tail %= Q_SIZE;
		q->Size++;
		return 1; // success
	}else
		return 0; // failure
}
unsigned char Q_Dequeue(Q_T * q) {
	// Must check to see if queue is empty before dequeueing
	unsigned char t=0;
	if (!Q_Empty(q)) {
		t = q->Data[q->Head];
		q->Data[q->Head++] = 0; // to simplify debugging
		q->Head %= Q_SIZE;
		q->Size--;
	}
	return t;
}


//enumeration state_t to hold the state of led, on or off
typedef enum{
	led_on,
	led_off
} state_t;

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
	
	// ensure rx are disabled before configuration
	UART2->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
	
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
	UART2->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	UART2->C2 |= UART_C2_RIE_MASK;
}

//delay for about 1 second
void delay1(){
	while(count < time){
		count++;
	}
	count = 0;
}

void offRGB(void){
	//turn off all leds
	PTB->PSOR = MASK(RED_LED);
	PTB->PSOR = MASK(GREEN_LED);
	PTD->PSOR = MASK(BLUE_LED);	
}

//control LED state using parameter color
void led_control(int LED_COLOR, state_t state){
	//RED_LED
	if(LED_COLOR == RED_LED){
		if(state == led_on){
			PTB->PCOR = MASK(RED_LED);
		}
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

void control_thread (void *argument) {
  // ...
  for (;;) {
//		osThreadFlagsSet(REDLED_id,0x0000001);
//		osThreadFlagsSet(GREENLED_id,0x0000001);		
//		osThreadFlagsSet(BLUELED_id,0x0000001);		

		osEventFlagsSet(GREENLED_id,0x0000001);
		osDelay(1000);		
		osEventFlagsSet(BLUELED_id,0x0000001);
		osDelay(1000);		
		osEventFlagsSet(REDLED_id,0x0000001);
		osDelay(1000);		
	}
}

void led_green_thread (void *argument) {
  // ...
  for (;;) {		
		//osThreadFlagsWait(0x000000001, osFlagsWaitAny, osWaitForever);
		osEventFlagsWait(GREENLED_id, 0x000000001, osFlagsWaitAny, osWaitForever);
		led_control(GREEN_LED,led_on);
		osDelay(1000);
		led_control(GREEN_LED,led_off);
		osDelay(1000);
		
	}
}

void led_red_thread (void *argument) {
  // ...
  for (;;) {
		//osThreadFlagsWait(0x0000001, osFlagsWaitAny, osWaitForever);
		osEventFlagsWait(REDLED_id, 0x000000001, osFlagsWaitAny, osWaitForever);
		led_control(RED_LED,led_on);
		osDelay(1000);
		led_control(RED_LED,led_off);
		osDelay(1000);		
	}
}

void led_blue_thread (void *argument) {
  // ...
  for (;;) {
		//osThreadFlagsWait(0x0000001, osFlagsWaitAny, osWaitForever);
		osEventFlagsWait(BLUELED_id, 0x000000001, osFlagsWaitAny, osWaitForever);
		led_control(BLUE_LED,led_on);
		osDelay(1000);
		led_control(BLUE_LED,led_off);
		osDelay(1000);		
	}
}

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);

	//if receive any data 
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		rx_data = UART2->D;
		if(LED_MASK(rx_data) == LED_RED){
			if(BIT0_MASK(rx_data)){
				//
			}
		}
	}
	if(LED_MASK(rx_data) == LED_GREEN){
		if(BIT0_MASK(rx_data)){
			//
		}
	}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	InitGPIO();
	initUART2(BAUD_RATE);
	offRGB();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	control_id = osThreadNew(control_thread, NULL, NULL); 
	REDLED_id = osThreadNew(led_red_thread, NULL, NULL);    // Create application main thread
	GREENLED_id =osThreadNew(led_green_thread, NULL, NULL);
	BLUELED_id =osThreadNew(led_blue_thread, NULL, NULL);	
  osKernelStart();                      // Start thread execution
  for (;;) {}
}