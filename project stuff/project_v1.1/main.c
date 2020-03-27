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
/*----------------------------------------------------------*/

#define Q_SIZE (32)

unsigned int counter = 0;
unsigned int count = 0;
unsigned int time = 240000;

uint8_t rx_data = 0x69;

//osSemaphoreId_t myGREENSem;
//osSemaphoreId_t myREDSem;
//const osThreadAttr_t thread_attr = {.priority = osPriorityNormal1};

/*--------------------------------------------------------------------------------------------------*/
/* Delay routine */
static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__ASM("NOP");
		nof--;
	}
}

//delay for about 1 second
void delay1(){
	while(count < time){
		count++;
	}
	count = 0;
}

/*--------------------------------------------------------------------------------------------------*/
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
/*--------------------------------------------------------------------------------------------------*/

void InitGPIO(void)
{
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK));
	
	// Configure MUX settings to make all 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[11] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[11] |= PORT_PCR_MUX(1);
	PORTC->PCR[10] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[10] |= PORT_PCR_MUX(1);
	PORTC->PCR[6] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[6] |= PORT_PCR_MUX(1);
	PORTC->PCR[5] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[5] |= PORT_PCR_MUX(1);
	PORTC->PCR[4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[4] |= PORT_PCR_MUX(1);
	PORTC->PCR[3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[3] |= PORT_PCR_MUX(1);
	PORTC->PCR[0] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[0] |= PORT_PCR_MUX(1);
	PORTC->PCR[7] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[7] |= PORT_PCR_MUX(1);
	
	// Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTC->PDDR |= (MASK(11)|MASK(10)|MASK(6)|MASK(5)|MASK(4)|MASK(3)|MASK(0)|MASK(7));
	PTD->PDDR |= MASK(BLUE_LED);
}
/*--------------------------------------------------------------------------------------------------*/

/* Init UART2 */
void initUART2(uint32_t baud_rate)
{
	uint32_t divisor, bus_clock;
	
	// Enable clock to UART and Port E
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// Connect UART to pins for PTE22, PTE23
	PORTE->PCR[22] = PORT_PCR_MUX(4);
	PORTE->PCR[23] = PORT_PCR_MUX(4);
	
	// Ensure TX and RX are disabled before configuration
	UART2->C2 &= ~(UARTLP_C2_TE_MASK | UARTLP_C2_RE_MASK);
	
	// Set baud rate to 4800 baud
	bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
	divisor = bus_clock/(baud_rate*16);
	UART2->BDH = UART_BDH_SBR(divisor>>8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	// No parity, 8 bits, two stop bits, other settings;
	UART2->C1 = UART2->S2 = UART2->C3 = 0;
	
	// Enable transmitter and receiver
	UART2->C2 = UART_C2_RE_MASK; // Not UART_C2_TE_MASK
	
	// Enable Interrupt
	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= UART_C2_RIE_MASK; // Not UART_C2_TIE_MASK
}
/*--------------------------------------------------------------------------------------------------*/

/* init PWM */
void initPWM(void){
	// Enable Clock to PORTB
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

	// Configure MUX settings to enable TPM
  	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
 	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);

	// Enable Clock to TPM
	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK;

	// Select the MCGFLLCLK clock or MCGPLLCLK/2
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	// Interesting values
	TPM1->MOD = 7500;	// 48MHz / (128 * 50)
	TPM1_C0V = 3750;	// 7500 / 2
	TPM2->MOD = 7500;
	TPM2_C0V = 3750;

	// Set clock and prescaler
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= TPM_SC_PS(7); // Not TPM_SC_CMOD(1)
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= TPM_SC_PS(7); // Not TPM_SC_CMOD(1)
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);

	// Set ELSB and ELSA for TPM1 and TPM2
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM1->SC &= ~TPM_SC_CMOD(1); // Disable PWM
	TPM2->SC &= ~TPM_SC_CMOD(1); // Disable PWM
}
/*--------------------------------------------------------------------------------------------------*/

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);

	//if receive any data 
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		rx_data = UART2->D;
		if(LED_MASK(rx_data) == LED_RED){
			if(BIT0_MASK(rx_data)){
				osSemaphoreRelease(myREDSem);
				TPM1->MOD = 143;
				TPM1_C0V = 42;
			}
		}
		if(LED_MASK(rx_data) == LED_GREEN){
			if(BIT0_MASK(rx_data)){
				osSemaphoreRelease(myGREENSem);
				TPM0->MOD = 143;
				TPM0_C0V = 42;
			}
		}
	}
}

/*--------------------------------------------------------------------------------------------------*/
//enumeration state_t to hold the state of led, on or off
typedef enum{
	led_on,
	led_off
} state_t;

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
			PTB->PSOR = MASK(GREEN_LED);
			PTD->PSOR = MASK(BLUE_LED);
		}
		else if(state == led_off){
			offRGB();
		}
	}
	//GREEN_LED
	if(LED_COLOR == GREEN_LED){
		if(state == led_on){
			PTB->PCOR = MASK(GREEN_LED);
			PTB->PSOR = MASK(RED_LED);
			PTD->PSOR = MASK(BLUE_LED);	
		}
		else if(state == led_off){
			offRGB();
		}
	}
	//BLUE_LED
	if(LED_COLOR == BLUE_LED){
		if(state == led_on){
			PTD->PCOR = MASK(BLUE_LED);	
			PTB->PSOR = MASK(RED_LED);
			PTB->PSOR = MASK(GREEN_LED);
		}
		else if(state == led_off){
			offRGB();
		}
	}
}

void led_green_thread (void *argument) {
  // ...
  for (;;) {
		osSemaphoreAcquire(myGREENSem, osWaitForever);
		
		led_control(GREEN_LED,led_on);
		osDelay(1000);
		led_control(GREEN_LED,led_off);
		osDelay(1000);
		
	}
}

void led_red_thread (void *argument) {
  // ...
  for (;;) {
		osSemaphoreAcquire(myREDSem, osWaitForever);
		
		led_control(RED_LED,led_on);
		osDelay(1000);
		led_control(RED_LED,led_off);
		osDelay(1000);		
	}
}
/*--------------------------------------------------------------------------------------------------*/
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  InitGPIO();
  initUART2(BAUD_RATE);
  offRGB();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
