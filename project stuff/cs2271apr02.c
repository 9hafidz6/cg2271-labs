#include "RTE_Components.h"
#include CMSIS_device_header
#include <stdbool.h>
#include "cmsis_os2.h"
#include "song.h"

#define MASK(x) (1 << (x))
#define RED_LED 18					// PortB Pin 18 (LED RED)
#define GREEN_LED 19				// PortB Pin 19 (LED GREEN)
#define BLUE_LED 1					// PortD Pin 1  (LED BLUE)
#define PTB0_Pin 0					// PortB Pin 0  (PWM)
#define PTB1_Pin 1					// PortB Pin 1  (PWM)
#define PTB2_Pin 2					// PortB Pin 2  (PWM)
#define PTB3_Pin 3					// PortB Pin 3  (PWM)
#define UART_TX_PORTE22 22	// PortE Pin 22 (BT TX)
#define UART_RX_PORTE23 23	// PortE Pin 23 (BT RX)

#define LED_MASK(x) (x & 0x7C) //mask to contrl which component, 0011 1100
#define BIT0_MASK(x) (x & 0x01) //mask to control within each component, 0000 0001 
#define BIT00_MASK(x) (x & 0x03) //0000 0011

#define BAUD_RATE 9600

osThreadId_t left_motor_flag; //event flag for each threads, controlled by thread tbrain
osThreadId_t right_motor_flag;
//osThreadId_t green_led_flag;
//osThreadId_t red_led_flag;
//osThreadId_t audio_flag;

int rx_data = 0x69;
bool stationary = true;

void InitGPIO(void){
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK));
	
	// Configure MUX settings to make all 3 pins GPIO
	//might not need
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	
	//setting GPIO for the Green LED	
	PORTC->PCR[9] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[9] |= PORT_PCR_MUX(1);
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
	PTC->PDDR |= (MASK(9)|MASK(11)|MASK(10)|MASK(6)|MASK(5)|MASK(4)|MASK(3)|MASK(0)|MASK(7));
	PTD->PDDR |= MASK(BLUE_LED);
}

void ledControl(int c){
	// The LEDs are active LOW
	switch(c){
		case 1: PTB->PDOR |= MASK(RED_LED);   // Off red led
			break;
		case 2: PTB->PDOR |= MASK(GREEN_LED); // Off green led
			break;
		case 3: PTD->PDOR |= MASK(BLUE_LED);  // Off blue led
			break;
		case 4: PTB->PDOR &= ~MASK(RED_LED);  // On red led
			break;
		case 5: PTB->PDOR &= ~MASK(GREEN_LED);// On green led
			break;
		case 6: PTD->PDOR &= ~MASK(BLUE_LED); // On blue led
			break;
		default: ; // Invalid input
	}
}

void offRGB(void){
	ledControl(1);
	ledControl(2);
	ledControl(3);
}

void initPWM(void) {
	// Enable Clock to PORTB
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

	// Configure MUX settings to enable TPM
  PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
 	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);
	PORTC->PCR[1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[1] |= PORT_PCR_MUX(4);

	// Enable Clock to TPM
	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK;
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;

	// Select the MCGFLLCLK clock or MCGPLLCLK/2
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	// Interesting values
	TPM1->MOD = 7500;	// 48MHz / (128 * 50)
	TPM1_C0V = 3750;	// 7500 / 2
	TPM1_C1V = 3750;	
	TPM2->MOD = 7500;
	TPM2_C0V = 3750;
	TPM2_C1V = 3750;	
	TPM0->MOD = 7500;
	TPM0_C0V = 3750;

	// Set clock and prescaler
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= TPM_SC_PS(7); // Not TPM_SC_CMOD(1)
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= TPM_SC_PS(7); // Not TPM_SC_CMOD(1)
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= TPM_SC_PS(7); 
	TPM0->SC |= TPM_SC_CMOD(1); // WE WANT TO ENABLE
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);

	// Set ELSB and ELSA for TPM1 and TPM2 (channel 0)
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	// Set ELSB and ELSA for TPM1 and TPM2 (channel 1)
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM1->SC &= ~TPM_SC_CMOD(1); // Disable PWM
	TPM2->SC &= ~TPM_SC_CMOD(1); // Disable PWM
}

void setFreq(int freq){
  if (freq > 0) {
    // For a 48MHz clk at 128 prescaler
  	int clk = 48000000 / 128;
  	int mod = clk / freq;
  	int cov = mod / 2; // 50% Duty Cycle
  	TPM0->MOD = mod;
  	TPM0_C0V = cov;
  }
}

void Init_UART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
	// enable clock to UART and Port E
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// connect UART to pins for PTE22, PTE23
	PORTE->PCR[22] = PORT_PCR_MUX(4);
	PORTE->PCR[23] = PORT_PCR_MUX(4);
	
	// ensure TX and RX are disabled before configuration
	UART2->C2 &= ~(UARTLP_C2_TE_MASK | UARTLP_C2_RE_MASK);
	
	// Set baud rate to 4800 baud
	bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
	divisor = bus_clock/(baud_rate*16);
	UART2->BDH = UART_BDH_SBR(divisor>>8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	// No parity, 8 bits, two stop bits, other settings;
	UART2->C1 = UART2->S2 = UART2->C3 = 0;
	
	// Enable transmitter and receiver
	UART2->C2 = UART_C2_RE_MASK;
	
	// Enable Interrupt
	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= UART_C2_RIE_MASK;
}

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_TDRE_MASK) {
		// On Transmit
	}
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		// On Receive
		rx_data = UART2->D;
	}
	if (UART2->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | UART_S1_FE_MASK | UART_S1_PF_MASK)) {
		// handle the error
		// clear the flag
	}
}

void delay(int d){
	while(d--);
}

void tBrain(void *argument){
	for(;;){
		switch(LED_MASK(rx_data)){ //0x3C, 0011 1100, mask with value is rx_data to choose component
			case 2: //if the robot is stationary, set event
							//if(){}
							//osThreadFlagsSet(green_led_flag,0x0001);
							break;
			case 4: //if the robot is stationary
							//if(){}
							//osThreadFlagsSet(red_led_flag,0x0001);							
							break;
			case 8: //if rx_data is 0b10xx
							osThreadFlagsSet(right_motor_flag, 0x0001);
							stationary = false;
							break;
		  case 16://if rx_data is 0b100xx  
							osThreadFlagsSet(left_motor_flag, 0x0001);
							stationary = false;
							break;
			case 32:// Buzzer
							if(BIT0_MASK(rx_data)) TPM0->SC |= TPM_SC_CMOD(1); //Enable
							else TPM0->SC &= ~TPM_SC_CMOD(1); //Disable
							break;
			case 64:TPM2->SC |= TPM_SC_CMOD(1); // Enable TPM2
							TPM2_C0V = 0; // 30%  Duty Cycle
							TPM2_C1V = 2250; // 30%  Duty Cycle
			
							TPM1->SC |= TPM_SC_CMOD(1); // Enable TPM2
							TPM1_C0V = 0; // 30%  Duty Cycle
							TPM1_C1V = 2250; // 30%  Duty Cycle
			
							stationary = false;
							break;
			default:TPM1->SC &= ~TPM_SC_CMOD(1); //Disable TPM1
							TPM2->SC &= ~TPM_SC_CMOD(1); //Disable TPM2
							stationary = true;							
		}
	}
}

void red_tLed(void *argument){
	for(;;){
		//osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
		if(stationary){
			//turn on red led 500ms freq
			PTC->PDOR |= MASK(9);
			osDelay(500);
			PTC->PDOR &= ~MASK(9);
			osDelay(500);
		}
		else{
			//turn on red led 250ms freq
			PTC->PDOR |= MASK(9);
			osDelay(250);
			PTC->PDOR &= ~MASK(9);
			osDelay(250);
		}
	}
}

void green_tLed(void *argument){
	int array[] = {11,10,6,5,4,3,0,7};
	for (;;) {
		//osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
		if(!stationary){
			for(int i=0; i<8; i++){
				if(stationary){break;}
				PTC->PDOR = MASK(array[i]);
				osDelay(1000);
			}
		}
		else{
			PTC->PDOR = 0b110011111001; //on all green led
			osDelay(1000);			
		}
	}	
}

void right_tMotor(void *argument){
	for(;;){
		//osEventFlagsWait(right_motor_flag,0x0001, osFlagsWaitAny,osWaitForever);
		osThreadFlagsWait(0x0001, osFlagsWaitAny,osWaitForever);
		switch(BIT00_MASK(rx_data)){
			case 0: TPM2->SC &= ~TPM_SC_CMOD(1); // Disable TPM2
							break;
			case 1: TPM2->SC |= TPM_SC_CMOD(1); // Enable TPM2
							TPM2_C0V = 5250; // 30%  Duty Cycle
							TPM2_C1V = 0;
							break;
			case 2: TPM2->SC |= TPM_SC_CMOD(1); // Enable TPM2
							TPM2_C0V = 2250; // 70%  Duty Cycle
							TPM2_C1V = 0;
							break;
			default: TPM2->SC &= ~TPM_SC_CMOD(1); // Disable TPM2
							 stationary = true;
		}
	}	
}

void left_tMotor(void *argument){
	for(;;){
		//osEventFlagsWait(left_motor_flag,0x0001, osFlagsWaitAny,osWaitForever);
		osThreadFlagsWait(0x0001, osFlagsWaitAny,osWaitForever);
		switch(BIT00_MASK(rx_data)){
			case 0: TPM1->SC &= ~TPM_SC_CMOD(1); // Disable TPM1
							break;
			case 1: TPM1->SC |= TPM_SC_CMOD(1); // Enable TPM1
							TPM1_C0V = 5250; // 30%  Duty Cycle
							TPM1_C1V = 0;
							break;
			case 2: TPM1->SC |= TPM_SC_CMOD(1); // Enable TPM1
							TPM1_C0V = 2250; // 70%  Duty Cycle
							TPM1_C1V = 0;
							break;
			default: TPM1->SC &= ~TPM_SC_CMOD(1); // Disable TPM1
							 stationary = true;
		}
	}
}

void tAudio(void *argument){
	for(;;){
		//osThreadFlagsWait(0x0001, osFlagsWaitAny,osWaitForever);
		for(int i=0; i < 203; i++){
			int wait = duration[i] * songspeed;
			//tone(buzzer,notes[i],wait);          //tone(pin,frequency,duration)
			//TPM0_C0V = notes[i];s
			setFreq(notes[i]);
			osDelay(wait);
		}
	}	
}

const osThreadAttr_t thread_attr = {
	.priority = osPriorityBelowNormal2
};

int main (void) {
  // System Initialization
  SystemCoreClockUpdate();
	InitGPIO();
	initPWM();
	Init_UART2(BAUD_RATE);
	offRGB();
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	
	osThreadNew(tBrain,NULL,&thread_attr); 				// Create application main thread
	
	left_motor_flag = osThreadNew(left_tMotor,NULL,NULL);
	right_motor_flag = osThreadNew(right_tMotor,NULL,NULL);
	/*red_led_flag =*/ osThreadNew(red_tLed,NULL,NULL);
	/*green_led_flag =*/ osThreadNew(green_tLed,NULL,NULL);
	/*audio_flag =*/ osThreadNew(tAudio,NULL,NULL);
  
	osKernelStart();                      // Start thread execution
  	
	for (;;) {}
}
