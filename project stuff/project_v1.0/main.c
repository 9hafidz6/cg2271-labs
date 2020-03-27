/*--------------------------------------------------------------------------------------------------------------------------------------------*/
#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"

#include "song.h"

/*--------------------------------------------------------------------------------------------------------------------------------------------*/

#define MASK(x) (1 << (x))
#define RED_LED 18					// PortB Pin 18 (LED RED)
#define GREEN_LED 19				// PortB Pin 19 (LED GREEN)
#define BLUE_LED 1					// PortD Pin 1  (LED BLUE)
/*--------------------------------------------------------------------------------------------------------------------------------------------*/
#define PTB0_Pin 0					// PortB Pin 0  (PWM)
#define PTB1_Pin 1					// PortB Pin 1  (PWM)
#define PTB2_Pin 2					// PortB Pin 2  (PWM)
#define PTB3_Pin 3					// PortB Pin 3  (PWM)
#define UART_TX_PORTE22 22	// PortE Pin 22 (BT TX)
#define UART_RX_PORTE23 23	// PortE Pin 23 (BT RX)
/*--------------------------------------------------------------------------------------------------------------------------------------------*/
#define LED_MASK(x) (x & 0x0E) //mask to contrl which component
#define BIT0_MASK(x) (x & 0x01) //mask to control within each component
/*--------------------------------------------------------------------------------------------------------------------------------------------*/
#define BAUD_RATE 9600
/*--------------------------------------------------------------------------------------------------------------------------------------------*/
osEventFlagsId_t left_motor_flag;
osEventFlagsId_t right_motor_flag;
osEventFlagsId_t green_led_flag;
osEventFlagsId_t red_led_flag;

int rx_data = 0x69;
/*--------------------------------------------------------------------------------------------------------------------------------------------*/

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
/*--------------------------------------------------------------------------------------------------------------------------------------------*/

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
/*--------------------------------------------------------------------------------------------------------------------------------------------*/

void initPWM(void) {
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
/*--------------------------------------------------------------------------------------------------------------------------------------------*/

void setFreq(int freq){
  if (freq > 0) {
  	double period = 1.0 / freq;
  	double period_clk = 1.0 / (1.0 * 48000000 / 128);
  	int mod = (period / period_clk) - 1;
  	int cov = (mod + 1) / 2;
  	TPM1->MOD = mod;
  	TPM1_C0V = cov;
  }
}
/*--------------------------------------------------------------------------------------------------------------------------------------------*/

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
	//Q_Init(&TxQ);
	//Q_Init(&RxQ);
}

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_TDRE_MASK) {
		//UART2->C2 &= ~UART_C2_TIE_MASK;
	}
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		// On Receive
		rx_data = UART2->D;
		//TPM1->SC |= TPM_SC_CMOD(1); // Enable TPM1
		//TPM2->SC |= TPM_SC_CMOD(1); // Enable TPM2

//		switch(rx_data){
//			case 1: TPM1->SC &= ~TPM_SC_CMOD(1); //Disable TPM1
//					break;
//			case 2: TPM1->SC |= TPM_SC_CMOD(1); // Enable TPM1
//					TPM1_C0V = 2250; // 30%  Duty Cycle
//					break;
//			case 3: TPM1->SC |= TPM_SC_CMOD(1); // Enable TPM1
//					TPM1_C0V = 5250; // 70% Duty Cycle
//					break;
//			case 4: TPM2->SC &= ~TPM_SC_CMOD(1); //Disable TPM2
//					break;
//			case 5: TPM2->SC |= TPM_SC_CMOD(1); // Enable TPM2
//					TPM2_C0V = 2250; // 30%  Duty Cycle
//					//TPM1_C1V = 2250;
//					break;
//			case 6: TPM2->SC |= TPM_SC_CMOD(1); // Enable TPM2
//					TPM2_C0V = 5250; // 70% Duty Cycle
//					//TPM1_C1V = 5250;
//					break;
//			default:TPM1->SC &= ~TPM_SC_CMOD(1); //Disable TPM1
//					TPM2->SC &= ~TPM_SC_CMOD(1); //Disable TPM2
//		}
	}
	if (UART2->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | UART_S1_FE_MASK | UART_S1_PF_MASK)) {
		// handle the error
		// clear the flag
	}
}
/*--------------------------------------------------------------------------------------------------------------------------------------------*/

void delay(int d){
	while(d--);
}
/*--------------------------------------------------------------------------------------------------------------------------------------------*/

void tBrain(void *argument){
	for(;;){
		switch(LED_MASK(rx_data)){ //0x0E mask with value is rx_data to choose component
			case 2: osEventFlagsSet(red_led_flag,0x0001);							
							break;
			case 4: //control the green led
							break;
			case 8: osEventFlagsSet(right_motor_flag, 0x0001);
							break;
			case 16:osEventFlagsSet(left_motor_flag, 0x0001); 
							break;
			case 32: //control the buzzer
							break;
			default:TPM1->SC &= ~TPM_SC_CMOD(1); //Disable TPM1
							TPM2->SC &= ~TPM_SC_CMOD(1); //Disable TPM2
		}
	}
}

void left_tMotor(void *argument){
	for(;;){
		osEventFlagsWait(left_motor_flag,0x0001, osFlagsWaitAny,osWaitForever);
		switch(BIT0_MASK(rx_data)){
			case 1: TPM1->SC |= TPM_SC_CMOD(1); // Enable TPM1
							TPM1_C0V = 2250; // 30%  Duty Cycle
							break;
			case 2: TPM1->SC |= TPM_SC_CMOD(1); // Enable TPM1
							TPM1_C0V = 5250; // 70%  Duty Cycle
							break;
			case 3: TPM1->SC &= ~TPM_SC_CMOD(1); // Disable TPM1
							break;
			default: TPM1->SC &= ~TPM_SC_CMOD(1); // Disable TPM1
		}
	}
}

void right_tMotor(void *argument){
	for(;;){
		osEventFlagsWait(right_motor_flag,0x0001, osFlagsWaitAny,osWaitForever);
		switch(BIT0_MASK(rx_data)){
			case 1: TPM2->SC |= TPM_SC_CMOD(1); // Enable TPM2
							TPM2_C0V = 2250; // 30%  Duty Cycle
							break;
			case 2: TPM2->SC |= TPM_SC_CMOD(1); // Enable TPM2
							TPM2_C0V = 5250; // 70%  Duty Cycle
							break;
			case 3: TPM2->SC &= ~TPM_SC_CMOD(1); // Disable TPM2
							break;
			default: TPM2->SC &= ~TPM_SC_CMOD(1); // Disable TPM2
		}
	}	
}

void red_tLed(void *argument){
	for(;;){
		osEventFlagsWait(red_led_flag, 0x0001, osFlagsWaitAny, osWaitForever);
		switch(BIT0_MASK(rx_data)){
			//red led on or off
			case 1: 
			case 2: 
			default: offRGB();
		}
	}
}

void green_tLed(void *argument){
	int array[] = {11,10,6,5,4,3,0,7};
	for (;;) {
		for(int i=0; i<8; i++){
			PTC->PDOR = MASK(array[i]);
			osDelay(1000);
		}
	}	
}

void tAudio(void *argument){
	for(;;){
		for(int i=0; i < 203; i++){
			int wait = duration[i] * songspeed;
			//tone(buzzer,notes[i],wait);          //tone(pin,frequency,duration)
			osDelay(wait);
		}
	}	
}
/*--------------------------------------------------------------------------------------------------------------------------------------------*/

int main (void) {
  // System Initialization
  SystemCoreClockUpdate();
	InitGPIO();
	initPWM();
	Init_UART2(BAUD_RATE);
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  //osThreadNew(app_main, NULL, NULL);    // Create application main thread
	osThreadNew(tBrain,NULL,NULL);
	osThreadNew(left_tMotor,NULL,NULL);
	osThreadNew(right_tMotor,NULL,NULL);
	osThreadNew(red_tLed,NULL,NULL);
	osThreadNew(green_tLed,NULL,NULL);
	osThreadNew(tAudio,NULL,NULL);
  osKernelStart();                      // Start thread execution
  
	left_motor_flag = osEventFlagsNew(NULL);	//create event flags for threads
	right_motor_flag = osEventFlagsNew(NULL);
	red_led_flag = osEventFlagsNew(NULL);
	green_led_flag = osEventFlagsNew(NULL); 
	
	for (;;) {}
}

//1110 (E)	
//	0100 (5)
//	0101 (6)
//		0010 (3)
//		0011 (2)
//			1000 (8)
//			1001 (9)
//led mask(E) -> 3 values
//bit(11) -> 4 values
 