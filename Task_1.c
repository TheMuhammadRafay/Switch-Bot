/*
ES-|| __SEMESTER PROJECT__:
		'SWITCH BOT'
		PROJECT MEMBERS:
MOHAMMAD SAAD (2021-MC-09)
MOHAMMAD SAAD YASEEN(2021-MC-37)
MOHAMMAD RAFAY (2021-MC-39)
*/

//Libraries:
#include "TM4C123GH6PM.h"
#include <stdint.h>
#include <stdlib.h>
void Delay(unsigned long counter); // used to add delay
void HC05_init(void); // Initialize UART5 module for HC-05
char Bluetooth_Read(void); //Read data from Rx5 pin of TM4C123
void Bluetooth_Write(unsigned char data); // Transmit a character to HC-05 over Tx5 pin 
void Bluetooth_Write_String(char *str); // Transmit a string to HC-05 over Tx5 pin 

void timer0A_Prescaler(int sec);
unsigned volatile long j;

void PLL_Init(void);
void GPIOA_Init(void);
void PB6_as_M0PWM0_Init(void);
void GPIOA_Handler(void);
void PWM_Module0_Channel0_Init(void);
void clocks_delay(volatile uint32_t clocks);

unsigned static int state = 0;
int timerFlag = 1;  // Flag to control timer

int main(void)
{		
    PLL_Init();
    PB6_as_M0PWM0_Init();
    PWM_Module0_Channel0_Init();
		GPIOA_Init();//Manual ovewrite

		HC05_init(); // call HC05_init() to initialze UART5 of TM4C123GH6PM
	
	/* Set PF1, PF2 and PF3 as digital output pins */
	
		SYSCTL->RCGCGPIO |= 0x20;   /* enable clock to GPIOF */
    GPIOF->DIR |= 0x0E;         //set PF1, PF2 and PF3 as digital output pin
    GPIOF->DEN |= 0x0E;         // CON PF1, PF2 and PF3 as digital GPIO pins
  	Delay(10);

    while(1)
    {
			
        char c = Bluetooth_Read();          /* get a character from UART5 */
			// Convert string to integer:
				int intValue_Timer = (int)c;
				
		/* Check the received character and take action to control onboard LEDs of TM4C123 */
		/* Send status string to Andriod app after turning on/off LEDs */

				if( c=='A' && state==0){
					GPIOF->DATA |=(1<<1);
					GPIOF->DATA &=~(1<<2);
					// Set PWM to rotate to 90 degrees
				  PWM0->_0_CMPA = 102300;
					state=1;
				}
				else if( c=='B'&& state==1){
					GPIOF->DATA &=~(1<<1);
					GPIOF->DATA |=(1<<2);
					// Set PWM to return to 0 degrees
					PWM0->_0_CMPA = 153450;
					state=0;
				}
				else if((1<= intValue_Timer && intValue_Timer<= 60)){
					GPIOF->DATA &=~(1<<1);
					GPIOF->DATA &=~(1<<2);
					GPIOF->DATA |=(1<<3);
					timer0A_Prescaler(intValue_Timer);
					if (state==0){
						
						// Set PWM to move to 90 degrees
						PWM0->_0_CMPA = 102300;
						GPIOF->DATA &=~(1<<2);
						GPIOF->DATA |=(1<<1);
						state=1;
					}
					else{
						// Set PWM to move to 0 degrees
						PWM0->_0_CMPA = 153450;
						GPIOF->DATA &=~(1<<1);
						GPIOF->DATA |=(1<<2);
						state=0;
					}
					GPIOF->DATA &=~(1<<3);
				}
    }
}
void GPIOA_Handler(void)
{
    // Button on PA4 pressed
    if (GPIOA->MIS & 0x10)
    { 
			Delay(100);
			if(state==0){
				// Set PWM to move to 90 degrees
        PWM0->_0_CMPA = (2000 / 20) * 1023;//red on
				GPIOF->DATA &=~(1<<2);
				GPIOF->DATA |=(1<<1);
				state = 1 ;				
				
			}
			else{
				// Set PWM to return to 0 degrees
        PWM0->_0_CMPA = (3000 / 20) * 1023;
				GPIOF->DATA &=~(1<<1);
				GPIOF->DATA |=(1<<2);
				state = 0;
			}
       timerFlag=0;
        // Clear the interrupt flag
        GPIOA->ICR = 0x10;
    }
}

void GPIOA_Init(void)
{
   
    // Enable clock for GPIO Port A
    SYSCTL->RCGCGPIO |= 0x01;
    while (!(SYSCTL->PRGPIO & 0x01)); // Wait until GPIOA is ready

    // Configure PA4 as input
    GPIOA->DIR &= ~0x10; // Input direction for PA4
    GPIOA->DEN |= 0x10;  // Enable digital pin functionality
    GPIOA->PUR |= 0x10;  // Enable pull-up resistor for PA4

    // Enable interrupt on PA4
    GPIOA->IS &= ~0x10;   // Edge-sensitive
    GPIOA->IBE &= ~0x10;  // Not both edges
    GPIOA->IEV |= 0x10;   // Rising edges
    GPIOA->ICR |= 0x10;   // Clear any prior interrupt
    GPIOA->IM |= 0x10;    // Unmask interrupt

    // Enable and set priority for GPIO Port A interrupt (Assuming interrupt number 0 for GPIO Port A)
    NVIC->IP[0] = 3 << 5;      // Set priority to 3
    NVIC->ISER[0] |= 1 << 0;    // Enable interrupt for GPIO Port A
}
void GPIOF_Handler(void)
{
    // Button on PF0 pressed
    if (GPIOF->MIS & 0x01)
    {
        // Set PWM to rotate to 180 degrees
        //PWM0->_0_CMPA = 278000 + ((314000 - 278000) * 180) / 180;
				GPIOF->DATA |=(1<<1);
				GPIOF->DATA &=~(1<<2);
			 PWM0->_0_CMPA = (2000 / 20) * 1023;

    }

    // Button on PF4 pressed
    if (GPIOF->MIS & 0x10)
    {
        // Set PWM to return to 0 degrees
        PWM0->_0_CMPA = (3000 / 20) * 1023;
				GPIOF->DATA &=~(1<<1);
				GPIOF->DATA |=(1<<2);
    }
		timerFlag=0;
    GPIOF->ICR = 0x11; // Clear the interrupt flag
}
void timer0A_Prescaler(int sec){
	int i=0;
	
	// Step 1: Enable Timer Clock on timer0
	SYSCTL->RCGCTIMER |= 0x01;		// b 0000 0001
	for (j =0; j < 3 ; j++)		// at least 3 clock cyles
	
	// Step 2: Ensure Timer is disabled before making any changes
	TIMER0->CTL = 0x00;					// TAEN = 0, i.e., timer0A is disablled
	
	// Step 3: Select Mode of Operation of timer0 (Split/cancatenated/RTC)
	TIMER0->CFG = 0x04;					// timer0 is used as a 16-bit (split) timer, i.e., timer0A
	TIMER0->TAMR = 0x02; 				// TAMR = 2 (periodic), TACDIR = 0 (count-down)
	
	// Step 4: Load counter start value 
	TIMER0->TAPR = 250-1;					// 16000000 Hz/250 = 64000 Hz
	TIMER0->TAILR = 32000-1;			// 64000/32000 = 2 Hz (0.5 second)

	// Step 5: Interrupt configurations
	TIMER0->ICR = 0x01;					// Clear timer status flag (TATORIS, TATOMIS)
	
	// Step 6: Enable the Timer and start counting
	TIMER0->CTL |= 0x01;					// TAEN = 1
	
	// One iteration of this loop generates 1 mili-second delay
	for( i = 0; i<120*sec*timerFlag; i++){
		// Step 7: Poll TATORIS (Time-Out Raw Interrupt Status) bit to check timer0A timeout
		while((TIMER0->RIS & 0x01)==0); // Wait for timeout flag to set
		
		// After time-out, timer relaoads automatically and start counting
		// Clear timer status flag i.e., TATORIS flag
		TIMER0->ICR = 0x01;
	}
	// Disable Timer 
	TIMER0->CTL = 0x00;			// TAEN = 0, i.e., timer0A is disablled
}

void PLL_Init(void){

  // 0) Use RCC2
  SYSCTL->RCC2 |=  0x80000000;  // USERCC2

  // 1) bypass PLL while initializing
  SYSCTL->RCC2 |=  0x00000800;  // BYPASS2, PLL bypass

  // 2) select the crystal value and oscillator source
  SYSCTL->RCC = (SYSCTL->RCC &~ 0x000007C0) + 0x00000540;  // clear XTAL field, bits 10-6  // 10101, configure for 16 MHz crystal
  SYSCTL->RCC2 &= ~0x00000070;  // configure for main oscillator source

  // 3) activate PLL by clearing PWRDN
  SYSCTL->RCC2 &= ~0x00002000;

  // 4) set the desired system divider
  SYSCTL->RCC2 |= 0x40000000;   // use 400 MHz PLL
  SYSCTL->RCC2 = (SYSCTL->RCC2 &~ 0x1FC00000) + (24<<22);      // configure for 16 MHz clock / / clear system clock divider +

  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL->RIS&0x00000040)==0){}  // wait for PLLRIS bit

  // 6) enable use of PLL by clearing BYPASS
  SYSCTL->RCC2 &= ~0x00000800;
	
}

void PB6_as_M0PWM0_Init(void){
	// Step 1: Clock enable on PortB
	SYSCTL->RCGCGPIO |= 0x02;		// 0b 0010 0000 
	for (j =0; j < 3 ; j++)			// at least 3 clock cyles
	
	// Step 2: APB is selected for PortB by selecting
	// 0x40025000 as Base Address in DATA section
	
	// Step 3: Enable alternate functionality on PortB
	GPIOB->AFSEL |= 0x40;				// 0b 0000 0100
	
	// Step 4: Enable digital pin functionaliy on PortF pin 6
	GPIOB->DEN |= 0x40; // Digital enable for PB6
	
	// Step 5: Set PortB pin 6 as an output pin
	GPIOB->DIR |= 0x40; // PB6 as Output pin
	
	// Step 6: Configure PortB pin 6 as M0PWM6 pin (Table 10-2 of Datasheet, page # 651)
	GPIOB->PCTL &= 0xF0FFFFFF;		// clear the bit fields
	GPIOB->PCTL |= 0x04000000;
}

void PWM_Module0_Channel0_Init(void){
	
	// Step 1: Clock Gating Control of PWM Module 1
	SYSCTL->RCGCPWM |= 0x01;		// b 0000 0010
	for (j =0; j < 3 ; j++)			// at least 3 clock cyles
	SYSCTL->RCC	&= (~(1<<20));		// disable clock signal divisor for PWM
	
	// Step 2: For PWM Channel configurations
	// we need check which PWM block our PWM Pin blongs to. For our case PF2
	// is M1PWM6 which is a part of PWM block 3 
	// Read any register description for identifying the block, e.g., CTL
	
	// Step 3: Disable Generator 0 before performing configurations
	// Step 4: Select Counter 0 Count-down mode
	PWM0->_0_CTL = 0x00;
	
	// Step 5: Set Load value for 50 Hz
	PWM0->_0_LOAD = 320000;
	
	// Step 6: Set Compare Register Value to set  duty cycle
	PWM0->_0_CMPA = 1590;
	
	// Step 7: Perform Generating Control Register configuration
	// PWM signal HIGH when counter relaoded and LOW when matches CMPA Value
	PWM0->_0_GENA |= 0x8C;
	
	// Step 8: Enable generator 0 counter
	PWM0->_0_CTL |= 0x01;
	
	// Step 9: Enalbe PWM Module 0 Channel 0 Output
	PWM0->ENABLE |= 0x01;
}

void HC05_init(void)
{
	SYSCTL->RCGCUART |= 0x20;  /* enable clock to UART5 */
    SYSCTL->RCGCGPIO |= 0x10;  /* enable clock to PORTE for PE4/Rx and PE5/Tx */
    Delay(1);
    /* UART0 initialization */
    UART5->CTL = 0;         /* UART5 module disbable */
    UART5->IBRD = 104;      /* for 9600 baud rate, integer = 104 */
    UART5->FBRD = 11;       /* for 9600 baud rate, fractional = 11*/
    UART5->CC = 0;          /*select system clock*/
    UART5->LCRH = 0x60;     /* data lenght 8-bit, not parity bit, no FIFO */
    UART5->CTL = 0x301;     /* Enable UART5 module, Rx and Tx */

    /* UART5 TX5 and RX5 use PE4 and PE5. Configure them digital and enable alternate function */
    GPIOE->DEN = 0x30;      /* set PE4 and PE5 as digital */
    GPIOE->AFSEL = 0x30;    /* Use PE4,PE5 alternate function */
    GPIOE->AMSEL = 0;    /* Turn off analog function*/
    GPIOE->PCTL = 0x00110000;     /* configure PE4 and PE5 for UART */
}
char Bluetooth_Read(void)
{
    char data;
    while ((UART5->FR & (1 << 4)) != 0); // wait until Rx buffer is not full
    data = (char)(UART5->DR & 0xFF);    // mask out unnecessary bits
    return data;
}

void Bluetooth_Write(unsigned char data)  
{
    while((UART5->FR & (1<<5)) != 0); /* wait until Tx buffer not full */
    UART5->DR = data;                  /* before giving it another byte */
}


void Delay(unsigned long counter)
{
	unsigned long i = 0;
	
	for(i=0; i< counter; i++);
}
