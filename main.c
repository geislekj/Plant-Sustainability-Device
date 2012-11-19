/********************************************************************
* FileName:        main.c
* Processor:       PIC18F4520
* Compiler:        MPLAB C18 v.3.06 
*
* This file is the control for the plant sustanability device
*                                                                     
*
*       Author               Date              Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	//Kevin Geisler			10/31/2012

/**  Header Files **************************************************/     
#include <p18f4520.h>
#include <timers.h> 
#include <delays.h>
#include <adc.h>
#include <pwm.h>

/** Configuration Bits *********************************************/     
#pragma config OSC = INTIO67  // EC = External 4MHz Crystal for PICDEM board only
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config BOREN = OFF
#pragma config XINST = OFF

/** Define Constants Here ******************************************/
#define MOTORCLOCKTICKS 100
#define MOTORSPEED 120
#define MOTORSTOP 0
#define SERVOCLOCKTICKS 1540
#define OPENEDVALVE 300
#define CLOSEDVALVE 1000
#define NUMBEROFPLANTS 2;
#define TOTALTIMERCOUNT 150;

/** Local Function Prototypes **************************************/
void low_isr(void);
void high_isr(void);
void setupIRSensor (void);
int readIRSensor(void);
void setTimer2PWM(void);
void setupMotor(void);
void startMotor(void);
void stopMotor(void);
void setupServo(void);
void openValve(void);
void closeValve(void);
void startTimer0(void);
void startTimer1(void);
void setupButtons(void);
void moveToLocation(int newPosition);

/** Declare Interrupt Vector Sections ****************************/
#pragma code high_vector=0x08
void interrupt_at_high_vector(void)
{
   _asm goto high_isr _endasm
}

#pragma code low_vector=0x18
void interrupt_at_low_vector(void)
{
   _asm goto low_isr _endasm
}

/** Global Variables ***********************************************/
int currentPosition = 0;
int servoState = 0;
int plantTimerCount = 0;
int valveCounter = 0;
int plantBeingWatered = 0;

//CONSTANTS
int totalplants = 2;
int totalTimerCount = 150/4;
int valveTotalTick = 6; //218;

/*******************************************************************
* Function:        void main(void)
********************************************************************/
#pragma code
void main (void)
{
	//Set Clock Frequency = 500kHz
	OSCCONbits.IRCF2 = 0;
	OSCCONbits.IRCF2 = 1;
	OSCCONbits.IRCF2 = 1;


	// Enable Global interrupts
	INTCONbits.GIE = 1;		//  Enable High priority interrupt

	//Setup IR Sensor
	setupIRSensor();
	
	//Initializes the timer2 for PWM for Motor and Servo
	setTimer2PWM();
	
	//Setup Motor
	setupMotor();
	
	//Setup Timer for servo
	startTimer0();	

	//Setup Timer for timed water
	startTimer1();
	
	//SetupButtonInterupt
	setupButtons();

	while (1)
    {
		// This area loops forever
    }
}

/*****************************************************************
* Additional Helper Functions
******************************************************************/


/*****************************************************************
* Function:			void setupButtons(void);
* Input Variables:	none
* Output Return:	none
* Overview:			Setups the interupts for buttons RB0-2
******************************************************************/
void setupButtons()
{
	RCONbits.IPEN = 0;
	TRISB = 0b00000111;
	
	//RB0
	INTCONbits.INT0IE = 1;
	INTCON2bits.INTEDG0 = 0;
	
	//RB1
	INTCON3bits.INT1IE = 1;
	INTCON2bits.INTEDG1 = 0;

	//RB2
	INTCON3bits.INT2IE = 1;
	INTCON2bits.INTEDG2 = 0;
	
	
	INTCONbits.INT0IF = 0;
	INTCON3bits.INT1IF = 0;
	INTCON3bits.INT2IF = 0;
}

/*****************************************************************
* Function:			setupIRSensor
* Input Variables:	none
* Output Return:	none
* Overview:			Setups the ADC for the IR
******************************************************************/
void setupIRSensor()
{
	ADCON1=0b00001101;
	TRISA=0b00001111;

    OpenADC(ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_12_TAD,
    ADC_CH0 & ADC_INT_OFF &ADC_REF_VDD_VSS,
    0b00001101);

}

/*****************************************************************
* Function:			readIRSensor
* Input Variables:	none
* Output Return:	1 if there is IR
*					0 if there is no IR
* Overview:			Reads the valus for the IR and determs if there is IR
******************************************************************/
int readIRSensor()
{
    int ra0 = 0;
    SetChanADC(ADC_CH0);
    ConvertADC();
    while(BusyADC());
    ra0 = ReadADC();
    
    if(ra0 >= 500){
    	return 0;
    }else{
    	return 1;
    }	
}	

/*****************************************************************
* Function:			setTime2PWM
* Input Variables:	none
* Output Return:	none
* Overview:			Initializes the timer 2 for pusle width modulation
******************************************************************/
void setTimer2PWM()
{
	OpenTimer2(TIMER_INT_OFF & T2_PS_1_1);	
}	

/*****************************************************************
* Function:			setupMotor
* Input Variables:	none
* Output return:	none
* Overview:			Sets up the motor control using PWM
******************************************************************/
void setupMotor(){
	OpenPWM1(MOTORCLOCKTICKS);
	SetDCPWM1(MOTORSTOP);
}	

/*****************************************************************
* Function:			startMotor
* Input Variables:	void
* Output return:	none
* Overview:			Moves the motor
******************************************************************/
void startMotor(){
	SetDCPWM1(MOTORSPEED);
}	

/*****************************************************************
* Function:			stopMotor
* Input Variables:	void
* Output return:	none
* Overview:			Stops the motor
******************************************************************/
void stopMotor(){
	SetDCPWM1(MOTORSTOP);
}	

/*****************************************************************
* Function:			openValve(void);
* Input Variables:	void
* Output return:	none
* Overview:			opens the valve by the servo
******************************************************************/
void openValve(){
	servoState = 1;
}	

/*****************************************************************
* Function:			closeValve(void);
* Input Variables:	void
* Output return:	none
* Overview:			closes the valve by the servo
******************************************************************/
void closeValve(){
	servoState = 0;
}	

/*****************************************************************
* Function:			startTimer0
* Input Variables:	none
* Output Return:	none
* Overview:			Initializes the timer 1 for servo
******************************************************************/

void startTimer0()
{
	// Setup the timer with a 1:1 prescaler with 16 bits resolution
	// Therefore the timer0 freq is 500 kHz / 4 / 4 = 31.250 kHz
	OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_4 );
	// Should take a little over 2 seconds to overflow the counter from TMR0 = 0
	// If you write in a different starting value for TMR0 it'll overflow sooner

	WriteTimer0(0);
	INTCONbits.TMR0IF = 0;
}	

/*****************************************************************
* Function:			startTimer1
* Input Variables:	none
* Output Return:	none
* Overview:			Initializes the timer 1 for watering for a long period of time
******************************************************************/

void startTimer1()
{
	// Setup the timer with a 1:1 prescaler with 16 bits resolution
	// Therefore the timer0 freq is 500 kHz / 4 / 4 = 31.250 kHz
	OpenTimer1( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_4 );
	// Should take a little over 2 seconds to overflow the counter from TMR0 = 0
	// If you write in a different starting value for TMR0 it'll overflow sooner

	WriteTimer1(0);
	PIR1bits.TMR1IF = 0;  
}	

/*****************************************************************
* Function:			
* Input Variables:	int position
* Output Return:	none
* Overview:			moves the susan to a given plant location
******************************************************************/

void moveToLocation(int newPosition)
{
	if(newPosition != currentPosition){
		startMotor();
		Delay10KTCYx(7);
		while(!readIRSensor());
		stopMotor();
		
		//Track the current location of the plant
		currentPosition++;	
		
		if(currentPosition == 3){
			currentPosition = 0;	
		}	
	
		//recursive call to move the the next position if this is not the required state	
		if(newPosition != currentPosition){
			moveToLocation(newPosition);
		}
	}
}


/*****************************************************************
* Function:        void high_isr(void)
* Possible sources of interrupt - none
* Overview:
******************************************************************/
#pragma interrupt high_isr
void high_isr(void)
{
	
	//Timer for servo
	if(INTCONbits.TMR0IF)  
	{						
		PORTBbits.RB3 = 1;
		if(servoState == 0){
			Delay1TCY();
  	 	  	Delay1TCY();
  	 	  	//Delay10TCYx(6);
		}else{
			Delay1TCY();
      		Delay1TCY();
      		Delay1TCY();
      		Delay1TCY();
      		Delay1TCY();
      		Delay1TCY();
      		Delay1TCY();
  	 	  	Delay10TCYx(5);
  	 	  	Delay100TCYx(4);
  		}	 
  		PORTBbits.RB3 = 0;	  
		INTCONbits.TMR0IF = 0;	// Clear interrupt flag for timer 0
		WriteTimer0(64911);

	}
	//code for servo state = 1 and regular timer
	if(PIR1bits.TMR1IF && servoState == 1){
		valveCounter++;	
		if(valveCounter == valveTotalTick){
			closeValve();	
			valveCounter = 0;
		}	
		PIR1bits.TMR1IF = 0;	// Clear interrupt flag for timer 0
	}
	else if(PIR1bits.TMR1IF && plantTimerCount == totalTimerCount){
		//Waters all the plants	
	
		moveToLocation(plantBeingWatered);
		openValve();
		
		if(plantBeingWatered == totalplants){
			plantTimerCount = 0;
			plantBeingWatered = -1;
		}
	
		plantBeingWatered++;
		PIR1bits.TMR1IF = 0;	// Clear interrupt flag for timer 0
	}	
	
	//Timer for timed water sequence
	else if(PIR1bits.TMR1IF)  
	{		
		plantTimerCount++;
		PIR1bits.TMR1IF = 0;	// Clear interrupt flag for timer 0
	}
	
	// interupt for button to water plant 0
	if(INTCONbits.INT0IF){
		moveToLocation(0);
		openValve();
		INTCONbits.INT0IF = 0;
		
		
	}	
	// interupt for button to water plant 1
	if(INTCON3bits.INT1IF){
		moveToLocation(1);
		openValve();
		INTCON3bits.INT1IF = 0;

	}	
	// interupt for button to water plant 2
	if(INTCON3bits.INT2IF){
		moveToLocation(2);
		openValve();
		INTCON3bits.INT2IF = 0;
	}	
}


/******************************************************************
* Function:        void low_isr(void)
* Possible sources of interrupt - none
* Overview:
********************************************************************/
#pragma interruptlow low_isr
void low_isr(void)
{
	// Add code here for the low priority Interrupt Service Routine (ISR)
}




/******************************************************************************************************
												EXTRA CRAP SECTION  -IGNORE 
*******************************************************************************************************/
