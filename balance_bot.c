//____________testing done with an old 9V battery, 7.26 V, and a pretty good 9V

// team segmundo
// meam 510 -- L6



// header files
#include "m_general.h"
#include "m_usb.h"

// subroutines
int MoveAvg8(int NewValue);
void set_ADC(void);
void update_ADC(void);
void calibrate(void);
void set_Timer(void);
void configure_MotorPins(void);
void drive_motors(void);
void horizon(void);

// global variables
#define DEBUG 1
#define CLOCK 0
#define PIN 4
#define ADC_LENGTH 3
#define SAMPLE_FREQ 5000 //--> may not have the greatest effect on the movement, noticeably worse at 100 though
static int RunSum=0;
static int RunSum_Buffer[8] = {0,0,0,0,0,0,0,0};
static unsigned char Newest = 0;
static unsigned char Oldest = 1;

//float timestep = 1/SAMPLE_FREQ;
float timestep = 1/SAMPLE_FREQ;
//_______________________________________ gyro variables, read in through F0
signed int gyroOffset;
signed int gyroVal;
signed int gyroValOld;
signed int gyroDiff;
int ADC_F0;
//_______________________________________ x accelerometer variables, read in through B4
signed int xAccelOffset;
signed int xAccelVal;
signed int xAccelValOld;
signed int xAccelDiff;
int ADC_B4;
//_______________________________________ gains
float alpha = 0.75; //infHor xAc filter --> 
float beta = 0.8; //infHor gyro filter --> high end (like .98) is poor, low end (like .25) is good

float gyroGain = 0.15; //--> .5 felt nice, oh yea
float xAccelGain;
int angleThreshold = 2; // default to 5

float angle = 0;
float angleOld = 0;
float omega = 100;
int gyroThreshold = 2;

int angleMax = 2;
volatile uint8_t dsend = 0;


//_______________________________________ main routine
int main(void)
{
	if (DEBUG){
  	m_usb_init(); 				// connect usb
  }
  
	sei();									// Enable global interrupts
 	m_clockdivide(CLOCK); 	// set to 16 MHz
	configure_MotorPins();	// set motor pins
  set_ADC(); 							// set our ADC up
  update_ADC(); 					// grab ADC values for gyros & accelerometers
	calibrate(); 						// set offset values
	set_Timer();						// sets up a timer, Mode 4 up to OCR1A

	while(1)
	{
		xAccelGain = (float)(1-gyroGain); 			// set the x accelerometer gain
		drive_motors();
			
		if (dsend){
			update_ADC();
			horizon();
			
			gyroDiff = gyroVal - gyroOffset;
			xAccelDiff = xAccelVal - xAccelOffset;
			angleOld = angle;												// save the old angle for calculating change in angle

			//COMPLIMENTARY FILTER: angle(new) = (0.98)*((omega*timestep) + angle) + (0.02)*(Xaccel)
			//omega(new) = ((angle/time)+omega)		
			angle = (float)( gyroGain*( (gyroDiff*timestep) + angle) + xAccelGain*(xAccelDiff) ); // compute the new angle
//			omega = (float)((angle - angleOld)/timestep ); // compute 
//			omega = (float)( gyroDiff + (angle - angleOld)/timestep ); // compute 
			omega = (float)( gyroDiff ); // compute 


			dsend=0;
			
			if(DEBUG){
				m_usb_tx_string("Gyro: ");
				m_usb_tx_int(gyroDiff);
				m_usb_tx_string("      Xaccel:");
				m_usb_tx_int(xAccelDiff);
				m_usb_tx_string("      GyroOffset:");
				m_usb_tx_int( gyroOffset );
				m_usb_tx_string("      AccelOffset:");
				m_usb_tx_int(xAccelOffset);
				m_usb_tx_string("      Angle:");
				m_usb_tx_int( angle );
				m_usb_tx_string("      Omega:");
				m_usb_tx_int( omega );
				m_usb_tx_string("\n");
			}
		}
	}
}

//_______________________________________ Subroutine for calibrating sensors
void calibrate(void){ // calibrate the gyros and accelerometers
	//m_usb_tx_string("\n Calibrating gyro & accelerometers.");	// pause and then set the initial values for gyro, accelerometers
	m_red(TOGGLE);
	m_wait(1000); // wait for 3s to stabilize
	m_red(TOGGLE);
	m_wait(1000); // wait for 3s to stabilize
	m_red(TOGGLE);
	m_wait(1000); // wait for 3s to stabilize
	m_red(TOGGLE);
	update_ADC();	// update offset values
	gyroOffset = gyroVal;
	xAccelOffset = xAccelVal;
	gyroValOld = gyroVal;
	xAccelValOld = xAccelVal;
	
	/*	
	m_usb_tx_string("\n Calibration completed. Gyro:");
	m_usb_tx_int(gyroOffset);
	m_usb_tx_string("\n X accelerometer:");
	m_usb_tx_int(xAccelOffset);
	m_usb_tx_string("\n Z accelerometer:");
	m_usb_tx_int(zAccelOffset);
	*/
	
	set(PORTF,5);			// Enable motors
	set(PORTF,7);			// ^
	
	m_red(ON);
}

//_______________________________________ Subroutine for setting motor pins
void configure_MotorPins(void){
	m_disableJTAG();			// Turn off JTAG
	set(DDRF, 4);				// Set F4 as output
	set(DDRF, 5);				// Set F5 as output --> enable line
	set(DDRF, 6);				// Set F6 as output
	set(DDRF, 7);				// Set F7 as output --> enable line
	set(DDRB, 7);				// Set B7 as output __> second driver
	set(DDRD, 0);				// Set D0 as output __> second driver
}

//_______________________________________ Subroutine for driving motors
void drive_motors(void){

	if( omega > 0 ){
		clear(PORTF,4);	// Drive forwards
		clear(PORTF,6);	// ^
		set(PORTB, 7);				// Set B7 as output
		set(PORTD, 0);				// Set D0 as output
	}
	else if( omega < 0 ){
		set(PORTF,4);	// Drive forwards
		set(PORTF,6);	// ^
		clear(PORTB, 7);				// Set B7 as output
		clear(PORTD, 0);				// Set D0 as output
	}

	if( angle > 1.4*angleThreshold ){
		clear(PORTF,4);	// Drive forwards
		clear(PORTF,6);	// ^
		set(PORTB, 7);				// Set B7 as output
		set(PORTD, 0);				// Set D0 as output
	}
	else if( angle < -angleThreshold ){
		set(PORTF,4);	// Drive forwards
		set(PORTF,6);	// ^
		clear(PORTB, 7);				// Set B7 as output
		clear(PORTD, 0);				// Set D0 as output
	}

/*
	if( angle > angleMax ){
		clear(PORTF,4);	// Drive forwards
		clear(PORTF,6);	// ^
	}
	else if( angle < -angleMax ){
		set(PORTF,4);	// Drive forwards
		set(PORTF,6);	// ^
	}
	
	else if ( (abs(gyroDiff) > gyroThreshold) || ( abs(angle) > angleThreshold ) ){// If change of angle is not zero...		
		if ( gyroDiff > 0)		// If change of angle is positive...		
		{
			set(PORTF,4);	// Drive forwards
			set(PORTF,6);	// ^
		}
		else							// Else if change of angle is negative...
		{
			clear(PORTF,4);			// Drive backwards
			clear(PORTF,6);			// ^
		}
	}
	*/
	// if about to fall, go balls to the wall
}	

//_______________________________________ inFa-nay-nay's Horizon
void horizon(void){
	gyroVal = (float)beta*gyroValOld + gyroVal*(1-beta);
	xAccelVal = (float)alpha*xAccelValOld + xAccelVal*(1-alpha);	
}

//_______________________________________ Subroutine for moving average
int MoveAvg8(int NewValue){ // Moving Average, 8
	RunSum = RunSum - RunSum_Buffer[Oldest] + NewValue;
	RunSum_Buffer[Newest] = NewValue;
	Newest = (Newest+1) & 0x07;
	Oldest = (Oldest+1) & 0x07;
	return(RunSum >> 3); 		// divide by 8 using logical shift operation
}

//_______________________________________ subroutine for setting ADCs
void set_ADC(void){
	//_____________________________________ set ADC values
	clear(ADMUX, REFS1); // voltage Reference - set to VCC
	set(ADMUX, REFS0);   // ^
	
	//clear(ADMUX, REFS1); // voltage Reference - set to Vref, the Aref pin, 3.4V
	//clear(ADMUX, REFS0);   // ^

	set(ADCSRA, ADPS2); // set the ADC clock prescaler, divide 16 MHz by 128 (set, set, set)
	set(ADCSRA, ADPS1); // ^
	set(ADCSRA, ADPS0); // ^
	
	set(DIDR0, ADC0D);  // disable the 0 digital input
  set(DIDR2, ADC11D); // disable the 11 digital input
}

//_______________________________________ Subroutine for setting timer
void set_Timer(void){
		OCR1A = (unsigned int)62500/SAMPLE_FREQ; //Set OCR1A to get desired frequency (OCR1A = 62500/SamplingFrequency)
		set(TCCR1B, CS12);		// Set Timer 1 Precaler to /256
		clear(TCCR1B, CS11);	//^
		clear(TCCR1B, CS10);		//^		
		
		clear(TCCR1B, WGM13);	// Set Timer 1 to count UP to OCR1A (mode 4), then reset
		set(TCCR1B, WGM12);		//^
		clear(TCCR1A, WGM11);	//^
		clear(TCCR1A, WGM10);	//^

		set(TIMSK1, OCIE1A);	//Call Timer 1 interrupt whenever TCNT1 mathces OCR1A
}


//_______________________________________ Subroutine for updating ADCs
void update_ADC(){ 		//update to current ADC values, set to ADC_F0, B4

	//-------------------> set pin F0 to read ADC values
	clear(ADCSRB, MUX5); // single-ended channel selection
	clear(ADMUX, MUX2); // ^
	clear(ADMUX, MUX1); // ^
	clear(ADMUX, MUX0); // ^
	
	set(ADCSRA, ADEN); // start conversion process
	set(ADCSRA, ADSC); // ^
	while(!check(ADCSRA,ADIF));
  gyroVal = ADC;
  set(ADCSRA, ADIF); // sets flag after conversion


	//-------------------> set pin B4 to read ADC values
	set(ADCSRB, MUX5);  // B4
	clear(ADMUX, MUX2); // ^
	set(ADMUX, MUX1);   // ^
	set(ADMUX, MUX0);   // ^

	set(ADCSRA, ADEN); // start conversion process
	set(ADCSRA, ADSC); // ^
	
	while(!check(ADCSRA,ADIF));
  xAccelVal = ADC;
  set(ADCSRA, ADIF); // sets flag after conversion
	
}


//_______________________________________ Timer1 Interrupt Code @ OCR1A
ISR(TIMER1_COMPA_vect){ // once per sampling period, update the ADC values
	m_green(TOGGLE);
	//update_ADC();
	dsend = 1;
}



