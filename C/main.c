/*

 +-------------------------------------+
 |                          Sig Vd Gnd |
 |  +---------+   5V O     PB0 [o o o] | 
 |  | 7805  O |   Vd O     PB1 [o o o] | 
 |  +---------+   V+ .     PB2 [o o o] | 
 |                         PB3 [o o o] | 
 |                         PB4 [o o o] | 
 |                         PB5 [o o o] | 
 |                         PB6 [o o o] | 
 |                         PB7 [o o o] | 
 |                         PA0 [o o o] | 
 |                         PA1 [o o o] | 
 |        +----------+     PA2 [o o o] | 
 |        |O         |     PA3 [o o o] | 
 |        |          |     PA4 [o o o] | 
 |        |          |     PA5 [o o o] | 
 |        |          |     PA6 [o o o] | 
 |        |          |     PA7 [o o o] | 
 |        |          |     PC7 [o o o] |
 |        |          |     PC6 [o o o] |
 |        |          |     PC5 [o o o] |
 |        | ATMEGA32 |     PC4 [o o o] |
 |        |          |     PC3 [o o o] |
 |        |          |     PC2 [o o o] |
 |        |          |     PC1 [o o o] |
 |        |          |     PC0 [o o o] |
 |        |          |     PD7 [o o o] | 
 |        |          |     PD2 [o o o] |
 |        |          |     PD3 [o o o] | 
 |        |          |     PD4 [o o o] | 
 |        |          |     PD5 [o o o] | 
 |        +----------+     PD6 [o o o] |
 |      E.D.S BABYBOARD III            |
 +-------------------------------------+

  balancing robot test
    analog input for control signal on ADC0
    PWM output for motors on PD5, PD6
      using "tompwm" where side A is inverted from side B, 50% is "stop"


*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "avrcommon.h"
//#include <avr/signal.h>




#define OUTPUT   1
#define INPUT    0


#define Kp0 1


void pwmInit(void) ;
void setPWM(uint8_t ch, uint8_t val) ;


/*****************************| VARIABLES |********************************/
 
volatile int AdcValues[8];
volatile unsigned char AdcUpdate;
unsigned char pwmcount;
 
/************************| FUNCTION DECLARATIONS |*************************/
 
void AnalogInit (void);
int  Analog (int n);
void send8 (unsigned int bits);

/****************************| CODE... |***********************************/


int main (void) {
 
  long temp;
  unsigned int pwm0, pwm1;

  // set up directions       

 // DDRA = (INPUT << PA0 | INPUT << PA1 |INPUT << PA2 |INPUT << PA3 |INPUT << PA4 |INPUT << PA5 |INPUT << PA6 |INPUT << PA7);
 
  DDRB = (INPUT << PB0 | INPUT << PB1 |INPUT << PB2 |INPUT << PB3 |INPUT << PB4 |INPUT << PB5 |INPUT << PB6 |INPUT << PB7);
  DDRC = (INPUT << PC0 | INPUT << PC1 |INPUT << PC2 |INPUT << PC3 |INPUT << PC4 |INPUT << PC5 |INPUT << PC6 );
  DDRD = (INPUT << PD0 | INPUT << PD1 |INPUT << PD2 |INPUT << PD3 |INPUT << PD4 |OUTPUT << PD5 |OUTPUT << PD6 |INPUT << PD7);

  pwm0 = 128;  pwm1 = 128;
  AdcUpdate = 0;
  
  AnalogInit();
  pwm0Init();
  
  // turn interrupts on
  sei();
  
  // calculate servo loops
  while(1){
  
    while(AdcUpdate == 0) NOP();  // limit loop freq to the adc rate
    AdcUpdate = 0;    
    
    temp = (AdcValues[0] - 512)  ;
    temp *= ABS(temp);
    temp /= 80;
          
    //deadband
    if (inBounds(temp, 0, 25))  temp += 25;
    if (inBounds(temp, -25, 0)) temp -= 25;
    
    pwm0 = limit(temp, -127, 127) + 127;  
    
    setPWM(0, pwm0);  setPWM(1, pwm0);   // both channels the same.

    
  }

}

//------------------------| FUNCTIONS |------------------------


void setPWM(uint8_t ch, uint8_t val) {
  if (ch) {
     OCR0A = val;  
  } else {
     OCR0B = val;
  } 

}

int Analog (int n) {
  return AdcValues[n & 7];
}


/*

initialize pwm channels

16Mhz
/64 = ~1khz
/256 = ~240hz
/1024 = ~61Hz

*/
void pwm0Init() {
  // clear pwm levels
  OCR0A  = 0; 
  
  // set up WGM, clock, and mode for timer 0
  TCCR0A = 1 << COM0A1 | /* ** normal polarity */
           0 << COM0A0 | /*   this bit 1 for interted, 0 for normal  */
	   1 << COM0B1 | /* ** normal polarity */
           0 << COM0B0 | /*   this bit 1 for interted, 0 for normal  */
           1 << WGM00 | /* fast pwm */
           1 << WGM01  ;
          
  TCCR0B = 1 << CS00  ;  /* CLKio /1 */

  OCR0A = 128;  // 50% duty is stop.
  OCR0B = 128;
  
 }


void pwm2Init() {
  // clear pwm levels
  OCR2A = 128;
  OCR2B = 128;  
  
  // set up WGM, clock, and mode for timer 0
  TCCR2A = 1 << COM2A1 | /* ** normal polarity */
           0 << COM2A0 | /*   this bit 1 for interted, 0 for normal  */
	   1 << COM2B1 | /* ** normal polarity */
           0 << COM2B0 | /*   this bit 1 for interted, 0 for normal  */
           1 << WGM20 | /* fast pwm */
           1 << WGM21  ;
          
  TCCR2B = 1 << CS20  ;  /* CLKio /1 */


 }
 
 

void AnalogInit (void) {  

  // Activate ADC with Prescaler 
  ADCSRA =  1 << ADEN  |
            1 << ADSC  | /* start a converstion, irq will take from here */
            0 << ADATE |
            0 << ADIF  |
            1 << ADIE  | /* enable interrupt */
            1 << ADPS2 |
            1 << ADPS1 |
            1 << ADPS0 ;
                        
  ADMUX = (1<<REFS0);     // channel 0     
  
}


// reduced to only convert 4 channels.
ISR(ADC_vect) {
  int i;
  
  i = ADMUX & 3;       // what channel we on?
  AdcValues[i] = ADC;  // save value
  i++;                 // next value
  ADMUX = (i & 3) | (1<<REFS0);       // select channel
  ADCSRA |= _BV(ADSC); // start next conversion
  
  if (i == 3) AdcUpdate++;
  
  return;
}


