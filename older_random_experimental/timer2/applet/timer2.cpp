#define TOGGLE_IO        1  //Arduino pin to toggle in timer ISR

#include "WProgram.h"
void SetupTimer1();
void setup(void);
void loop(void);
int counter;
 
void SetupTimer1()
{

  //Timer2 Settings: Timer Prescaler /8, mode 0
  //The /8 prescale gives us a good range to work with so we just hard code this for now.
  TCCR1A = 0;
  TCCR1B = 1<<CS12 | 0<<CS11 | 0<<CS10; 

  //Timer2 Overflow Interrupt Enable   
  TIMSK1 = 1<<TOIE2;

}

//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {

  counter ++;
  
}

void setup(void)
{
  //Set the pin we want the ISR to toggle for output.
  pinMode(TOGGLE_IO,OUTPUT);
  
  //Start the timer and get the timer reload value.
  SetupTimer1();
}


void loop(void)
{
    if (TCNT1 > 500)
   {
     digitalWrite(TOGGLE_IO,!digitalRead(TOGGLE_IO));  
     TCNT1 = 0;
   }
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

