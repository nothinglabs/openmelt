#define TOGGLE_IO        1  //Arduino pin to toggle in timer ISR

int counter;
 
void SetupTimer1()
{

  TCCR1A = 0;                               //mode = 0
  TCCR1B = 1<<CS12 | 0<<CS11 | 0<<CS10;     //prescaler = 256

  //Timer2 Overflow Interrupt Enable   
  TIMSK1 = 1<<TOIE2;

}

//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {
  //  unused...
}

void setup(void)
{
  //Set the pin we want the ISR to toggle for output.
  pinMode(TOGGLE_IO,OUTPUT);
  
  //Start the timer
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
