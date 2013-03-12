// 3060   removed all ruddr, etc code
//  
// TRYING TO IMPLEMENT RUDDER = FULL THROTTLE rev 3030


 // added config switch from rich code to mine
// "Open Melt" - Open Source Translational Drift / Melty Brain Robot
// WinAVR-20090313  **************  RJW SPINNING TORTOISE VERSION BRUSHLESS  ***********
// removed on off and added true pwm to escs, based on throttle input
// rich's braking code or over 50% only


#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/sleep.h>
#include <avr/eeprom.h> 

//the following define statements setup which pins go to what
//it's a bit ugly - but it keeps all pin assignment stuff in one place

//pins for receiver
//note - these pins must be on PORT B so they're included in PCINT0
#define throttle_pin (PINB & 1<<PINB0)   					//portb.0 - throttle
#define set_throttle_pin_as_input() DDRB &= ~(1<<DDB0) 

//next four lines are for default control-stick port mappings - if using new-style "B" Baby Orangutan
// (comment them out if using an older non-"B" Baby Orangutan	
	#define leftright_pin (PINB & 1<<PINB1)					//portb.1 - left / right (for use with new "B" Baby Orangutan)
	#define set_leftright_pin_as_input() DDRB &= ~(1<<DDB1) 

	#define forwardback_pin (PINB & 1<<PINB2)					//portb.2 - forward / back (for use with new "B" Baby Orangutan)
	#define set_forwardback_pin_as_input() DDRB &= ~(1<<DDB2) 
	
/*
#define rudder_pin (PINB & 1<<PINB4)   					// portB.4 - rudder
#define set_rudder_pin_as_input() DDRB &= ~(1<<DDB4) 

#define gear_pin (PINB & 1<<PINB5)   					//portB.5 - gear sw
#define set_gear_pin_as_input() DDRB &= ~(1<<DDB5) 
*/
	


//definitions for LED pin
#define set_led_on() PORTD |= _BV(PD5)						//portd.5 - heading indicator LED control PIN (powers LED connected to motor controller on M1a / M1b)
#define set_led_off() PORTD &= ~_BV(PD5)	
#define toggle_led() PORTD ^= 1<<PIND5	
#define set_led_pin_as_output() DDRD |= 1<<DDD5


//definitions for accelerometer data pin
#define ADC_PORT_FOR_ACCEL 4									//adc.4 - accelerometer analog input 4 (pc4)
#define set_accel_data_pin_as_input() DDRC &= ~(1<<DDC4) 

//definitions for accelpower pin
#define set_accelpower_pin_on() PORTD |= 1<<PIND0				//portd.0 - accel power
#define set_accelpower_pin_as_output() DDRD |= 1<<DDD0



//the following receiver values may be modified to compensate for minor differences in radio systems
//all values are actually times - units are number of times the Atmel's timer fires between the PWM signal going high - then returning to low

long rotation_count = 0;									//continuously incremented - used for misc. stuff

int throttle_low = 375;   	                               //changed from 390 RJW throttle low RX value - includes some "slack" - value at which motors should actually kick in
int throttle_high = 565;									//100% value for throttle (if the signal goes significantly over this - it's considered bad)

int forwardback_center = 469;                              //center value for forward / back RX 
int forwardback_forwardthresh = +30;                       //center + 30 
int forwardback_backthresh = -30;                          //center - 30



// 20Mhx/64 (prescaler) = 312,500 or 312.5k  | convert to ms= 1000/312500 = .0032 | 1.2ms/.0032 = 375 = low |1.5ms = 468.75 centered | 1.8ms = 562.5 high
// based on spektrum receiver with travels at 100%

int heading_center = 469;                                  //center value for heading left/right RX
int heading_leftthresh = +20;                              //center + 20
int heading_rightthresh = -20;                             //center - 20

//int rudder_center = 469;                                  //center value for heading left/right RX
//int rudder_leftthresh = -20;                              //center + 20
//int rudder_rightthresh = +20;                             //center - 20



//int gear_low = 380;   	                               //changed from 390 RJW throttle low RX value - includes some "slack" - value at which motors should actually kick in
//int gear_high = 560;									//100% value for throttle (if the signal goes significantly over this - it's considered bad)






float min_rpm = 500;                     //minimum RPM for translation / throttling to kick in (if lower than - gets full power)
float max_allowed_rpm = 8000;                                //max_rpm allowed before cutting power
float max_g = 2000;                    	 //max g's before cutting power      *****changed from 190 rjw ******

float radius = .6;                        	 //effective radius of circle for accel (centimeters) - adjust as needed   ****** changed from 4.5  rjw *****
															//(lower numbers = "faster" tracking)
															
float g_per_adc_increment = .625;                            //10mv / g, 5mv per single increment from ADC up to 1024

	//the following variables can be tweaked to adjust for any tracking errors that seem to only happen when driving forward / backward
			//these don't seem needed (and default to 1.00) with the new floating point math  / improved motor control
			//it's possible they might still be needed depending on the physical bot construction - so tweak if neccessary
float forward_comp = 1.00;                                //heading compensation when going forward (1.00 = none)
float backward_comp = 1.00;                               //heading compensation when going back (1.00 = none)

float tracking_comp = 1.0;                       //tracking compensation - variable that accounts for differences in accelerometer placement
			                        //this is what gets set / saved in tracking adjustment in config mode
						//defaults to 1 (no adjustment)

float led_adjust = 63;                                     	//  **** changed from 13 orig = 63  RJW ********offset in % for LED heading
										//default value of 63 is about right if:
										//motor 1 is at 9 o-clock
										//motor 2 is at 3 o-clock (or if you're just using one motor)
										//LED is at 6 o-clock

int base_accel = 498;                             //default base (0g) value for accelerometer (this gets automatically reset in config mode)

float turn_speed = .0009;          	//greater number = faster steering -  increased / decreased to make directional control more / less responsive


long x;                                           		  	//general loop variable

unsigned int accel_raw_data;                           	//raw accelerometer data
float accel_read;                               	      	//single used to store accelerometer data

float delay_loop;                              	       	//used to add extra delay

int full_power_spin;                          		      	//if set to 1 - we're just spinning at full power (no translation)

int in_config_mode;											//set to 1 if we're in config mode

// int alternate_motor_cycle = 0;              				//flipped between 1 and 2 each spin - alternates which motor is used for power each cycle when not moving

int forward;                                    		   	//is 1 if robot is supposed to be going forward
int backward;                                  			   	//is 1 if robot is supposed to be going back

//int rudderyes;
//int gearyes;

float begin_brake;                               			 //point in spin to start brake
float end_brake;                                             //point in spin to end brake

int flashy_led;                        	        		   	//set to 1 if heading led is pulsing (used to indicate throttle)

float full_spin_time_ms;                               	//how long it takes for the robot to rotate once in milliseconds

float half_spin_time;                              		//delaytime refers to time spent in each "cycle"

float led_on;                                          		//offset in milliseconds for front LED to come on
float led_off;                                         		//offset in milliseconds for front LED to come on

float led_ref;                                         		//used to count through both cycles for LED reference point
long led_hold_over;

float throttle_percent;                             		//percentage of full throttle (spin rate)
float config_mode_throttle_percent;							//second copy of throttle percent used in config mode


float braking_length;                                  		//length of braking cycle in MS - used for throttling

float g;                                             		//g force the accelerometer is seeing

float rpm;                                           		//current RPM of robot

float steering_multiplier;                                 //total rotation time is multiplied for this amount to adjust for steering input from transmitter

int got_centered_forwardback = 0;			//set to true in safety loop once we've gotten a forwardback reading that looks centered 
                                            //- used to prevent going into config mode on boot
int rotations_since_throttle_was_set = 0;		//tracks number of times the bot has rotated without getting throttle data (safety)
int throttle_up_count = 0;				//counts number of times a "power-up" throttle has been received in a row (safety)

							//RC data channels - declared volatile since they're shared with the interrupt handler
volatile long leftright;                                    //heading RC channel
volatile long forwardback;                                  //forward/back RC channel
volatile long throttle;                                     //throttle RC channel
//volatile long rudder;
//volatile long gear;



int throttle_hilow;                               			//indicate if given RC channel was hi or low on last read
int forwardback_hilow;
int leftright_hilow;
//int rudder_hilow;
//int gear_hilow;



int led_is_on_now;                                        	//used to keep track if tracking LED should be on now or not

long max_observed_rpm = 0;                                   //tracks fastest RPM for last spin-up

long throttle_hightime = 0;            			//track value of timer for different channels upon going RX signal going HIGH
long forwardback_hightime = 0;                              
long leftright_hightime = 0;               
long rudder_hightime = 0;
long gear_hightime = 0;




// below used to convert throttle_ percentage to integer before assigning to OCR22A AND B

int bm1;
int bm2;

//float target_rpm;
//float new_rpm;
float save_throttle_percent;
//int rpm_locked;
float braking_length_adjust;



//EPROM variables - for saved configuration data
uint16_t EEMEM saved_data_valid;				//this value is set to 128 to indicate as a flag that other ROM values are good
uint16_t EEMEM led_adjust_save;									//led_adjust value - saved to ROM
uint16_t EEMEM tracking_comp_save_word1;						//tracking adjustment value - saved to ROM
uint16_t EEMEM tracking_comp_save_word2;						//tracking adjustment value - saved to ROM
uint16_t EEMEM heading_center_save;								//RC left/right center value - saved to ROM
uint16_t EEMEM base_accel_save;									//Accelerometer 0G value - saved to ROM


//eeprom_read_word(&base_accel_save);


void motors_off(void);
void motors_fullon(void); 

void throttle_change(void);
void forwardback_change(void);
void leftright_change(void);
//void rudder_change(void);
//void gear_change(void);



void SetupTimer1(void);
void SetupTimer2(void);   //added 7-20-1- for brushless  rjw
void wait24us(void);
void setup(void);
void loop(void);
void adc_init(void);
int read_adc(void);
void safety_and_idle(void);
void do_spin_180(int);
void main_calculations(void);
void config_mode(void);
void get_config_constants(void);
void load_config(void);
void save_config(void);
void reset_rc(void);

int main(void)
{
	TCNT2=0;  // added 7/20/2010 rjw for brushless  *********************************************************  




		
	setup();			//do initial setup stuff - set pins / interrupts / etc.
			
	load_config();		//try to load configuration data from ROM
				
	//execute the main loop indefinitely...
	


	DDRD |= 1<<DDD3;   //output
	PORTD &= ~_BV(PD3); //off

	OCR2B = 0;  //set low - just one motor
	
	
	while (1)
	{		
		
					//the lines before main_calculations don't have their execution time accounted for in the code 
					//but only take approximately 2us to execute (measured) - (0.006% of a rotation at 2000 rpm)
												
		rotations_since_throttle_was_set ++;	//used as a safety counter - if no good throttle data is received for certain number of rotations 
      	rotation_count ++;                      // - the bot shuts down
				
		safety_and_idle();						//does safety check / sees if we're just sitting idle - also checks if config mode is requested
		
	//	alternate_motor_cycle = !alternate_motor_cycle; //?     //alternates alternate_motor_cycle - used to balance spin / avoid favoring one motor

		led_hold_over = 0;						//reset the LED counter
		
		main_calculations();					//reads accel data and does all the math
												//takes about 400us - but is measured real-time / compensated for

		do_spin_180(1);							//1st 180 degrees of spin

		led_hold_over = TCNT1;					//carry over led_counter from last spin
				
		main_calculations();					//read accel / do the main calculations again
						//this is really only here to assure timing "balance" betweeen the two rotation cycle halves
	//time doing math is tracked - but since time doing calculations is outside the motor loop - if it all falls in one cycle or the other...
								//..it may still be enough to cause a slight bias when translating
								//should really present a very small amount of time (less than 1% at 2000rpm) 
   							//since we're resampling the accel - this may also provide a little better accuracy

		do_spin_180(2);							//2nd 180 degrees of spin
				
	}
	
	
	return(0);

}



void safety_and_idle(void)
{
	
	sei();  //enable interrupts (needed to get transmitter data)
	
	//if throttle is lower than throttle_low - or is over 100 beyond throttle_high - bot stays powered down
	//also - if we've gone more than 11 rotations without getting fresh throttle data - assume something has gone wrong / shutdown
	//since max allowed rotation time is 400ms - should always fail-safe in under 5 seconds
	//in addition - requires 4 good "throttle up" reads in a row before allowing the loop to be left 
	//(hopefully prevents stray RC data from causing spin-up)
		

	while (throttle < throttle_low || throttle > (throttle_high + 100) || rotations_since_throttle_was_set > 11 || throttle_up_count < 4)
	{


		motors_off();			//motors are off while sitting idle

		if (throttle < throttle_low || throttle > (throttle_high + 100)) throttle_up_count = 0;		//single low / bad throttle resets the counter to 0
		if (throttle > throttle_low && throttle < (throttle_high + 100)) throttle_up_count ++;		//if the throttle has been moved high - increment the counter

		//interrupt blinking if stick isn't centered (helps to verify TX is working)
		if ( leftright > (heading_center + heading_leftthresh) ) {set_led_on(); _delay_ms(200);}
		if ( leftright < (heading_center + heading_rightthresh) ) {set_led_on(); _delay_ms(200);}

		//sit there and flash LED
		
		toggle_led();
		_delay_ms(30);		
		

		//slower LED flash if in config mode
		if (in_config_mode == 1) {set_led_off(); _delay_ms(200);}		
		
		
		//verifies we got a centered forwardback stick at least once before allowing config mode (prevents boot directly into config mode if fail-safe is below center)
		if (forwardback > (forwardback_center + forwardback_backthresh) && forwardback < (forwardback_center + forwardback_forwardthresh))
		{

			_delay_ms(10);
			if (forwardback > (forwardback_center + forwardback_backthresh) && forwardback < (forwardback_center + forwardback_forwardthresh))	//check it again to be sure
			{
				got_centered_forwardback = 1;
			}
		}
 

		//check for enter / leave config mode
		if (forwardback < (forwardback_center + forwardback_backthresh) && got_centered_forwardback == 1)		//is the stick being held back?
		{

			//wait a bit to make sure stick is being held...
			_delay_ms(1000);
			//still being held back - then enter / leave config mode
			if (forwardback < (forwardback_center + forwardback_backthresh))
			{
				in_config_mode = !in_config_mode;
				_delay_ms(1500);				//delay a bit longer to help assure config_mode isn't toggled again

				cli();                	//disable interrupts - seems like a good idea before saving stuff to ROM
				if (in_config_mode == 1) get_config_constants();		//read + set a few constants prior to actually going into config mode
				if (in_config_mode == 0) save_config();				//if we're exiting config mode - save the configuration
				sei();  //re-enable interrupts

			}

			
		}
		      
/*
// below code to use left right stick to change from new config to old

	if (leftright < (heading_center + heading_rightthresh) && (throttle < throttle_low) && (use_code == 0))
		{
		   use_code = 1;
			set_led_off();
			
			//if we haven't recorded an RPM - show a little status flash to show we have signal
				for (x = 0; x < 3; x++)
				{
					set_led_on();
					_delay_ms (5);
					set_led_off();
					_delay_ms (800);				
				}
			}
if (leftright > (heading_center + heading_leftthresh) && (throttle < throttle_low) && (use_code == 1))
		{
		   use_code = 0;
			set_led_off();
			
			//if we haven't recorded an RPM - show a little status flash to show we have signal
				for (x = 0; x < 5; x++)
				{
					set_led_on();
					_delay_ms (5);
					set_led_off();
					_delay_ms (600);				
				}
			}
			*/

//*********************************************************************
	  
		// if stick is forward - flash out highest rpm this boot
		while (forwardback > (forwardback_center + forwardback_forwardthresh) && throttle < throttle_low)
		{
		   
			set_led_off();
			
			//if we haven't recorded an RPM - show a little status flash to show we have signal
			if (max_observed_rpm == 0)
			{
				for (x = 0; x < 15; x++)
				{
					set_led_on();
					_delay_ms (5);
					set_led_off();
					_delay_ms (30);				
				}
			}
			
			_delay_ms (800);
			
			x = 49;					//little confusing - but this effectively rounds up (600 rpm = 6 flashes, 650 rpm = 7 flashes)
			while ((x < (max_observed_rpm)) && (forwardback > (forwardback_center + forwardback_forwardthresh)) && throttle < throttle_low)
			{
				x = x + 100;		
				set_led_on();
				_delay_ms (50);
				set_led_off();
				_delay_ms (400);
			
			}

		}


	}

}

void load_config(void)
{

	float tracking_word1;		//first word of tracking_comp
	float tracking_word2;		//second word of tracking_comp

	//only load config data if "saved_data_valid" indicates it was saved previously
	if (eeprom_read_word(&saved_data_valid) == 128)
	{

		led_adjust = eeprom_read_word(&led_adjust_save);	//loads led offset
		tracking_word1 = (eeprom_read_word(&tracking_comp_save_word1));		//loads tracking comp
		tracking_word2 = (eeprom_read_word(&tracking_comp_save_word2));		//loads tracking comp
		heading_center = eeprom_read_word(&heading_center_save);	//loads heading_center
		base_accel = eeprom_read_word(&base_accel_save);		//loads base accelerometer value
	
		tracking_word1 = tracking_word1 / 1000;	//converts 1st tracking comp word back to float
		
		tracking_word2 = tracking_word2 / 1000;		//converts 2nd tracking comp word back to float
		tracking_word2 = tracking_word2 / 10000;	//put it in the correct decimal place
		
		tracking_comp = tracking_word1 + tracking_word2;	//puts the two floats together
	}

}

void save_config(void)
{
	
	long tracking_word1;	//first word of tracking_comp
	float tracking_word2;	//second word of tracking_comp
	
	//this code busts up tracking_comp (float) into two words for storage to ROM (there are probably cleaner ways to do this)
	
	tracking_word1 = tracking_comp * 1000;							//mulitply tracking_comp by 1000 to get 1st word
	tracking_word2 = ((tracking_comp * 1000) - tracking_word1);	//amount that didn't make it into word1 goes into word2
	tracking_word2 = tracking_word2 * 10000;						//multiply that by 10,000
		
	//EPROM variables - for saved configuration data
	eeprom_write_word(&saved_data_valid, 128);	//used as an indicator that saved data is good
	eeprom_write_word(&led_adjust_save, led_adjust);	//saves out led offset
	eeprom_write_word(&tracking_comp_save_word1, tracking_word1);	//saves out tracking calibration word1 (converted to integer)
	eeprom_write_word(&tracking_comp_save_word2, tracking_word2);	//saves out tracking calibration word2 (converted to integer)
	eeprom_write_word(&heading_center_save, heading_center);	//saves out RC center value for left/right 
	eeprom_write_word(&base_accel_save, base_accel);	//saves out accelerometer 0G value


}

void get_config_constants(void)
{

	// sample and set the accelerometer base value (average a bunch of samples)

	base_accel = 0;
	for (x = 0; x < 20; x++)
	{
		base_accel = base_accel + read_adc();               				//get accel data
		_delay_ms(10);
	}
	base_accel = base_accel / 20;


	// sample and set the left / right center value for the control stick (average a bunch of samples)

	heading_center = 0;
	for (x = 0; x < 20; x++)
	{
		heading_center = heading_center + leftright;               		
		_delay_ms(20);

	}
	heading_center = heading_center / 20;


}

void config_mode(void)
{
	

	//basic tracking adjustment code (is under 50%)
	if (config_mode_throttle_percent < 50)
	{
	
		flashy_led = 0;			//flashy LED off unless something is changing
		//flashy LED gets turned off to indicate change
		
		//   *******************  changed below from .003 to .001  more precise ??  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX)))))))))))))))))))))))))))))
		
		if ( leftright > (heading_center + heading_leftthresh) ) {tracking_comp = tracking_comp + (tracking_comp * 0.002); flashy_led = 1;}
		if ( leftright < (heading_center + heading_rightthresh) ) {tracking_comp = tracking_comp - (tracking_comp * 0.002); flashy_led = 1;}	

		if (tracking_comp < 0.1) tracking_comp = 0.1;
		if (tracking_comp > 10) tracking_comp = 10;
	}	


	//heading adjustment code (when throttle is between 50% and 90%)
	if (config_mode_throttle_percent >= 50 && config_mode_throttle_percent < 90)
	{
		
		flashy_led = 1;				//pulse the LED to indicate we're in heading adjustment


		//   *******************  changed below from 1 to .1  more precise ??  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX)))))))))))))))))))))))))))))


		if ( leftright > (heading_center + heading_leftthresh) ) {led_adjust = led_adjust + 1; flashy_led = 0;}   //flashing gets turned back off to indicate change
		if ( leftright < (heading_center + heading_rightthresh) ) {led_adjust = led_adjust - 1; flashy_led = 0;}

		if (led_adjust < 0) led_adjust = 100;
		if (led_adjust > 100) led_adjust = 0;

	}	
	
	//if we're above 90% throttle - the bot is effectively in normal drive mode - with throttle locked at 50%


}


//sets up all the pins and interrupts
void setup(void)

{

	adc_init();		//init the ADC...

	set_throttle_pin_as_input();
	set_leftright_pin_as_input();  
	set_forwardback_pin_as_input();
	
	//set_rudder_pin_as_input();
	//set_gear_pin_as_input();
	

  
	set_accel_data_pin_as_input();
	
	set_accelpower_pin_as_output();
	set_accelpower_pin_on();								//turn on power for accel (accel is connected to chip for power)


	
	set_led_pin_as_output();

//	set_motor1_pin_as_output();
//	set_motor2_pin_as_output();

	PORTB |= 1 << PORTB3;  // RJW brushless  ******************************************************
    PORTD |= 1 << PORTD3; //  RJW brushless
	
	set_led_on();						                    //turn on signal LED before timers so it comes on immediately
	

	//enable pin change interrupt - any changes on PORTB trigger interrupt
	PCMSK0 = 0xFF;
	PCICR = 1<<PCIE0; 
	  
	SetupTimer1();   //fire up timer1 (2 bytes) - accessed via TCNT1 variable

        SetupTimer2();   //setup timer 2 for brushless    added 7-20-10   RJW  *******************************************

	motors_off();   //make sure those motors are off...


	//flash LED on boot	(fast - so visible if spinning)
	for (x = 1; x <= 250; x++)
	{
		toggle_led();
		_delay_ms(1);
	}


	//flash LED on boot (slower)
	for (x = 1; x <= 15; x++)
	{
		toggle_led();
		_delay_ms(15);
	}

	throttle = 0;   //make sure throttle is off at boot
	
//	target_rpm = 0;
//	new_rpm = 0;
	save_throttle_percent = 0;


}



void main_calculations(void)

{

	reset_rc();				//resets existing RC data - must get called before timer1 is reset to prevent errors
	
	TCNT1 = 0;				//resets timer (used to track for time spent outside motor drive loop)
							//by resetting timer at beginning over each call to main_calculations - the calc time effectively gets 
							//included in the do_spin loop


	cli();                	//disable interrupts - bad things seem to happen if the RC interrupts get triggered while doing math...
							//RC data is not updated while in this code

	//Are we going forward or backwards?
	if ( forwardback > (forwardback_center + forwardback_forwardthresh)) forward = 1; else forward = 0;
	if ( forwardback < (forwardback_center + forwardback_backthresh)) backward = 1; else backward = 0;


//	if ( rudder < 400 ) rudderyes = 1; else rudderyes = 0;



	flashy_led = 0;											//by default LED isn't flashy
	
	/*
	
    if ((forward == 1) || (backward == 1 ))   //  *********rjw code to switch configs for testing*******************************************
	{
      if (rpm_locked == 0) 
	  
	     target_rpm = rpm; 
		 save_throttle_percent = throttle_percent;  //rjw //rjw
	     rpm_locked = 1;
	  
	  }
	  
	  else 
	  
	  {
	  rpm_locked = 0;
	  save_throttle_percent = 0;
	  
		
	}
	
	*/

	accel_raw_data = read_adc();               				//get accel data
	
	accel_read = accel_raw_data;                              //move it over to single in case we want to do floating point
	accel_read = accel_read - base_accel;                     //compensate for base (2.5v) level
	g = accel_read * g_per_adc_increment;                    //convert to g's

	rpm = g * 89445;                                //calculate RPM from g's - derived from "G = 0.00001118 * r * RPM^2"
	rpm = rpm / radius;
	rpm = pow(rpm, .5);								
	
	if (rpm > max_observed_rpm) max_observed_rpm = rpm;		//update max_observed_rpm if current rpm is higher
	
		
	
	full_spin_time_ms = rpm / 60;                                      //convert RPM to duration of each spin in milliseconds
		
	if (full_spin_time_ms == 0) full_spin_time_ms = 1;                          //must prevent any possible division by zero!!!
	full_spin_time_ms = 1 / full_spin_time_ms;
	full_spin_time_ms = full_spin_time_ms * 1000;									//seconds to milliseconds

	full_spin_time_ms = full_spin_time_ms * tracking_comp;                      //compensate with user-set tracking adjustment

  
	if ( forward == 1 ) full_spin_time_ms = full_spin_time_ms * forward_comp;   //extra compensation if going forward
	if ( backward == 1 ) full_spin_time_ms = full_spin_time_ms * backward_comp; //extra compensation if going backward



	//converts throttle reading from remote into percentage
	throttle_percent = ((throttle - throttle_low) * 100) / (throttle_high - throttle_low);
	if ( throttle_percent < 10 ) throttle_percent = 10;    //   CHANGED FROM 12 boh  **** RJW ******    don't got under X%	 (throttle percent is only set if we exit safety - so lack of "0" throttle isn't a problem)
	if ( throttle_percent > 100 ) throttle_percent = 100;    //don't got over 100%
	
	
//   XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXOOOOOOOOOOOOOOOOOO  below is config rpm percent  OOOOOOOOOOOOOXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX



	config_mode_throttle_percent = throttle_percent;				//second copy of variable used in config mode (since we're otherwise locking it at 50%)
	if (in_config_mode == 1) throttle_percent = 30;		//*************  rjw changed from 50	//throttle is locked at 50 percent in config mode




	//calculates + modifies changes to heading based on input from transmitter - not done if in config mode (and not in the 90+% normal drive mode)
	
	if (in_config_mode == 0 || config_mode_throttle_percent > 90)
	{
		steering_multiplier = heading_center - leftright;
		steering_multiplier = steering_multiplier * turn_speed;
		steering_multiplier = 1 - steering_multiplier;		//starts with 1 as a base value (ie - if it was 0.0 it becomes 1.0 - so there's no change in heading)
		full_spin_time_ms = full_spin_time_ms * steering_multiplier;
	}
	

	half_spin_time = full_spin_time_ms / 2;                          //sets period in MS for each half of spin

	//caps on timing if going too slow or fast
	if ( half_spin_time > 200) half_spin_time = 200;			//slowest allowed - 200ms per half-cycle = 150rpm
	if ( half_spin_time < 4) half_spin_time = 4;				//fastest - 5ms per half-cycle = 6000rpm


	//set heading beacon size and location

	led_on = full_spin_time_ms * led_adjust;
	led_on = led_on / 100;
	led_off = full_spin_time_ms / 3;            //led signal is 33% of circle
	led_off = led_off + led_on;


	if (led_off >= full_spin_time_ms ) //if led_off is "later" or at end of cycle - shift led_off behind by one cycle
	{                        
		led_off = led_off - full_spin_time_ms;
	}

	if ( led_off < 1 ) led_off = led_off + full_spin_time_ms;

	//throttling

	full_power_spin = 0;
	//if ( rpm < min_rpm ) full_power_spin = 1;             //if we're under the minimum RPM for translation - do the full power spin!
	if ( g > max_g || rpm > max_allowed_rpm ) throttle_percent = 10; 	     //if we're over max alowed G's or RPM - reduce throttle
	
	
	
	    bm1 = (int)(((throttle_percent)* .7) + 78.5);  //added .5 to round up inthat mcu will truncate 
		bm2 = (int)(((throttle_percent)* .7) + 78.5);

        if (bm1 > 135) bm1 = 135; // 130 = 2600 rpm
        if (bm2 > 135) bm2 = 135;

	
		braking_length = half_spin_time * 25;                  
		braking_length = braking_length / throttle_percent;  
		
		begin_brake = half_spin_time / 2;
		begin_brake = begin_brake - braking_length;

		end_brake = half_spin_time / 2;
		end_brake = end_brake + braking_length;
	
		if ( begin_brake < 1 )	begin_brake = 1; 
  
   
		/*
	     if (in_config_mode == 0 && rudderyes == 1)  // *********************** added to go full throttle on rudder command
		 {
		  
		  bm1 = 148;
          bm2 = 148; 
		   
		   
		 }
         */
	
	//if throttle is at or over 50% throttle - adjust time spent in braking
	if ( throttle_percent > 50 )
	{                          

		flashy_led = 1;                                        //flash the LED to indicate we're in fast mode

	
	}
	

	
	if (in_config_mode == 1) config_mode();	//do config_mode stuff if we're in configuration mode
		
	//adjustments - convert times from milliseconds to timer ticks
	led_on = led_on * 312.5;
	led_off = led_off * 312.5;
	half_spin_time = half_spin_time * 312.5;
	begin_brake = begin_brake * 312.5;
	end_brake = end_brake * 312.5;

	sei();  //enable interrupts to allow updating of transmitter data - out of all the critical stuff
	

}



void do_spin_180(int spin_cycle)
{	

	while (TCNT1 < half_spin_time)
	{
	
		if (TCNT1 < begin_brake) //motors_left();				//full power if we haven't entered braking yet
		
		{
		  
		  OCR2A = bm1;
		  //OCR2B = bm2;
		} 
		  
		if (TCNT1 > begin_brake) // motors_left();				//full power if we're after braking
		{
		  
		  OCR2A = bm1;
		  //OCR2B = bm2;
		} 
		

		led_ref = TCNT1 + led_hold_over;			//provides continuous LED tracking between the two do_spin loops

		if ( TCNT1 > begin_brake && TCNT1 < end_brake )        //switch to single motor as soon as entering braking cycle
		{
			//if sitting still
			if ( forward == 0 && backward == 0 )
			{
				
			    
		        OCR2A = bm1;
		        //OCR2B = bm2;	
				

			}

			//if ( going forward / back set motors appropriately (this is "where it happens")
			if ( forward == 1)
			{	
				if (spin_cycle == 1) //motor1_on();
				
			   {
				OCR2A = 148; //141;
				//OCR2B = 95;  //101;//91 too slow?
			   } 

				
				
				if (spin_cycle == 2) //motor2_on();
				
			   {
				OCR2A = 95;
				//OCR2B = 148;
			   } 

			}

			if ( backward == 1)
			{	
				if (spin_cycle == 1) //motor2_on();

			   {
				OCR2A = 95;
				//OCR2B = 148;
			   } 
		
				
				if (spin_cycle == 2) //motor1_on();
							   
			   {
				OCR2A = 148;
				//OCR2B = 95;
			   } 

				
				
			}
				
		}

		if ( TCNT1 > end_brake ) //motors_left();                 //if we hit end of brake cycle - go to full power
         {
		    
		    OCR2A = bm1;
		    //OCR2B = bm2;
		 } 

       

		
		//following code handles turning on and off LED (little confusing)
		
		if (led_on > led_off)
		{
			led_is_on_now = 1;
			if ( led_ref > led_off ) led_is_on_now = 0;
			if ( led_ref > led_on ) led_is_on_now = 1;
		}


		if (led_off > led_on)
		{
		
			led_is_on_now = 0;
			if ( led_ref > led_on ) led_is_on_now = 1;
			if ( led_ref > led_off ) led_is_on_now = 0;
		
		}
	
	
		if ( led_is_on_now == 1 )
		{
			//flash the LED if we're in flashy mode - otherwise it's just on
			if ( flashy_led == 1 )
			{
				
				if ((TCNT1 / 160) % 2 == 0) set_led_on(); else set_led_off();
			}
			else
			{
				set_led_on();
			}
		}

		if ( led_is_on_now == 0 )
		{
			set_led_off();
		}
		
	}

}


void motors_off(void)
{

        OCR2A = 78;
       // OCR2B = 78;

}




void motors_fullon(void)
{

        OCR2A = 148; //156?
       // OCR2B = 148;
    
}


//main interrupt handler - is called any time any ports on PORTB change

ISR (PCINT0_vect)
{
	
	//check all RC channels to see if they were updated
	if (throttle_hilow != throttle_pin) throttle_change();
	if (leftright_hilow != leftright_pin) leftright_change();
	if (forwardback_hilow != forwardback_pin) forwardback_change();
    //if (rudder_hilow != rudder_pin) rudder_change();
    //if (gear_hilow != gear_pin) gear_change();

}


void reset_rc(void)
{
	//this routine voids existing RC data - used if timer is going to get reset
	throttle_hightime = 0;
	leftright_hightime = 0;
	forwardback_hightime = 0;
    //rudder_hightime = 0;
	//gear_hightime = 0;


}

//updates RC channels any time specified pin goes high/low
//following 3 routines are all identical except for channels and some safety code in throttle_change (not easy to consolidate them)
void throttle_change(void)
{
	
	//did the pin go HIGH? - then note time
	if ( throttle_pin != 0 ) throttle_hightime = TCNT1;    

	//did the pin go low? - then set timer value as value for this channel / if timer has overflowed then ignore
	if ( throttle_pin == 0 && throttle_hightime != 0 && TCNT1 > throttle_hightime)
	{		
		throttle = ((throttle * 85) + ((TCNT1 - throttle_hightime) * 15)) / 100;		//smoothed RC data
																						//throttle is highly smoothed to help prevent accidental spin-down
		rotations_since_throttle_was_set = 0;					 //note that throttle was successfully set (for safety)
	}


	throttle_hilow = throttle_pin;                                  //make note of pin state for reference next time interrupt is triggered...	
	
}

void leftright_change(void)
{
	//did the pin go HIGH? - then note time
	if (leftright_pin != 0 ) leftright_hightime = TCNT1;    

	//did the pin go low? - then set timer value as value for this channel / if timer has overflowed then ignore
	if (leftright_pin == 0 && leftright_hightime != 0 && TCNT1 > leftright_hightime)
	{
			leftright = ((leftright * 50) + ((TCNT1 - leftright_hightime) * 50)) / 100;		//smoothed RC data
	}
  
	leftright_hilow = leftright_pin;                                  //make note of pin state for reference next time interrupt is triggered...	
		
}

void forwardback_change(void)
{
	//did the pin go HIGH? - then note time
	if (forwardback_pin != 0 ) forwardback_hightime = TCNT1;    

	//did the pin go low? - then set timer value as value for this channel / if timer has overflowed then ignore
	if (forwardback_pin == 0 && forwardback_hightime != 0 && TCNT1 > forwardback_hightime) forwardback = TCNT1 - forwardback_hightime;      
  
	forwardback_hilow = forwardback_pin;                               //make note of pin state for reference next time interrupt is triggered...

}


/*
void rudder_change(void)
{
	//did the pin go HIGH? - then note time
	if (rudder_pin != 0 ) rudder_hightime = TCNT1;    

	//did the pin go low? - then set timer value as value for this channel / if timer has overflowed then ignore
	if (rudder_pin == 0 && rudder_hightime != 0 && TCNT1 > rudder_hightime)
	{
			rudder = ((rudder * 50) + ((TCNT1 - rudder_hightime) * 50)) / 100;		//smoothed RC data
	}
  
	rudder_hilow = rudder_pin;                                  //make note of pin state for reference next time interrupt is triggered...	
		
}


void gear_change(void)
{
	
	//did the pin go HIGH? - then note time
	if ( gear_pin != 0 ) gear_hightime = TCNT1;    

	//did the pin go low? - then set timer value as value for this channel / if timer has overflowed then ignore
	if ( gear_pin == 0 && gear_hightime != 0 && TCNT1 > gear_hightime)
	{		
		gear = ((gear * 85) + ((TCNT1 - gear_hightime) * 15)) / 100;		//smoothed RC data
																						//throttle is highly smoothed to help prevent accidental spin-down
		//rotations_since_throttle_was_set = 0;					 //note that throttle was successfully set (for safety)
	}


	gear_hilow = gear_pin;                                  //make note of pin state for reference next time interrupt is triggered...	
	
}

*/



void SetupTimer1(void)
{

	TCCR1A = 0;                               //mode = 0
	TCCR1B = 0<<CS12 | 1<<CS11 | 1<<CS10;    //prescaler = 64

}

void SetupTimer2(void)
{

// OCR2A - PB3                                                              
// OCR2B - PD3                                                               

DDRB |= 1 << DDB3; 
//DDRD |= 1 << DDD3;
TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
TCCR2B = _BV(CS22) | _BV(CS20);
 
}


void adc_init(void)

{
 
  	// Free running Mode
	ADCSRB = 0x00;

   // Set ADCSRA Register in ATMega168
   ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);

   // Set ADMUX Register in ATMega168
   ADMUX=ADC_PORT_FOR_ACCEL;	

   
}


int read_adc(void)
{

    unsigned int adc_data;
	
	// Start conversion by setting ADSC on ADCSRA Register
	  ADCSRA |= (1<<ADSC);

	// wait until convertion complete ADSC=0 -> Complete
    while (ADCSRA & (1<<ADSC));
	    
	adc_data = ADCL;   

	//shift from low level to high level ADC, from 8bit to 10bit

	adc_data += (ADCH<<8);	  

	return (adc_data);

}



