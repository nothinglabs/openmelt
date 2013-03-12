//Melty B - Translational Drift / Melty Brain combat robot
//Copyright Rich Olson - 2005-2009


// WARNING / DISCLAIMER:
// YOU MUST READ THIS!!!!

// This code is provided for your use without warranty
// It is not advertised as being for any specific use
// The author is making this code available for your intellectual curiosity
// If you should choose to attempt compiling / deploying it - you are placing your life in danger!

// Semi-autonomous systems can be extremely dangerous

// You should assume at any time the robot when powered may "spin-up" (even if not instructed to by the operator)
//    and cause corresponding damage / injury / death.
// This may be caused by any number of errors in implementation - However - have no apparent cause at all.
// THIS IS A REAL CONCERN - MOTOR "GLITCHES" HAVE BEEN OBSERVED ON SOME PLATFORMS DURING TESTING - THAT HAVE NEVER BEEN FULLY UNDERSTOOD / EXPLAINED
// In addition - this code may have "bugs" that may further endanger anyone attempting to utilize it

// A 1lbs spinning robot is likely capable of causing significant injury
// Larger robots may very plausibly kill someone.
// SPECIFICALLY - BUILDING A TRANSLATIONAL DRIFT ROBOT OVER ABOUT 6 LBS IS ONE OF THE MORE DANGEROUS THINGS I CAN IMAGINE DOING

// If you are intent on placing your life in danger by building a semi-autonomous robot
//		you may want to consider building a less-dangerous (under 1lbs) test-platform to better understand how things work

// The author is not responsible for any damages / injury / death resulting from use of this code
// Your life is in your hands - don't sue me.



// Hardware List:

// MCU Board: Pololu (pololu.com) Baby Orangutan / B-328 (or B-168) Robot Controller (Atmega 328 / 20MHZ crystal)
// Accelerometer: Freescale 200g MMA2301Eg (Mouser.com) 
// NPN Darlington transistor(s) to drive motor(s)
// 		MJ11032G - moderate power - about 15 amps @ 10v (see data sheet for more info)
//		TIP120 (Radio Shack) - useable for very small test platforms 
//		Jaycar SY-4086 - Solid state relay - 100 amps / 32V (requires an extra diode / minor wiring differences - read the spec sheet closely)
// Some kind of motor(s)
// 		Either 1 or 2 motors may be used (maybe more if you get creative)
// A heading LED (1 to 3 Watts)
// 5V Regulator (see note below)
// Some resistors (detailed in wiring notes)


// General Notes:

// This system uses an accelerometer to calculate the rate of rotation based on G-forces around a given radius.
// That data is then used to light up an LED once per rotation - giving the appearance of the "front" of the robot.
// The user can adjust the heading beacon by moving the remote control left or right.  
// To move - the system turns on the motor(s) on when that motor is in the correct position to result in a net movement the direction the robot is intended to go.
// For example - if the heading beacon is on approximately between 10 o-clock and 2 o-clock it indicates movement will be towards 12 o-clock.
// Pushing "forward" on the remote causes the robot to turn on the motor(s) between 6 and 12 o-clock each rotation - the net direction of travel being towards 12 o-clock.
// When the robot is desired to be standing still - the system alternates when the motor(s) are powered to cancel out any translation.


// This code assumes the motors will be driving the bot counter-clockwise (you may be able to change this just by reversing some channels on your transmitter)

// Motors are always on or off.  Throttle control is achieved by adjusting the duration the motor(s) are on each rotation.
//		For instance - at low throttle - the motors may only be on in the example above between 8 and 10 o-clock)
//		At high throttle - the motors may only be on between 5 and 1 o-clock

// When spinning up motor(s) will be on all the time until a certain RPM is reached (this is set in a variable in the code)

// This code will work with 1 or 2 motors without any special configuration changes (in fact - you may not immediately notice if a motor fails)

// The only thing this code depends on specific to the Baby Organgutan as opposed to any other Atmel board is its motor driver which is used to drive the heading LED
// This could of course be replaced with a transistor.  A low-power LED could also be driven directly by the Atmel (probably good enough for testing - but not combat).

// If you are using an earlier (non-B) Baby Orangutan - some pins may need to be re-assigned

// Mixing on the transmitter should be off.  Servo reversing should also be off (play with this if it seems needed).

// The voltage regulator will need to be capable of powering the LED - which may draw over 500ma (current draw from other components is probably under 100ma).
// This may not sound like a lot - but with conventional regulators - the higher the source voltage - the lower the available current.
// A standard regulator is probably fine up to about 11.1v driving an LED at 2 Watts.  Above that - a "switching" style regulator may be required.

// You can get away with mounting the MMA2301Eg at about any angle - but the pinned side with the "dot" needs to be towards the "center" of the bot
// Intentionally mounting the accelerometer at angle (say 45 degrees) - effectively decreases the G's it sees and increases its range (you'll of course need to calibrate)
// Use http://www.djblabcare.co.uk/djb/info/6/User_Tools to verify you're mounting the accelerometer at a position that doesn't exceed its limits

// If you're having problems with reboots / the receiver cutting out / or losing calibration  - the cause is often low voltage
// If you think you're going under 7.4v under load - there's a pretty decent chance any problems are voltage related
// In short - running off two LiPo cells at 7.4v nominal may work with a light motor load - but if you're having trouble - going to 11.1v might help


// Physical Build Notes

// More weight seems to result in poorer translation
// More power seems to result in better translation
// What works well small may not scale up (larger / heavier designs have generally performed poorer - even if the power seemed ample)
// Softer wheels seem to result in better translation (--foam-- works really well)
// One-wheel bots with hard wheels seem specifically prone to "bounce" off floor imperfections when spinning - resulting in poor translation
// If a one-wheeled bot is scraping - look at how it's balanced "vertically" - ie try moving weight higher / lower 
// A spinning robot may see internal static G forces approaching 200G's - remember this may spike MUCH higher upon impact - expect things to fail in unexpected ways



// Wiring Notes:

// Google datasheets for pin layout on all items
// This list should include all non-obvious required connections
// You are ultimately designing your own circuit - I'm assuming you can figure out how to wire in a voltage regulator / etc.
// If this list seems short - there really are only a handful of connections.  The wiring isn't that complicated.


// Accelerometer
// The Freescale 200g MMA2301Eg is a surface mount component - Sparkfun.com's 20-pin SOIC to DIP Adapter may be used to help with soldering
// VDD goes to PD0 on Baby Orangutan - (power for the accelerometer is sourced from the microcontroller)
// VSS goes to Ground
// VOut goes to PC4 on Baby Orangutan


// Receiver
// Needs at least one of the "positive" pins connected to 5v (probably from voltage regulator)
// Needs at least one of the "ground" pins connected to ground
// Signal for Elevon (forward / back) channel goes to PB4 
// Signal for Aileron (left / right) channel goes to PB3
// Signal for Throttle channel goes to PB0


// Darlington Driver(s) / Motor(s)
// The "Base" on the darlington driver connects to a 100 ohm resistor
//		The opposite side of the resistor connects to PD2
//		The resistor may not technically be needed - but assures current levels don't excede what the chip can provide
// "Emitter" on darlington connects to Ground

// + Motor Terminal connects to + Battery
// - Motor Terminal connects to "Collector" on Darlington
//		If the robot turns the wrong direction - swap the connections

// If using a second Motor / Darlington Driver - the connections are the same - except that the Darlington's "Base" should go to PD4 (again through a 100ohm resistor)


// LED
// The LED is driven using the Baby Orangutan's motor driver
// The motor driver can source 1 amp continuous - don't exceed this.
// Determine a resistor of appropriate value based on an LED calculator (Google it - note - the "source" voltage is about 5v)
// If needed - you can use a higher value resistor to reduce the current draw (this can help if your voltage regulator seems too hot)
// Positive side of the LED should connect to the resistor
// 		Other side of resistor goes to M1A on the Orangutan
// Negative side of the LED goes to M1B on the Orangutan
// If the LED doesn't light up - it might be connected backwards


// Baby Organgutan
// VIN needs 5V (probably from the voltage regulator) 
// GND goes to ground
// All other pin connections are listed previously under the corresponding components 


// LED Placement
// The LED should be placed approximately 60 degrees - counter-clockwise from the first (or only) drive motor (the one connected to the darlington connected to PD2)
// To put it another way - if drive motor 1 is at 12 o-clock - the LED should be around 7 o-clock
// If the robot doesn't seem to drive quite "forward" - the led_adjust variable may be modified
// The possible ranges for led_adjust go from 0 to 100 - however some values may result the heading vanishing at certain trajectories (this is a BUG)
// If you have this problem - try a higher or lower value ("wrapping" from a very high value down to a very low value or vice-versa may help) - or just move the LED itself


// Pre-compile Calibration
// First - attempt to estimate the "radius" the accelerometer is from the center of the bot
// If you've intentionally angled the accelerometer - increase that value by some percentage (take a guess - figure 2x increase for 45 degrees)
// Place that value (in centimeters) into the "radius" variable in the code
// This is only a guess at calibration - you'll need to fine-tune things with the robot actually running


// First Spin-Up / Calibration
// Documentation on configuring the bot at runtime goes here....


// LED Status
// When LED flashes slowly - it indicates it's not seeing a signal -or- it's getting a signal where the control stick isn't centered
// When LED flashes quickly - it indicates it has a signal with the control stick centered
// Note - a receiver providing a "fail safe" signal looks like a real signal to the bot


// Driving Notes

// Robot will (hopefully) remain still until given throttle
// Should spin in place once throttle is applied (assuming control stick is centered)
// Move control stick left or right to adjust heading (fully proportional)
// Move stick forward or backwards to go forwards or backwards (non-proportional)
// Yes - it drives kind of like a regular robot does

// Minor inaccuracies in tracking can be compensated for using steering (actually adjusting the transmitter's trim isn't recommended for a couple reasons)
// Driving constantly forward / adjusting steering to head towards the target seems to work well
// In general - best "translation" (ie movement) speed occurs at roughly half-throttle (full throttle may result in slow translation)
// The heading light will "pulse" over 50% throttle to provide feedback to the driver



// That's it! - here's the code....




#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/sleep.h>

//the following define statements setup which pins go to what
//it's a bit ugly - but it keeps all pin assignment stuff in one place

//pins for receiver
//note - these pins must be on PORT B so they're included in PCINT0
#define throttle_pin (PINB & 1<<PINB0)   					//portb.0 - throttle
#define set_throttle_pin_as_input() DDRB &= ~(1<<DDB0) 

#define leftright_pin (PINB & 1<<PINB1)						//portb.1 - left / right
#define set_leftright_pin_as_input() DDRB &= ~(1<<DDB1) 

#define forwardback_pin (PINB & 1<<PINB2)					//portb.2 - forward / back
#define set_forwardback_pin_as_input() DDRB &= ~(1<<DDB2) 


//definitions for LED pin
#define set_led_on() PORTD |= _BV(PD5)						//portd.5 - heading indicator LED control PIN (powers LED connected to motor controller on M1a / M1b)
#define set_led_off() PORTD &= ~_BV(PD5)	
#define toggle_led() PORTD ^= 1<<PIND5	
#define set_led_pin_as_output() DDRD |= 1<<DDD5

//definitions for motor1 pin
#define set_motor1_on() PORTD |= _BV(PD2)						//portd.2 - motor 1
#define set_motor1_off() PORTD &= ~_BV(PD2)	
#define set_motor1_pin_as_output() DDRD |= 1<<DDD2

//definitions for motor2 pin
#define set_motor2_on() PORTD |= _BV(PD4)						//portd.4 - motor 2
#define set_motor2_off() PORTD &=  ~_BV(PD4)		
#define set_motor2_pin_as_output() DDRD |= 1<<DDD4

//definitions for accelerometer data pin
#define ADC_PORT_FOR_ACCEL 4									//adc.4 - accelerometer analog input 4 (pc4)
#define set_accel_data_pin_as_input() DDRC &= ~(1<<DDC4) 

//definitions for accelpower pin
#define set_accelpower_pin_on() PORTD |= 1<<PIND0				//portd.0 - accel power
#define set_accelpower_pin_as_output() DDRD |= 1<<DDD0



//the following receiver values may be modified to compensate for minor differences in radio systems
//all values are actually times - units are number of times the Atmel's timer fires between the PWM signal going high - then returning to low

int throttle_low = 400;   	                               //throttle low RX value - includes some "slack" - value at which motors should actually kick in
int throttle_high = 560;

int forwardback_center = 457;                              //center value for forward / back RX
int forwardback_forwardthresh = 472;                       //center + 15 
int forwardback_backthresh = 442;                          //center - 15

int heading_center = 456;                                  //center value for heading left/right RX
int heading_rightthresh = 441;                             //center - 15
int heading_leftthresh = 471;                              //center + 15

float min_rpm = 400;                                         //minimum RPM for translation / throttling to kick in (if lower than - gets full power)
float max_allowed_rpm = 2750;                                //max_rpm allowed before cutting power
float max_g = 5000;                                          //max g's   before cutting power

float radius = 3.0;                                      	 //effective radius of circle for accel (centimeters) - adjust as needed
															//(lower numbers = "faster" tracking)
															
float g_per_adc_increment = .5;                            //10mv / g, 5mv per single increment from ADC up to 1024


															//the following two values add very minor adjustments to help compensate for time spent outside the actual motor drive loop
															//these are based on a bunch of testing - shouldn't need to be changed unless the code is changed significantly
															//ultimately they have a very small improvement in tracking (changing them to 0.0 won't hurt stuff much)
float fudge_factor = .31;                                  //each cycle is reduced by this many ms to compensate for fixed time events
															

float fudge_factor_per_ms = .008;                           //adds this much more correction per each MS in rotation


															//these variables can be tweaked to adjust for any tracking errors that seem to only happen when driving forward / backward
															//this really shouldn't happen - but may occur due to friction that occur during translational movement
															//they probably depend on the physical construction of the bot / drive system
float forward_comp = .997;                                 //heading compensation when going forward (1.00 = none)
float backward_comp = 1.003;                               //heading compensation when going back (1.00 = none)

float tracking_comp = 1.0;                                 //tracking compensation defaults to 1 (no adjustment)

long led_adjust = 10;                                      //offset in % for LED heading (1 to 100) - note - some values may not work! (10 and 39 are tested and known to be good)
															//10 is a verified good value for drive motor 1 at 12 o-clock - and LED around 7 o-clock
															//adjust as neeed

int base_accel = 498;                                      //default base (0g) value for accelerometer

float turn_speed = .0009; 		                           	//greater number = faster steering -  increased / decreased to make directional control more / less responsive


long x;                                           		  	//general loop variable

unsigned int accel_raw_data;                           	//raw accelerometer data
float accel_read;                               	      	//single used to store accelerometer data

float delay_loop;                              	       	//used to add extra delay

int full_power_spin;                          		      	//if set to 1 - we're just spinning at full power (no translation)

int alternate_motor_cycle = 0;              				//flipped between 1 and 2 each spin - alternates which motor is used for power each cycle when not moving

int forward;                                    		   	//is 1 if robot is supposed to be going forward
int backward;                                  			   	//is 1 if robot is supposed to be going back

float total_fudge_factor;									//total amount of fixed compensation per spin

int begin_brake;                               			   	//point in spin to start brake
int end_brake;                                             //point in spin to end brake

int flashy_led;                        	        		   	//set to 1 if heading led is pulsing (used to indicate throttle)

float periodms;                               				//how long it takes for the robot to rotate once
long periodms_long;                               			//how long it takes for the robot to rotate once

float delaytime_single;                              		//delaytime refers to time spent in each "cycle"
long delaytime_long;                                  		//Used for actual for / next loops (integer)

long led_on;                                          		//offset in milliseconds for front LED to come on
long led_off;                                         		//offset in milliseconds for front LED to come on

long led_ref;                                         		//used to count through both cycles for LED reference point

int throttle_percent;                             			//percentage of full throttle (spin rate)

long power_kill_length;                               		//used for throttling - time in MS for which spin power is cut short
long power_kill_part1;                                		//used for throttling - if before this time in cycle - power is cut
long power_kill_part2;                                		//used for throttling - if after this time in cycle - power is cut

long braking_length;                                  		//length of braking cycle in MS - used for throttling

float g;                                             		//g force the accelerometer is seeing

float rpm;                                           		//current RPM of robot

float steering_multiplier;                                 //total rotation time is multiplied for this amount to adjust for steering input from transmitter

float digitdif;                                      		//used in nasty code to convert periodms into an integer

															//RC data channels - declared volatile since they're shared with the interrupt handler
volatile int leftright;                                    //heading RC channel
volatile int forwardback;                                  //forward/back RC channel
volatile int throttle;                                     //throttle RC channel


int throttle_hilow;                               			//indicate if given RC channel was hi or low on last read
int forwardback_hilow;
int leftright_hilow;

int led_is_on_now;                                        	//used to keep track if tracking LED should be on now or not

int throttle_hightime = 0;                         			//track value of timer for different channels upon going RX signal going HIGH
int forwardback_hightime = 0;                              
int leftright_hightime = 0;                               


void motors_off(void);
void motor1_on(void);
void motor2_on(void);
void motors_left(void);

void throttle_change(void);
void forwardback_change(void);
void leftright_change(void);
void SetupTimer1(void);
void wait24us(void);
void setup(void);
void loop(void);
void adc_init(void);
int read_adc(void);

int main(void)
{
	
	//do initial setup stuff - set pins / interrupts / etc.
	setup();
			
	//execute the main loop indefinitely...
	
	while (1)
	{
		
		loop();			//main loop with everything in it...
		
	}
	
	
	return(0);

}


//sets up all the pins and interrupts
void setup(void)

{

	adc_init();		//init the ADC...

	set_throttle_pin_as_input();
	set_leftright_pin_as_input();  
	set_forwardback_pin_as_input();
  
	set_accel_data_pin_as_input();
	
	set_accelpower_pin_as_output();
	set_accelpower_pin_on();								//turn on power for accel (accel is connected to chip for power)


	
	set_led_pin_as_output();

	set_motor1_pin_as_output();
	set_motor2_pin_as_output();
	
	set_led_on();						                    //turn on signal LED before timers so it comes on immediately
	

	//enable pin change interrupt - any changes on PORTB trigger interrupt
	PCMSK0 = 0xFF;
	PCICR = 1<<PCIE0; 
	  
	SetupTimer1();   //fire up timer1 (2 bytes) - accessed via TCNT1 variable

	motors_off();   //make sure those motors are off...


	//flash LED on boot
	for (x = 1; x <= 10; x++)
	{
		toggle_led();
		_delay_ms(200);
	}

	throttle = 0;   //make sure throttle is off at boot


}



void loop(void)

{

		
	//if throttle is lower than throttle_low - or is over 100 beyond throttle_high -  bot stays powered down 
	
	sei();  //enable interrupts 


	while ( throttle < throttle_low || throttle > (throttle_high + 100))
	{


		motors_off();

		//interrupt blinking if stick isn't centered (helps to verify TX is working)
		if ( leftright > heading_leftthresh ) _delay_ms(200);
		if ( leftright < heading_rightthresh ) _delay_ms(200);

		//sit there and flash LED
		
		toggle_led();
		_delay_ms(50);		


	}




	cli();                //disable interrupts - bad things seem to happen if the RC interrupts get triggered while doing math...

	//Are we going forward or backwards?
	if ( forwardback > forwardback_forwardthresh) forward = 1; else forward = 0;
	if ( forwardback < forwardback_backthresh) backward = 1; else backward = 0;

	flashy_led = 0;

	accel_raw_data = read_adc();               //get accel data 
	accel_read = accel_raw_data;                              //move it over to single in case we want to do floating point
	accel_read = accel_read - base_accel;                     //compensate for base (2.5v) level
	g = accel_read * g_per_adc_increment;                    //convert to g's

	rpm = g * 89445;                                //calculate RPM from g's - derived from "G = 0.00001118 * r * RPM^2"

	rpm = rpm / radius;

	rpm = pow(rpm, .5);
	
	
	periodms = rpm / 60;                                      //convert RPM to duration of each spin in milliseconds
		
	if (periodms == 0) periodms = 1;                          //must prevent any possible division by zero!!!
	periodms = 1 / periodms;
	periodms = periodms * 1000;

	periodms = periodms * tracking_comp;                      //compensate with user-set tracking adjustment


	alternate_motor_cycle = !alternate_motor_cycle;     //alternates alternate_motor_cycle - used to balance spin

  
	if ( forward == 1 ) periodms = periodms * forward_comp;   //extra compensation if going forward
	if ( backward == 1 ) periodms = periodms * backward_comp; //extra compensation if going backward


	
	total_fudge_factor = fudge_factor + (fudge_factor_per_ms * periodms);
	periodms = periodms - total_fudge_factor;                       					   //compensate for time spent doing reads / calculations
	

	delaytime_single = periodms / 2;                          //sets period in MS for each half of spin

	//converts throttle reading from remote into percentage (400 = low, 560 = high)
	throttle_percent = (throttle - throttle_low) + 40;    //this gives a range from about 20% to 100%
	throttle_percent = throttle_percent / 2;
	if ( throttle_percent > 100 ) throttle_percent = 100;    //don't got over 100%



	//calculates + modifies changes to heading based on input from transmitter
	steering_multiplier = heading_center - leftright;
	steering_multiplier = steering_multiplier * turn_speed;
	steering_multiplier = 1 - steering_multiplier;		//starts with 1 as a base value (ie - if it was 0.0 it becomes 1.0 - so there's no change in heading)
	delaytime_single = delaytime_single * steering_multiplier;
	
	
	delaytime_long = delaytime_single;

	digitdif = delaytime_single - delaytime_long;


	//code that waits X microseconds to compensate for integer math
	digitdif = digitdif  * 10;                                 //converts digit dif to 50's of microseconds


	//caps on timing if going too slow or fast
	if ( delaytime_long > 500) delaytime_long = 500;
	if ( delaytime_long < 5) delaytime_long = 5;

	periodms = delaytime_long * 2;                           //we re-use periodms (full cycle length) for LED calculations - so it needs to be updated with any timing adjustments
	periodms_long = periodms;

	//set heading beacon size and location

	led_on = periodms * led_adjust;
	led_on = led_on / 100;
	led_off = periodms / 3;                                  //led signal is 33% of circle
	led_off = led_off + led_on;


	if (led_off >= periodms_long ) //if led_off is "later" or at end of cycle - shift led_off behind by one cycle
	{                        
		led_off = led_off - periodms_long;
	}

	if ( led_off < 1 ) led_off = led_off + periodms_long;

	if ( g > max_g && throttle_percent > 20 ) throttle_percent = 20;       //if we're over max RPM for translation - reduce throttle

	//throttling

	full_power_spin = 0;
	if ( rpm < min_rpm ) full_power_spin = 1;                //if we're under the minimum RPM for translation - do the full power spin!
	if ( rpm > max_allowed_rpm ) throttle_percent = 6;       //if we're over max RPM for translation - reduce power
	
	
	//if throttle is at or over 50% throttle - adjust time spent in braking
	if ( throttle_percent > 50 )
	{                          

		flashy_led = 1;                                        //flash the LED to indicate we're in fast mode

		braking_length = delaytime_long * 25;                  
		braking_length = braking_length / throttle_percent;    

		begin_brake = delaytime_long / 2;
		begin_brake = begin_brake - braking_length;

		end_brake = delaytime_long / 2;
		end_brake = end_brake + braking_length;
	
		if ( begin_brake < 1 )	begin_brake = 1;               //make sure begin_brake isn't getting set to 0
			
		power_kill_part1 = 0;                                  //power_kill not used if throttle over 50%
		power_kill_part2 = delaytime_long;

	}


	if ( throttle_percent <= 50 )                           //if throttle under 50% - kill the motors for a portion of each spin
	{
		begin_brake = 1;
		end_brake = delaytime_long;

		power_kill_length = 50 - throttle_percent;           //set time in each cycle to cut power (throttling)
		power_kill_length = power_kill_length * delaytime_long;
		power_kill_length = power_kill_length / 150;

		power_kill_part1 = power_kill_length;
		power_kill_part2 = delaytime_long - power_kill_length;

	}


	if ( full_power_spin == 1 )      //if we're actually doing full power this spin (no translation) - ignore any calculations / reset variables
	{
		end_brake = 1;
		begin_brake = 0;

		power_kill_part1 = 0;
		power_kill_part2 = delaytime_long;
	}


	sei();  //enable interrupts - out of all the critical stuff

	led_ref = 0;

	//Do translational drift driving

	//Cycle 1 ("front" 180 degrees of spin)

	for (x = 1; x <= delaytime_long; x++)
	{

		motors_left();                                        //start off under full power

		led_ref = led_ref + 1;

		if ( x >= begin_brake && x < end_brake )         //switch to single motor as soon as entering braking cycle
		{
			//if sitting still
			if ( forward == 0 && backward == 0 )
			{
				if ( alternate_motor_cycle == 0 ) motor1_on();  //alternates which motor is used each cycle if ( sitting still
				if ( alternate_motor_cycle == 1 ) motor2_on();  //this prevents unwanted "translation" due to any imbalances
			}

			//if ( going forward / back set motors appropriately (this is "where it happens")
			if ( forward == 1 ) motor1_on();
			if ( backward == 1 ) motor2_on();
		}

		if ( x >= end_brake ) motors_left();                 //if we hit end of brake cycle - go to full power

		if ( x < power_kill_part1 ) motors_off();            //if th rottle is less that 100% - kill power at appropriate time
		if ( x > power_kill_part2 ) motors_off();            //if throttle is less that 100% - kill power at appropriate time

		if ( led_ref == led_on ) led_is_on_now = 1;         //turn on heading led
		if ( led_ref == led_off ) led_is_on_now = 0;        //turn off heading led

		
		if ( led_is_on_now == 1 )
		{
			//flash the LED if we're in flashy mode - otherwise it's just on
			if ( flashy_led == 1 ) toggle_led(); else set_led_on();
		}

		if ( led_is_on_now == 0 ) set_led_off();

		_delay_ms(1);

	}

	for (delay_loop = 1; delay_loop <  digitdif; delay_loop++)
	{                           
		//in theory we should wait 25 microseconds - but I'm assuming the overhead takes up 1us
		wait24us();                                             
	}

  
	//Cycle 2 (back 180 degrees of spin) - same as above except motors are reversed (there are some moderately-good reasons this isn't a subroutine)


	for (x = 1; x <= delaytime_long; x++)
	{

		motors_left();                                        //start off under full power

		led_ref = led_ref + 1;

		if ( x >= begin_brake && x < end_brake )         //switch to single motor as soon as entering braking cycle
		{
			//if sitting still
			if ( forward == 0 && backward == 0 )
			{
				if ( alternate_motor_cycle == 0 ) motor2_on();  //alternates which motor is used each cycle if ( sitting still
				if ( alternate_motor_cycle == 1 ) motor1_on();  //this prevents unwanted "translation" due to any imbalances
			}

			//if ( going forward / back set motors appropriately (this is "where it happens")
			if ( forward == 1 ) motor2_on();
			if ( backward == 1 ) motor1_on();
		}

		if ( x >= end_brake ) motors_left();                //if we hit end of brake cycle - go to full power

		if ( x < power_kill_part1 ) motors_off();           //if throttle is less that 100% - kill power at appropriate time
		if ( x > power_kill_part2 ) motors_off();           //if throttle is less that 100% - kill power at appropriate time

		if ( led_ref == led_on ) led_is_on_now = 1;         //turn on heading led
		if ( led_ref == led_off ) led_is_on_now = 0;        //turn off heading led

		
		if ( led_is_on_now == 1 )
		{
			//flash the LED if we're in flashy mode - otherwise it's just on
			if ( flashy_led == 1 ) toggle_led(); else set_led_on();
		}

		if ( led_is_on_now == 0 ) set_led_off();

		_delay_ms(1);


	}

	for (delay_loop = 1; delay_loop <  digitdif; delay_loop++)
	{                           
		//in theory we should wait 25 microseconds - but I'm assuming the overhead takes up 1us
		wait24us();                                             
	}
	
	

}



void wait24us(void)
{
	_delay_us(24);
}


void motors_off(void)
{
	set_motor1_off();
	set_motor2_off();
}


void motors_left(void)
{
	set_motor1_on();
	set_motor2_on();
}


void motor1_on(void)
{
	set_motor1_on();
	set_motor2_off();
}

void motor2_on(void)
{
	set_motor1_off();
	set_motor2_on();
}


//main interrupt handler - is called any tmie any ports on PORTB change

ISR (PCINT0_vect)
{
	
	//check all RC channels to see if they were updated
	if (throttle_hilow != throttle_pin) throttle_change();
	if (leftright_hilow != leftright_pin) leftright_change();
	if (forwardback_hilow != forwardback_pin) forwardback_change();
	

}


//updates RC channels any time specified pin goes high/low
//following 3 routines are all identical except for channels (not easy to consolidate them)
void throttle_change(void)
{
	
	//did the pin go HIGH? - then note time
	if ( throttle_pin != 0 ) throttle_hightime = TCNT1;    

	//did the pin go low? - then set timer value as value for this channel / if timer has overflowed then ignore
	if ( throttle_pin == 0 && throttle_hightime != 0 && TCNT1 > throttle_hightime) throttle = TCNT1 - throttle_hightime;      
  

	throttle_hilow = throttle_pin;                                  //make note of pin state for reference next time interrupt is triggered...	
}

void leftright_change(void)
{
	//did the pin go HIGH? - then note time
	if (leftright_pin != 0 ) leftright_hightime = TCNT1;    

	//did the pin go low? - then set timer value as value for this channel / if timer has overflowed then ignore
	if (leftright_pin == 0 && leftright_hightime != 0 && TCNT1 > leftright_hightime) leftright = TCNT1 - leftright_hightime;      
  
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



void SetupTimer1(void)
{

	TCCR1A = 0;                               //mode = 0
	TCCR1B = 0<<CS12 | 1<<CS11 | 1<<CS10;    //prescaler = 64

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


//Timer1 overflow interrupt vector handler
//this is only used if the overflow handler is enabled (which it isn't in this code)
ISR(TIMER1_OVF_vect)
{
	//  unused...
}



