//Melty B - Translational Drif (t / Melty Brain combat robot
//www.spambutcher.com

//This code is provided for your use without warranty / etc...

//this code is the general purpose melty-B code (as of 3/31/09)
//includes improved safety

//Arduino translation - 5/6/09

//Hardware
//Atmega 168 / 20MHZ crystal
//accelerometer: Freescale 200g MMA2301Eg (Mouser.com)
//MCU Board: Pololu Baby Orangutan / Mega168 (pololu.com)

#include <avr/interrupt.h>
#include <avr/io.h>
#include "pins_arduino.h"
#include <math.h>

//interrupt related stuff
volatile uint8_t *port_to_pcmask[] = { 
  &PCMSK0,  &PCMSK1,  &PCMSK2};
typedef void (*voidFuncPtr)(void);
volatile static voidFuncPtr PCintFunc[24] = {
  NULL};
volatile static uint8_t PCintLast[3];

long x;                                               //general variables
float y;

int accel_raw_data;                                  //raw accelerometer data
float accel_read;                                    //single used to store accelerometer data

float delay_loop;                                    //used to add extra delay

long got_throttle;                                 //used for safety

int rangecheck;                                   //used to read RC values / verify they're in range before assigning

int full_power_spin;                              //if set to 1 - we're just spinning at full power (no translation)
int cut_power;                                    //if set to 1 - motors are off for this rotation

int configmode;                                    //is 1 when in configmode

int alternate_motor_cycle;                        //flipped between 1 and 2 each spin - alternates which motor is used for power each cycle when not moving

int forward;                                      //is 1 if robot is supposed to be going forward
int backward;                                     //is 1 if robot is supposed to be going back

int begin_brake;                                  //point in spin to start brake
int end_brake;                                    //point in spin to end brake

int flashy_led;                                   //set to 1 if heading led flashes

float periodms;                                      //how long it takes for the robot to rotate once
long periodms_long;                                   //how long it takes for the robot to rotate once

float delaytime_single;                              //delaytime refers to time spent in each "cycle"
long delaytime_long;                                  //Used for actual for / next loops (integer)

int in_tracking_adjust;                           //is 1 if robot is in tracking adjustment mode

long tail_start;                                      //offset in milliseconds for tail LED to come on
long tail_end;                                        //offset in milliseconds for tail LED to come on

long led_on;                                          //offset in milliseconds for front LED to come on
long led_off;                                         //offset in milliseconds for front LED to come on

long led_ref;                                         //used to count through both cycles for LED reference point

int no_led;                                       //true if the LED recalc is off for this rotation

int throttle_percent;                             //percentage of full throttle (spin rate)

long power_kill_length;                               //used for throttling - time in MS for which spin power is cut short
long power_kill_part1;                                //used for throttling - if before this time in cycle - power is cut
long power_kill_part2;                                //used for throttling - if after this time in cycle - power is cut

long braking_length;                                  //length of braking cycle in MS - used for throttling

float g;                                             //g force the accelerometer is seeing

float rpm;                                           //current RPM of robot
float max_rpm;                                       //max RPM of robot

float add_delay;                                     //used to calculate changes in heading

float digitdif;                                      //used in nasty code to convert periodms into an integer

int leftright;                                    //heading RC channel
int rudder;                                       //rudder RC channel
int forwardback;                                  //forward/back RC channel
int throttle;                                     //throttle RC channel

int left_strafe;
int right_strafe;

int throttle_hilow;                               //indicate if given RC channel was hi or low on last read
int forwardback_hilow;
int leftright_hilow;
int rudder_hilow;

int throttle_hightime = 0;                               //track value of timer for different channels upon going RX signal going HIGH
int forwardback_hightime = 0;                              
int leftright_hightime = 0;                               
int rudder_hightime = 0;

int throttle_low = 400;                                  //throttle low RX value - includes some "slack" - value at which motors should actually kick in
int throttle_high = 560;                         

//full forward = 576 / full back = 376
int forwardback_center = 476;                                  //center value for forward / back RX
int forwardback_forwardthresh = 491;                          //center + 15
int forwardback_backthresh = 461;                             //center - 15

//full right = 568 / full left = 368
int heading_center = 468;                                  //center value for heading left/right RX
int heading_leftthresh = 461;                              //center - 7
int heading_rightthresh = 475;                             //center + 7

//rudder right = 370 / rudder left = 570 
int rudder_center = 470;                                   //center value for rudder RX
int rudder_leftthresh = 510;                               //center + 40
int rudder_rightthresh = 430;                              //center - 40

int strafing_is_off = 1;                                 //is strafing disabled? (strafing has some quirks / may cause problems)
int dont_alternate_motors = 0;                           //changes motor cycling when not translating (for flight)

int led_is_on_now;                                        //used to keep track if tracking LED should be on now or not

int min_rpm = 300;                                         //minimum RPM for translation / throttling to kick in
int max_allowed_rpm = 2750;                                //max_rpm allowed before cutting power
int max_g = 5000;                                          //max g's before cutting power

float radius = 5;                                            //effective radius of circle for accel (centimeters) - seems to be off sometimes...
float g_per_adc_increment = .2;                              //10mv / g, 5mv per single increment up to 1024

float fudge_factor = .54;                                    //each cycle is reduced by this many ms to compensate for fixed time events
                                                              //each analog read = 0.7ms * 2 + x for calc time

float forward_comp = 1.00;                                   //heading compensation when going forward (1.00 = none)
float backward_comp = .99;                                   //heading compensation when going back  (1.00 = none)

int tracking_comp = 1;                                        //tracking compensation defaults to 1 (no adjustment)

long led_adjust = 39;                                          //offset in % for LED heading (1 to 100)
long led_adjust_backup = led_adjust;                           //(used to reset if not straffing)
int load_heading = 1;                                        //if set to 0 - will use as heading as set above as opposed to loading from rom

int base_accel = 503;                                      //default base (0g) value for accelerometer

int throttle_percent_max = 100;                            //throttle maxes at 100% (normal - allows less translation / HIGHer top speed)
int throttle_percent_min = 0;                              //throttle starts at 0% (normal)

float exponential = 1.18;                                          //greater number = more exponential steering (normal)


int throttle_pin = 8;       //portb.0 - throttle  (87 = low, 115 = middle, 148 = HIGH)
int leftright_pin = 11;     //portb.3 - left / right (425 = left, 469 = center, 541 = right)
int forwardback_pin = 12;   //portb.4 - forward / back (145 = forward, 114 = center, 83 = back)
int rudder_pin = 13;        //portb.5 - rudder (425 = left, 469 = center, 541 = right)

int accelpower_pin = 4;    //accel power (unused for now)
int led_pin = 5;           //portd.5 - heading indicator LED (on motor controller m1 / m2)
int motor1_pin = 7;        //portd.3 - motor 1
int motor2_pin = 4;        //motor 2 (unused for now)
int acceldata_pin = 4;      //adc.4   - analog input 4 (pc4)

/*
tracking_comp_store As Eram Single                      //used to store tracking adjustment in ROM
 led_adjust_store As Eram Long                           //used to store LED tracking adjustment in ROM (which way is "forward")
 base_accel_store As Eram Single                         //used to store base level for accelerometer in ROM
 Eprom_check As Eram integer                             //used to store validate stored ROM value
 */


// setup pin change interrupts / based on code by Mike Cook

void PCattachinterrupt(uint8_t pin, void (*userFunc)(void))
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t slot;
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  }
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }
  slot = port * 8 + (pin % 8);
  PCintFunc[slot] = userFunc;
  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
}

// common code for isr handler. "port" is the PCINT number.
static void PCint(uint8_t port) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;

  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }

  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = port * 8 + i;
      if (PCintFunc[pin] != NULL) {
        PCintFunc[pin]();
      }
    }
  }
}

void setup()

{

  pinMode(throttle_pin, INPUT);  
  pinMode(leftright_pin, INPUT);  
  pinMode(forwardback_pin, INPUT);  
  pinMode(rudder_pin, INPUT);  

  pinMode(acceldata_pin, INPUT);  
  pinMode(accelpower_pin, OUTPUT);  

  pinMode(led_pin, OUTPUT);  

  pinMode(motor1_pin, OUTPUT);  
  pinMode(motor2_pin, OUTPUT);  

  digitalWrite(led_pin, HIGH);                           //turn on signal LED before timers so it comes on immediately
  digitalWrite(accelpower_pin, HIGH);                    //turn on power for accel (if ( accel is connected to chip for power)


  //setup interrupts for RX pins
  PCattachinterrupt(throttle_pin, throttle_change);
  PCattachinterrupt(leftright_pin,  leftright_change);
  PCattachinterrupt(forwardback_pin,  forwardback_change);
  PCattachinterrupt(rudder_pin,  rudder_change);

  SetupTimer1();   //fire up timer1 (2 bytes) - accessed via TCNT1 variable

  configmode = 0;                                              //we're not in configmode to start

  motors_off();                                                  //make sure those motors are off...

  throttle = 0;   //make sure throttle is off at boot

  //flash LED on boot
  for (x = 1; x <= 20; x++)
  {
    digitalWrite(led_pin, !digitalRead(led_pin));
    delay(20);
  }

  /*
  if ( Eprom_check = 555 )
   {                                   //load config data from eprom if ( eprom_check was set to 555
   tracking_comp = tracking_comp_store;
   if ( load_heading == 1 ) led_adjust = led_adjust_store;
   if ( load_heading == 1 ) led_adjust_backup = led_adjust_store;
   base_accel = base_accel_store;
   }
   */

}


void loop()

{

  
  //if throttle is lower than throttle_low - or is over 100 beyond throttle_high -  bot stays powered down 

  while ( throttle < throttle_low || throttle > (throttle_high + 100))
  {

    motors_off();

    //interrupt blinking if ( stick isn//t centered
    //used to help debug if ( center is off
    if ( leftright > heading_rightthresh ) delay(200);
    if ( leftright < heading_leftthresh ) delay(200);


    //if ( stick is pulled back (and not in configmode) - flash out highest RPM
    if ( forwardback < forwardback_backthresh && configmode == 0 )
    {

      digitalWrite(led_pin, LOW);
      delay(500);

      for (y = 1; y <= max_rpm; y = y + 100)
      {
        digitalWrite(led_pin, HIGH);
        delay(100);
        digitalWrite(led_pin, LOW);
        delay(300);

        if ( throttle >= throttle_low ) y = max_rpm;               //abort if throttle gets pushed up

      }

      if ( throttle < throttle_low ) delay(2000);               //only wait if throttle is still low

    }


    //if ( stick is upper right - ) { toggle configmode
    if ( forwardback > forwardback_forwardthresh && leftright > rudder_leftthresh )
    {

      //wait a bit to make sure stick is being held...
      delay(1400);

      if ( forwardback > forwardback_forwardthresh && leftright > rudder_leftthresh )
      {

        if ( configmode == 0 )
        {
          configmode = 1;

          //assign base_accel
          accel_raw_data = analogRead(acceldata_pin);
          base_accel = accel_raw_data;

        }
        else
        {
          configmode = 0;

          /*               //write out new data to ROM
           tracking_comp_store = tracking_comp;          //write out config data to ROM
           led_adjust_store = led_adjust;
           base_accel_store = base_accel;
           Eprom_check = 555;                            //write out arbitrary value to validate tracking_comp was written out
           */
        }

        delay(500);

      }

    }



    if ( forwardback < forwardback_backthresh && leftright < rudder_rightthresh && configmode == 1 )
    { 
      ////if ( stick is held to back left while ( in config mode
      //reset heading adjustment

      //wait a bit to make sure stick is being held...
      delay(1400);

      if ( forwardback < forwardback_backthresh && leftright < rudder_rightthresh && configmode == 1 )
      {

        tracking_comp = 1;                            //reset heading adjustment

        for ( x = 1; x <= 70; x++)
        {
          digitalWrite(led_pin, !digitalRead(led_pin));
          delay(20);
        }
        digitalWrite(led_pin, LOW);
        delay(1000);
      }

    }


    //sit there and flash LED

    digitalWrite(led_pin, LOW);
    delay(50);

    if ( configmode == 1 )
    {
      for ( x = 1; x <=10; x++)
      {
        delay(30);
        digitalWrite(led_pin, !digitalRead(led_pin));
      }
      digitalWrite(led_pin, LOW);
      delay(150);
    }

    digitalWrite(led_pin, HIGH);
    delay(50);

  }


  //safety
  //if we haven't gotten the throttle reading in 5 cycles - set throttle to 0
  got_throttle = got_throttle + 1;
  if ( got_throttle > 100 ) got_throttle = 100;
  if ( got_throttle > 5) throttle = 0;


  //reset max RPM
  max_rpm = 0;

  cli();                //disable interrupts - bad things seem to happen if the RC interrupts get triggered while doing math...



  //Are we going forward or backwards?
  if ( forwardback > forwardback_forwardthresh) {
    forward = 1;
  } 
  else {
    forward = 0;
  }
  if ( forwardback < forwardback_backthresh) {
    backward = 1;
  } 
  else {
    backward = 0;
  }

  flashy_led = 0;

  accel_raw_data = analogRead(acceldata_pin);               //get accel data 
  accel_read = accel_raw_data;                              //move it over to single in case we want to do floating point
  accel_read = accel_read - base_accel;                     //compensate for base (2.5v) level
  g = accel_read * g_per_adc_increment;                    //convert to g's

  rpm = 28.45 * radius;                                     //calculate RPM from g's  - rpm  = (g/(28.45* radius ))^0.5 *1000
  if (rpm == 0) rpm = 1;                                    //must prevent any possible division by zero!!!
  rpm = g / rpm;
  rpm = pow(rpm, .5);
  rpm = rpm * 1000;

  if ( rpm > max_rpm )  max_rpm = rpm;

  periodms = rpm / 60;                                      //convert RPM to duration of each spin in milliseconds
  if (periodms == 0) periodms = 1;                          //must prevent any possible division by zero!!!
  periodms = 1 / periodms;
  periodms = periodms * 1000;

  periodms = periodms * tracking_comp;                      //compensate with user-set tracking adjustment

//test!
periodms = 100;

  if ( alternate_motor_cycle == 1 ) { 
    alternate_motor_cycle = 2;
  } 
  else {
    alternate_motor_cycle = 1;
  }       //alternates alternate_motor_cycle - used to balance spin

  //strafing

  if ( strafing_is_off == 0 )              //don't do strafing stuff if it's off
  {

    if ( no_led == 1 )  //skip straffing for one cycle if we had a change last time (setting no_led to 1)
    {                                          
      no_led = 0;
    }
    else
    {
      no_led = 0;                                                  //make sure LED is recalc on unless we decide to turn it off

      led_adjust = led_adjust_backup;

      if ( rudder > rudder_leftthresh )
      {
        if ( left_strafe == 0 )
        {
          periodms = periodms * 1.25;                            //one time adjustment to turn 90 degrees
          no_led = 1;                                            // led recalc off this cycle - otherwise will look glitchy
        }
        led_adjust = led_adjust - 25;
        left_strafe = 1;
        forward = 1;
        backward = 0;
      }  
      else
      {
        if ( left_strafe == 1 )
        {
          periodms = periodms * .75;                             //correct for prior straffing
          no_led = 1;                                            // led recalc off this cycle - otherwise will look glitchy
        }
        left_strafe = 0;
      }

      if ( rudder < rudder_rightthresh )
      {
        if ( right_strafe == 0 )
        {
          periodms = periodms * .75;                             //one time adjustment to turn 90 degrees
          no_led = 1;                                            // led recalc off this cycle - otherwise will look glitchy
        }
        led_adjust = led_adjust + 25;
        right_strafe = 1;
        forward = 1;
        backward = 0;
      }
      else
      {
        if ( right_strafe == 1 )
        {
          periodms = periodms * 1.25;                            //correct for prior straffing
          no_led = 1;                                            // led recalc off this cycle - otherwise will look glitchy
        }
        right_strafe = 0;
      }
    }
  }

  if ( forward == 1 ) periodms = periodms * forward_comp;   //extra compensation if going forward
  if ( backward == 1 ) periodms = periodms * backward_comp; //extra compensation if going backward


  periodms = periodms - fudge_factor;                       //compensate for time spent doing reads / calculations


  //make sure led_adjust didn't get set above or below limits

  if ( led_adjust < 1 ) led_adjust = led_adjust + 100;
  if ( led_adjust > 100 ) led_adjust = led_adjust - 100;

  delaytime_single = periodms / 2;                          //sets period in MS for each half of spin

  //converts throttle reading from remote into percentage (400 = low, 560 = high)
  throttle_percent = (throttle - throttle_low) + 40;    //this gives a range from about 20% to 100%
  throttle_percent = throttle_percent / 2;
  if ( throttle_percent > 100 ) throttle_percent = 100;    //don't got over 100%

  //driver moves stick left and right until the bot tracks correctly
  //data is written into eprom next time the robot spins down

  in_tracking_adjust = 0;

  //tracking adjustment - if throttle is neither forward or back and throttle is under 50% go into tracking adjustment mode

  if ( configmode == 1 && forward == 0 && backward == 0 && throttle_percent < 50 )
  {

    in_tracking_adjust = 1;
    flashy_led = 1;                                    //to indicate heading adjust most - LED flashing is on

    if ( leftright > heading_rightthresh || leftright < heading_leftthresh )
    {

      flashy_led = 0;                                //turn off flashing to show is actively being adjusted

      add_delay = heading_center - leftright;
      add_delay = add_delay * delaytime_single;

      //              add_delay = pow(add_delay , exponential);    //disabled - need to fix problem with negative values

      add_delay = add_delay / 1400000; //(was 14000000)

      tracking_comp = tracking_comp + add_delay;

    }

  }

  //adjust led direction if throttle is over 50% and in configmode + not going back / forward

  if ( configmode == 1 && forward == 0 && backward == 0 && throttle_percent >= 50 )
  {

    in_tracking_adjust = 1;
    flashy_led = 0;                                        //to indicate LED adjust most - LED flashing is off

    if ( leftright < heading_leftthresh )
    {
      led_adjust = led_adjust + 1;
      flashy_led = 1;                                      //turn on flashing to indicate change
    }

    if ( leftright > heading_rightthresh )
    {
      led_adjust = led_adjust - 1;
      flashy_led = 1;                                      //turn on flashing to indicate change
    }

    if ( led_adjust < 1 ) led_adjust = 100;               //"wrap" around when adjusting LED direction
    if ( led_adjust > 100 ) led_adjust = 1;

    backward = 0;                                          //don//t really go backwards

    led_adjust_backup = led_adjust;

  }


  if ( in_tracking_adjust == 0 )
  {                           

    //don't do normal heading adjustments if we're doing tracking adjustments

    //normal drive heading change
    //this code adds or subtracts a percentage of delaytime based on the heading data from the remote
    //don't do if in configmode

    if ( leftright > heading_rightthresh || leftright < heading_leftthresh )
    {

      add_delay = heading_center - leftright;
      add_delay = add_delay * delaytime_single;

      //add_delay = Pow(add_delay , exponential);    //disabled - need to fix problem with negative values

      add_delay = add_delay / 700;  //(was 7700)

      delaytime_single = delaytime_single + add_delay;

    }

  }

  if ( configmode == 1) throttle_percent = 50;             //ignore throttle if in configmode

  delaytime_long = delaytime_single;

  digitdif = delaytime_single - delaytime_long;


  //code that waits X microseconds to compensate for integer math
  digitdif = digitdif  * 10;                                 //converts digit dif ( to 50's of microseconds

  for (delay_loop =1; delay_loop <= digitdif; delay_loop++)
  {
    wait25us();                                             
  }

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

  tail_start = periodms_long * 17;                         //code to calculate position of LED tail
  tail_start = tail_start / 60;
  tail_start = tail_start + led_off;

  tail_end = periodms_long * 6;
  tail_end = tail_end / 60;
  tail_end = tail_end + tail_start;


  if ( tail_start >= periodms_long )
  {
    tail_start = tail_start - periodms_long;
  }

  if ( tail_end >= periodms_long )
  {
    tail_end = tail_end - periodms_long;
  }

  if ( g > max_g && throttle_percent > 20 ) throttle_percent = 20;       //if we're over max RPM for translation - reduce throttle

  //throttling

  if ( throttle_percent > throttle_percent_max ) throttle_percent = throttle_percent_max;
  if ( throttle_percent < throttle_percent_min ) throttle_percent = throttle_percent_min;


  full_power_spin = 0;
  cut_power = 0;

  if ( rpm < min_rpm ) full_power_spin = 1;                //if we're under the minimum RPM for translation - do the full power spin!


  if ( rpm > max_allowed_rpm ) throttle_percent = 6;       //if we're over max RPM for translation - reduce power

  //if throttle is at or over 50% throttle - adjust time spent in braking
  if ( throttle_percent > 50 )
  {                          

    flashy_led = 1;                                        //flash the LED to indicate we're in fast mode

    braking_length = delaytime_long * 25;                  //original
    braking_length = braking_length / throttle_percent;    

    begin_brake = delaytime_long / 2;
    begin_brake = begin_brake - braking_length;

    end_brake = delaytime_long / 2;
    end_brake = end_brake + braking_length;

    if ( begin_brake < 1 )
    {
      begin_brake = 1;               //make sure begin_brake isn//t getting set to 0
      power_kill_part1 = 0;                                  //power_kill not used if throttle over 50%
      power_kill_part2 = delaytime_long;
    }

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


  if ( no_led == 1 )  //kill tail if initiating strafe
  {                 
    tail_start = 0;
    tail_end = 0;
  }

  sei();  //enable interrutps - out of all the critical stuff

  if ( full_power_spin == 1 )      //reset variables for full power spin

  {

    end_brake = 1;
    begin_brake = 0;

    power_kill_part1 = 0;
    power_kill_part2 = delaytime_long;

  }


  //Do translational drift driving


//test!
flashy_led = 1;

    //Cycle 1 (front 180 degrees of spin)

  led_ref = 0;

  for (x = 1; x <= delaytime_long; x++)
  {

    motors_left();                                        //start off under full power

    led_ref = led_ref + 1;

    if ( x >= begin_brake && x < end_brake )         //switch to single motor as soon as entering braking cycle
    {
      //if sitting still
      if ( forward == 0 && backward == 0 )
      {
        if ( alternate_motor_cycle == 1 ) motor1_on();  //alternates which motor is used each cycle if ( sitting still
        if ( alternate_motor_cycle == 2 ) motor2_on();  //this prevents unwanted "translation" due to any imbalances
      }

      //if ( going forward / back set motors appropriately (this is "where it happens")
      if ( forward == 1 ) motor1_on();
      if ( backward == 1 ) motor2_on();
    }

    if ( x >= end_brake ) motors_left;                 //if ( we hit end of brake cycle - go to full power

    if ( x < power_kill_part1 ) motors_off;            //if ( throttle is less that 100% - kill power at appropriate time
    if ( x > power_kill_part2 ) motors_off;            //if ( throttle is less that 100% - kill power at appropriate time

    if ( cut_power == 1 ) motors_off;                   //if ( this is a no-power spin

    if ( led_ref == led_on ) led_is_on_now = 1;         //turn on heading led
    if ( led_ref == led_off ) led_is_on_now = 0;        //turn off heading led

    if ( no_led == 1 ) led_is_on_now = 0;

    if ( led_is_on_now == 1 )
    {
      if ( flashy_led == 1 ) {
        digitalWrite(led_pin, !digitalRead(led_pin));
      } 
      else {
        digitalWrite(led_pin, HIGH);
      }
    }

    if ( led_is_on_now == 0 ) digitalWrite(led_pin, LOW);
    turn_on_tail;

    delay_1_ms;

  }


  for (delay_loop = 1; delay_loop <  digitdif; delay_loop++)
  {                           //extra delay to compensate for integer math (other half done earlier)
    wait25us();                                             
  }

  //Cycle 2 (back 180 degrees of spin) - pretty much everything works the same...


  for (x = 1; x <= delaytime_long; x++)    //each loop is 1ms (delaytime is length of 180 degrees of cycle)
  {

    motors_left;                                        //start off under full power

    led_ref = led_ref + 1;

    if ( x >= begin_brake && x < end_brake ) {         //switch to single motor as soon as entering braking cycle


        if ( forward = 0 && backward == 0 )              //if ( sitting still
      {
        if ( dont_alternate_motors == 1 )            //swap motors once per rotation when not moving
        {
          if ( alternate_motor_cycle == 1 ) motor1_on;
          if ( alternate_motor_cycle == 2 ) motor2_on;
          else
            if ( alternate_motor_cycle == 1 ) motor2_on;       //alternates which motor is used each cycle if sitting still
          if ( alternate_motor_cycle == 2 ) motor1_on;       //this prevents unwanted "translation" due to any imbalances
        }                                                     //if just using one wheel - this "pulses" it
      }

      //if ( going forward / back set motors appropriately (this is "where it happens")
      if ( forward = 1 ) motor2_on;
      if ( backward = 1 ) motor1_on;

    }

    if ( x >= end_brake ) motors_left;                 //if we hit end of brake cycle - go to full power

    if ( x < power_kill_part1 ) motors_off;            //if throttle is less that 100% - kill power at appropriate time
    if ( x > power_kill_part2 ) motors_off;            //if throttle is less that 100% - kill power at appropriate time

    if ( cut_power == 1 ) motors_off;                   //if this is a no-power spin

    if ( led_ref == led_on ) led_is_on_now = 1;        //turn on heading led
    if ( led_ref == led_off ) led_is_on_now = 0;        //turn off heading led

    if ( no_led == 1 ) led_is_on_now = 0;

    if ( led_is_on_now == 1 )
    {
      if ( flashy_led == 1 ) {
        digitalWrite(led_pin, !digitalRead(led_pin));
      } 
      else {
        digitalWrite(led_pin, HIGH);
      }
    }

    if ( led_is_on_now == 0 ) digitalWrite(led_pin, LOW);

    turn_on_tail;
    delay_1_ms;
  }

}

void delay_1_ms()
{
//arduino doesn't seem to pay attention to the 20mhz listed for the organgutan in boards.txt....
//    delayMicroseconds(1250);
delay(0);
}

void turn_on_tail()
{
  
  if ( tail_start <= tail_end )
  {
    if ( led_ref >= tail_start && led_ref <= tail_end ) digitalWrite(led_pin, HIGH);
  }
  else
  {
    if ( led_ref >= tail_start ) digitalWrite(led_pin, HIGH);
    if ( led_ref <= tail_end ) digitalWrite(led_pin, HIGH);
  }
}


void wait25us()
{

}


void motors_off()
{
  digitalWrite(motor1_pin, LOW);
  digitalWrite(motor2_pin, LOW);
}


void motors_left()
{
  digitalWrite(motor1_pin, HIGH);
  digitalWrite(motor2_pin, HIGH);

}


void motor1_on()
{
  digitalWrite(motor1_pin, HIGH);
  digitalWrite(motor2_pin, LOW);
}

void motor2_on()
{
  digitalWrite(motor1_pin, LOW);
  digitalWrite(motor2_pin, HIGH);
}


//updates throttle any time specified pin goes high/low
void throttle_change()
{
    //did the pin go HIGH? - then note time
    if ( digitalRead(throttle_pin) == 1 ) throttle_hightime = TCNT1;    

    //did the pin go low? - then set timer value as value for this channel / if timer has overflowed then ignore
    if ( digitalRead(throttle_pin) == 0 && throttle_hightime != 0 && TCNT1 > throttle_hightime) throttle = TCNT1 - throttle_hightime;      
  
    throttle_hilow = digitalRead(throttle_pin);                                  //make note of pin state for reference next time interrupt is triggered...
}

//updates throttle any time specified pin goes high/low
void leftright_change()
{
    //did the pin go HIGH? - then note time
    if ( digitalRead(leftright_pin) == 1 ) leftright_hightime = TCNT1;    

    //did the pin go low? - then set timer value as value for this channel / if timer has overflowed then ignore
    if ( digitalRead(leftright_pin) == 0 && leftright_hightime != 0 && TCNT1 > leftright_hightime) leftright = TCNT1 - leftright_hightime;      
  
    leftright_hilow = digitalRead(leftright_pin);                                  //make note of pin state for reference next time interrupt is triggered...
}

//updates throttle any time specified pin goes high/low
void forwardback_change()
{
    //did the pin go HIGH? - then note time
    if ( digitalRead(forwardback_pin) == 1 ) forwardback_hightime = TCNT1;    

    //did the pin go low? - then set timer value as value for this channel / if timer has overflowed then ignore
    if ( digitalRead(forwardback_pin) == 0 && forwardback_hightime != 0 && TCNT1 > forwardback_hightime) forwardback = TCNT1 - forwardback_hightime;      
  
    forwardback_hilow = digitalRead(forwardback_pin);                                  //make note of pin state for reference next time interrupt is triggered...
}

//updates throttle any time specified pin goes high/low
void rudder_change()
{
    //did the pin go HIGH? - then note time
    if ( digitalRead(rudder_pin) == 1 ) rudder_hightime = TCNT1;    

    //did the pin go low? - then set timer value as value for this channel / if timer has overflowed then ignore
    if ( digitalRead(rudder_pin) == 0 && rudder_hightime != 0 && TCNT1 > rudder_hightime) rudder = TCNT1 - rudder_hightime;      
  
    rudder_hilow = digitalRead(rudder_pin);                                  //make note of pin state for reference next time interrupt is triggered...
}





void SetupTimer1()
{

  TCCR1A = 0;                               //mode = 0
  TCCR1B = 0<<CS12 | 1<<CS11 | 1<<CS10;     //prescaler = 64

//  TIMSK1 = 1<<TOIE2;      //Timer1 Overflow interrupt Enable   

}

SIGNAL(PCINT0_vect) {
  PCint(0);
}
SIGNAL(PCINT1_vect) {
  PCint(1);
}
SIGNAL(PCINT2_vect) {
  PCint(2);
}

//Timer1 overflow interrupt vector handler
ISR(TIMER1_OVF_vect) {
  //  unused...
}


void misc()
{
  
//just some random diagnostic stuff here - not used....  
  
//accelerometer test - just sits there and blinks if accel is over certain value
/*  while (1)
  {
    delay(20);
    accel_raw_data = analogRead(acceldata_pin);
    if (accel_raw_data > 503) digitalWrite(led_pin, !digitalRead(led_pin)); 
  }
*/




//rx test
/*
  while (1)
  {
    delay(20);
    if (leftright == 468) digitalWrite(led_pin, !digitalRead(led_pin)); 
  }
*/


  
  
  
  
}
