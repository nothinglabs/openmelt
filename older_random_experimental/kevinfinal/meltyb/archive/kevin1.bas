'Melty B - Translational Drift / Melty Brain combat robot
'www.spambutcher.com

'This code is provided for your use without warranty / etc...

'for Kevin's bot

'Hardware:
'Atmega 168 / 20MHZ crystal
'Bascom AVR Compiler (need commercial version due to size)
'Motor control: STMicroelectronics BU941ZT Darlington drivers (Mouser.com)
'Accelerometer: Freescale 200G MMA2301EG (Mouser.com)
'MCU Board: Pololu Baby Orangutan / Mega168 (pololu.com)

'portb.0 - Throttle  (87 = low, 115 = middle, 148 = high)
'portb.1 - Left / Right (425 = left, 469 = center, 541 = right)
'portb.2 - Forward / Back (145 = forward, 114 = center, 83 = back)


'adc.4 accelerometer (PC4)
'portd.5 - heading indicator LED (front)
'portd.3 - heading indicator LED (tail)
'portd.2 - motor 1
'portd.4 - motor 2 (unused this rev.)
'portd.0 - accel power


'unused
   'portd.6 - voltage divider / monitor
   'portb.5 - Rudder (425 = left, 469 = center, 541 = right)


$crystal = 20000000                                         ' used crystal frequency
$hwstack = 32                                               ' default use 32 for the hardware stack
$swstack = 10                                               ' default use 10 for the SW stack
$framesize = 40                                             ' default use 40 for the frame space
'$baud = 9600

Config Portd = Output
Config Portb = Input
Config Portc.0 = Input

Config Adc = Single , Prescaler = Auto , Reference = Avcc   'accelerometer is compared to internal 2.5v voltage

'make sure those motors are off...
Portd.2 = 0
Portd.4 = 0


Dim A As Byte
Dim X As Long                                               'general variables
Dim Y As Single

Dim Voltage As Word                                         'get voltage from battery

Dim Accel_raw_data As Word                                  'raw accelerometer data
Dim Accel_read As Single                                    'single used to store accelerometer data

Dim Delay_loop As Single                                    'used to add extra delay

Dim Got_throttle As Long                                    'used for safety

Dim Rangecheck As Integer                                   'used to read RC values / verify they're in range before assigning

Dim Full_power_spin As Integer                              'if set to 1 - we're just spinning at full power (no translation)
Dim Cut_power As Integer                                    'if set to 1 - motors are off for this rotation

Dim Configmode As Integer

Dim Alternate_motor_cycle As Integer                        'flipped between 1 and 2 each spin - alternates which motor is used for power each cycle when not moving

Dim Forward As Integer                                      '1 if robot is supposed to be going forward
Dim Backward As Integer                                     '1 if robot is supposed to be going back

Dim Begin_brake As Integer                                  'point in spin to start brake
Dim End_brake As Integer                                    'point in spin to end brake

Dim Flashy_led As Integer                                   'set to 1 heading led flashes

Dim Periodms As Single                                      'how long it takes for the robot to rotate once
Dim Periodms_long As Long                                   'how long it takes for the robot to rotate once

Dim Delaytime_single As Single                              'Delaytime refers to time spent in each "cycle"
Dim Delaytime_long As Long                                  'Used for actual for / next loops (integer)

Dim In_tracking_adjust As Integer                           '1 if robot is in tracking adjustment mode

Dim Tail_start As Long                                      'offset in milliseconds for LED to come on
Dim Tail_end As Long                                        'offset in milliseconds for LED to come on

Dim Led_on As Long                                          'offset in milliseconds for LED to come on
Dim Led_off As Long                                         'offset in milliseconds for LED to come on

Dim Load_heading As Integer                                 'if heading offset is loaded from rom or not


Dim Led_adjust As Long                                      'offset in % for LED heading
Dim Led_adjust_backup As Long                               'offset in % for LED heading (used to reset if not straffing)

Dim Led_ref As Long                                         'used to count through both cycles for LED reference point

Dim No_led As Integer                                       'true if the LED recalc is off for this rotation


Dim Throttle_percent As Integer                             'percentage of full throttle (spin rate)

Dim Power_kill_length As Long                               'used for throttling - time in MS for which spin power is cut short
Dim Power_kill_part1 As Long                                'used for throttling - if before this time in cycle - power is cut
Dim Power_kill_part2 As Long                                'used for throttling - if after this time in cycle - power is cut

Dim Braking_length As Long                                  'length of braking cycle in MS - used for throttling

Dim G As Single                                             'g force the accelerometer is seeing

Dim Rpm As Single                                           'current RPM's of robot
Dim Max_rpm As Single                                       'current RPM's of robot

Dim Add_delay As Single                                     'used to calculate changes in heading
Dim Exponential As Single                                   'add_delay is raised to the power of this variable for exponential steering

Dim Digitdif As Single                                      'used in nasty code to convert periodms into an integer
Dim Rand1 As Integer
Dim Randsingle As Single

Dim Leftright As Integer                                    'heading RC channel
Dim Rudder As Integer                                       'rudder RC channel
Dim Forwardback As Integer                                  'forward/back RC channel
Dim Throttle As Integer                                     'throttle RC channel

Dim Left_strafe As Boolean
Dim Right_strafe As Boolean

Dim Shutdown As Integer                                     'if set to 1 - robot goes into safety mode

Dim Throttle_hilow As Boolean                               'indicate if given RC channel was hi or low on last read
Dim Forwardback_hilow As Boolean
Dim Leftright_hilow As Boolean
Dim Rudder_hilow As Boolean

Dim Leftright_last As Integer                               'track value of timer upon going high
Dim Rudder_last As Integer

Enable Interrupts
Enable Pcint0
On Pcint0 Rc_change                                         'call RC_change anytime RC pins go up or down
'Pcmsk0 = &B00111001                                         'sets mask so that only RC pins trigger interrupt
Pcmsk0 = &B11111111                                         'sets mask so that anything changing on B port triggers interrupt

'Setup timers for RC READ
Config Timer0 = Timer , Prescale = 256                      'forward / back
Config Timer1 = Timer , Prescale = 64                       'timer1 used for left/right - provides higher resolution
Config Timer2 = Timer , Prescale = 256                      'throttle

Portd.5 = 1                                                 'turn on signal LED before timers so it comes on immediately

Start Timer0                                                'start timers for reading RC
Start Timer1
Start Timer2

Disable Timer0                                              'disabling timer overflow interrupts (may or may not be needed)
Disable Timer1
Disable Timer2

Dim Led_is_on_now As Integer                                'used to keep track if tracking LED should be on now or not

Dim Tracking_comp_store As Eram Single                      'used to store tracking adjustment in ROM
Dim Led_adjust_store As Eram Long                           'used to store LED tracking adjustment in ROM (which way is "forward")
Dim Base_accel_store As Eram Single                         'used to store base level for accelerometer in ROM
Dim Full_charge_voltage_store As Eram Long                  'used to store low voltage alarm threshhold in ROM
Dim Eprom_check As Eram Integer                             'used to store validate stored ROM value

Dim Low_voltage As Integer
Dim Full_charge_voltage As Long
Dim Low_voltage_cutoff As Long

Dim Tracking_comp As Single                                 'user compensation for tracking error

Dim Eprom_single_read As Single                             'used to read from eprom

Dim Base_accel As Single                                    'ADC value for accel with no motion

Dim Strafing_is_off As Integer                              'is strafing off?

Dim Dont_alternate_motors As Integer                        'changes motor cycling when not translating

Declare Sub Wait_one_ms                                     'wait one millisecond (contains other stuff that happens during time...)

Declare Sub Motors_off                                      'motors off
Declare Sub Motors_left                                     'both motors on
Declare Sub Motor1_on                                       'turn motor 1 on
Declare Sub Motor2_on                                       'turn motor 2 on
Declare Sub Turn_on_tail                                    'handles LED for tail

A = 1                                                       'value set to be always "true"
Configmode = 0                                              'we're not in configmode to start

Exponential = 1.18                                          'greater number = more exponential steering (normal)
'Exponential = 1.28                                          'greater number = more exponential steering (flight)

Const Heading_center = 477                                  'center value for heading
Const Heading_leftthresh = 470                              'center - 7
Const Heading_rightthresh = 484                             'center + 7

Const Rudder_center = 475                                   'center value for rudder
Const Rudder_leftthresh = 515                               'center + 40
Const Rudder_rightthresh = 435                              'center - 40

Const Min_rpm = 200                                         'minimum RPM for translation / throttling to kick in

Const Max_allowed_rpm = 3750                                'max_rpm allowed before cutting power
Const Max_g = 500                                           'max G's before cutting power

Const Radius = 2.02                                         'effective radius of circle for accel (centimeters) - seems to be off sometimes...
Const G_per_adc_increment = .2                              '10mv / g, 5mv per single increment up to 1024

Const Fudge_factor = .54                                    'each cycle is reduced by this many ms to compensate for fixed time events
                                                             'each analog read = 0.7ms * 2 + x for calc time


'for combat - 200g accel
'Const Radius = 2.02                                         'effective radius of circle for accel (centimeters) - seems to be off sometimes...
'Const G_per_adc_increment = .5                              '10mv / g, 5mv per single increment up to 1024


Strafing_is_off = 1                                         'disables strafing (for flight)

Dont_alternate_motors = 0                                   'changes motor cycling when not translating (for flight)

Const Forward_comp = 1.00                                   'heading compensation when going forward (1.00 = none)
Const Backward_comp = .99                                   'heading compensation when going back  (1.00 = none)
                                                            'default variable values if nothing is read in from eprom
Tracking_comp = 1                                           'tracking compensation defaults to 1 (no adjustment)

Led_adjust = 39                                             'default setting for heading offset (1 to 100)
Led_adjust_backup = Led_adjust

Load_heading = 1                                            'if set to 0 - will use as heading as set above as opposed to loading from rom

Full_charge_voltage = 70
Base_accel = 489

'Const Throttle_percent_max = 50                             'throttle max (for flight - maximizes translation)
Const Throttle_percent_max = 100                            'throttle maxes at 100% (normal - allows less translation / higher top speed)

'Const Throttle_percent_min = 50                             'throttle starts higher (for flight)
Const Throttle_percent_min = 0                              'throttle starts at 0% (normal)


Motors_off                                                  'make sure those motors are off...

If Eprom_check = 555 Then                                   'load config data from eprom if eprom_check was set to 555
   Tracking_comp = Tracking_comp_store
   If Load_heading = 1 Then Led_adjust = Led_adjust_store
   If Load_heading = 1 Then Led_adjust_backup = Led_adjust_store
   Full_charge_voltage = Full_charge_voltage_store
   Base_accel = Base_accel_store
End If

Portd.0 = 1                                                 'turn on power for accel (if accel is connected to chip for power)

Start Adc                                                   'start ADC for accelerometer

'Accel Test
'While A = 1
'   Accel_raw_data = Getadc(4)
'   If Accel_raw_data > 489 Then Toggle Portd.5
'   Waitms 50
'Wend

Motors_off                                                  'make sure those motors are off...


For X = 1 To 800                                            'blink on boot-up
Toggle Portd.5
Toggle Portd.3
Waitms 1
Motors_off                                                  'make sure those motors are off...
Next X


For X = 1 To 20                                             'blink on boot-up
Toggle Portd.5
Toggle Portd.3
Waitms 20
Motors_off                                                  'make sure those motors are off...
Next X



'prevent boot...
'While A = 1
'   Motors_off
'wend


While A = 1                                                 'main loop

'if throttle is lower than 90 bot stays powered down
   While Throttle < 90 Or Throttle > 200 Or Shutdown = 1

      Motors_off

      'interrupt blinking if stick isn't centered
      'used to help debug if center is off
      If Leftright > Heading_rightthresh Then Waitms 200
      If Leftright < Heading_leftthresh Then Waitms 200


      'if stick is pulled back (and not in configmode) - flash out highest RPM
      If Forwardback < 105 And Forwardback > 0 And Configmode = 0 Then

         Portd.5 = 0
         Waitms 500

         For Y = 1 To Max_rpm Step 100

            Portd.5 = 1
            Waitms 100
            Portd.5 = 0
            Waitms 300

            If Throttle > 90 Then Y = Max_rpm               'abort if throttle gets pushed up

         Next Y

            If Throttle < 90 Then Waitms 2000               'only wait if throttle is still low

      End If


      'if stick is upper right - then toggle configmode
      If Forwardback > 125 And Leftright > Rudder_leftthresh Then

         'wait a bit to make sure stick is being held...
         Waitms 1400

         If Forwardback > 125 And Leftright > Rudder_leftthresh Then

            If Configmode = 0 Then
               Configmode = 1

               'assign base_accel
                  Accel_raw_data = Getadc(4)
                  Base_accel = Accel_raw_data

            Else
               Configmode = 0

               'write out new data to ROM
               Tracking_comp_store = Tracking_comp          'write out config data to ROM
               Led_adjust_store = Led_adjust
               Full_charge_voltage_store = Full_charge_voltage
               Base_accel_store = Base_accel
               Eprom_check = 555                            'write out arbitrary value to validate tracking_comp was written out

            End If

            Waitms 500

         End If

      End If


       If Forwardback < 105 And Leftright > Rudder_leftthresh And Configmode = 1 Then       ''if stick is held to back right while in config mode
                                                                                 'reset full_charge_voltage
                                                                                'intended to be set when battery is at full charge
         'wait a bit to make sure stick is being held...
         Waitms 1400

         If Forwardback < 105 And Leftright > Rudder_leftthresh And Configmode = 1 Then

               Full_charge_voltage = Getadc(6)

               For X = 1 To 70                              'blink to indicate new full_charge_voltage set
               Toggle Portd.5
               Waitms 20
               Next X
               Portd.5 = 0
               Waitms 1000

         End If

         If Forwardback < 105 And Leftright > Rudder_leftthresh And Configmode = 1 Then       'if they're still holding back right - disable voltage monitoring

               Full_charge_voltage = 0                      'set full_charge_voltage to 0 - in effect disabling voltage monitoring

               For X = 1 To 70                              'blink again to inidicate voltage monitoring disabled
               Toggle Portd.5
               Waitms 20
               Next X
               Portd.5 = 0
               Waitms 1000

         End If


      End If


      If Forwardback < 105 And Leftright < Rudder_rightthresh And Configmode = 1 Then       ''if stick is held to back left while in config mode
                                                                                 'reset heading adjustment

         'wait a bit to make sure stick is being held...
         Waitms 1400

         If Forwardback < 105 And Leftright < Rudder_rightthresh And Configmode = 1 Then

               Tracking_comp = 1                            'reset heading adjustment

               For X = 1 To 70
               Toggle Portd.5
               Waitms 20
               Next X
               Portd.5 = 0
               Waitms 1000

         End If

      End If


      'sit there and flash LED

      Portd.5 = 0
      Waitms 50

      If Configmode = 1 Then
         For X = 1 To 10
            Waitms 30
            Toggle Portd.5
         Next X
         Portd.5 = 0
         Waitms 150
      End If

      Portd.5 = 1
      Waitms 50


   Wend


   'safety
   'if we haven't gotten the throttle reading in 5 cycles - set throttle to 0
   Got_throttle = Got_throttle + 1
   If Got_throttle > 100 Then Got_throttle = 100
   If Got_throttle > 5 Then Throttle = 0


   'reset max RPM
   Max_rpm = 0

   Disable Interrupts                                       'bad things seem to happen if the RC interrupts get triggered while doing math...

   'Are we going forward or backwards?
   If Forwardback > 125 And Forwardback < 250 Then Forward = 1 Else Forward = 0
   If Forwardback < 100 And Forwardback > 50 Then Backward = 1 Else Backward = 0

   Flashy_led = 0

   Low_voltage = 0

   '325 = 8.4v
   '487 = 12.6v
   '389 = 10v
   '345 = 8.6v
   '311 = 8v

   Low_voltage_cutoff = Full_charge_voltage * 85            'low_voltage_cutoff is x% of full charge voltage
   Low_voltage_cutoff = Low_voltage_cutoff / 100

   Voltage = Getadc(6)

   If Voltage < Low_voltage_cutoff Then Low_voltage = 1
'   If Voltage < 37 Then Low_voltage = 1                     'hard coded to about 7.1V

   Low_voltage = 0                                          ' tail always on / override low-voltage warning


   Accel_raw_data = Getadc(4)                               'get accel data (word)
   Accel_read = Accel_raw_data                              'move it over to single in case we want to do floating point
   Accel_read = Accel_read - Base_accel                     'compensate for base (2.5v) level
   G = Accel_read * G_per_adc_increment                     'convert to G's

   Rpm = 28.45 * Radius                                     'calculate RPM from G's  - rpm  = (G/(28.45* radius ))^0.5 *1000
   Rpm = G / Rpm
   Rpm = Rpm ^ .5
   Rpm = Rpm * 1000

   If Rpm > Max_rpm Then Max_rpm = Rpm

   Periodms = Rpm / 60                                      'convert RPM to duration of each spin in milliseconds
   Periodms = 1 / Periodms
   Periodms = Periodms * 1000

   Periodms = Periodms * Tracking_comp                      'compensate with user-set tracking adjustment

   If Alternate_motor_cycle = 1 Then Alternate_motor_cycle = 2 Else Alternate_motor_cycle = 1       'alternates Alternate_motor_cycle - used to balance spin

If Strafing_is_off = 1 Then Goto Strafing_off               'strafing is off for air-testing

If No_led = 1 Then                                          'skip straffing for one cycle if we had a change last time (setting no_led to 1)
   No_led = 0
   Goto Strafing_off
End If

'strafing
No_led = 0                                                  'make sure LED is recalc on unless we decide to turn it off

Led_adjust = Led_adjust_backup

If Rudder > Rudder_leftthresh Then
   If Left_strafe = 0 Then
      Periodms = Periodms * 1.25                            'one time adjustment to turn 90 degrees
      No_led = 1                                            ' led recalc off this cycle - otherwise will look glitchy
   End If
   Led_adjust = Led_adjust - 25
   Left_strafe = 1
   Forward = 1
   Backward = 0
Else
   If Left_strafe = 1 Then
      Periodms = Periodms * .75                             'correct for prior straffing
      No_led = 1                                            ' led recalc off this cycle - otherwise will look glitchy
   End If
   Left_strafe = 0
End If


If Rudder < Rudder_rightthresh Then
   If Right_strafe = 0 Then
      Periodms = Periodms * .75                             'one time adjustment to turn 90 degrees
      No_led = 1                                            ' led recalc off this cycle - otherwise will look glitchy
   End If
   Led_adjust = Led_adjust + 25
   Right_strafe = 1
   Forward = 1
   Backward = 0
Else
   If Right_strafe = 1 Then
      Periodms = Periodms * 1.25                            'correct for prior straffing
      No_led = 1                                            ' led recalc off this cycle - otherwise will look glitchy
   End If
   Right_strafe = 0
End If

Strafing_off:

   If Forward = 1 Then Periodms = Periodms * Forward_comp   'extra compensation if going forward
   If Backward = 1 Then Periodms = Periodms * Backward_comp 'extra compensation if going backward


   Periodms = Periodms - Fudge_factor                       'compensate for time spent doing reads / calculations


'make sure led_adjust didn't get set above or below limits

If Led_adjust < 1 Then Led_adjust = Led_adjust + 100
If Led_adjust > 100 Then Led_adjust = Led_adjust - 100

   Delaytime_single = Periodms / 2                          'sets period in MS for each half of spin

'converts throttle reading from remote into percentage (Throttle - 87 = low, 115 = middle, 148 = high)
   Throttle_percent = Throttle - 90
   Throttle_percent = Throttle_percent * 2
   If Throttle_percent > 100 Then Throttle_percent = 100    'don't got over 100%


'driver moves stick left and right until the bot tracks correctly
'data is written into eprom next time the robot spins down

   In_tracking_adjust = 0

'tracking adjustment - if throttle is neither forward or back and throttle is under 50% then go into tracking adjustment mode

   If Configmode = 1 And Forward = 0 And Backward = 0 And Throttle_percent < 50 Then

          In_tracking_adjust = 1
          Flashy_led = 1                                    'to indicate heading adjust most - LED flashing is on

         If Leftright > Heading_rightthresh Or Leftright < Heading_leftthresh Then

              Flashy_led = 0                                'turn off flashing to show is actively being adjusted

              Add_delay = Heading_center - Leftright
              Add_delay = Add_delay * Delaytime_single

              Add_delay = Power(add_delay , Exponential)    'yes - this shouldn't work for negative numbers - but in this language it does...

              Add_delay = Add_delay / 14000000

              Tracking_comp = Tracking_comp + Add_delay

         End If

    End If

   'adjust led direction if throttle is over 50% and in configmode + not going back / forward

   If Configmode = 1 And Forward = 0 And Backward = 0 And Throttle_percent >= 50 Then

      In_tracking_adjust = 1
      Flashy_led = 0                                        'to indicate LED adjust most - LED flashing is off

      If Leftright < Heading_leftthresh Then
        Led_adjust = Led_adjust + 1
        Flashy_led = 1                                      'turn on flashing to indicate change
      End If

      If Leftright > Heading_rightthresh Then
        Led_adjust = Led_adjust - 1
        Flashy_led = 1                                      'turn on flashing to indicate change
      End If

      If Led_adjust < 1 Then Led_adjust = 100               '"wrap" around when adjusting LED direction
      If Led_adjust > 100 Then Led_adjust = 1

      Backward = 0                                          'don't really go backwards

      Led_adjust_backup = Led_adjust

   End If


   If In_tracking_adjust = 0 Then                           'don't do normal heading adjustments if we're doing tracking adjustments

      'normal drive heading change
      'this code adds or subtracts a percentage of delaytime based on the heading data from the remote
      'don't do if in configmode

         If Leftright > Heading_rightthresh Or Leftright < Heading_leftthresh Then

              Add_delay = Heading_center - Leftright
              Add_delay = Add_delay * Delaytime_single

              Add_delay = Power(add_delay , Exponential)    'yes - this shouldn't work for negative numbers - but in this language it does...

              Add_delay = Add_delay / 7700

              Delaytime_single = Delaytime_single + Add_delay

         End If

   End If

   If Configmode = 1 Then Throttle_percent = 50             'ignore throttle if in configmode


   Delaytime_long = Delaytime_single

   Digitdif = Delaytime_single - Delaytime_long


   'code that waits X microseconds to compensate for integer math
   Digitdif = Digitdif * 10                                 'converts digit dif to 50's of microseconds
   For Delay_loop = 1 To Digitdif                           'does half of delay (other half done between cycles)
      Waitus 35                                             '15us is used for looping (measured)
   Next Delay_loop

   'caps on timing if going too slow or fast
   If Delaytime_long > 500 Then Delaytime_long = 500
   If Delaytime_long < 5 Then Delaytime_long = 5

    Periodms = Delaytime_long * 2                           'we re-use periodms (full cycle length) for LED calculations - so it needs to be updated with any timing adjustments
    Periodms_long = Periodms

   'set heading beacon size and location

    Led_on = Periodms * Led_adjust
    Led_on = Led_on / 100
    Led_off = Periodms / 3                                  'led signal is 33% of circle
    Led_off = Led_off + Led_on


    If Led_off => Periodms_long Then                        'if led_off is "later" or at end of cycle - shift led_off behind by one cycle
       Led_off = Led_off - Periodms_long
    End If

    If Led_off < 1 Then Led_off = Led_off + Periodms_long

    Tail_start = Periodms_long * 17                         'code to calculate position of LED tail
    Tail_start = Tail_start / 60
    Tail_start = Tail_start + Led_off

    Tail_end = Periodms_long * 6
    Tail_end = Tail_end / 60
    Tail_end = Tail_end + Tail_start


    If Tail_start => Periodms_long Then
        Tail_start = Tail_start - Periodms_long
    End If

    If Tail_end => Periodms_long Then
        Tail_end = Tail_end - Periodms_long
    End If

    If G > Max_g And Throttle_percent > 40 Then Throttle_percent = 40       'if we're over max RPM for translation - reduce throttle

   'throttling

    If Throttle_percent > Throttle_percent_max Then Throttle_percent = Throttle_percent_max
    If Throttle_percent < Throttle_percent_min Then Throttle_percent = Throttle_percent_min


   Full_power_spin = 0
   Cut_power = 0

   If Rpm < Min_rpm Then Full_power_spin = 1                'if we're under the minimum RPM for translation - do the full power spin!

'full kill on throttle if over RPM
   If Rpm > Max_allowed_rpm Then Cut_power = 1              'if we're over max RPM for translation - cut power

'just reduce to certain percentage
'   If Rpm > Max_allowed_rpm Then Throttle_percent = 6       'if we're over max RPM for translation - reduce power


    If Throttle_percent > 50 Then                           'if throttle is at or over 50% throttle - adjust time spent in braking

      Flashy_led = 1                                        'flash the LED to indicate we're in fast mode

      Braking_length = Delaytime_long * 25                  'original
      Braking_length = Braking_length / Throttle_percent    'braking_length =  Delaytime_long / 4 when throttle_percent = 100
                                                            'braking_length = Delaytime_long / 2 when throttle_percent = 50
      Begin_brake = Delaytime_long / 2
      Begin_brake = Begin_brake - Braking_length

      End_brake = Delaytime_long / 2
      End_brake = End_brake + Braking_length

      If Begin_brake < 1 Then Begin_brake = 1               'make sure begin_brake isn't getting set to 0

      Power_kill_part1 = 0                                  'power_kill not used if throttle over 50%
      Power_kill_part2 = Delaytime_long

    End If


    If Throttle_percent <= 50 Then                          'if throttle under 50% - kill the motors for a portion of each spin

        Begin_brake = 1
        End_brake = Delaytime_long

        Power_kill_length = 50 - Throttle_percent           'set time in each cycle to cut power (throttling)
        Power_kill_length = Power_kill_length * Delaytime_long
        Power_kill_length = Power_kill_length / 150

        Power_kill_part1 = Power_kill_length
        Power_kill_part2 = Delaytime_long - Power_kill_length

    End If


'    Low_voltage = 1                                         'kills tail

      If Low_voltage = 1 Or No_led = 1 Then                 'kill tail if voltage is low / initiating strafe
            Tail_start = 0
            Tail_end = 0
      End If



   Enable Interrupts                                        'out of all the critical stuff

   If Full_power_spin = 1 Then

      'reset variables for full power spin
      End_brake = 1
      Begin_brake = 0

        Power_kill_part1 = 0
        Power_kill_part2 = Delaytime_long

   End If



    'Do translational drift driving

   'Cycle 1 (front 180 degrees of spin)


   Led_ref = 0

      For X = 1 To Delaytime_long                           'each loop is 1ms (delaytime is length of 180 degrees of cycle)

         Motors_left                                        'start off under full power

         Led_ref = Led_ref + 1

         If X => Begin_brake And X < End_brake Then         'switch to single motor as soon as entering braking cycle

            'if sitting still
            If Forward = 0 And Backward = 0 Then
               If Alternate_motor_cycle = 1 Then Motor1_on  'alternates which motor is used each cycle if sitting still
               If Alternate_motor_cycle = 2 Then Motor2_on  'this prevents unwanted "translation" due to any imbalances
            End If

            'if going forward / back set motors appropriately (this is "where it happens")
            If Forward = 1 Then Motor1_on
            If Backward = 1 Then Motor2_on
         End If

         If X => End_brake Then Motors_left                 'if we hit end of brake cycle - go to full power

         If X < Power_kill_part1 Then Motors_off            'if throttle is less that 100% - kill power at appropriate time
         If X > Power_kill_part2 Then Motors_off            'if throttle is less that 100% - kill power at appropriate time

         If Cut_power = 1 Then Motors_off                   'if this is a no-power spin

         If Led_ref = Led_on Then Led_is_on_now = 1         'turn on heading led
         If Led_ref = Led_off Then Led_is_on_now = 0        'turn off heading led

         If No_led = 1 Then Led_is_on_now = 0

         If Led_is_on_now = 1 Then
               If Flashy_led = 1 Then Toggle Portd.5 Else Portd.5 = 1
         End If

         If Led_is_on_now = 0 Then Portd.5 = 0

         Wait_one_ms

      Next X

   For Delay_loop = 1 To Digitdif                           'extra delay to compensate for integer math (other half done earlier)
      Waitus 35                                             '15us is used for looping (measured)
   Next Delay_loop

   'Cycle 2 (back 180 degrees of spin) - pretty much everything works the same...


      For X = 1 To Delaytime_long                           'each loop is 1ms (delaytime is length of 180 degrees of cycle)

         Motors_left                                        'start off under full power

         Led_ref = Led_ref + 1

         If X => Begin_brake And X < End_brake Then         'switch to single motor as soon as entering braking cycle
            'if sitting still

            If Forward = 0 And Backward = 0 Then
               If Dont_alternate_motors = 1 Then            'swap motors once per rotation when not moving
                  If Alternate_motor_cycle = 1 Then Motor1_on
                  If Alternate_motor_cycle = 2 Then Motor2_on
               Else
                  If Alternate_motor_cycle = 1 Then Motor2_on       'alternates which motor is used each cycle if sitting still
                  If Alternate_motor_cycle = 2 Then Motor1_on       'this prevents unwanted "translation" due to any imbalances
               End If                                       'if just using one wheel - this "pulses" it
            End If

            'if going forward / back set motors appropriately (this is "where it happens")
            If Forward = 1 Then Motor2_on
            If Backward = 1 Then Motor1_on

         End If

         If X => End_brake Then Motors_left                 'if we hit end of brake cycle - go to full power

         If X < Power_kill_part1 Then Motors_off            'if throttle is less that 100% - kill power at appropriate time
         If X > Power_kill_part2 Then Motors_off            'if throttle is less that 100% - kill power at appropriate time

         If Cut_power = 1 Then Motors_off                   'if this is a no-power spin

         If Led_ref = Led_on Then Led_is_on_now = 1         'turn on heading led
         If Led_ref = Led_off Then Led_is_on_now = 0        'turn off heading led

         If No_led = 1 Then Led_is_on_now = 0

         If Led_is_on_now = 1 Then
               If Flashy_led = 1 Then Toggle Portd.5 Else Portd.5 = 1
         End If

         If Led_is_on_now = 0 Then Portd.5 = 0

         Wait_one_ms

      Next

Wend


Sub Turn_on_tail
         Portd.3 = 0
         If Tail_start <= Tail_end Then
            If Led_ref => Tail_start And Led_ref <= Tail_end Then Portd.3 = 1
         Else
            If Led_ref => Tail_start Then Portd.3 = 1
            If Led_ref <= Tail_end Then Portd.3 = 1
         End If
End Sub

Sub Wait_one_ms
    Waitus 700                                              'total wait time is 1ms - split up in middle to pulse tail
    Turn_on_tail
    Waitus 300                                              'total wait time is 1ms - split up in middle to pulse tail
End Sub

Sub Motors_off
      Portd.2 = 0
      Portd.4 = 0
End Sub


Sub Motors_left
      Portd.2 = 1
      Portd.4 = 1
End Sub


Sub Motor1_on
      Portd.2 = 1
      Portd.4 = 0
End Sub


Sub Motor2_on
      Portd.2 = 0
      Portd.4 = 1
End Sub




'Reads RC data - triggered by RCINT anytime one of the RC pins goes high or low
'Uses timers to determine how long since the signal went high
Rc_change:

'valid range 50 to 200

   If Pinb.2 <> Forwardback_hilow Then

      If Pinb.2 = 0 Then                                    'did the pin go low? - then set timer value as value for this channel...
         Forwardback = Timer0
      End If


      If Pinb.2 = 1 Then                                    'did the pin go high? - then reset timer...
         Timer0 = 0
      End If

   End If

   If Pinb.0 <> Throttle_hilow Then

      If Pinb.0 = 0 Then

         If Timer2 < 200 Then                               'only set if within bounds
            If Timer2 > 50 Then
               Throttle = Timer2
               Throttle = Throttle - 8                      'throttle "trim"
               Got_throttle = 0                             'set that we got the throttle successfully
            End If
         End If

      End If


      If Pinb.0 = 1 Then
         Timer2 = 0
      End If

   End If


'these two are a bit different since they share the same 16-bit timer
'valid range 275-700

   If Pinb.1 <> Leftright_hilow Then

      If Pinb.1 = 0 Then
         Rangecheck = Timer1 - Leftright_last
         If Rangecheck > 275 Then
            If Rangecheck < 700 Then
               Leftright = Rangecheck                       'if the timer overflowed - just discard it
            End If
         End If
      End If


      If Pinb.1 = 1 Then
         Leftright_last = Timer1
      End If

   End If


   If Pinb.5 <> Rudder_hilow Then

      If Pinb.5 = 0 Then
         Rangecheck = Timer1 - Rudder_last
         If Rangecheck > 275 Then
            If Rangecheck < 700 Then
               Rudder = Rangecheck                          'if the timer overflowed - just discard it
            End If
         End If
      End If


      If Pinb.5 = 1 Then
         Rudder_last = Timer1
      End If

   End If


   Throttle_hilow = Pinb.0                                  'make note of all pin states for reference next time interrupt is triggered...
   Forwardback_hilow = Pinb.2
   Leftright_hilow = Pinb.1
   Rudder_hilow = Pinb.5


Return