'Melty B - Translational Drift / Melty Brain combat robot
'www.spambutcher.com

'This code is provided for your use without warranty / etc...

Hardware:
'Atmega 168 / 20MHZ crystal
'Bascom AVR Compiler (need commercial version due to size)
'Motor control: STMicroelectronics BU941ZT Darlington drivers (Mouser.com)
'Accelerometer: Freescale 200G MMA2301EG (Mouser.com)
'MCU Board: Pololu Baby Orangutan / Mega168 (pololu.com)

'portb.0 - Throttle  (87 = low, 115 = middle, 148 = high)
'portb.3 - Left / Right (425 = left, 469 = center, 541 = right)
'portb.4 - Forward / Back (145 = forward, 114 = center, 83 = back)

'adc.4 accelerometer
'adc.6 battery voltage
'portd.5 - heading indicator LED (on motor controller)
'portd.2 - motor 1
'portd.4 - motor 2
'portd.7 - accel power


$crystal = 20000000                                         ' used crystal frequency
$hwstack = 32                                               ' default use 32 for the hardware stack
$swstack = 10                                               ' default use 10 for the SW stack
$framesize = 40                                             ' default use 40 for the frame space
$baud = 9600

Config Portd = Output
Config Portb = Input


Config Adc = Single , Prescaler = Auto , Reference = Avcc   'accelerometer is compared to internal 2.5v voltage

Dim A As Byte                                               'general variables
Dim X As Long
Dim Y As Single

Dim Voltage As Word                                         'get voltage from battery

Dim Accel_raw_data As Word                                  'raw accelerometer data
Dim Accel_read As Single                                    'single used to store accelerometer data

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
Dim Led_adjust As Long                                      'offset in milliseconds for LED to come on

Dim Led_ref As Long                                         'used to count through both cycles for LED reference point

Dim Throttle_percent As Integer                             'percentage of full throttle (spin rate)

Dim Power_kill_length As Long                               'used for throttling - time in MS for which spin power is cut short
Dim Power_kill_part1 As Long                                'used for throttling - if before this time in cycle - power is cut
Dim Power_kill_part2 As Long                                'used for throttling - if after this time in cycle - power is cut


Dim Braking_length As Long                                  'length of braking cycle in MS - used for throttling

Dim G As Single                                             'g force the accelerometer is seeing

Dim Rpm As Single                                           'current RPM's of robot
Dim Max_rpm As Single                                       'current RPM's of robot

Dim Add_delay As Single                                     'used to calculate changes in heading

Dim Digitdif As Single                                      'used in nasty code to convert periodms into an integer
Dim Rand1 As Integer
Dim Randsingle As Single


Dim Leftright As Integer                                    'heading RC channel
Dim Forwardback As Integer                                  'forward/back RC channel
Dim Throttle As Integer                                     'throttle RC channel

Dim Shutdown As Integer                                     'if set to 1 - robot goes into safety mode

Dim Throttle_hilow As Boolean                               'indicate if given RC channel was hi or low on last read
Dim Forwardback_hilow As Boolean
Dim Leftright_hilow As Boolean

Dim Max_allowed_rpm As Single                               'max_rpm allowed

Enable Interrupts
Enable Pcint0
On Pcint0 Rc_change                                         'call RC_change anytime RC pins go up or down
Pcmsk0 = &B00011001                                         'sets mask so that only RC pins trigger interrupt

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

Const Heading_center = 477                                  'center value for heading
Const Heading_leftthresh = 472                              'center - 5
Const Heading_rightthresh = 482                             'center + 5

Const Min_rpm = 500                                         'minimum RPM for translation / throttling to kick in

Const Radius = 2.02                                         'effective radius of circle for accel (centimeters) - seems to be off sometimes...
Const G_per_adc_increment = .5                              '10mv / g, 5mv per single increment up to 1024

Dim Base_accel As Single                                    'ADC value for accel with no motion

Const Forward_comp = 1.00                                   'heading compensation when going forward
Const Backward_comp = .99                                   'heading compensation when going back

Declare Sub Motors_off                                      'motors off
Declare Sub Motors_left                                     'both motors on
Declare Sub Motor1_on                                       'turn motor 1 on
Declare Sub Motor2_on                                       'turn motor 2 on

Max_allowed_rpm = 20000                                     'max speed allowed

A = 1                                                       'value set to be always "true"

Motors_off                                                  'make sure those motors are off...

Configmode = 0

Tracking_comp = 1                                           'tracking compensation defaults to 1 (no adjustment)
Led_adjust = 50
Full_charge_voltage = 70
Base_accel = 489

If Eprom_check = 555 Then                                   'load config data from eprom if eprom_check was set to 555
   Tracking_comp = Tracking_comp_store
   Led_adjust = Led_adjust_store
   Full_charge_voltage = Full_charge_voltage_store
   Base_accel = Base_accel_store
End If

Portd.7 = 1                                                 'turn on power for accel (if accel is connected to chip for power)

Start Adc                                                   'start ADC for accelerometer

For X = 1 To 20                                             'blink on boot-up
Toggle Portd.5
Waitms 20
Next X

While A = 1                                                 'main loop

'if throttle is lower than 90 bot stays powered down
   While Throttle < 90 Or Throttle > 200 Or Shutdown = 1

      Motors_off

      'interrupt blinking if stick isn't centered
      'used to help debug if center is off
      If Leftright > Heading_rightthresh Then Waitms 100
      If Leftright < Heading_leftthresh Then Waitms 100


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
      If Forwardback > 125 And Leftright > 515 Then

         'wait a bit to make sure stick is being held...
         Waitms 1400

         If Forwardback > 125 And Leftright > 515 Then

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


       If Forwardback < 105 And Leftright > 515 And Configmode = 1 Then       ''if stick is held to back right while in config mode
                                                                                 'reset full_charge_voltage
                                                                                'intended to be set when battery is at full charge
         'wait a bit to make sure stick is being held...
         Waitms 1400

         If Forwardback < 105 And Leftright > 515 And Configmode = 1 Then

               Full_charge_voltage = Getadc(6)

               For X = 1 To 70                              'blink to indicate new full_charge_voltage set
               Toggle Portd.5
               Waitms 20
               Next X
               Portd.5 = 0
               Waitms 1000

         End If

         If Forwardback < 105 And Leftright > 515 And Configmode = 1 Then       'if they're still holding back right - disable voltage monitoring

               Full_charge_voltage = 0                      'set full_charge_voltage to 0 - in effect disabling voltage monitoring

               For X = 1 To 70                              'blink again to inidicate voltage monitoring disabled
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


   'reset max RPM
   Max_rpm = 0

   Disable Interrupts                                       'bad things seem to happen if the RC interrupts get triggered while doing math...

   'Are we going forward or backwards?




'driver moves stick left and right until the bot tracks correctly
'data is written into eprom next time the robot spins down



   'nasty code to convert Delaytime_single into Delaytime_long
   'randomly adds 1 to delaytime_long a percentage of time proportionate to how close
   'the decimal portion of the number is to 1 (1.4 becomes 2 40% of the time)
   'this in effect improves the accuracy of tracking / steering (yes, there are better ways to handle this)



   'caps on timing if going too slow or fast
   If Delaytime_long > 250 Then Delaytime_long = 250
   If Delaytime_long < 5 Then Delaytime_long = 5


   'set heading beacon size and location
    Led_on = Periodms * Led_adjust
    Led_on = Led_on / 100
    Led_off = Periodms / 3                                  'led signal is 33% of circle
    Led_off = Led_off + Led_on

    Periodms_long = Periodms

    If Led_off => Periodms_long Then                        'if led_adjust is "later" or at end of cycle - shift led_off behind by one cycle
        Led_off = Led_off - Periodms_long
    End If

    If Led_on < 1 Then Led_on = 1
    If Led_off < 1 Then Led_off = 1



   'throttling





   Full_power_spin = 0
   Cut_power = 0

   If Rpm < Min_rpm Then Full_power_spin = 1                'if we're under the minimum RPM for translation - do the full power spin!
   If Rpm > Max_allowed_rpm Then Cut_power = 1              'if we're over max RPM for translation - cut power


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
            If Alternate_motor_cycle = 1 Then Motor1_on     'alternates which motor is used each cycle if sitting still
            If Alternate_motor_cycle = 2 Then Motor2_on     'this prevents unwanted "translation" due to any imbalances

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

         If Led_is_on_now = 0 Then Portd.5 = 0

         If Led_is_on_now = 1 Then
            If Flashy_led = 1 Then Toggle Portd.5 Else Portd.5 = 1
         End If

         If Led_ref => Tail_start And Led_ref <= Tail_end Then Portd.5 = 1
         Waitms 1                                           'wait 1ms

      Next X


   'Cycle 2 (back 180 degrees of spin) - pretty much everything works the same...



      For X = 1 To Delaytime_long                           'each loop is 1ms (delaytime is length of 180 degrees of cycle)

         Motors_left                                        'start off under full power

         Led_ref = Led_ref + 1

         If X => Begin_brake And X < End_brake Then         'switch to single motor as soon as entering braking cycle
            'if sitting still
            If Alternate_motor_cycle = 1 Then Motor2_on     'alternates which motor is used each cycle if sitting still
            If Alternate_motor_cycle = 2 Then Motor1_on     'this prevents unwanted "translation" due to any imbalances
                                                             'if just using one wheel - this "pulses" it

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

         If Led_is_on_now = 0 Then Portd.5 = 0

         If Led_is_on_now = 1 Then
            If Flashy_led = 1 Then Toggle Portd.5 Else Portd.5 = 1
         End If

         If Led_ref => Tail_start And Led_ref <= Tail_end Then Portd.5 = 1
         Waitms 1                                           'wait 1ms

      Next


Wend


Sub Motors_off
      Portd.2 = 0
      Portd.4 = 0
End Sub


Sub Motors_left
      Portd.2 = 1
      Portd.4 = 1
End Sub


Sub Motor1_on
      Portd.2 = 0
      Portd.4 = 1
End Sub


Sub Motor2_on
      Portd.2 = 1
      Portd.4 = 0
End Sub


'Reads RC data - triggered by RCINT anytime one of the RC pins goes high or low
'Uses timers to determine how long since the signal went high
Rc_change:

   If Pinb.4 <> Forwardback_hilow Then

      If Pinb.4 = 0 Then                                    'did the pin go low? - then set timer value as value for this channel...
         Forwardback = Timer0
      End If


      If Pinb.4 = 1 Then                                    'did the pin go high? - then reset timer...
         Timer0 = 0
      End If

   End If


   If Pinb.3 <> Leftright_hilow Then

      If Pinb.3 = 0 Then

         If Timer1 < 650 Then                               'only set if within bounds
            If Timer1 > 300 Then
               Leftright = Timer1
            End If
         End If


      End If


      If Pinb.3 = 1 Then
         Timer1 = 0
      End If

   End If



   If Pinb.0 <> Throttle_hilow Then

      If Pinb.0 = 0 Then

         If Timer2 < 200 Then                               'only set if within bounds
            If Timer2 > 50 Then
               Throttle = Timer2
            End If
         End If

      End If


      If Pinb.0 = 1 Then
         Timer2 = 0
      End If

   End If

   Throttle_hilow = Pinb.0                                  'make note of all pin states for reference next time interrupt is triggered...
   Forwardback_hilow = Pinb.4
   Leftright_hilow = Pinb.3


Return