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
'portd.6 - heading indicator LED
'portd.3 - motor 1
'portd.4 - motor 2

$crystal = 20000000                                         ' used crystal frequency
$hwstack = 32                                               ' default use 32 for the hardware stack
$swstack = 10                                               ' default use 10 for the SW stack
$framesize = 40                                             ' default use 40 for the frame space
$baud = 9600

Config Portd = Output
Config Portb = Input

Config Adc = Single , Prescaler = Auto , Reference = Internal       'accelerometer is compared to internal 2.5v voltage

Dim A As Byte                                               'general variables
Dim X As Long
Dim Y As Single

Dim Voltage As Word                                         'get voltage from battery

Dim Accel_raw_data As Word                                  'raw accelerometer data
Dim Accel_read As Single                                    'single used to store accelerometer data

Dim Full_power_spin As Integer                              'if set to 1 - we're just spinning at full power (no translation)
Dim Cut_power As Integer                                    'if set to 1 - motors are off for this rotation

Dim Alternate_motor_cycle As Integer                        'flipped between 1 and 2 each spin - alternates which motor is used for power each cycle when not moving

Dim Forward As Integer                                      '1 if robot is supposed to be going forward
Dim Backward As Integer                                     '1 if robot is supposed to be going back

Dim Begin_brake As Integer                                  'point in spin to start brake
Dim End_brake As Integer                                    'point in spin to end brake

Dim Turbo As Integer                                        'set to 1 if in special superfast mode (less translation)

Dim Periodms As Single                                      'how long it takes for the robot to rotate once
Dim Delaytime_single As Single                              'Delaytime refers to time spent in each "cycle"
Dim Delaytime_long As Long                                  'Used for actual for / next loops (integer)

Dim In_tracking_adjust As Integer                           '1 if robot is in tracking adjustment mode

Dim Led_on As Long                                          'offset in milliseconds for LED to come on
Dim Led_off As Long                                         'offset in milliseconds for LED to come on

Dim Led_ref As Long                                         'used to count through both cycles for LED reference point

Dim Throttle_percent As Integer                             'percentage of full throttle (spin rate)

Dim Power_kill_offset As Long                               'used for throttling - MS offset in spin at which power is cut

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

Dim Serialdata As String * 10

Dim Throttle_hilow As Boolean                               'indicate if given RC channel was hi or low on last read
Dim Forwardback_hilow As Boolean
Dim Leftright_hilow As Boolean

Dim Max_allowed_rpm As Single                               'max_rpm allowed

Dim Rc_count As Integer                                     'count number of spins since last setting of throttle data (used for safety)

Enable Interrupts
Enable Pcint0
On Pcint0 Rc_change                                         'call RC_change anytime RC pins go up or down
Pcmsk0 = &B00011001                                         'sets mask so that only RC pins trigger interrupt

'Setup timers for RC READ
Config Timer0 = Timer , Prescale = 256                      'forward / back
Config Timer1 = Timer , Prescale = 64                       'timer1 used for left/right - provides higher resolution
Config Timer2 = Timer , Prescale = 256                      'throttle

Portd.6 = 1                                                 'turn on signal LED before timers so it comes on immediately

Start Timer0                                                'start timers for reading RC
Start Timer1
Start Timer2

Disable Timer0                                              'disabling timer overflow interrupts (may or may not be needed)
Disable Timer1
Disable Timer2

Dim Led_is_on_now As Integer                                'used to keep track if tracking LED should be on now or not

Dim Tracking_comp_store As Eram Single                      'used to store tracking adjustment in ROM

Dim Tracking_comp_check As Eram Integer                     'used to store validate stored ROM value

Dim Low_voltage As Integer

Dim Tracking_comp As Single                                 'user compensation for tracking error

Dim Eprom_single_read As Single                             'used to read from eprom

Const Heading_center = 469                                  'center value for heading
Const Heading_leftthresh = 465
Const Heading_rightthresh = 473

Const Min_rpm = 700                                         'minimum RPM for translation / throttling to kick in

Const Radius = 2.2915                                       'effective radius of circle for accel (centimeters) - seems to be off sometimes...
Const G_per_adc_increment = .5                              '10mv / g, 5mv per single increment up to 1024
Const Base_accel = 504.5                                    'ADC value for accel with no motion

Const Forward_comp = 1.00                                   'heading compensation when going forward
Const Backward_comp = .99                                   'heading compensation when going back


Declare Sub Motors_off                                      'motors off
Declare Sub Motors_left                                     'both motors on
Declare Sub Motor1_on                                       'turn motor 1 on
Declare Sub Motor2_on                                       'turn motor 2 on


Max_allowed_rpm = 20000                                     'max speed allowed

A = 1                                                       'value set to be always "true"

Tracking_comp = 1                                           'tracking compensation defaults to 1 (no adjustment)

If Tracking_comp_check = 555 Then Tracking_comp = Tracking_comp_store       'get tracking_comp from ROM only if Tracking_comp_check was set to 555

Rc_count = 0                                                'make sure rc_count is 0...

Start Adc                                                   'start ADC for accelerometer




While A = 1                                                 'main loop

'   Rc_count = Rc_count + 1                                  'increment RC_count to check for safety (is reset to 0 each time throttle is succesfully received from RC)

   'if no rc for 15 spins then shutdown (set throttle to 0)
'   If Rc_count > 15 Then
'      Throttle = 0
'   End If


'if throttle is lower than 90 bot stays powered down
   While Throttle < 90 Or Throttle > 200 Or Shutdown = 1

      Motors_off


      'if stick is pulled back - flash out highest RPM
      If Forwardback < 105 Then


         For Y = 1 To Max_rpm Step 100

            Portd.6 = 1
            Waitms 100
            Portd.6 = 0
            Waitms 300

            If Throttle > 90 Then Y = Max_rpm               'abort if throttle gets pushed up

         Next Y

            If Throttle < 90 Then Waitms 2000               'only wait if throttle is still low

      Max_allowed_rpm = 20000                               'max speed allowed

      End If


     'if stick is upper right - go into low RPM demo mode
      If Forwardback > 135 And Leftright > 525 Then

         For Y = 1 To 60

            Toggle Portd.6
            Waitms 25

           If Throttle > 90 Then Y = 60                     'abort if throttle gets pushed up

         Next Y

            If Throttle < 90 Then Max_allowed_rpm = 900     'only reset speed if throttle still low after flashing

      End If



      'sit there and flash LED

      Portd.6 = 0
      Waitms 50
      Portd.6 = 1
      Waitms 50


      'if the tracking compensation has been changed by the driver - write it out to Eprom
      Eprom_single_read = Tracking_comp_store               'reads tracking_comp from eprom
      If Tracking_comp <> Eprom_single_read Then            'write out only if changed from last time - otherwise will kill eeprom
         Tracking_comp_store = Tracking_comp                'write out current tracking compensation - done here since writting to ROM takes time...
         Tracking_comp_check = 555                          'write out arbitrary value to validate tracking_comp was written out
      End If

      'debug data

      'Serialdata = Str(leftright)
      'Print Serialdata;
      'Print ", ";

'      Serialdata = Str(forwardback)
 '     Print Serialdata;
 '     Print ", ";

  '    Serialdata = Str(throttle)
  '    Print Serialdata;
  '    Print ", ";

   '   Serialdata = Str(tracking_comp)
   '   Print "Tracking_comp:";
   '   Print Serialdata

    '  Serialdata = Str(periodms)
    '  Print "Last periodms:";
    '  Print Serialdata

      Print

   Wend


   'reset max RPM
   Max_rpm = 0

   Disable Interrupts                                       'bad things seem to happen if the RC interrupts get triggered while doing math...

   'Are we going forward or backwards?
   If Forwardback > 120 And Forwardback < 250 Then Forward = 1 Else Forward = 0
   If Forwardback < 105 And Forwardback > 50 Then Backward = 1 Else Backward = 0

   Voltage = Getadc(6)                                      'get voltage data (word)

   If Voltage < 385 Then Low_voltage = 1 Else Low_voltage = 0       '395 is about 6.2v

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
   If Forward = 1 Then Periodms = Periodms * Forward_comp   'extra compensation if going forward
   If Backward = 1 Then Periodms = Periodms * Backward_comp 'extra compensation if going backward

   Periodms = Periodms - .07                                'each accel read = .07 ms
   Periodms = Periodms - .07                                'each voltage read = .07 ms

   If Alternate_motor_cycle = 1 Then Alternate_motor_cycle = 2 Else Alternate_motor_cycle = 1       'alternates Alternate_motor_cycle - used to balance spin

   Delaytime_single = Periodms / 2                          'sets period in MS for each half of spin

'converts throttle reading from remote into percentage (Throttle - 87 = low, 115 = middle, 148 = high)
   Throttle_percent = Throttle - 80
   Throttle_percent = Throttle_percent * 2
   If Throttle_percent > 100 Then Throttle_percent = 100    'don't got over 100%


'tracking adjustment - if throttle is between 1/3 and 1/2 - go into tracking adjustment mode (at full normal speed)
'driver moves stick left and right until the bot tracks correctly
'data is written into eprom next time the robot spins down

   If Throttle < 100 Then

         In_tracking_adjust = 1
         Throttle_percent = 75                              '75% speed

         If Leftright < Heading_leftthresh Then
            Tracking_comp = Tracking_comp + .002
         End If

         If Leftright > Heading_rightthresh Then
            Tracking_comp = Tracking_comp - .002
            If Tracking_comp < .04 Then Tracking_comp = .04 'don't let it get set too low...
         End If

         If Forwardback < 105 Then
            Tracking_comp = 1                               'if stick is pulled backward during heading adjustment - reset to 1
            Backward = 0
         End If

    Else

        In_tracking_adjust = 0

      'normal drive heading change
      'this code adds or subtracts a percentage of delaytime based on the heading data from the remote

        Add_delay = Heading_center - Leftright
        Add_delay = Add_delay * Delaytime_single
        Add_delay = Add_delay / 2000

        Delaytime_single = Delaytime_single + Add_delay

   End If


   'nasty code to convert Delaytime_single into Delaytime_long
   'randomly adds 1 to delaytime_long a percentage of time proportionate to how close
   'the decimal portion of the number is to 1 (1.4 becomes 2 40% of the time)
   'this in effect improves the accuracy of tracking / steering (yes, there are better ways to handle this)

   Delaytime_long = Delaytime_single
   Digitdif = Delaytime_single - Delaytime_long
   Digitdif = Digitdif * 100
   Rand1 = Rnd(100)
   Randsingle = Rand1
   If Digitdif > Randsingle Then Delaytime_long = Delaytime_long + 1


   'caps on timing if going too slow or fast
   If Delaytime_long > 250 Then Delaytime_long = 250
   If Delaytime_long < 5 Then Delaytime_long = 5


    Power_kill_offset = Throttle_percent * Delaytime_long   'set time in each cycle to cut power (throttling)
    Power_kill_offset = Power_kill_offset / 100


   'Do translation ("braking") for full cycle
    Begin_brake = 1
    End_brake = Delaytime_long

    Led_on = 1
    Led_off = Periodms / 2


   Turbo = 0


'if we're at top speed ("Turbo") - only do translation for 3/4 of each cycle (start 1/8th late / end 1/8th early)
   If Throttle > 136 Then

      Turbo = 1

      Begin_brake = Delaytime_long / 8
      End_brake = Delaytime_long / 8
      End_brake = End_brake * 7


    Led_on = 1
    Led_off = Periodms / 2


   End If

   Full_power_spin = 0
   Cut_power = 0

   If Rpm < Min_rpm Then Full_power_spin = 1                'if we're under the minimum RPM for translation - do the full power spin!
   If Rpm > Max_allowed_rpm Then Cut_power = 1              'if we're over max RPM for translation - cut power



   'special ultra-high-speed mode - does full_power_spin every other cycle if throttle is set to max
   'reduced translation - robot tends to drift forward on its own since motors aren't being switched
   'if it goes backwards change alternate_motor_cycle to 1
   If Throttle > 146 And Alternate_motor_cycle = 2 Then Full_power_spin = 1
'   If Throttle > 146 Then Full_power_spin = 1

   Enable Interrupts                                        'out of all the critical stuff

   If Cut_power = 1 Then Full_power_spin = 0

   Led_ref = 0

   If Full_power_spin = 1 Then

      Motors_left                                           'full power!

      Led_ref = 0

      For X = 1 To Delaytime_long
         Led_ref = Led_ref + 1

         If Led_ref = Led_on Then Led_is_on_now = 1         'turn on heading led
         If Led_ref = Led_off Then Led_is_on_now = 0        'turn off heading led

         If Led_is_on_now = 0 Then Portd.6 = 0

         If Led_is_on_now = 1 Then
            If In_tracking_adjust = 1 Or Turbo = 1 Then Toggle Portd.6 Else Portd.6 = 1
         End If

         Waitms 1
      Next X

      For X = 1 To Delaytime_long
         Led_ref = Led_ref + 1

         If Led_ref = Led_on Then Led_is_on_now = 1         'turn on heading led
         If Led_ref = Led_off Then Led_is_on_now = 0        'turn off heading led

         If Led_is_on_now = 0 Then Portd.6 = 0

         If Led_is_on_now = 1 Then
            If In_tracking_adjust = 1 Or Turbo = 1 Then Toggle Portd.6 Else Portd.6 = 1
         End If

         Waitms 1
      Next X


   Else

   'Do translational drift driving

   'Cycle 1 (front 180 degrees of spin)

   Motors_left                                              'start off under full power

   Led_ref = 0

      For X = 1 To Delaytime_long                           'each loop is 1ms (delaytime is length of 180 degrees of cycle)

         Led_ref = Led_ref + 1

         If X = Begin_brake Then                            'switch to single motor as soon as entering braking cycle
            'if sitting still
            If Alternate_motor_cycle = 1 Then Motor1_on     'alternates which motor is used each cycle if sitting still
            If Alternate_motor_cycle = 2 Then Motor2_on     'this prevents unwanted "translation" due to any imbalances

            'if going forward / back set motors appropriately (this is "where it happens")
            If Forward = 1 Then Motor1_on
            If Backward = 1 Then Motor2_on
         End If

         If X = End_brake Then Motors_left                  'if we hit end of brake cycle - go to full power

         If X > Power_kill_offset Then Motors_off           'if throttle is less that 100% - kill throttle at appropriate time
         If Cut_power = 1 Then Motors_off                   'if this is a no-power spin

         If Led_ref = Led_on Then Led_is_on_now = 1         'turn on heading led
         If Led_ref = Led_off Then Led_is_on_now = 0        'turn off heading led

         If Led_is_on_now = 0 Then Portd.6 = 0

         If Led_is_on_now = 1 Then
            If In_tracking_adjust = 1 Or Turbo = 1 Then Toggle Portd.6 Else Portd.6 = 1
         End If

         If Low_voltage = 1 And Alternate_motor_cycle = 1 Then Portd.6 = 1

         Waitms 1                                           'wait 1ms

      Next X


   'Cycle 2 (back 180 degrees of spin) - pretty much everything works the same...

      Motors_left                                           'start off under full power

      For X = 1 To Delaytime_long                           'each loop is 1ms (delaytime is length of 180 degrees of cycle)

         Led_ref = Led_ref + 1

         If X = Begin_brake Then                            'switch to single motor as soon as entering braking cycle
            'if sitting still
            If Alternate_motor_cycle = 1 Then Motor2_on     'alternates which motor is used each cycle if sitting still
            If Alternate_motor_cycle = 2 Then Motor1_on     'this prevents unwanted "translation" due to any imbalances

            'if going forward / back set motors appropriately (this is "where it happens")
            If Forward = 1 Then Motor2_on
            If Backward = 1 Then Motor1_on
         End If

         If X = End_brake Then Motors_left                  'if we hit end of brake cycle - go to full power

         If X > Power_kill_offset Then Motors_off           'if throttle is less that 100% - kill throttle at appropriate time
         If Cut_power = 1 Then Motors_off                   'if this is a no-power spin


         If Led_ref = Led_on Then Led_is_on_now = 1         'turn on heading led
         If Led_ref = Led_off Then Led_is_on_now = 0        'turn off heading led

         If Led_is_on_now = 0 Then Portd.6 = 0

         If Led_is_on_now = 1 Then
            If In_tracking_adjust = 1 Or Turbo = 1 Then Toggle Portd.6 Else Portd.6 = 1
         End If

         If Low_voltage = 1 And Alternate_motor_cycle = 1 Then Portd.6 = 1

         Waitms 1                                           'wait 1ms

      Next

   End If

Wend


Sub Motors_off
      Portd.3 = 0
      Portd.4 = 0
End Sub


Sub Motors_left
      Portd.3 = 1
      Portd.4 = 1
End Sub


Sub Motor1_on
      Portd.3 = 0
      Portd.4 = 1
End Sub


Sub Motor2_on
      Portd.3 = 1
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
               Rc_count = 0                                 'got throttle data - reset rc_count
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