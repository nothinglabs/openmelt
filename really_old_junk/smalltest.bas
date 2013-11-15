'Melty B - Translational Drift / Melty Brain combat robot
'www.spambutcher.com

'This code is provided for your use without warranty / etc...

'Hardware:
'Atmega 168 / 20MHZ crystal
'Bascom AVR Compiler (need commercial version due to size)
'Motor control: STMicroelectronics BU941ZT Darlington drivers (Mouser.com)
'Accelerometer: Freescale 200G MMA2301EG (Mouser.com)
'MCU Board: Pololu Baby Orangutan / Mega168 (pololu.com)

'portb.0 - Throttle  (87 = low, 115 = middle, 148 = high)
'portb.3 - Left / Right (425 = left, 469 = center, 541 = right)
'portb.4 - Forward / Back (145 = forward, 114 = center, 83 = back)

'adc.4 accelerometer
'portd.5 - heading indicator LED (on motor controller)
'portd.2 - motor 1
'portd.4 - motor 2

$crystal = 20000000                                         ' used crystal frequency
$hwstack = 32                                               ' default use 32 for the hardware stack
$swstack = 10                                               ' default use 10 for the SW stack
$framesize = 40                                             ' default use 40 for the frame space

Config Portd = Output
Config Portb = Input

Config Adc = Single , Prescaler = Auto , Reference = Avcc   'accelerometer is compared to internal 2.5v voltage

Dim A As Byte                                               'general variables
Dim X As Long

Dim Accel_raw_data As Word                                  'raw accelerometer data
Dim Accel_read As Single                                    'single used to store accelerometer data

Dim Alternate_motor_cycle As Integer                        'flipped between 1 and 2 each spin - alternates which motor is used for power each cycle when not moving

Dim Forward As Integer                                      '1 if robot is supposed to be going forward
Dim Backward As Integer                                     '1 if robot is supposed to be going back


Dim Begin_brake As Integer                                  'point in spin to start brake
Dim End_brake As Integer                                    'point in spin to end brake

Dim Periodms As Single                                      'how long it takes for the robot to rotate once
Dim Periodms_long As Long                                   'how long it takes for the robot to rotate once

Dim Delaytime_single As Single                              'Delaytime refers to time spent in each "cycle"
Dim Delaytime_long As Long                                  'Used for actual for / next loops (integer)

Dim Led_on As Long                                          'offset in milliseconds for LED to come on
Dim Led_off As Long                                         'offset in milliseconds for LED to come on

Dim Led_ref As Long                                         'used to count through both cycles for LED reference point

Dim G As Single                                             'g force the accelerometer is seeing

Dim Rpm As Single                                           'current RPM's of robot

Dim Add_delay As Single                                     'used to calculate changes in heading

Dim Leftright As Integer                                    'heading RC channel
Dim Forwardback As Integer                                  'forward/back RC channel
Dim Throttle As Integer                                     'throttle RC channel

Dim Throttle_hilow As Boolean                               'indicate if given RC channel was hi or low on last read
Dim Forwardback_hilow As Boolean
Dim Leftright_hilow As Boolean

Enable Interrupts
Enable Pcint0
On Pcint0 Rc_change                                         'call RC_change anytime RC pins go up or down
Pcmsk0 = &B00011001                                         'sets mask so that only RC pins trigger interrupt

'Setup timers for RC READ
Config Timer0 = Timer , Prescale = 256                      'forward / back
Config Timer1 = Timer , Prescale = 64                       'timer1 used for left/right - provides higher resolution
Config Timer2 = Timer , Prescale = 256                      'throttle

Start Timer0                                                'start timers for reading RC
Start Timer1
Start Timer2

Disable Timer0                                              'disabling timer overflow interrupts (may or may not be needed)
Disable Timer1
Disable Timer2

Dim Led_is_on_now As Integer                                'used to keep track if tracking LED should be on now or not

Const Heading_center = 477                                  'center value for heading
Const Heading_leftthresh = 472                              'center - 5
Const Heading_rightthresh = 482                             'center + 5

Const Radius = 2.35                                         'effective radius of circle for accel (centimeters) - seems to be off sometimes...
Const G_per_adc_increment = .5                              '10mv / g, 5mv per single increment up to 1024
Const Base_accel = 591                                      'value reported by the accelerometer when it's at 0g

Declare Sub Motors_off                                      'motors off
Declare Sub Motors_left                                     'both motors on
Declare Sub Motor1_on                                       'turn motor 1 on
Declare Sub Motor2_on                                       'turn motor 2 on

A = 1                                                       'value set to be always "true"

Start Adc                                                   'start ADC for accelerometer

While A = 1                                                 'main loop

'if throttle is lower than 90 bot stays powered down
   While Throttle < 90 Or Throttle > 200

      Portd.5 = 1                                           'turn led on
      Motors_off                                            'sit there with motors off

   Wend

   Disable Interrupts                                       'bad things seem to happen if the RC interrupts get triggered while doing math...

   'Are we going forward?
   If Forwardback > 120 Then Forward = 1 Else Forward = 0
   If Forwardback < 105 Then Backward = 1 Else Backward = 0

   Accel_raw_data = Getadc(4)                               'get accel data (word)
   Accel_read = Accel_raw_data                              'move it over to single in case we want to do floating point
   Accel_read = Accel_read - Base_accel                     'compensate for base (2.5v) level
   G = Accel_read * G_per_adc_increment                     'convert to G's

   Rpm = 28.45 * Radius                                     'calculate RPM from G's  - rpm  = (G/(28.45* radius ))^0.5 *1000
   Rpm = G / Rpm
   Rpm = Rpm ^ .5
   Rpm = Rpm * 1000

   Periodms = Rpm / 60                                      'convert RPM to duration of each spin in milliseconds
   Periodms = 1 / Periodms
   Periodms = Periodms * 1000

   If Alternate_motor_cycle = 1 Then Alternate_motor_cycle = 2 Else Alternate_motor_cycle = 1       'alternates Alternate_motor_cycle - used to balance spin

   Delaytime_single = Periodms / 2                          'sets period in MS for each half of spin

   'normal drive heading change
   'this code adds or subtracts a percentage of delaytime based on the heading data from the remote

        Add_delay = Heading_center - Leftright
        Add_delay = Add_delay * Delaytime_single
        Add_delay = Add_delay / 2300
        Delaytime_single = Delaytime_single + Add_delay

        Delaytime_long = Delaytime_single

   'set heading beacon size and location
    Led_on = 1
    Led_off = Periodms / 3                                  'led signal is 33% of circle

    Begin_brake = 1
    End_brake = Delaytime_long

   Enable Interrupts                                        'out of all the critical stuff

   'Do translational drift driving

   'Cycle 1 (front 180 degrees of spin)

   Led_ref = 0

      For X = 1 To Delaytime_long                           'each loop is 1ms (delaytime is length of 180 degrees of cycle)

         Motors_left                                        'start off under full power

         Led_ref = Led_ref + 1

         If X => Begin_brake And X < End_brake Then
            If Alternate_motor_cycle = 1 Then Motor1_on Else Motor2_on       'alternates which motor is used each cycle if sitting still

            If Forward = 1 Then Motor1_on                   'if going forward / back set motors appropriately (this is "where it happens")
            If Backward = 1 Then Motor2_on
         End If

         If X => End_brake Then Motors_left                 'if we hit end of brake cycle - go to full power

         If Led_ref > Led_on Then Portd.5 = 1               'turn on heading led
         If Led_ref > Led_off Then Portd.5 = 0              'turn off heading led

         Waitms 1                                           'wait 1ms

      Next X


   'Cycle 2 (back 180 degrees of spin) - pretty much everything works the same...


      For X = 1 To Delaytime_long                           'each loop is 1ms (delaytime is length of 180 degrees of cycle)

         Motors_left                                        'start off under full power

         Led_ref = Led_ref + 1

         If X => Begin_brake And X < End_brake Then
            If Alternate_motor_cycle = 1 Then Motor2_on Else Motor1_on       'alternates which motor is used each cycle if sitting still

            If Forward = 1 Then Motor2_on                   'if going forward / back set motors appropriately (this is "where it happens")
            If Backward = 1 Then Motor1_on
         End If

         If X => End_brake Then Motors_left                 'if we hit end of brake cycle - go to full power

         If Led_ref > Led_on Then Portd.5 = 1               'turn on heading led
         If Led_ref > Led_off Then Portd.5 = 0              'turn off heading led

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

   If Pinb.4 <> Forwardback_hilow Then

      If Pinb.4 = 0 Then Forwardback = Timer0 Else Timer0 = 0       'did the pin go low? - then set timer value as value for this channel...

                                                                         'otherwise reset timer
   End If


   If Pinb.3 <> Leftright_hilow Then

      If Pinb.3 = 0 Then Leftright = Timer1 Else Timer1 = 0

   End If


   If Pinb.0 <> Throttle_hilow Then

      If Pinb.0 = 0 Then

         If Timer2 < 200 Then                               'only set if within bounds
               Throttle = Timer2
         End If

      Else

         Timer2 = 0

      End If

   End If

   Throttle_hilow = Pinb.0                                  'make note of all pin states for reference next time interrupt is triggered...
   Forwardback_hilow = Pinb.4
   Leftright_hilow = Pinb.3

Return