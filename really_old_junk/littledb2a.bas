                                                           '-----------------------------------------------------------------------------------------
'name                     : instr.bas
'copyright                : (c) 1995-2005, MCS Electronics
'purpose                  : INSTR function demo
'micro                    : Mega48
'suited for demo          : yes
'commercial addon needed  : no
'-----------------------------------------------------------------------------------------


$crystal = 20000000                                         ' used crystal frequency
$hwstack = 32                                               ' default use 32 for the hardware stack
$swstack = 10                                               ' default use 10 for the SW stack
$framesize = 40                                             ' default use 40 for the frame space
$baud = 9600

'portb.0 - Throttle  (87 = low, 114 = middle, 148 = high)
'portb.3 - Left / Right (425 = left, 469 = center, 541 = right)
'portb.4 - Forward / Back (145 = forward, 119 = center, 83 = back)


'adc.5 accelerometer

'portd.5 - LED

Config Adc = Single , Prescaler = Auto , Reference = Internal

Config Portd = Output
Config Portb = Input

Enable Interrupts
Enable Pcint0
On Pcint0 Rc_change
Pcmsk0 = &B00011001

Dim A As Byte
Dim X As Integer
Dim Z As Integer

Dim Accel_read As Word
Dim Accel_read_raw As Word
Dim Accel_read2 As Word

Dim Base_accel As Word

Dim Rcread As Integer

Dim Rc_cycle As Integer

Dim Forward As Integer
Dim Backward As Integer

Dim Begin_brake As Integer
Dim End_brake As Integer

Dim Delaytime As Long

Dim Led_on_time As Long
Dim Led_off_time As Long

Dim Halfdelay As Long

Dim Radius As Single

Dim Forward_comp As Single
Dim Backward_comp As Single

Dim Rpm As Single
Dim G As Single
Dim Gmax As Single
Dim Gloop As Single

Dim Periodms As Single

Dim Add_delay As Single

Dim Leftright As Integer
Dim Forwardback As Integer

Dim Forwardback_comp As Long

Dim Throttle As Integer

Dim Leftright1 As Integer
Dim Forwardback1 As Integer
Dim Throttle1 As Integer

Dim Shutdown As Integer

Dim Serialdata As String * 10

Declare Sub Motors_off
Declare Sub Motors_left
Declare Sub Motors_break1
Declare Sub Motors_break2
Declare Sub Test_loop


Declare Sub Check_invert

Declare Sub Read_rc
Declare Sub Accel_test
Declare Sub Read_random_rc

Dim Norc As Integer
Dim Inrc As Integer

Dim Throttle_hilow As Boolean
Dim Forwardback_hilow As Boolean
Dim Leftright_hilow As Boolean

Dim Rc_count As Integer


Dim Heading_center As Long
Dim Forwardback_center As Long

Dim Heading_leftthresh As Long
Dim Heading_rightthresh As Long


Dim X_delta As Single
Dim Y_delta As Single
Dim Stick_angle As Single
Dim Last_stick_angle As Single

Dim Delta_stick_angle As Single




'Setup times for RC READ

Config Timer0 = Timer , Prescale = 256
Config Timer1 = Timer , Prescale = 64
Config Timer2 = Timer , Prescale = 256

Start Timer0
Start Timer1
Start Timer2

Disable Timer0
Disable Timer1
Disable Timer2

Heading_center = 469
Forwardback_center = 476

Heading_leftthresh = 465
Heading_rightthresh = 473

Last_stick_angle = 0


'Radius = .28                                                'effective radius of circle for accel
Radius = 2.35                                               'effective radius of circle for accel

Forward_comp = .94                                          'compensation when going forward
Backward_comp = 1.06                                        'compensation when going back


Base_accel = 512                                            'value for accel with no motion

A = 1

Rc_count = 0




'Motors_break1

'Waitms 1000
'Motors_off
'End




While A = 1

Rc_count = Rc_count + 1

'if no rc for 15 spins then shutdown
Norc = 0
If Rc_count > 15 Then
   Throttle = 0
   Norc = 1
End If

'If G > 13 Then Shutdown = 1

'If throttle is down or off the charts - do nothing

   While Throttle < 115 Or Throttle > 200 Or Shutdown = 1

      'keep timer1 reset
'      Timer1 = 0

      Motors_off


      Serialdata = Str(leftright)
      Print Serialdata;
      Print ", ";

      Serialdata = Str(forwardback)
      Print Serialdata;
      Print ", ";

Goto Skip1

      Serialdata = Str(throttle)
      Print Serialdata;
      Print ", ";

      Serialdata = Str(accel_read_raw)
      Print "last accel:";
      Print Serialdata

      Serialdata = Str(g)
      Print "Last g:";
      Print Serialdata

      Serialdata = Str(periodms)
      Print "Last periodms:";
      Print Serialdata

      Accel_read2 = Getadc(5)
      Serialdata = Str(accel_read2)
      Print "Current accel:";
      Print Serialdata


      Serialdata = Str(norc)
      Print "NORC: ";
      Print Serialdata;
      Print ", ";


      Print

      Skip1:

      Waitms 100

      Portd.6 = 1
      Waitms 10
      Portd.6 = 0



'      Serialdata = Str(stick_angle)
'      Print "angle: ";
'      Print Serialdata;
'      Print ", ";

      Serialdata = Str(delta_stick_angle)
      Print "delta angle: ";
      Print Serialdata;
      Print ", ";



      Serialdata = Str(delaytime)
      Print "delta angle: ";
      Print Serialdata

      Waitms 10


   Wend


'   If Forwardback > 120 And Forwardback < 250 Then Forward = 1 Else Forward = 0
'   If Forwardback < 105 And Forwardback > 50 Then Backward = 1 Else Backward = 0

   Disable Interrupts

   Start Adc
   Accel_read_raw = Getadc(5)
   Stop Adc

   Accel_read = Accel_read_raw
   If Accel_read < Base_accel Then Accel_read = Base_accel
   Accel_read = Accel_read - Base_accel
   G = Accel_read / 2                                       '10mv / g, 5mv per single increment up to 1024

   Rpm = 28.45 * Radius                                     'calculate RPM from G's  - rpm  = (G/(28.45* radius ))^0.5 *1000

'   If Forward = 1 Then Rpm = Rpm * Forward_comp
'   If Backward = 1 Then Rpm = Rpm * Backward_comp

   Rpm = G / Rpm
   Rpm = Rpm ^ .5
   Rpm = Rpm * 1000
   Periodms = Rpm / 60
   Periodms = 1 / Periodms
   Periodms = Periodms * 1000


   'alternate RC read between cycles to keep balance - also used to balance spin
   If Rc_cycle = 1 Then Rc_cycle = 2 Else Rc_cycle = 1

   'Adjust for heading



      Forwardback_comp = Forwardback * 4
      X_delta = Heading_center - Leftright
      Y_delta = Forwardback_center - Forwardback_comp

      Last_stick_angle = Stick_angle
      Stick_angle = Atn2(y_delta , X_delta)
      Stick_angle = Stick_angle * 180
      Stick_angle = Stick_angle / 3.14
      Stick_angle = 180 - Stick_angle

      Delta_stick_angle = Stick_angle - Last_stick_angle

      If Delta_stick_angle > 180 Then Delta_stick_angle = Delta_stick_angle - 360
      If Delta_stick_angle < -180 Then Delta_stick_angle = Delta_stick_angle + 360


'      Forward = 1

   Delta_stick_angle = Delta_stick_angle * Periodms

   Add_delay = Delta_stick_angle * Periodms
   Add_delay = Add_delay / 360
   Add_delay = Add_delay / 8

   Periodms = Periodms + Add_delay


   Dim Digitdif As Single
   Dim Rand1 As Integer
   Dim Randsingle As Single

   Delaytime = Periodms
   Digitdif = Periodms - Delaytime
   Digitdif = Digitdif * 100
   Rand1 = Rnd(100)
   Randsingle = Rand1

   If Digitdif > Randsingle Then Delaytime = Delaytime + 1


   Delaytime = Periodms / 2



   'caps on timing if going too slow or fast
   If Delaytime > 250 Then Delaytime = 250
   If Delaytime < 5 Then Delaytime = 5


   Enable Interrupts

      Halfdelay = Delaytime / 2
      Led_on_time = Delaytime / 4
      Led_on_time = Led_on_time * 3
      Led_off_time = Led_on_time


'do brake for 3/4 cycle

'    Begin_brake = Delaytime / 8
'    End_brake = Delaytime / 8
'    End_brake = End_brake * 7

'Do brake for full cycle

    Begin_brake = 1
    End_brake = Delaytime

'Cycle 1 (front 180 degrees of spin)



   If Rc_cycle = 1 Then Motors_break1
   If Rc_cycle = 2 Then Motors_break2

   For X = 1 To Delaytime

      If X = Begin_brake Then
         If Forward = 1 Then Motors_break1
         If Backward = 1 Then Motors_break2
      End If

      If X = End_brake Then Motors_left

'      If G > 9 Then Motors_off
      Waitms 1
      If X > Led_off_time Then Portd.6 = 0
   Next X

'Cycle 2 (back 180 degrees of spin)

   If Rc_cycle = 1 Then Motors_break1
   If Rc_cycle = 2 Then Motors_break2

   For X = 1 To Delaytime

      If X = Begin_brake Then
         If Forward = 1 Then Motors_break2
         If Backward = 1 Then Motors_break1
      End If

      If X = End_brake Then Motors_left


'      If G > 9 Then Motors_off
      Waitms 1
      If X > Led_on_time Then Portd.6 = 1
   Next

Wend


Return


Sub Motors_off
      Portd.3 = 0
      Portd.4 = 0
End Sub


Sub Motors_left
      Portd.3 = 1
      Portd.4 = 1
End Sub


Sub Motors_break1
      Portd.3 = 0
      Portd.4 = 1
End Sub


Sub Motors_break2
      Portd.3 = 1
      Portd.4 = 0
     ' Motors_off
End Sub




'Reads RC data - triggered by RCINT
Rc_change:

   Inrc = 1

   If Pinb.4 <> Forwardback_hilow Then

      If Pinb.4 = 0 Then
         Forwardback = Timer0
      End If


      If Pinb.4 = 1 Then
         Timer0 = 0
      End If

   End If


   If Pinb.3 <> Leftright_hilow Then

      'only set location if within bounds
      If Pinb.3 = 0 Then
         Leftright = Timer1


         If Timer1 < 650 Then
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
         Throttle = Timer2
         Rc_count = 0                                       'got throttle data - reset rc_count
      End If


      If Pinb.0 = 1 Then
         Timer2 = 0
      End If

   End If

   Throttle_hilow = Pinb.0
   Forwardback_hilow = Pinb.4
   Leftright_hilow = Pinb.3

Return




