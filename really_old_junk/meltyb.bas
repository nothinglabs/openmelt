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

'portc.1 motor 1 direction
'portc.2 motor 1 direction
'portc.0 motor 1 enable

'portc.4 motor 2 direction
'portd.0 motor 2 direction
'portc.3 motor 2 enable

'portd.4 LED

'portb.0 - Throttle  (124 = low, 190 = high)
'portb.3 - Left / Right (124 = left, 160 = center, 190 = right)
'portb.4 - Forward / Back (190 = forward, 158 = center, 124 = back)
'portb.5 - Invert (over 150 = on)

'adc.6 accelerometer

Config Adc = Single , Prescaler = Auto , Reference = Internal

Config Portd = Output
Config Portb = Input

Config Portc = Output
Config Portc.5 = Input

Dim A As Byte
Dim X As Integer
Dim Z As Integer
Dim Zz As Integer

Dim Pwmloop1 As Integer
Dim Pwmloop2 As Integer

Dim Accel_read As Word
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
Dim Rpm As Single
Dim G As Single
Dim Periodms As Single

Dim Add_delay As Integer

Dim Pwmofflength As Integer

Dim Leftright As Integer
Dim Forwardback As Integer
Dim Throttle As Integer

Dim Pwmspeed As Integer

Dim Invert As Integer

Dim Inspin As Integer

Dim Rc As Integer

Declare Sub Motors_on
Declare Sub Motors_off
Declare Sub Motors_forward
Declare Sub Motors_back
Declare Sub Motors_left
Declare Sub Motors_right
Declare Sub Normal_drive
Declare Sub Motors_break1
Declare Sub Motors_break2
Declare Sub Motors_break3

Declare Sub Check_invert

Declare Sub Read_rc
Declare Sub Accel_test
Declare Sub Read_random_rc


Declare Sub Pwm(byval Pwmduty As Integer , Byval Pwmlength As Integer)


Rc = 1

A = 1

'Radius = 2.5
Radius = 2.3

Start Adc

Motors_off

Invert = 0

Inspin = 0

Read_rc

While A = 1

   'Read_random_rc

'If throttle is down or off the charts - go to normal drive
   While Throttle < 140 Or Throttle > 250
      Call Normal_drive
      Read_rc
     'Status Light
      Toggle Portd.4
   Wend

   Inspin = 1

   Accel_read = Getadc(5)

   If Accel_read < 548 Then Accel_read = 548

   Accel_read = Accel_read - 547

   G = Accel_read * .044

   'calculate RPM from G's  - rpm  = (G/(28.45* radius ))^0.5 *1000

   Rpm = 28.45 * Radius
   Rpm = G / Rpm
   Rpm = Rpm ^ .5
   Rpm = Rpm * 1000

   Periodms = Rpm / 60
   Periodms = 1 / Periodms
   Periodms = Periodms * 1000

   Delaytime = Periodms / 2

   'full throttle!
    Pwmspeed = 10

   'fixed compensation - 46ms measured total for RC read overhead
   Delaytime = Delaytime - 23

   'alternate RC read between cycles to keep balance
   If Rc_cycle = 1 Then Rc_cycle = 2 Else Rc_cycle = 1

   'if going too slow - cap rotation at 300ms * 2
   If Delaytime > 300 Then Delaytime = 300

   'Adjust for heading

   If Leftright < 156 Then
      Add_delay = 158 - Leftright
      Add_delay = Add_delay / 3
      Delaytime = Delaytime + Add_delay
   End If

   If Leftright > 164 Then
      Add_delay = Leftright - 158
      Add_delay = Add_delay / 3
      Delaytime = Delaytime - Add_delay
   End If

      Halfdelay = Delaytime / 2
      Led_on_time = Delaytime / 4
      Led_on_time = Led_on_time * 3
      Led_off_time = Led_on_time


      If Forwardback > 170 And Forwardback < 350 Then Forward = 1 Else Forward = 0
      If Forwardback < 150 And Forwardback > 50 Then Backward = 1 Else Backward = 0


'do brake for 3/4 cycle

'      Begin_brake = Delaytime / 8
'      End_brake = Delaytime / 8
'      End_brake = End_brake * 7

'Do brake for full cycle

      Begin_brake = 1
      End_brake = Delaytime


'Cycle 1 setup (front 180 degrees of spin)

      'Cycle 1


      Motors_left

      Motors_on
      For X = 1 To Delaytime

         If X = Begin_brake Then
            If Forward = 1 Then Motors_break1
            If Backward = 1 Then Motors_break2
         End If

         If X = End_brake Then Motors_left

         Waitms 1
         'Call Pwm(pwmspeed , 1)
         If X > Led_off_time Then Portd.4 = 0
      Next X
      Motors_off

      'motors on for RC check
      Motors_left
      Motors_on
      If Rc_cycle = 1 Then Read_rc
      Motors_off

'Cycle 2 Setup (back 180 degrees of spin)


     'Cycle 2

      Motors_left

      Motors_on
      For X = 1 To Delaytime

         If X = Begin_brake Then
            If Forward = 1 Then Motors_break2
            If Backward = 1 Then Motors_break1
         End If

         If X = End_brake Then Motors_left

         Waitms 1
         'Call Pwm(pwmspeed , 1)
         If X > Led_on_time Then Portd.4 = 1
      Next
      Motors_off

      'motors on for RC check
      Motors_left
      Motors_on
      If Rc_cycle = 2 Then Read_rc
      Motors_off

Wend



'Sub to handle non-translational drift driving

Sub Normal_drive

   'check if invert switch has been flipped on remote
   Check_invert

   If Leftright < 155 And Leftright > 50 Then
      Motors_left
      Pwmspeed = 158 - Leftright
      Pwmspeed = Pwmspeed / 3
      Pwmspeed = Pwmspeed + 1
      Call Pwm(pwmspeed , 75)

   Else

      If Leftright > 164 And Leftright < 350 Then
         Motors_right
         Pwmspeed = Leftright - 158
         Pwmspeed = Pwmspeed / 3
         Pwmspeed = Pwmspeed + 1
         Call Pwm(pwmspeed , 75)

      Else

         If Forwardback > 164 And Forwardback < 350 Then
            Motors_forward
            Pwmspeed = Forwardback - 158
            Pwmspeed = Pwmspeed / 3
            Pwmspeed = Pwmspeed + 2
            Call Pwm(pwmspeed , 75)
         End If

         If Forwardback < 155 And Forwardback > 50 Then
            Motors_back
            Pwmspeed = 158 - Forwardback
            Pwmspeed = Pwmspeed / 3
            Pwmspeed = Pwmspeed + 2
            Call Pwm(pwmspeed , 75)
         End If

      End If

   End If


End Sub




Sub Motors_on
      Portc.0 = 1
      Portc.3 = 1
End Sub


Sub Motors_off
      Portc.0 = 0
      Portc.3 = 0
End Sub


Sub Motors_left
      'motor 1 direction
      Portc.1 = 0
      Portc.2 = 1

      'motor 2 direction
      Portc.4 = 0
      Portd.0 = 1
End Sub



Sub Motors_right
      'motor 1 direction
      Portc.1 = 1
      Portc.2 = 0

      'motor 2 direction
      Portc.4 = 1
      Portd.0 = 0
End Sub


Sub Motors_forward
      'motor 1 direction
      Portc.1 = 1
      Portc.2 = 0

      'motor 2 direction
      Portc.4 = 0
      Portd.0 = 1
End Sub


Sub Motors_back
      'motor 1 direction
      Portc.1 = 0
      Portc.2 = 1

      'motor 2 direction
      Portc.4 = 1
      Portd.0 = 0
End Sub


Sub Motors_break1
      'motor 1 direction
      Portc.1 = 0
      Portc.2 = 0

      'motor 2 direction
      Portc.4 = 0
      Portd.0 = 1
End Sub


Sub Motors_break2
      'motor 1 direction
      Portc.1 = 0
      Portc.2 = 1

      'motor 2 direction
      Portc.4 = 0
      Portd.0 = 0
End Sub


'unused sub
Sub Motors_break3
      'motor 1 direction
      Portc.1 = 0
      Portc.2 = 0

      'motor 2 direction
      Portc.4 = 0
      Portd.0 = 0
End Sub



Sub Pwm(byval Pwmduty As Integer , Byval Pwmlength As Integer)

'pwmlength is in ms
'pwmduty is 1 to 10

If Pwmduty > 10 Then Pwmduty = 10

Pwmofflength = 10 - Pwmduty

For Pwmloop1 = 1 To Pwmlength

   Motors_on
   For Pwmloop2 = 1 To Pwmduty
      Waitus 100
   Next Pwmloop2

   Motors_off
   For Pwmloop2 = 1 To Pwmofflength
      Waitus 100
   Next Pwmloop2

Next Pwmloop1


End Sub


Sub Read_rc

   Pulsein Rcread , Pinb , 0 , 1
   Throttle = Rcread

   Pulsein Rcread , Pinb , 4 , 1
   Forwardback = Rcread

   Pulsein Rcread , Pinb , 3 , 1
   Leftright = Rcread

End Sub


'Unused sub

Sub Read_random_rc

   If Rc = 1 Then
      Pulsein Rcread , Pinb , 0 , 1
      Throttle = Rcread
      Rc = 2

      Else

      If Rc = 2 Then
         Pulsein Rcread , Pinb , 4 , 1
         Forwardback = Rcread
         Rc = 3

      Else

         If Rc = 3 Then
            Pulsein Rcread , Pinb , 3 , 1
            Leftright = Rcread
            Rc = 1
         End If

   End If
   End If



End Sub


Sub Check_invert
   Pulsein Rcread , Pinb , 5 , 1
   If Rcread > 150 Then Invert = 1 Else Invert = 0
End Sub



'Unused sub

Sub Accel_test

A = 1

While A = 1

   Accel_read = Getadc(5)

   If Accel_read < 547 Then Accel_read = 547

   Accel_read = Accel_read - 547

   G = Accel_read * .048

   If G > .95 Then Portd.4 = 1 Else Portd.4 = 0

Wend


End Sub