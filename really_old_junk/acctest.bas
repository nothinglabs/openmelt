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
'portc.5 motor 2 direction
'portc.3 motor 2 enable

'portd.4 LED

'portb.0 - Throttle  (124 = low, 190 = high)
'portb.3 - Left / Right (124 = left, 160 = center, 190 = right)
'portb.4 - Forward / Back (124 = forward, 158 = center, 190 = back)

'adc.6 accelerometer

Config Adc = Single , Prescaler = Auto , Reference = Off

Config Pind.3 = Output

Dim A As Byte
Dim X As Integer

Dim Pwmloop1 As Integer
Dim Pwmloop2 As Integer

dim throttle as integer

Dim Accel_read As Word
Dim Rcread As Integer

Dim Delaytime As Long
Dim Radius As Single
Dim Rpm As Single
Dim G As Single
Dim Periodms As Single

Dim Pwmofflength As Integer

Dim S As String * 10



Declare Sub Motors_on
Declare Sub Motors_off
Declare Sub Motors_forward
Declare Sub Motors_back
Declare Sub Motors_left
Declare Sub Motors_right
Declare Sub Normal_drive

Declare Sub Pwm(byval Pwmduty As Integer , Byval Pwmlength As Integer)



Config Portd = Output

Config Portb = Output
Config Portc = Output

Config Pind.0 = Input
'Config Pind.4 = Output
Config Pinb.0 = Input

Portd.4 = 0

A = 1


Portd.5 = 0


Config Adc = Single , Prescaler = Auto , Reference = Internal



A = 1

Radius = 2.5

Start Adc



While A = 1

  Accel_read = Getadc(6)

  S = Str(accel_read) + Chr(10) + Chr(13)

  Serout S , 0 , D , 3 , 9600 , 0 , 8 , 1
  Waitms 240

'  Toggle Portd.4

Wend
