'-----------------------------------------------------------------------------------------
'name                     : instr.bas
'copyright                : (c) 1995-2005, MCS Electronics
'purpose                  : INSTR function demo
'micro                    : Mega48
'suited for demo          : yes
'commercial addon needed  : no
'-----------------------------------------------------------------------------------------


'Portb.5 = 1
'$loader = $1c00

'$loadersize = 1024


$crystal = 20000000                                         ' used crystal frequency
$hwstack = 32                                               ' default use 32 for the hardware stack
$swstack = 10                                               ' default use 10 for the SW stack
$framesize = 40                                             ' default use 40 for the frame space



Dim A As Byte
Dim B As Long

Dim Back As Byte
Dim Loopa As Byte

Dim Ahead As Byte

Dim Rcread As Integer
Dim Lastread As Long
Dim Timetotal As Integer

Dim Adjust As Integer

Config Portd = Output

Config Portc.5 = Input
Config Portc.4 = Output
Config Portb.5 = Output






Config Adc = Single , Prescaler = Auto , Reference = Off

Dim Test As Integer

Dim X As Integer

'adc.6 accelerometer

Dim S As String * 10



Dim Accel_read As Word


A = 1



While A = 1

   B = 0

   While B < 8000
      Portb.1 = 1
      Waitus 11
      Portb.1 = 0
      Waitus 11
      B = B + 1
   Wend

   Toggle Portd.1
Wend


