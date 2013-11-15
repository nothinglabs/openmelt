Dim Init_volt As Word                                       'voltage compensation stuff - not currently using
Dim Current_volt As Word
Dim Volt_adjust As Single

   'Current_volt = Getadc(6)                                   'not using voltage compensation currently...



' Sets RPM manually using throttle (not using)
'   Rpm = Throttle - 10
'   Rpm = Rpm * 14


'adjust rpm for voltage (not using)
'   Volt_adjust = Current_volt / Init_volt
'   Volt_adjust = Volt_adjust * 100
'   Rpm = Rpm * Volt_adjust
'   Rpm = Rpm / 100

'Start Adc                       'stores initial voltage level for voltage compenstation - not using...
'Init_volt = Getadc(6)
'Stop Adc
