# two-axis-microstep-controller
The present repository is a controller software for PIC 16F877 with support:
* two axis micro step (support for three axis)
* PWM out signal
* one typical servo motor pin
* speed control for final movement (based on the two axis)

The software supports the follow microsteps chips
* LB1946 microstep chip (the ones present in Epson Stylus Color C60)
* A national microstep controller
* you can implement another microstep tables
 
The maximiun step output is 12KHz for a 20MHz clock.

The software utilizes fixed point math.
The motion information is supplied in steps in XX:XX:XX:FF:FF:FF format where:
XX are the integer part
FF are the franctional part

A windows sample software is provided exampling the usage.
