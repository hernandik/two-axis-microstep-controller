# two-axis-microstep-controller
The present project is a controller software for PIC 16F877 with support to:
* two axis micro step (support for three axis, with minor changes)
* PWM out signal (for use as main current control)
* one typical PWM for servo motor pin
* microsteps with speed control for final movement (based on the two axis)
* 3 lines command buffer

The software supports the follow microsteps chips
* LB1946 microstep chip (the ones present in Epson Stylus Color C60)
* A national microstep controller
* you can implement another microstep tables
 
The maximiun step output is 10KHz for a 20MHz clock. This is due the math and serialization routines for the LB1946.
With a paralell or simple serial routine, is possible to achieve better resolution.

The software utilizes fixed point math.
The motion information is supplied in steps in XX:XX:XX:FF:FF:FF format where:
XX are the integer part
FF are the franctional part

A windows sample software is provided exampling the usage.
