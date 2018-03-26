# -*- coding: utf-8 -*-

# Examples to control software PWM port, Port D and receiving status
from time import sleep
import serial

ser = serial.Serial('COM8', 38400, timeout=0.2)
print(ser.name)

# Port D control, pins 4 to 7
ser.flushInput()
for i in range(0,10):
    o = 'M0%s' % str(i%2)
    ser.write(o.encode('ascii'))
    sleep(0.1)

# Test of status receiving and port D control
ser.flushInput()
last_a = 0
for i in range(1,10):
    ser.write(b'S')
    a = ""
    a = ser.read(150)
    a = a[:30]
    if(last_a != a):
        print("##############\ni={}\n{}\n".format(i, a.decode('unicode_escape')))
    else:
        print("OK %d" % i)
    last_a = a
    o = 'M0%s' % str(i%2)
    ser.write(o.encode('ascii'))
    a = ser.read(2)
    print(a)

# Test software PWM port [0-3]
# Usable range of values of 4 to 17 for default TIMER0 value
for i in range(0,20):
    # motor1 range 4 - 17
    for i in range(4,17):
        ser.write(b'A1%c' % i)
        sleep(0.1)
        print(i)
    
    for i in reversed(range(4,17)):
        ser.write(b'A1%c' % i)
        sleep(0.1)
        print(i)

ser.close()
