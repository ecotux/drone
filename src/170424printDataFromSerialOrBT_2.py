## Get data from serial cable (port='/dev/ttyACM1') or bluetooth (port='/dev/rfcomm0')

import serial
import sys
 
ser = serial.Serial(
    port = '/dev/ttyACM1',
#   port = '/dev/rfcomm0',
    baudrate = 38400,
    bytesize = serial.EIGHTBITS,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    timeout = None,
    xonxoff = False,      # disable software flow control
    rtscts = False,       # disable hardware (RTS/CTS) flow control
    dsrdtr = False,     # disable hardware (DSR/DTR) flow control
    writeTimeout = 2      # timeout for write
)
 
command = "";
while True:
    response = ser.readline()
    if len(response) == 0:
        print "Arduino is not responding!\n"
        sys.stdout.flush()
    else:
        print response,
        sys.stdout.flush()
