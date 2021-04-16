import serial
import time

data_stream = serial.Serial('/dev/ttyACM3', 115200, timeout=0)

datum = ""
append = False
data_found = False

while 1:
    while data_found==False:
        incoming_char = data_stream.read()

        if append:
            if incoming_char == '}':
                print(datum)
                datum = ""
                data_found = True
                append = False
            else:
                datum += incoming_char

        if (incoming_char=='{'):
            append = True
    data_found = False
    time.sleep(.02)