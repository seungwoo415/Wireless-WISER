import serial
import time

ser = serial.Serial(
    '/dev/cu.usbmodem1101',  # make sure it's cu, not tty
    baudrate=115200,
    timeout=0               # non-blocking
)

# REQUIRED for Zephyr CDC ACM
ser.dtr = True
ser.rts = True 

print("Connected") 

while True:
    ser.write(b'1')
    print("Sent: 1")

    data = ser.read(256)
    if data:
        print("Got:", data)

    time.sleep(0.2)