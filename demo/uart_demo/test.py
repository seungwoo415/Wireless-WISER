import serial
import time

# Use cu (Call Out) port, 1s timeout
ser = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=1)

# Critical Zephyr Handshake
ser.dtr = True
ser.rts = True

print("Waiting for nRF52 to detect connection...")
time.sleep(2)  # Give it time to pass the while(!dtr) loop

# Clear any boot-up garbage
ser.reset_input_buffer()

print("Sending: P")
ser.write(b'P')
ser.flush()

# Wait for response
response = ser.read(1)

if response == b'O':
    print("✨ SUCCESS: Received 'O' from nRF52!")
elif response == b'':
    print("❌ FAILURE: Timed out (Received nothing)")
else:
    print(f"❓ FAILURE: Received unexpected data: {response}")

ser.close()