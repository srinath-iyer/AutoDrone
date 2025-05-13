import serial
import time

ser = serial.Serial('/dev/serial0', 115200, timeout=1)
ser.flush()
i = 0
while True:
    # Send a message to the ESP32
    ser.write(b"Hi from Pi!\n")

    # Read from ESP32 if available
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        print(f"{i}: ESP32 says: {line}")
        i+=1
    
    time.sleep(2)
