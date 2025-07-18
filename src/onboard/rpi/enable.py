# from uart_esp32 import OnboardComms
# comms = OnboardComms()
# # for i in range(10):
# #     comms.test_signal()
# while True:
#     comms.get_latest_line()

import serial

ser = serial.Serial('/dev/serial0', 115200, timeout=0)
buffer = b""
while True:
    with open("log.txt", 'a+') as f:
        data = ser.read(64)
        if data:
            buffer += data
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                f.write(line.decode().strip()+"\n")
                print("[RX]", line.decode().strip())

