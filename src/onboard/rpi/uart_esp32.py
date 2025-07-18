"""
This file containts all methods for the ESP32 UART communication with the Raspberry Pi.
"""

import serial
import threading
from queue import Queue
from state import State
import time
import select

class OnboardComms:
    def __init__(self):
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=0)
        self.ser.flush()
        print("Serial port initialized")
        self.test_signal()
        self.rx_queue = Queue()
        self.listener_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listener_thread.start()
        time.sleep(0.1)  # Give the thread time to start
        print("Serial port initialized")
        #self.test_signal()


    def listen_loop(self):
        buffer = b""
        while True:
            rlist, _, _ = select.select([self.ser.fileno()], [], [], 0.05)
            if rlist:
                try:
                    data = self.ser.read(64)
                    if not data:
                        time.sleep(0.005)  # back off a bit
                        continue  # don't process empty reads
                    buffer += data
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        line = line.decode('utf-8').strip()
                        self.rx_queue.put(line)
                except Exception as e:
                    print(f"[UART] Exception in listener: {e}")



    def send_command(self, command: str):
        self.ser.write(command.encode('utf-8'))
        print(f"Command sent: {command}")

    def send_state(self, state: State):
        self.ser.write(str(state).encode('utf-8'))
        print(f"State sent: {state}")

    def get_latest_line(self):
        """Non-blocking: Returns the latest UART line if available."""
        if not self.rx_queue.empty():
            return self.rx_queue.get()
    
    def test_signal(self):
        self.send_command("/test_signal")
        start_time = time.perf_counter()
        print(start_time)
        elapsed_time = 0
        while elapsed_time < 5: # 5 second timeout
            get_latest_line = self.get_latest_line()
            if get_latest_line is not None:
                end_time = time.perf_counter()
                elapsed_time = end_time - start_time
                if get_latest_line == "/test_signal_received":
                    print("Received successful message from ESP32:", get_latest_line)
                    print("Latency:", elapsed_time)
                    break
                else:
                    print("Received unexpected message from ESP32:", get_latest_line)
                    print("Latency:", elapsed_time)
                    break
            else:
                end_time = time.perf_counter()
                elapsed_time = end_time - start_time
                time.sleep(0.001)
        if elapsed_time > 5:
            print("No response from ESP32")
