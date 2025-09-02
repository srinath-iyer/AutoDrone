"""
This file handles the calculations for drone state between the ESP32 and the vision system.
"""
import time

class State:
    def __init__(self, x=0, y=0, z=0, yaw=0, pitch=0, roll=0, vx=0, vy=0, vz=0, vision_corrected=False):
        self.x = x  # X position in meters
        self.y = y  # Y position in meters
        self.z = z  # Z position in meters
        self.yaw = yaw  # Yaw angle in degrees
        self.pitch = pitch  # Pitch angle in degrees
        self.roll = roll  # Roll angle in degrees\
        self.vx = vx  # Velocity in X in m/s
        self.vy = vy  # Velocity in Y in m/s
        self.vz = vz
        self.vision_corrected = vision_corrected
        self.latest_time = time.perf_counter()  # Initialize latest_time to current time

    def update_state(self, time, x, y, z, yaw, pitch, roll, vx, vy, vz, vision_corrected):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.vision_corrected = vision_corrected
        self.latest_time = time
    
    def __str__(self):
        return f"{self.latest_time}: State(x={self.x}, y={self.y}, z={self.z}, yaw={self.yaw}, pitch={self.pitch}, roll={self.roll}, vx={self.vx}, vy={self.vy}, vz={self.vz}, vision_corrected={self.vision_corrected})"

state = State()