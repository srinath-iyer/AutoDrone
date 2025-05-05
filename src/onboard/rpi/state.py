"""
This file handles the calculations for drone state between the ESP32 and the vision system.
"""

class State:
    def __init__(self, x=0, y=0, z=0, yaw=0, pitch=0, roll=0):
        self.x = x  # X position in meters
        self.y = y  # Y position in meters
        self.z = z  # Z position in meters
        self.yaw = yaw  # Yaw angle in degrees
        self.pitch = pitch  # Pitch angle in degrees
        self.roll = roll  # Roll angle in degrees


    def __str__(self):
        return f"State(x={self.x}, y={self.y}, z={self.z}, yaw={self.yaw}, pitch={self.pitch}, roll={self.roll})"