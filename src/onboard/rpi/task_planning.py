"""
This file contains all task-planning related commands that are sent to the ESP32 at a high level.
"""

import asyncio

class TaskPlanning:
    
    @staticmethod
    async def go_to_pose(x, y, z, yaw, roll, pitch):
        pass

    @staticmethod
    async def takeoff(altitude = 1.0):
        pass

    @staticmethod
    async def land():
        pass
