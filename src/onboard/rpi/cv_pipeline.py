# cv_pipeline.py

import numpy as np
import cv2
import time
import threading
from picamera2 import Picamera2
from collections import deque

class CVPipeline(threading.Thread):
    def __init__(self):
        super().__init__(name="CVPipeline", daemon=True)
        self.load_camera_intrinsics()
        self.frame_buffer = deque(maxlen=2)
        self.time_buffer = deque(maxlen=2)
        self.rate = 30  # target FPS
        self.vx = 0.0
        self.vy = 0.0
        self.latest_debug_frame = None
        self.start_camera()

    def load_camera_intrinsics(self):
        calib = np.load("camera_calibration.npz")
        self.K = calib["K"]
        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]

    def start_camera(self):
        self.picam2 = Picamera2()
        self.picam2.video_configuration.main.size = (160, 120)
        self.picam2.video_configuration.main.format = "RGB888"
        self.picam2.configure("video")
        self.picam2.start()

    def capture_frame(self):
        t = time.perf_counter()
        frame = self.picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        self.frame_buffer.append(gray)
        self.time_buffer.append(t)

    def optical_flow(self):
        if len(self.frame_buffer) < 2:
            return self.vx, self.vy

        prev_gray = self.frame_buffer[0]
        gray = self.frame_buffer[1]
        dt = self.time_buffer[1] - self.time_buffer[0]
        z = 1.0  # meters

        if dt <= 0 or z < 0.01:
            return self.vx, self.vy

        prev_pts = cv2.goodFeaturesToTrack(prev_gray, maxCorners=300, qualityLevel=0.01, minDistance=3)
        if prev_pts is None:
            return self.vx, self.vy

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None)

        # Filter valid points
        good_prev = prev_pts[status == 1]
        good_curr = curr_pts[status == 1]

        if len(good_prev) == 0 or len(good_curr) == 0:
            return self.vx, self.vy

        displacements = good_curr - good_prev
        dx = np.median(displacements[:, 0])
        dy = np.median(displacements[:, 1])

        raw_vx = (dx * z) / (self.fx * dt)
        raw_vy = (dy * z) / (self.fy * dt)

        self.vx = 0.7 * self.vx + 0.3 * raw_vx
        self.vy = 0.7 * self.vy + 0.3 * raw_vy

        # Draw tracked points and motion vectors
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for (p0, p1) in zip(good_prev, good_curr):
            x0, y0 = int(p0[0]), int(p0[1])
            x1, y1 = int(p1[0]), int(p1[1])
            cv2.arrowedLine(vis, (x0, y0), (x1, y1), (0, 255, 0), 1, tipLength=0.3)

        self.latest_debug_frame = vis
        return self.vx, self.vy


    def run(self):
        while True:
            with open("log.txt", "a") as f:
                self.capture_frame()
                self.optical_flow()
                f.write(f"{time.perf_counter()}, {self.vx}, {self.vy}\n")