import tkinter as tk
from tkinter import ttk, scrolledtext
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time
import threading
import queue
import serial
import random
import sys
from datetime import datetime
import os
import copy

from uart_esp32 import OnboardComms

class DronePIDTuner:
    def __init__(self, master):
        self.master = master
        self.master.title("Drone PID Tuner")
        self.master.geometry("1200x800")
        self.master.config(bg="#f0f0f0")
        
        # UART configuration
        self.Comms = OnboardComms()
        # self.Comms.listen_loop()
        self.Comms.test_signal()
        self.sent_log_queue = queue.Queue() # This is a queue that logs all commands sent to the ESP32. They will be timestamped strings and written to a file 

        # Data storage
        self.time_data = np.linspace(0, 10, 100)  # Last 10 seconds with 100 data points
        self.roll_data = np.zeros(100)
        self.pitch_data = np.zeros(100)
        self.yaw_data = np.zeros(100)
        self.thrust_data = np.zeros(100)
        
        # Create queues for thread-safe communication --> Explanation: queue.Queue() is a thread-safe because it has built-in locking.
        self.imu_data_queue = queue.Queue()
        self.command_queue = queue.Queue()
        self.log_queue = queue.Queue()  # Queue for log messages
        
        # PID values
        self.pid_values = {
            "roll": {"P": 1.0, "I": 0.1, "D": 0.05},
            "pitch": {"P": 1.0, "I": 0.1, "D": 0.05},
            "yaw": {"P": 1.0, "I": 0.1, "D": 0.05},
            "thrust": {"P": 1.0, "I": 0.1, "D": 0.05}
        }
        
        # Create GUI
        self.setup_gui()
        
        # Start plotting update loop
        self.master.after(100, self.update_plots)
        
        # Start log update loop
        self.master.after(100, self.update_logs)
        
    def setup_gui(self):
        # Create main frame with two sections
        main_frame = ttk.Frame(self.master)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left frame for PID controls
        left_frame = ttk.LabelFrame(main_frame, text="PID Controls")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=5, pady=5)
        
        # Right frame divided into plots and logs
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # IMU data plots at the top
        plot_frame = ttk.LabelFrame(right_frame, text="IMU Data")
        plot_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=0, pady=5)
        
        # Log display at the bottom
        #TODO: Delete this.
        log_frame = ttk.LabelFrame(right_frame, text="System Logs")
        log_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=False, padx=0, pady=5)
        
        # Create PID controls for each axis
        self.pid_inputs = {}
        self.pid_labels = {}
        
        for i, axis in enumerate(["roll", "pitch", "yaw", "thrust"]):
            axis_frame = ttk.LabelFrame(left_frame, text=f"{axis.capitalize()} PID")
            axis_frame.grid(row=i, column=0, padx=10, pady=5, sticky="ew")
            
            self.pid_inputs[axis] = {}
            self.pid_labels[axis] = {}
            
            # P, I, D inputs for each axis
            for j, pid_type in enumerate(["P", "I", "D"]):
                ttk.Label(axis_frame, text=f"{pid_type}:").grid(row=j, column=0, padx=5, pady=2)
                
                # Value label
                value_var = tk.StringVar(value=f"{self.pid_values[axis][pid_type]:.2f}")
                self.pid_labels[axis][pid_type] = value_var
                ttk.Label(axis_frame, textvariable=value_var, width=5).grid(row=j, column=1, padx=5, pady=2)
                
                # Numerical Input
                input = ttk.Entry(
                    axis_frame,
                    textvariable=value_var,
                    width=5,
                    validate="key", # Callback function to validate input after every keystroke.
                    validatecommand=(self.master.register(lambda value: value.replace('.', '', 1).isdigit()), '%P')

                )
                input.grid(row=j, column=2, padx=5, pady=2)
                self.pid_inputs[axis][pid_type] = input
        
        # Add connection status and control buttons
        control_frame = ttk.Frame(left_frame)
        control_frame.grid(row=len(self.pid_values), column=0, padx=10, pady=10, sticky="ew")
        
        # Send all PIDs button
        send_all_button = ttk.Button(control_frame, text="Send All PIDs", command=self.send_all_pids)
        send_all_button.grid(row=2, column=0, columnspan=2, padx=5, pady=5)
        
        # Create plots
        self.setup_plots(plot_frame)

    def init_log(self):
        """Initialize the log file"""
        directory_path = 'rpi/logs'
        num = len(os.listdir(directory_path)) + 1
        with open(directory_path + "/log" + str(num) + ".txt", 'w') as f:
            self.log_path = f.name

    def log(self, message):
        """Log a message to the log file and the console"""
        print(message)
        with open(self.log_path, 'a') as f:

            f.write(message + "\n")
        
    def update_logs(self):
        """Update the log display with queued messages"""
        # Process any new log messages
        log_updated = False
        max_messages = 50  # Process max 50 messages at once to prevent UI freeze
        
        for _ in range(max_messages):
            try:
                log_message = self.log_queue.get_nowait()
                
                
                
                self.log_queue.task_done()
                log_updated = True
            except queue.Empty:
                break
                
        # Schedule the next update
        self.master.after(100, self.update_logs)
        
    def setup_plots(self, frame):
        # Create figure with subplots
        self.fig = Figure(figsize=(8, 5), dpi=100)
        
        # Create 4 subplots for roll, pitch, yaw, and thrust
        self.axes = []
        plot_titles = ["Roll", "Pitch", "Yaw", "Thrust"]
        plot_colors = ["r", "g", "b", "m"]
        
        for i in range(4):
            ax = self.fig.add_subplot(4, 1, i+1)
            ax.set_title(plot_titles[i])
            ax.set_ylabel("Angle (deg)" if i < 3 else "Thrust (%)")
            ax.set_xlabel("Time (s)" if i == 3 else "")
            self.axes.append(ax)
        
        self.lines = []
        self.lines.append(self.axes[0].plot(self.time_data, self.roll_data, plot_colors[0])[0])
        self.lines.append(self.axes[1].plot(self.time_data, self.pitch_data, plot_colors[1])[0])
        self.lines.append(self.axes[2].plot(self.time_data, self.yaw_data, plot_colors[2])[0])
        self.lines.append(self.axes[3].plot(self.time_data, self.thrust_data, plot_colors[3])[0])
        
        self.fig.tight_layout()
        
        # Create canvas and add to frame
        self.canvas = FigureCanvasTkAgg(self.fig, frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def update_pid(self, axis, pid_type, value):
        # Update PID value
        value = float(value)
        self.pid_values[axis][pid_type] = value
        self.pid_labels[axis][pid_type].set(f"{value:.2f}")
        
        # Send updated PID to ESP32
        self.send_pid_command(axis)
        
    def send_pid_command(self, axis):
        """Format and send a PID command to the ESP32"""
        if not self.uart_connected:
            # self.log(f"Not connected, would send: SET_PID,{axis},{pid_type},{value:.2f}")
            return False
            
        # Format: "SET_PID,axis,type,value"
        command = f"/pid/{axis}/{self.pid_values[axis]['P']:.2f},{self.pid_values[axis]['I']:.2f},{self.pid_values[axis]['D']:.2f}"
        self.Comms.send_command(command)
        self.command_queue.put(command)
        self.log(f"Sending command: {command}")
        return True
        
    def send_all_pids(self):
        """Send all current PID values"""

            
        self.log("Sending all PID values to ESP32...")
        for axis in self.pid_values:
            self.send_pid_command(axis)
                
    def process_imu_data(self, data_line):
        """Process a line of data received from the ESP32"""
        try:
            # Split the line by commas
            parts = data_line.split(',')
            
            # Check message type
            if len(parts) >= 6 and parts[0] == "IMU":
                # Assuming format: "IMU,roll,pitch,yaw,thrust,timestamp"
                imu_data = {
                    "roll": float(parts[1]),
                    "pitch": float(parts[2]),
                    "yaw": float(parts[3]),
                    "thrust": float(parts[4]),
                    "timestamp": float(parts[5])
                }
                
                # Add to queue for UI thread to process
                self.imu_data_queue.put(imu_data)
                return True
                
            elif parts[0] == "PID" and len(parts) >= 4:
                # Handle PID value responses: "PID,axis,type,value"
                axis = parts[1]
                pid_type = parts[2]
                value = float(parts[3])
                
                # Update stored PID value and slider
                if axis in self.pid_values and pid_type in self.pid_values[axis]:
                    self.pid_values[axis][pid_type] = value
                    
                    # Schedule UI update on main thread
                    self.master.after(0, lambda: self.update_pid_ui(axis, pid_type, value))
                    
                self.log(f"Received PID value: {axis} {pid_type} = {value}")
                return True
                
            elif parts[0] == "ACK":
                # Handle acknowledgments from ESP32
                self.log(f"ESP32 acknowledged: {data_line}")
                return True
                
            elif parts[0] == "ERROR":
                # Handle error messages from ESP32
                self.log(f"ESP32 error: {data_line}")
                return True
                
            else:
                # Unknown data format
                self.log(f"Unknown data format: {data_line}")
                return False
                
        except (ValueError, IndexError) as e:
            self.log(f"Error parsing data '{data_line}': {e}")
            return False
            
    def update_pid_ui(self, axis, pid_type, value):
        """Update UI elements with PID value (called from main thread)"""
        # Update internal storage
        self.pid_values[axis][pid_type] = value
        
        # Update displayed value
        self.pid_labels[axis][pid_type].set(f"{value:.2f}")
        
        # Update slider (without triggering the callback)
        slider = self.pid_inputs[axis][pid_type]
        slider.set(value)
            
    def update_plots(self):
        if self.uart_connected:
            # Process any available IMU data
            while not self.Comms.rx_queue.empty():
                try:
                    imu_data = self.Comms.rx_queue.get_nowait()
                    
                    # Update data arrays (shift old data left)
                    self.roll_data = np.roll(self.roll_data, -1)
                    self.pitch_data = np.roll(self.pitch_data, -1)
                    self.yaw_data = np.roll(self.yaw_data, -1)
                    self.thrust_data = np.roll(self.thrust_data, -1)
                    
                    # Add new data points
                    self.roll_data[-1] = imu_data["roll"]
                    self.pitch_data[-1] = imu_data["pitch"]
                    self.yaw_data[-1] = imu_data["yaw"]
                    self.thrust_data[-1] = imu_data["thrust"]

                    self.log()
                    
                    self.Comms.rx_queue.task_done()
                except queue.Empty:
                    break
                    
            # Update plot lines
            self.lines[0].set_ydata(self.roll_data)
            self.lines[1].set_ydata(self.pitch_data)
            self.lines[2].set_ydata(self.yaw_data)
            self.lines[3].set_ydata(self.thrust_data)
            
            # Adjust y-axis limits if needed
            for i, ax in enumerate(self.axes):
                data = [self.roll_data, self.pitch_data, self.yaw_data, self.thrust_data][i]
                if len(data) > 0:  # Ensure there's data to analyze
                    ymin, ymax = min(data), max(data)
                    margin = max(1, (ymax - ymin) * 0.1)  # At least 1 unit margin, or 10% of range
                    ax.set_ylim(ymin - margin, ymax + margin)
            
            # Redraw the canvas
            self.canvas.draw()
            
        # Schedule the next update
        self.master.after(100, self.update_plots)
        
    def on_close(self):
        """Clean up when the application is closed"""
        self.running = False
        self.close_uart()
        
        # Restore original stdout
        sys.stdout = self.original_stdout
        
        if self.uart_thread.is_alive():
            self.uart_thread.join(timeout=1.0)
            
        self.master.destroy()

def main():
    root = tk.Tk()
    app = DronePIDTuner(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    main()