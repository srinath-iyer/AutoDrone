"random change delete later"

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

class DronePIDTuner:
    def __init__(self, master):
        self.master = master
        self.master.title("Drone PID Tuner")
        self.master.geometry("1200x800")
        self.master.config(bg="#f0f0f0")
        
        # UART configuration
        self.uart_port = "COM3"  # Change to appropriate port
        self.uart_baudrate = 115200
        self.uart_connected = False
        self.serial_conn = None
        
        # Operation mode
        self.simulation_mode = True  # Set to False to use real UART
        
        # Data storage
        self.time_data = np.linspace(0, 10, 100)  # Last 10 seconds with 100 data points
        self.roll_data = np.zeros(100)
        self.pitch_data = np.zeros(100)
        self.yaw_data = np.zeros(100)
        self.thrust_data = np.zeros(100)
        
        # Create queues for thread-safe communication
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
        
        # Redirect print statements to our custom log
        self.original_stdout = sys.stdout
        sys.stdout = self
        
        # Start UART communication thread
        self.running = True
        self.uart_thread = threading.Thread(target=self.uart_communication_loop)
        self.uart_thread.daemon = True
        self.uart_thread.start()
        
        # Start plotting update loop
        self.master.after(100, self.update_plots)
        
        # Start log update loop
        self.master.after(100, self.update_logs)
        
    def write(self, text):
        """Implement write method for print redirection"""
        if text.strip():  # Ignore empty lines
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            log_entry = f"[{timestamp}] {text}"
            self.log_queue.put(log_entry)
            # Also write to original stdout for console debugging
            self.original_stdout.write(text)
            
    def flush(self):
        """Required for stdout redirection"""
        self.original_stdout.flush()
        
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
        log_frame = ttk.LabelFrame(right_frame, text="System Logs")
        log_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=False, padx=0, pady=5)
        
        # Create PID controls for each axis
        self.pid_sliders = {}
        self.pid_labels = {}
        
        for i, axis in enumerate(["roll", "pitch", "yaw", "thrust"]):
            axis_frame = ttk.LabelFrame(left_frame, text=f"{axis.capitalize()} PID")
            axis_frame.grid(row=i, column=0, padx=10, pady=5, sticky="ew")
            
            self.pid_sliders[axis] = {}
            self.pid_labels[axis] = {}
            
            # P, I, D sliders for each axis
            for j, pid_type in enumerate(["P", "I", "D"]):
                ttk.Label(axis_frame, text=f"{pid_type}:").grid(row=j, column=0, padx=5, pady=2)
                
                # Value label
                value_var = tk.StringVar(value=f"{self.pid_values[axis][pid_type]:.2f}")
                self.pid_labels[axis][pid_type] = value_var
                ttk.Label(axis_frame, textvariable=value_var, width=5).grid(row=j, column=1, padx=5, pady=2)
                
                # Slider
                slider = ttk.Scale(
                    axis_frame, 
                    from_=0.0, 
                    to=5.0, 
                    value=self.pid_values[axis][pid_type],
                    orient=tk.HORIZONTAL,
                    length=200,
                    command=lambda value, a=axis, p=pid_type: self.update_pid(a, p, value)
                )
                slider.grid(row=j, column=2, padx=5, pady=2)
                self.pid_sliders[axis][pid_type] = slider
        
        # Add connection status and control buttons
        control_frame = ttk.Frame(left_frame)
        control_frame.grid(row=len(self.pid_values), column=0, padx=10, pady=10, sticky="ew")
        
        # Simulation mode toggle
        self.sim_mode_var = tk.BooleanVar(value=self.simulation_mode)
        sim_check = ttk.Checkbutton(
            control_frame, 
            text="Simulation Mode",
            variable=self.sim_mode_var,
            command=self.toggle_simulation_mode
        )
        sim_check.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        # Connection status and button
        self.conn_status_var = tk.StringVar(value="Disconnected")
        status_label = ttk.Label(control_frame, textvariable=self.conn_status_var, foreground="red")
        status_label.grid(row=1, column=0, padx=5, pady=5)
        
        self.connect_button = ttk.Button(control_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=1, column=1, padx=5, pady=5)
        
        # Send all PIDs button
        send_all_button = ttk.Button(control_frame, text="Send All PIDs", command=self.send_all_pids)
        send_all_button.grid(row=2, column=0, columnspan=2, padx=5, pady=5)
        
        # Request PIDs button (for real hardware)
        request_button = ttk.Button(control_frame, text="Request Current PIDs", command=self.request_current_pids)
        request_button.grid(row=3, column=0, columnspan=2, padx=5, pady=5)
        
        # UART port entry
        port_frame = ttk.Frame(control_frame)
        port_frame.grid(row=4, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        ttk.Label(port_frame, text="UART Port:").grid(row=0, column=0, padx=5, pady=2)
        self.port_var = tk.StringVar(value=self.uart_port)
        port_entry = ttk.Entry(port_frame, textvariable=self.port_var, width=15)
        port_entry.grid(row=0, column=1, padx=5, pady=2)
        
        # Clear log button
        clear_log_button = ttk.Button(control_frame, text="Clear Log", command=self.clear_log)
        clear_log_button.grid(row=5, column=0, columnspan=2, padx=5, pady=5)
        
        # Create log text area
        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, width=80, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.log_text.config(state=tk.DISABLED)  # Make read-only
        
        # Create plots
        self.setup_plots(plot_frame)
        
    def clear_log(self):
        """Clear the log display"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.log("Log cleared")
        
    def log(self, message):
        """Add a message to the log queue"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_queue.put(f"[{timestamp}] {message}")
        
    def update_logs(self):
        """Update the log display with queued messages"""
        # Process any new log messages
        log_updated = False
        max_messages = 50  # Process max 50 messages at once to prevent UI freeze
        
        for _ in range(max_messages):
            try:
                log_message = self.log_queue.get_nowait()
                
                # Update the text widget
                self.log_text.config(state=tk.NORMAL)
                self.log_text.insert(tk.END, log_message + "\n")
                self.log_text.see(tk.END)  # Scroll to bottom
                self.log_text.config(state=tk.DISABLED)
                
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
        
    def toggle_simulation_mode(self):
        """Toggle between simulation and real hardware mode"""
        self.simulation_mode = self.sim_mode_var.get()
        
        # If switching to real mode, disconnect first
        if not self.simulation_mode and self.uart_connected:
            self.close_uart()
            
        self.log(f"Simulation mode: {'ON' if self.simulation_mode else 'OFF'}")
        
    def update_pid(self, axis, pid_type, value):
        # Update PID value
        value = float(value)
        self.pid_values[axis][pid_type] = value
        self.pid_labels[axis][pid_type].set(f"{value:.2f}")
        
        # Send updated PID to ESP32
        self.send_pid_command(axis, pid_type, value)
        
    def send_pid_command(self, axis, pid_type, value):
        """Format and send a PID command to the ESP32"""
        if not self.uart_connected:
            self.log(f"Not connected, would send: SET_PID,{axis},{pid_type},{value:.2f}")
            return False
            
        # Format: "SET_PID,axis,type,value"
        command = f"SET_PID,{axis},{pid_type},{value:.2f}"
        self.command_queue.put(command)
        self.log(f"Sending command: {command}")
        return True
        
    def send_all_pids(self):
        """Send all current PID values"""
        if not self.uart_connected:
            self.log("Not connected to ESP32")
            return
            
        self.log("Sending all PID values to ESP32...")
        for axis in self.pid_values:
            for pid_type in self.pid_values[axis]:
                value = self.pid_values[axis][pid_type]
                self.send_pid_command(axis, pid_type, value)
                
    def request_current_pids(self):
        """Request current PID values from ESP32"""
        if not self.uart_connected:
            self.log("Not connected to ESP32")
            return False
            
        command = "GET_PIDS"
        self.command_queue.put(command)
        self.log(f"Requesting current PIDs from ESP32")
        return True
        
    def setup_uart(self):
        """Initialize the UART connection to the ESP32"""
        # If in simulation mode, just pretend we're connected
        if self.simulation_mode:
            self.uart_connected = True
            self.conn_status_var.set("Connected (Simulation)")
            self.connect_button.config(text="Disconnect")
            self.log("Connected in simulation mode")
            return True
            
        # Otherwise, try to establish a real connection
        try:
            self.uart_port = self.port_var.get()  # Get current value from entry field
            self.serial_conn = serial.Serial(
                port=self.uart_port,
                baudrate=self.uart_baudrate,
                timeout=0.1,  # Short timeout for non-blocking reads
                write_timeout=0.5
            )
            self.uart_connected = True
            self.conn_status_var.set(f"Connected to {self.uart_port}")
            self.connect_button.config(text="Disconnect")
            self.log(f"Connected to ESP32 on {self.uart_port}")
            return True
        except serial.SerialException as e:
            error_msg = f"Error connecting to {self.uart_port}: {e}"
            self.log(error_msg)
            self.conn_status_var.set(f"Error: {str(e)[:20]}...")
            return False
            
    def close_uart(self):
        """Close the UART connection"""
        if not self.simulation_mode and self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            
        self.uart_connected = False
        self.conn_status_var.set("Disconnected")
        self.connect_button.config(text="Connect")
        self.log("Disconnected from ESP32")
        
    def toggle_connection(self):
        """Toggle the UART connection state"""
        if self.uart_connected:
            self.close_uart()
        else:
            self.setup_uart()
            
    def uart_communication_loop(self):
        """UART communication thread - handles both simulation and real mode"""
        buffer = ""  # For real hardware mode
        
        while self.running:
            # Check if connected
            if not self.uart_connected:
                time.sleep(0.1)
                continue
                
            # === SIMULATION MODE ===
            if self.simulation_mode:
                # Generate random IMU data for simulation
                roll = 10 * np.sin(time.time() * 0.5) + random.uniform(-2, 2)
                pitch = 10 * np.cos(time.time() * 0.7) + random.uniform(-2, 2)
                yaw = 5 * np.sin(time.time() * 0.3) + random.uniform(-1, 1)
                thrust = 50 + 10 * np.sin(time.time() * 0.2) + random.uniform(-5, 5)
                
                # Put the data in the queue
                imu_data = {
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                    "thrust": thrust,
                    "timestamp": time.time()
                }
                self.imu_data_queue.put(imu_data)
                
                # Check for commands to send to simulated ESP32
                try:
                    command = self.command_queue.get_nowait()
                    self.log(f"[SIM] Sending to ESP32: {command}")
                    
                    # Simulate acknowledging the command
                    if command.startswith("SET_PID"):
                        self.log(f"[SIM] ESP32 acknowledged: ACK,{command}")
                    elif command == "GET_PIDS":
                        # Simulate ESP32 sending back current PIDs
                        for axis in self.pid_values:
                            for pid_type in self.pid_values[axis]:
                                value = self.pid_values[axis][pid_type]
                                self.log(f"[SIM] ESP32 sent: PID,{axis},{pid_type},{value:.2f}")
                                
                    self.command_queue.task_done()
                except queue.Empty:
                    pass
                    
                time.sleep(0.1)  # Simulate 10Hz data rate
                
            # === REAL HARDWARE MODE ===
            else:
                try:
                    # --- READ DATA FROM ESP32 ---
                    if self.serial_conn.in_waiting > 0:
                        new_data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='replace')
                        buffer += new_data
                        
                        # Process complete lines
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            self.process_imu_data(line.strip())
                            
                    # --- SEND COMMANDS TO ESP32 ---
                    # Check for commands to send
                    if not self.command_queue.empty():
                        command = self.command_queue.get_nowait()
                        self.log(f"Sending to ESP32: {command}")
                        
                        # Add newline terminator to command
                        command_bytes = (command + '\n').encode('utf-8')
                        bytes_written = self.serial_conn.write(command_bytes)
                        self.serial_conn.flush()  # Ensure data is sent immediately
                        
                        self.log(f"Sent {bytes_written} bytes")
                        self.command_queue.task_done()
                        
                except serial.SerialException as e:
                    self.log(f"Error with serial port: {e}")
                    self.close_uart()
                    time.sleep(1)  # Wait before retrying
                except Exception as e:
                    self.log(f"Unexpected error in UART thread: {e}")
                    
                time.sleep(0.01)  # Small delay to prevent CPU hogging
                
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
        slider = self.pid_sliders[axis][pid_type]
        slider.set(value)
            
    def update_plots(self):
        if self.uart_connected:
            # Process any available IMU data
            while not self.imu_data_queue.empty():
                try:
                    imu_data = self.imu_data_queue.get_nowait()
                    
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
                    
                    self.imu_data_queue.task_done()
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