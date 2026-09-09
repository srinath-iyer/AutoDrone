import argparse
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
from collections import deque, defaultdict
import threading
import tkinter as tk
from tkinter import filedialog, ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import os
from pathlib import Path
import time

class TopicVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("🚁 Drone Topic Visualizer")
        self.root.geometry("1400x900")
        
        # Data storage: field_name -> deque of values
        self.topics = defaultdict(lambda: deque(maxlen=5000))
        self.topic_metadata = {}  # topic_name -> num_fields
        self.max_points = 10000
        self.sample_count = 0
        self.line_buffer = ""  # Buffer for incomplete lines from streaming
        
        # Graph configuration
        self.graphs = {}  # graph_id -> config
        self.next_graph_id = 0
        self.file_handle = None
        self.reading_file = False
        self.last_file_pos = 0
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the tkinter UI"""
        # Top control panel
        control_frame = ttk.Frame(self.root)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        
        ttk.Button(control_frame, text="📂 Open/Stream Log File", command=self._open_file_dialog).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="🔍 Auto-Detect Latest Log", command=self._auto_detect_log).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="➕ Add Graph", command=self._add_graph_dialog).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="🗑️ Clear All", command=self._clear_all).pack(side=tk.LEFT, padx=5)
        
        self.status_label = ttk.Label(control_frame, text="No file loaded | 0 samples", relief=tk.SUNKEN)
        self.status_label.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        
        # Main canvas for plots
        canvas_frame = ttk.Frame(self.root)
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.fig, self.ax_main = plt.subplots(figsize=(14, 8))
        self.fig.patch.set_facecolor('#f0f0f0')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Add interactive pan/zoom toolbar similar to Plotly exploration.
        self.toolbar = NavigationToolbar2Tk(self.canvas, canvas_frame, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Topic list panel
        list_frame = ttk.LabelFrame(self.root, text="Available Topics")
        list_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)
        
        self.topic_listbox = tk.Listbox(list_frame, height=4, width=100, font=("Arial", 9))
        self.topic_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.topic_listbox.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.topic_listbox.config(yscrollcommand=scrollbar.set)
        
        # Horizontal scrollbar for long topic names
        h_scrollbar = ttk.Scrollbar(list_frame, orient=tk.HORIZONTAL, command=self.topic_listbox.xview)
        h_scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
        self.topic_listbox.config(xscrollcommand=h_scrollbar.set)
        
        # Start animation
        self.anim = FuncAnimation(self.fig, self._update_plots, interval=100, blit=False)
    
    def _open_file_dialog(self):
        """Open file picker or start streaming"""
        filename = filedialog.askopenfilename(
            title="Select drone log file",
            filetypes=[("Log files", "*.log *.txt"), ("All files", "*.*")]
        )
        
        if filename:
            self._start_streaming(filename)
    
    def _auto_detect_log(self):
        """Auto-detect the latest log file in common directories"""
        search_paths = [
            os.path.expanduser("~"),
            os.path.expanduser("~/Documents"),
            os.path.expanduser("~/Downloads"),
            "/tmp",
            ".",
        ]
        
        latest_file = None
        latest_time = 0
        
        for search_path in search_paths:
            if not os.path.exists(search_path):
                continue
            
            try:
                for file in Path(search_path).glob("*.log"):
                    mtime = os.path.getmtime(file)
                    if mtime > latest_time:
                        latest_time = mtime
                        latest_file = file
            except (PermissionError, OSError):
                continue
        
        if latest_file:
            self._start_streaming(str(latest_file))
            self.status_label.config(text=f"Streaming: {latest_file.name}")
        else:
            messagebox.showwarning("Not Found", "No log files found in common directories")
    
    def _start_streaming(self, filename):
        """Start streaming from a file"""
        try:
            if self.file_handle:
                self.file_handle.close()
            
            # Open with UTF-8 and error handling for non-ASCII bytes from ESP32
            self.file_handle = open(filename, 'r', encoding='utf-8', errors='ignore')
            self.reading_file = True
            self.last_file_pos = 0
            
            # Start file reading in background thread
            threading.Thread(target=self._read_file_thread, daemon=True).start()
            self.status_label.config(text=f"Streaming: {os.path.basename(filename)}")
        
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open file: {e}")
    
    def _read_file_thread(self):
        """Read file in background, continuously checking for new data"""
        while self.reading_file and self.file_handle:
            try:
                # Read from last known position
                self.file_handle.seek(self.last_file_pos)
                lines = self.file_handle.readlines()
                
                for line in lines:
                    # Handle incomplete lines from streaming
                    self.line_buffer += line
                    
                    # If line ends with newline, it's complete
                    if self.line_buffer.endswith('\n'):
                        self._parse_line(self.line_buffer.strip())
                        self.line_buffer = ""
                
                self.last_file_pos = self.file_handle.tell()
                
                # Small delay before checking again
                time.sleep(0.1)
            
            except Exception as e:
                print(f"Error reading file: {e}")
                time.sleep(1)
    
    def _parse_line(self, line):
        """Parse structured log: HH:MM:SS.mmm > TOPIC:val1,val2,val3"""
        if not line:
            return
        
        try:
            # Handle timestamp prefix: "HH:MM:SS.mmm > TOPIC:values"
            if ' > ' in line:
                # Extract the part after the timestamp
                _, line_content = line.split(' > ', 1)
            else:
                line_content = line
            
            # Now parse TOPIC:val1,val2,val3
            if ':' not in line_content:
                return
            
            parts = line_content.split(':', 1)
            if len(parts) != 2:
                return
            
            topic_name = parts[0].strip()
            values_str = parts[1].strip()
            
            # Skip if topic name has spaces or invalid chars
            if ' ' in topic_name or not topic_name or not topic_name[0].isalpha():
                return
            
            # Parse values
            try:
                values = [float(v.strip()) for v in values_str.split(',')]
            except ValueError:
                # Non-numeric data, skip
                return
            
            if not values:
                return
            
            # Store metadata
            if topic_name not in self.topic_metadata:
                self.topic_metadata[topic_name] = len(values)
            
            # Store each value as a field
            for i, val in enumerate(values):
                field_name = f"{topic_name}[{i}]"
                self.topics[field_name].append(val)
            
            self.sample_count += 1
            self.status_label.config(
                text=f"Loaded: {self.sample_count} samples | Topics: {len(self.topic_metadata)}"
            )
            self._update_topic_list()
        
        except Exception as e:
            # Silently skip malformed lines
            pass
    
    def _update_topic_list(self):
        """Update the topic listbox"""
        self.topic_listbox.delete(0, tk.END)
        for topic in sorted(self.topic_metadata.keys()):
            # Count samples for this topic
            sample_key = f"{topic}[0]"
            count = len(self.topics[sample_key]) if sample_key in self.topics else 0
            self.topic_listbox.insert(tk.END, f"📊 {topic} ({count} samples)")
    
    def _add_graph_dialog(self):
        """Show dialog to add a new graph"""
        dialog = tk.Toplevel(self.root)
        dialog.title("Add New Graph")
        dialog.geometry("400x350")
        dialog.resizable(False, False)
        
        ttk.Label(dialog, text="Enter topic name:", font=("Arial", 10, "bold")).pack(pady=10, padx=10)
        
        topic_entry = ttk.Entry(dialog, width=40)
        topic_entry.pack(pady=5, padx=10)
        
        ttk.Label(dialog, text="(e.g., 'IMU_ACCEL', 'BMP390_READING')", font=("Arial", 9), foreground="gray").pack(padx=10)
        
        ttk.Label(dialog, text="Fields to plot (optional):", font=("Arial", 10, "bold")).pack(pady=(15, 5), padx=10)
        
        ttk.Label(dialog, text="Leave empty to plot all. Examples: 0,1,2 or 0", font=("Arial", 9), foreground="gray").pack(padx=10)
        
        fields_entry = ttk.Entry(dialog, width=40)
        fields_entry.pack(pady=5, padx=10)
        
        ttk.Label(dialog, text="Graph title (optional):", font=("Arial", 10, "bold")).pack(pady=(15, 5), padx=10)
        
        title_entry = ttk.Entry(dialog, width=40)
        title_entry.pack(pady=5, padx=10)
        
        def add_graph():
            topic = topic_entry.get().strip()
            title = title_entry.get().strip() or topic
            fields_str = fields_entry.get().strip()
            
            if not topic:
                messagebox.showwarning("Error", "Topic name is required")
                return
            
            # Parse fields
            fields = None
            if fields_str:
                try:
                    fields = [int(f.strip()) for f in fields_str.split(',')]
                except ValueError:
                    messagebox.showerror("Error", "Invalid field numbers")
                    return
            
            self._create_graph(topic, fields, title)
            dialog.destroy()
        
        ttk.Button(dialog, text="Create Graph", command=add_graph).pack(pady=20)
    
    def _create_graph(self, topic, field_indices=None, title=None):
        """Create a new graph for a topic"""
        graph_id = self.next_graph_id
        self.next_graph_id += 1
        
        self.graphs[graph_id] = {
            'topic': topic,
            'field_indices': field_indices,
            'title': title or topic,
            'lines': {}
        }
        
        self._reorganize_plots()
    
    def _reorganize_plots(self):
        """Reorganize gridspec based on number of graphs"""
        self.fig.clear()
        
        if not self.graphs:
            self.ax_main = self.fig.add_subplot(111)
            self.ax_main.text(0.5, 0.5, 'No graphs added. Click "Add Graph" to start!',
                            ha='center', va='center', fontsize=14, fontweight='bold')
            self.ax_main.axis('off')
            return
        
        cols = min(2, len(self.graphs))
        rows = (len(self.graphs) + cols - 1) // cols
        
        gs = gridspec.GridSpec(rows, cols, figure=self.fig, hspace=0.4, wspace=0.3)
        
        for idx, (graph_id, graph_config) in enumerate(self.graphs.items()):
            ax = self.fig.add_subplot(gs[idx // cols, idx % cols])
            graph_config['ax'] = ax
            ax.set_title(graph_config['title'], fontweight='bold')
            ax.grid(True, alpha=0.3)
    
    def _update_plots(self, frame):
        """Update all plot data"""
        if not self.graphs:
            return
        
        for graph_id, graph_config in self.graphs.items():
            ax = graph_config['ax']
            ax.clear()
            
            topic = graph_config['topic']
            field_indices = graph_config['field_indices']
            
            # Collect all data for this topic
            topic_data = {}
            for field_key in self.topics.keys():
                if field_key.startswith(topic + '['):
                    try:
                        field_idx = int(field_key.split('[')[1].rstrip(']'))
                        topic_data[field_idx] = list(self.topics[field_key])
                    except (ValueError, IndexError):
                        pass
            
            if not topic_data:
                ax.text(0.5, 0.5, f'Waiting for "{topic}"...', ha='center', va='center', fontsize=10)
                ax.set_title(graph_config['title'], fontweight='bold')
                ax.axis('off')
                continue
            
            # Filter to requested fields
            if field_indices:
                topic_data = {k: v for k, v in topic_data.items() if k in field_indices}
            
            # Plot all fields
            colors = ['red', 'green', 'blue', 'orange', 'purple', 'brown', 'pink', 'cyan']
            field_labels = ['X', 'Y', 'Z', 'W', 'U', 'V', 'A', 'B']
            
            for plot_idx, (field_idx, values) in enumerate(sorted(topic_data.items())):
                if values:
                    color = colors[plot_idx % len(colors)]
                    label = field_labels[plot_idx % len(field_labels)]

                    # Keep rendering cost bounded for continuous streams.
                    plot_values = values[-self.max_points:] if len(values) > self.max_points else values
                    ax.plot(plot_values, label=label, color=color, linewidth=2.0, alpha=0.9)
            
            ax.set_title(graph_config['title'], fontweight='bold')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.set_xlabel('Sample')
            ax.set_ylabel('Value')
        
        self.fig.tight_layout()
        self.canvas.draw()
    
    def _clear_all(self):
        """Clear all data and graphs"""
        self.topics.clear()
        self.topic_metadata.clear()
        self.graphs.clear()
        self.next_graph_id = 0
        self.sample_count = 0
        self.topic_listbox.delete(0, tk.END)
        self._reorganize_plots()
        self.status_label.config(text="Cleared | 0 samples")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Drone topic visualizer")
    parser.parse_args()

    root = tk.Tk()
    app = TopicVisualizer(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("Visualizer closed")