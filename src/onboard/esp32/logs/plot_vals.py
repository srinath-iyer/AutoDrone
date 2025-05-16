import matplotlib.pyplot as plt
import datetime
import re

# Change this to your actual filename
filename = 'logs\device-monitor-250515-193158.log'

# Lists to store data
times = []
x_vals = []
y_vals = []
z_vals = []

# Regular expression to parse each line
line_re = re.compile(
    r'(\d{2}:\d{2}:\d{2}\.\d{3})\s+> scX:\s*([-+]?\d*\.\d+|\d+), Y:\s*([-+]?\d*\.\d+|\d+), Z:\s*([-+]?\d*\.\d+|\d+)'
)

with open(filename, 'r') as f:
    for line in f:
        match = line_re.match(line.strip())
        if match:
            t_str, x_str, y_str, z_str = match.groups()
            # Parse time as seconds since start
            t = datetime.datetime.strptime(t_str, "%H:%M:%S.%f")
            times.append(t)
            x_vals.append(float(x_str))
            y_vals.append(float(y_str))
            z_vals.append(float(z_str))

# Convert times to seconds since start
t0 = times[0]
t_seconds = [(t - t0).total_seconds() for t in times]

# Plotting
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

axs[0].plot(t_seconds, x_vals, label='X')
axs[0].set_ylabel('X')
axs[0].legend()

axs[1].plot(t_seconds, y_vals, label='Y', color='orange')
axs[1].set_ylabel('Y')
axs[1].legend()

axs[2].plot(t_seconds, z_vals, label='Z', color='green')
axs[2].set_ylabel('Z')
axs[2].set_xlabel('Time (s)')
axs[2].legend()

plt.tight_layout()
plt.show()