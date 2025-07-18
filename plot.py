import matplotlib.pyplot as plt

# Path to your data file
filename = "log.txt"

# Storage lists
timestamps = []
x_vals, y_vals, z_vals = [], [], []
roll_vals, pitch_vals, yaw_vals = [], [], []

with open(filename, 'r') as f:
    for line in f:
        if line.startswith("S:"):
            try:
                # Remove 'S:' prefix
                _, timestamp_str, rest = line.strip().split(':', 2)
                timestamp = int(timestamp_str)
                x, y, z, roll, pitch, yaw = map(float, rest.split(','))

                # Store parsed values
                timestamps.append(timestamp / 1e6)  # convert μs to seconds
                x_vals.append(x)
                y_vals.append(y)
                z_vals.append(z)
                roll_vals.append(roll)
                pitch_vals.append(pitch)
                yaw_vals.append(yaw)

            except ValueError:
                print(f"Malformed line: {line.strip()}")

# Plotting
plt.figure(figsize=(12, 8))

plt.subplot(2, 1, 1)
plt.plot(timestamps, x_vals, label='X')
plt.plot(timestamps, y_vals, label='Y')
plt.plot(timestamps, z_vals, label='Z')
plt.ylabel("Position (m)")
plt.legend()
plt.title("Pose Data vs Time")

plt.subplot(2, 1, 2)
plt.plot(timestamps, roll_vals, label='Roll')
plt.plot(timestamps, pitch_vals, label='Pitch')
# plt.plot(timestamps, yaw_vals, label='Yaw')
plt.xlabel("Time (s)")
plt.ylabel("Orientation (deg)")
plt.legend()

plt.tight_layout()
plt.show()
