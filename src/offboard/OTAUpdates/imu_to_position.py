import numpy as np
import matplotlib.pyplot as plt

# Accel bias to subtract
accel_bias = np.array([0.12178205, 0.453024, 10.15124524])

# Load data from file
def load_data(filename):
    timestamps = []
    accels = []

    with open(filename, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) < 4:
                continue
            timestamp = int(parts[0])
            accel = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
            timestamps.append(timestamp)
            accels.append(accel)

    return np.array(timestamps), np.array(accels)

# Unbias, integrate to velocity and then to position
def compute_position(timestamps, accels):
    # Convert timestamps from microseconds to seconds
    timestamps = timestamps / 1e6
    dt = np.diff(timestamps)

    # Unbias acceleration
    accels_unbiased = accels - accel_bias

    # Initialize velocity and position arrays
    velocities = [np.zeros(3)]
    positions = [np.zeros(3)]

    for i in range(1, len(accels_unbiased)):
        # Integrate acceleration to get velocity
        v = 0.5*velocities[-1] + 0.5*accels_unbiased[i-1] * dt[i-1]
        velocities.append(v)

        # Integrate velocity to get position
        p = 0.5*positions[-1] + 0.5*velocities[-1] * dt[i-1]
        positions.append(p)
    print(dt)
    return timestamps, np.array(positions)

# Plot X, Y, Z position vs time in subplots
def plot_subplots(timestamps, positions):
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(timestamps, positions[:, 0], color='r')
    axs[0].set_ylabel('X Position')
    axs[0].set_title('Position vs Time')

    axs[1].plot(timestamps, positions[:, 1], color='g')
    axs[1].set_ylabel('Y Position')

    axs[2].plot(timestamps, positions[:, 2], color='b')
    axs[2].set_ylabel('Z Position')
    axs[2].set_xlabel('Time (s)')

    plt.tight_layout()
    plt.show()

# Main function
def main():
    filename = 'src\offboard\OTAUpdates\data.txt'  # Replace with your actual file path
    timestamps, accels = load_data(filename)
    timestamps, positions = compute_position(timestamps, accels)
    plot_subplots(timestamps, positions)

if __name__ == '__main__':
    main()
