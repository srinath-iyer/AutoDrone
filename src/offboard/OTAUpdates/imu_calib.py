import numpy as np
import matplotlib.pyplot as plt

# ===== Calibration Step =====
def calibrate_imu(file_path):
    data = np.loadtxt(file_path, delimiter=",")
    
    accel = data[:, 1:4]   # accel_x, accel_y, accel_z
    gyro = data[:, 4:7]    # gyro_roll, gyro_pitch, gyro_yaw

    # Compute bias (mean)
    accel_bias = np.mean(accel, axis=0)
    gyro_bias = np.mean(gyro, axis=0)

    # Compute noise (std deviation)
    accel_noise = np.std(accel, axis=0)
    gyro_noise = np.std(gyro, axis=0)

    print("=== IMU Calibration Results ===")
    print(f"Accel Bias: {accel_bias}")
    print(f"Gyro Bias:  {gyro_bias}")
    print(f"Accel Noise (std): {accel_noise}")
    print(f"Gyro Noise (std):  {gyro_noise}")

    # Save to dictionary for reuse
    return {
        'accel_bias': accel_bias,
        'gyro_bias': gyro_bias,
        'accel_noise': accel_noise,
        'gyro_noise': gyro_noise
    }

# ===== Data Cleaning and Visualization =====
def clean_data(file_path, calibration):
    data = np.loadtxt(file_path, delimiter=",")
    timestamps = data[:, 0]
    accel = data[:, 1:4]
    gyro = data[:, 4:7]

    # Remove biases
    accel_cleaned = accel - calibration['accel_bias']
    gyro_cleaned = gyro - calibration['gyro_bias']

    # Plot raw vs cleaned
    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(timestamps, accel, label=["X", "Y", "Z"], alpha=0.5)
    axs[0].plot(timestamps, accel_cleaned, linestyle='--')
    axs[0].set_title("Accelerometer: Raw vs Cleaned")
    axs[0].legend(["Raw X", "Raw Y", "Raw Z", "Clean X", "Clean Y", "Clean Z"])
    axs[0].set_ylabel("m/s²")

    axs[1].plot(timestamps, gyro, label=["Roll", "Pitch", "Yaw"], alpha=0.5)
    axs[1].plot(timestamps, gyro_cleaned, linestyle='--')
    axs[1].set_title("Gyroscope: Raw vs Cleaned")
    axs[1].legend(["Raw Roll", "Raw Pitch", "Raw Yaw", "Clean Roll", "Clean Pitch", "Clean Yaw"])
    axs[1].set_ylabel("rad/s")
    axs[1].set_xlabel("Time (s)")

    plt.tight_layout()
    plt.show()

    return timestamps, accel_cleaned, gyro_cleaned

calibrate_imu("src\offboard\OTAUpdates\log.txt")
clean_data("src\offboard\OTAUpdates\log.txt",calibrate_imu("src\offboard\OTAUpdates\log.txt"))