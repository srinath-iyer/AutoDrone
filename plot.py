import matplotlib.pyplot as plt

# Load data from log.txt
time = []
vx = []
vy = []

with open("log.txt", "r") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue  # skip empty lines
        try:
            t, v_x, v_y = map(float, line.split(","))
            time.append(t)
            vx.append(v_x)
            vy.append(v_y)
        except ValueError:
            print(f"Skipping malformed line: {line}")

# Plot
fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

axs[0].plot(time, vx, label="vx", color='blue')
axs[0].set_ylabel("vx (m/s)")
axs[0].set_title("vx over Time")
axs[0].grid(True)

axs[1].plot(time, vy, label="vy", color='green')
axs[1].set_ylabel("vy (m/s)")
axs[1].set_xlabel("Time (s)")
axs[1].set_title("vy over Time")
axs[1].grid(True)

plt.tight_layout()
plt.show()
