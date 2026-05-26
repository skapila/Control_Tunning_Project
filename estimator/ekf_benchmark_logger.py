import matplotlib.pyplot as plt
import numpy as np
from collections import deque

# Settings
max_len = 300  # max data points on graph

# Buffers for position
x1, y1, z1 = deque(maxlen=max_len), deque(maxlen=max_len), deque(maxlen=max_len)  # Your EKF
x2, y2, z2 = deque(maxlen=max_len), deque(maxlen=max_len), deque(maxlen=max_len)  # ArduPilot

# Plot setup
plt.ion()
fig, axs = plt.subplots(3, 1, figsize=(10, 8))
lines = []

for ax in axs:
    line1, = ax.plot([], [], label='Your EKF')
    line2, = ax.plot([], [], label='ArduPilot')
    lines.append((line1, line2))
    ax.legend()
    ax.grid(True)

axs[0].set_title("X axis")
axs[1].set_title("Y axis")
axs[2].set_title("Z axis")

def update_plot():
    for i, (data_y1, data_y2, ax) in enumerate(zip([x1, y1, z1], [x2, y2, z2], axs)):
        x = np.arange(len(data_y1))
        lines[i][0].set_data(x, data_y1)
        lines[i][1].set_data(x, data_y2)
        ax.relim()
        ax.autoscale_view()

    plt.pause(0.001)

# Example usage in your loop:
# Replace with your actual data retrieval inside main loop
def simulate_live_comparison(full_ekf_state, ardupilot_ekf_state):
    x1.append(full_ekf_state[0])
    y1.append(full_ekf_state[1])
    z1.append(full_ekf_state[2])

    x2.append(ardupilot_ekf_state[0])
    y2.append(ardupilot_ekf_state[1])
    z2.append(ardupilot_ekf_state[2])

    update_plot()
