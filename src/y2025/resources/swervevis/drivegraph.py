from networktables import NetworkTables
import matplotlib.pyplot as plt
from collections import deque

from time import time

NetworkTables.initialize(server='127.0.0.1')
table = NetworkTables.getTable("SmartDashboard")

plt.ion()
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
fig.canvas.manager.set_window_title("Drive Motor Velocities (Real-Time)")
axs = axs.flatten()

titles = ["FL", "FR", "BL", "BR"]
lines = {}
for i, loc in enumerate(["FL", "FR", "BL", "BR"]):
    axs[i].set_title(f"{titles[i]} Drive Motor Velocity")
    axs[i].set_xlabel("Time (s)")
    axs[i].set_ylabel("Velocity (fps)")
    axs[i].set_xlim(0, 4)
    axs[i].set_ylim(-18, 18)
    lines[loc] = axs[i].plot([], [], label=loc)[0]
    axs[i].legend(loc="upper right")

value_fig, value_ax = plt.subplots(figsize=(6, 2))
value_fig.canvas.manager.set_window_title("Current Drive Motor Velocities")
value_ax.axis("off") 
value_text = value_ax.text(0.5, 0.5, "", ha="center", va="center", fontsize=12)

max_points = 10
data = {loc: deque(maxlen=max_points) for loc in ["FL", "FR", "BL", "BR"]}
timestamps = deque(maxlen=max_points)

start_time = time()
while True:
    timestamps.append(round(time() - start_time, 2) )

    current_values = {}
    for loc_index, loc in enumerate(["FL", "FR", "BL", "BR"]):
        vel = table.getNumber(f"SwerveDrivetrain/{loc}/readings/drive_motor_vel (fps)", 0.0)
        data[loc].append(vel)
        current_values[loc] = vel

        lines[loc].set_xdata(list(timestamps))
        lines[loc].set_ydata(data[loc])
        axs[loc_index].set_xlim(max(0, timestamps[0]), timestamps[-1])

    value_text.set_text("\n".join([f"{loc}: {current_values[loc]:.2f} fps" for loc in ["FL", "FR", "BL", "BR"]]))

    fig.canvas.draw()
    fig.canvas.flush_events()
    value_fig.canvas.draw()
    value_fig.canvas.flush_events()
