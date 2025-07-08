from networktables import NetworkTables
from time import sleep, time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize

NetworkTables.initialize(server='127.0.0.1')
table = NetworkTables.getTable(f"SmartDashboard")

positions_x = []
positions_y = []
velocities = []

plt.ion()
fig, ax1 = plt.subplots(figsize=(8, 5))

ax1.set_aspect('equal')
ax1.set_ylim(0, 720)
ax1.set_xlim(0, 350)
ax1.set_title("Path Visualization")
ax1.set_xlabel("X Position (in)")
ax1.set_ylabel("Y Position (in)")
ax1.grid(True, which='both', linestyle='--', linewidth=0.5)

norm = Normalize(vmin=0, vmax=15)
sm = plt.cm.ScalarMappable(cmap='viridis', norm=norm)
sm.set_array([])
cbar = plt.colorbar(sm, ax=ax1)
cbar.set_label("Velocity (ft/s)")

t = time()

while True:
    x = table.getNumber(f"SwerveDrivetrain/estimated_pose/position_x (in)", 0.0)
    y = table.getNumber(f"SwerveDrivetrain/estimated_pose/position_y (in)", 0.0)
    v = table.getNumber(f"SwerveDrivetrain/readings/velocity_mag (fps)", 0.0)

    if positions_x and positions_y:
        dx = abs(x - positions_x[-1])
        dy = abs(y - positions_y[-1])
        if dx > 25 or dy > 25:
            positions_x.clear()
            positions_y.clear()
            velocities.clear()

    positions_x.append(x)
    positions_y.append(y)
    velocities.append(v)

    positions_x = positions_x[-250:]
    positions_y = positions_y[-250:]
    velocities = velocities[-250:]

    points = np.array([positions_x, positions_y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap='viridis', norm=norm)
    lc.set_array(np.array(velocities))
    lc.set_linewidth(2)

    ax1.clear()
    ax1.add_collection(lc)
    ax1.plot(positions_x[-1:], positions_y[-1:], 'ro', label="Terminating Position")
    ax1.set_aspect('equal')
    ax1.set_ylim(0, 350)
    ax1.set_xlim(0, 350)
    ax1.set_title("Path Visualization")
    ax1.set_xlabel("X Position (in)")
    ax1.set_ylabel("Y Position (in)")
    ax1.legend()

    plt.draw()
    plt.pause(0.01)
    # sleep(0.01)