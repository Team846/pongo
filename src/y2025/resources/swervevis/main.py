from networktables import NetworkTables
from time import sleep
import numpy as np
import matplotlib.pyplot as plt

NetworkTables.initialize(server='127.0.0.1')
table = NetworkTables.getTable(f"SmartDashboard")

plt.ion()
fig, ax = plt.subplots()
fig.canvas.manager.set_window_title("Swerve Module Visualization")
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_aspect('equal')
ax.set_title("Swerve Module Visualization")
ax.set_xticks([])
ax.set_yticks([])

module_markers = {
    "FL": ax.quiver(1, 1, 0, 0, angles='xy', scale_units='xy', scale=1, color='r', label="FL"),
    "FR": ax.quiver(1, -1, 0, 0, angles='xy', scale_units='xy', scale=1, color='r', label="FR"),
    "BL": ax.quiver(-1, 1, 0, 0, angles='xy', scale_units='xy', scale=1, color='r', label="BL"),
    "BR": ax.quiver(-1, -1, 0, 0, angles='xy', scale_units='xy', scale=1, color='r', label="BR"),
}

target_markers = {
    "FL": ax.quiver(1, 1, 0, 0, angles='xy', scale_units='xy', scale=1, color='g', label="FL Target"),
    "FR": ax.quiver(1, -1, 0, 0, angles='xy', scale_units='xy', scale=1, color='g', label="FR Target"),
    "BL": ax.quiver(-1, 1, 0, 0, angles='xy', scale_units='xy', scale=1, color='g', label="BL Target"),
    "BR": ax.quiver(-1, -1, 0, 0, angles='xy', scale_units='xy', scale=1, color='g', label="BR Target"),
}

overall_arrow = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='b', label="Overall Direction")

loop_counter = 0
while True:
    loop_counter += 1
    total_dx = 0
    total_dy = 0

    for loc in ["FL", "FR", "BL", "BR"]:
        dir = table.getNumber(f"SwerveDrivetrain/{loc}/readings/steer_motor_pos (deg)", 0.0)
        vel = table.getNumber(f"SwerveDrivetrain/{loc}/readings/drive_motor_vel (fps)", 0.0) / 5.0

        dir = 90.0 - dir
        
        dir_rad = np.radians(dir)
        dx = vel * np.cos(dir_rad)
        dy = vel * np.sin(dir_rad)
        
        total_dx += dx
        total_dy += dy

        module_markers[loc].set_UVC(dx, dy)

        target_dir = table.getNumber(f"SwerveDrivetrain/{loc}/target/steer_dir (deg)", 0.0)
        target_vel = table.getNumber(f"SwerveDrivetrain/{loc}/target/drive_dc", 0.0) * 3.0

        target_dir = 90.0 - target_dir

        target_dir_rad = np.radians(target_dir)
        target_dx = target_vel * np.cos(target_dir_rad)
        target_dy = target_vel * np.sin(target_dir_rad)

        target_markers[loc].set_UVC(target_dx, target_dy)

        if loop_counter % 30 == 0:
            print(f"Module {loc}: Current -> Direction = {dir} degrees, Velocity = {vel} fps, dx = {dx}, dy = {dy}")
            print(f"Module {loc}: Target -> Direction = {target_dir} degrees, Velocity = {target_vel}, dx = {target_dx}, dy = {target_dy}")

    overall_arrow.set_UVC(total_dx / 4.0, total_dy / 4.0)

    fig.canvas.draw()
    fig.canvas.flush_events()
    sleep(0.05)