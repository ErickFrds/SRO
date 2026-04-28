#------------------------------#
# Author: Erick Faisal Firdaus
# NRP   : 5022231180
#------------------------------#

# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# %%
# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")

# %%
# 3. Simple Test: Post a message to CoppeliaSim status bar
sim.addLog(1, "Hello from Python!")
p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")
p3dx = sim.getObject('/PioneerP3DX')

rw = 0.195/2
rb = 0.381/2
d = 0.05

dt = 0.001
x_pose_int = 0.0
y_pose_int = 0.0
gamma_int = 0.0

sphere = sim.getObject('/Sphere')

# %%
# list penyimpan data plot
t_data = []
wr_data = []
wl_data = []
vx_data = []
wx_data = []

x_pose = []
y_pose = []

# %%
# parameter kontrol
Kv = 0.5
Kw = 0.8
goal_tolerance = 0.08

# batas kecepatan
vx_max = 0.7
wx_max = 2.5

def clamp(val, val_min, val_max):
    return max(val_min, min(val, val_max))

try:
    # 4. Main Loop
    start_time = time.time()
    prev_time = time.time()
    while (time.time() - start_time) < 90:

        # --- STUDENT CODE GOES HERE ---
        elapsed = time.time() - start_time
        print(f"Running... {elapsed:.1f}s", end="\r")

        # time difference
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        # Get Orientation
        euler_angle = sim.getObjectOrientation(p3dx, sim.handle_world)[2]

        # Get Pose of p3dx
        p3dx_position = sim.getObjectPosition(p3dx, sim.handle_world)
        p3dx_orientation = sim.getObjectOrientation(p3dx, sim.handle_world)

        # Get Pose of Sphere
        sphere_position = sim.getObjectPosition(sphere, sim.handle_world)

        # Transformation matrix robot w.r.t world
        T_world_robot = np.array([
            [math.cos(p3dx_orientation[2]), -math.sin(p3dx_orientation[2]), p3dx_position[0]],
            [math.sin(p3dx_orientation[2]),  math.cos(p3dx_orientation[2]), p3dx_position[1]],
            [0,                             0,                              1]
        ])

        # Represent Sphere wrt robot
        P_world_sphere = np.array([
            [sphere_position[0]],
            [sphere_position[1]],
            [1]
        ])

        sphere_pos_robot = np.linalg.inv(T_world_robot) @ P_world_sphere

        x_r = sphere_pos_robot[0][0]
        y_r = sphere_pos_robot[1][0]

        # jarak ke target
        rho = math.sqrt(x_r**2 + y_r**2)

        # kontrol
        if rho < goal_tolerance:
            vx = 0.0
            wx = 0.0
        else:
            vx = Kv * x_r
            wx = Kw * math.atan2(y_r, x_r)

        # saturasi kecepatan
        vx = clamp(vx, -vx_max, vx_max)
        wx = clamp(wx, -wx_max, wx_max)

        # inverse kinematics differential drive
        wr_vel = (vx + rb * wx) / rw
        wl_vel = (vx - rb * wx) / rw

        # set wheel velocity
        sim.setJointTargetVelocity(p3dx_RW, wr_vel)
        sim.setJointTargetVelocity(p3dx_LW, wl_vel)

        # W_P_S  → sphere position in world             (x, y)
        # W_P_R  → robot pose in world                  (x, y, theta)
        # R_P_S  → sphere position relative to robot    (x_r, y_r)

        sim.addLog(1, f"W_P_S({sphere_position[0]:.2f},{sphere_position[1]:.2f}) | W_P_R({p3dx_position[0]:.2f},{p3dx_position[1]:.2f},{p3dx_orientation[2]:.2f}) | R_P_S({x_r:.2f},{y_r:.2f})")
        # Odometry Space Velocity
        x_dot = vx * math.cos(euler_angle)
        y_dot = vx * math.sin(euler_angle)

        # Integrate Pose
        x_pose_int = x_pose_int + x_dot * dt
        y_pose_int = y_pose_int + y_dot * dt
        gamma_int = gamma_int + wx * dt

        # simpan data
        t_data.append(elapsed)
        wr_data.append(wr_vel)
        wl_data.append(wl_vel)
        vx_data.append(vx)
        wx_data.append(wx)

        x_pose.append(x_pose_int)
        y_pose.append(y_pose_int)

        time.sleep(0.1)

finally:
    # stop robot first
    sim.setJointTargetVelocity(p3dx_RW, 0)
    sim.setJointTargetVelocity(p3dx_LW, 0)

    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")