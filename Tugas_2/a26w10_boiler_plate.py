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


# %%
# list penyimpan data plot
t_data = []
wr_data = []
wl_data = []
vx_data = []
wx_data = []

x_pose = []
y_pose = []


try:
    # 4. Main Loop 
    start_time = time.time()
    prev_time = time.time()
    while (time.time() - start_time) < 30:
        
        # --- STUDENT CODE GOES HERE ---
        # Example: Print elapsed time
        elapsed = time.time() - start_time
        print(f"Running... {elapsed:.1f}s", end="\r")

        # time difference
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        
        wr_vel = sim.getJointTargetVelocity(p3dx_RW)
        wl_vel = sim.getJointTargetVelocity(p3dx_LW)

        vx = (wr_vel + wl_vel) * rw / 2
        wx = (wr_vel - wl_vel) * rw / rb

        # Get Orientation
        euler_angle = sim.getObjectOrientation(p3dx, sim.handle_world)[2]

        # Odometry Space Velocity
        x_dot =  vx*math.cos(euler_angle) # euler_angle[2] (punya sensor absolut) atau gamma_int
        y_dot =  vx*math.sin(euler_angle)

        #Integrate Pose
        x_pose_int = x_pose_int + x_dot * dt
        y_pose_int = y_pose_int + y_dot * dt
        gamma_int = gamma_int + wx * dt


        # sim.addLog(1, f"RW:{wr_vel:.1f}, LW:{wl_vel:.1f}:")
        sim.addLog(1, f"Vx:{vx:.1f}m/s, Wx:{wx:.1f}rad/s")
        sim.addLog(1, f"x_dot:{x_dot:.1f}m/s, y_dot:{y_dot:.1f}m/s, gamma_int:{gamma_int:.1f}, x_int:{x_pose_int:.1f}m,  y_int:{y_pose_int:.1f}m, dt:{dt:.1f}")

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
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")

# # %%
# # 6. Plot hasil assignment

# fig, ax = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# # subplot atas: joint velocity
# ax[0].plot(t_data, wr_data, label=r'$\dot{\phi}_R$ (rad/s)')
# ax[0].plot(t_data, wl_data, label=r'$\dot{\phi}_L$ (rad/s)')
# ax[0].set_ylabel('Joint Velocity (rad/s)')
# ax[0].set_title('Temporal Plot of P3DX Joint Velocity')
# ax[0].grid(True)
# ax[0].legend()

# # subplot bawah: body velocity
# ax[1].plot(t_data, vx_data, label=r'$V_x$ (m/s)')
# ax[1].plot(t_data, wx_data, label=r'$\omega$ (rad/s)')
# ax[1].set_xlabel('Time (s)')
# ax[1].set_ylabel('Body Velocity')
# ax[1].set_title('Temporal Plot of P3DX Body Velocity')
# ax[1].grid(True)
# ax[1].legend()

# plt.tight_layout()
# plt.show()


# Odometry

plt.figure()
plt.plot(x_pose,y_pose)
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Robot Path")
plt.grid(True)
plt.show()