# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")


# 3. Simple Test: Post a message to CoppeliaSim status bar
sim.addLog(1, "Hello from Python!")
p3dx = sim.getObject("/PioneerP3DX")
p3dx_rw = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_lw = sim.getObject("/PioneerP3DX/leftMotor")
disc = sim.getObject("/Disc")


rw = 0.195/2
rb = 0.318/2
d = 0.05

dt = 0.01
x_dot_int = 0.0
y_dot_int = 0.0
gamma_int = 0.0

x_odom = []
y_odom = []

try:
    # 4. Main Loop (Run for 10 seconds)
    start_time = time.time()
    elapsed_prev = 0.0
    while (time.time() - start_time) < 120:
        
        # --- STUDENT CODE GOES HERE ---
        # Example: Print elapsed time
        elapsed = time.time() - start_time
        print(f"Running... {elapsed:.1f}s", end="\r")

        # time difference
        dt = elapsed - elapsed_prev
        elapsed_prev = elapsed

        # Get Disc orientation
        euler_angle = sim.getObjectOrientation(disc, sim.handle_world)

        # Get Pose of p3dx
        p3dx_position = sim.getObjectPosition(p3dx, sim.handle_world)
        p3dx_orientation = sim.getObjectOrientation(p3dx, sim.handle_world)

        # Get Pose of disc
        disc_position = sim.getObjectPosition(disc, sim.handle_world)

        # Transformation matrix robot w.r.t world (Using Z angle rotation)
        T_world_robot = np.array([[math.cos(p3dx_orientation[2]), -math.sin(p3dx_orientation[2]), p3dx_position[0]],
                                  [math.sin(p3dx_orientation[2]), math.cos(p3dx_orientation[2]), p3dx_position[1]],
                                    [0, 0, 1]])
        # Represent Sphere wrt robot
        disc_pos_robot = np.linalg.inv(T_world_robot) @ np.array([[disc_position[0]], [disc_position[1]], [1]])

        # Get Error Orientation
        error_gamma = euler_angle[2] - p3dx_orientation[2] 

        error_gamma = np.atan2(math.sin(error_gamma), math.cos(error_gamma))  # Normalize angle to [-pi, pi]

        #Euclidean Distance
        rho = math.sqrt(disc_pos_robot[0,0]**2 + disc_pos_robot[1,0]**2)

        # Threshold Distance
        rho_threshold = 0.08
        shift = 0.5

        gain_distance = np.exp(-(rho - shift)/rho_threshold)
        gain_distance = min(gain_distance, 1.0)

        wo = 0.5 * error_gamma 

        # Calculate forward velocity
        vx = 0.3*disc_pos_robot[0,0]

        # Calculate angular velocity
        wx = 0.9*math.atan2(disc_pos_robot[1,0], disc_pos_robot[0,0])

        w_total = gain_distance * wo + (1 - gain_distance) * wx

        # Calculate wheel velocities
        wr_vel = (vx + (rb*w_total)/2)/rw   
        wl_vel = (vx - (rb*w_total)/2)/rw
        print(f"\n wo {wo:2f}, wx {wx:2f}, w_total {w_total:2f}, gain {gain_distance:2f}")

        # Set wheel velocity
        sim.setJointTargetVelocity(p3dx_rw, wr_vel)
        sim.setJointTargetVelocity(p3dx_lw, wl_vel)

        

finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")
