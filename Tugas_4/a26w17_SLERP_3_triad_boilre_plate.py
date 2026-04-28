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
cb1 = sim.getObject("/Cuboid[0]")
cb2 = sim.getObject("/Cuboid[1]")
cb3 = sim.getObject("/Cuboid[2]")

try:
    # 4. Main Loop (Run for 10 seconds)
    start_time = time.time()
    elapsed_prev = 0.0
    t_max = 10.0  # Total time for interpolation (in seconds)
    while (time.time() - start_time) < t_max:

        def quaternion_slerp(q1, q2, t):
            """
            Spherical Linear Interpolation antara dua quaternion q1 dan q2.
            Parameter t berada di range [0.0, 1.0].
            """
            pass

        # Convert Euler angles to quaternions (CoppeliaSim X-Y-Z Intrinsic Convention)
        def euler_to_quat_coppelia(euler):
            pass
        

        # get euler angles (alpha, beta, gamma) of cb1 and cb2
        euler_cube1 = sim.getObjectOrientation(cb1, sim.handle_world)  # returns (alpha, beta, gamma) in radians
        euler_cube2 = sim.getObjectOrientation(cb2, sim.handle_world)

        # Calculate quaternions from Euler angles
        quat_cube1_from_euler = euler_to_quat_coppelia(euler_cube1)
        quat_cube2_from_euler = euler_to_quat_coppelia(euler_cube2)

        q_start = quat_cube1_from_euler # Identitas
        q_end = quat_cube2_from_euler # Rotasi 90 derajat di sumbu Y
        t_now = time.time() - start_time
        t = t_now / t_max  # Normalisasi waktu ke dalam range [0, 1]

        q_mid = quaternion_slerp(q_start, q_end, t)

        # Set cb3 orientation using the interpolated quaternion
        sim.setObjectQuaternion(cb3, sim.handle_world, q_mid.tolist())

finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")