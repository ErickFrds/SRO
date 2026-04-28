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

pt1 = sim.getObject("/Sphere[0]")
pt2 = sim.getObject("/Sphere[1]")
pt3 = sim.getObject("/Sphere[2]")

try:
    # 4. Main Loop (Run for 10 seconds)
    start_time = time.time()
    elapsed_prev = 0.0
    while (time.time() - start_time) < 60*10:
        
        # Get Euler angles of Cuboid[0] and Cuboid[1]
        euler_cube1 = sim.getObjectOrientation(cb1, sim.handle_world)
        euler_cube2 = sim.getObjectOrientation(cb2, sim.handle_world)

        # # Get quaternions (x, y, z, w) of cb1 and cb2 relative to world
        # quat_cube1 = sim.getObjectQuaternion(cb1, sim.handle_world)
        # quat_cube2 = sim.getObjectQuaternion(cb2, sim.handle_world)

        # Convert Euler angles to YPR rotation matrices
        def rotation_matrix(euler_angles):
            roll, pitch, yaw = euler_angles
            # X-axis rotation (roll)
            R_x = np.array([[1, 0, 0],
                             [0, np.cos(roll), -np.sin(roll)],
                             [0, np.sin(roll), np.cos(roll)]])
            
            # Y-axis rotation (pitch)
            R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                             [0, 1, 0],
                             [-np.sin(pitch), 0, np.cos(pitch)]])
            
            # Z-axis rotation (yaw)
            R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                             [np.sin(yaw), np.cos(yaw), 0],
                             [0, 0, 1]])
            
            # Combine rotations: R = R_z * R_y * R_x
            return R_x @ R_y @ R_z

         # Convert Euler angles to quaternions (CoppeliaSim X-Y-Z Intrinsic Convention)
        def euler_to_quat_coppelia(euler):
            alpha, beta, gamma = euler
            
            # Setengah sudut
            cx = math.cos(alpha * 0.5)
            sx = math.sin(alpha * 0.5)
            cy = math.cos(beta * 0.5)
            sy = math.sin(beta * 0.5)
            cz = math.cos(gamma * 0.5)
            sz = math.sin(gamma * 0.5)

            # Perhitungan Quaternion X-Y-Z
            w = cx * cy * cz - sx * sy * sz
            x = sx * cy * cz + cx * sy * sz
            y = cx * sy * cz - sx * cy * sz
            z = cx * cy * sz + sx * sy * cz
            
            return np.array([x, y, z, w])

        # def quat_to_rot(q):
        #     x, y, z, w = q
        #     # Normalisasi (penting!)
        #     q = q / np.linalg.norm(q)
        #     x, y, z, w = q
        #     return np.array([
        #         [1-2*(y**2 + z**2),   2*(x*y - z*w),     2*(x*z + y*w)],
        #         [2*(x*y + z*w),       1-2*(x**2 + z**2), 2*(y*z - x*w)],
        #         [2*(x*z - y*w),       2*(y*z + x*w),     1-2*(x**2 + y**2)]
        #     ])

        # Compute full rotation matrices
        # R_cube1 = rotation_matrix(euler_cube1)
        # R_cube2 = rotation_matrix(euler_cube2)

        # R_cube1 = quat_to_rot(quat_cube1)
        # R_cube2 = quat_to_rot(quat_cube2)

        R_cube1 = euler_to_quat_coppelia(euler_cube1)
        R_cube2 = euler_to_quat_coppelia(euler_cube2)


        # Print matrices
        print("Rotation matrix for Cube 1 (RPY):\n", R_cube1)
        print("Rotation matrix for Cube 2 (RPY):\n", R_cube2)
        
        # Compute rotation matrix of Cube 2 relative to Cube 1
        R_rel = R_cube1.T @ R_cube2  # since R^{-1} = R^T for rotation matrices @(TRANSPOSE)
        
        sim.setObjectPosition(pt1, R_rel[:, 0].tolist(), cb1)
        sim.setObjectPosition(pt2, R_rel[:, 1].tolist(), cb1)
        sim.setObjectPosition(pt3, R_rel[:, 2].tolist(), cb1)
        # time.sleep(1)

finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")