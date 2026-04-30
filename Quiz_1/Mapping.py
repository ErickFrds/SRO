#%%
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

#%%
print("Program Started")

client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(False)
sim.startSimulation()

w1_Handle = sim.getObject("/rightMotor")
w2_Handle = sim.getObject("/leftMotor")
s3_Handle = sim.getObject("/ultrasonicSensor[3]")
p3dx_Handle = sim.getObject("/PioneerP3DX")
# list position = sim.getObjectPosition(int objectHandle, 
                                    #   int relativeToObjectHandle = sim.handle_world)


def transformMat(alpha, beta, gamma, tx, ty, tz):
    # 1. Individual Rotation Matrices (3x3)
    rotx = np.array([
        [1, 0, 0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha),  math.cos(alpha)]
        ])
    roty = np.array([
        [ math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)]
        ])
    rotz = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma),  math.cos(gamma), 0],
        [0,0,1]
        ])
    # 2. Total Rotation Matrix (R_total, 3x3)
    rot_total = np.matmul(rotx, roty)
    rot_total = np.matmul(rot_total, rotz)
    # 3. Translation Vector (t, 3x1)
    trans_vector = np.array([
                    [tx],
                    [ty],
                    [tz]
                    ])
    # 4. Create the 3x4 Transformation Matrix
    R_t_3x4  = np.hstack((rot_total, trans_vector))
    # 5. Create the Homogeneous Row [0 0 0 1] (1x4)
    homogeneous_row = np.array([[0, 0, 0, 1]])
    # 6. Vertically stack to create the 4x4 Homogeneous Transformation Matrix
    transform_matrix_4x4 = np.vstack((R_t_3x4, homogeneous_row))
    return transform_matrix_4x4
    

d_xyyaw = []
d_t = []
dat_ultrasound3 = np.zeros((4, 1))

t_prv = 0.0

sim.addLog(1,"get vel start")
time.sleep(2)
start_time = time.time()
while True:
    t_now = time.time() - start_time
    if t_now > 10:
        break;

    # get pose
    bod_pos_xyz = sim.getObjectPosition(p3dx_Handle) 
    bod_pos_abg = sim.getObjectOrientation(p3dx_Handle) 

    # read sensor
    prox_res, prox_dist, prox_point, prox_obj, prox_n = sim.readProximitySensor(s3_Handle)
    if (prox_res): # prox_dist>0.1 and prox_dist< 5
        sensor_reading = np.array([
            [0],
            [0],
            [prox_dist],
            [1]
        ])
        
        sensor2body_mat = transformMat(np.deg2rad(-90), np.deg2rad(80), np.deg2rad(-180), 0.209, 0.027, 0.068)
        sensor2body_pos = np.matmul(sensor2body_mat, sensor_reading)
        body2world_mat = transformMat(0, 0, bod_pos_abg[2], bod_pos_xyz[0], bod_pos_xyz[1], 0)
        sensor2odom_pos = np.matmul(body2world_mat, sensor2body_pos) # koyoke masih body
        
        # save
        dat_ultrasound3 = np.hstack((dat_ultrasound3, sensor2body_pos)) 
        d_t.append(t_now)
    # save
    d_xyyaw.append([bod_pos_xyz[0],
                    bod_pos_xyz[1],
                    bod_pos_abg[2]])
    

    t_prv = t_now
    sim.addLog(1,f"x,y,yaw,t="
                f"{bod_pos_xyz[0]:.2f}m,{bod_pos_xyz[1]:.2f}m, "
                f"{math.degrees(bod_pos_abg[2]):.2f}deg, "
                f"{t_now:.2f}s")

sim.addLog(1,"sim com ended")

# convert to np array
dat_xyyaw=np.array(d_xyyaw)
dat_t=np.array(d_t)
dat_ultrasound3 = dat_ultrasound3[:, 1:] #remove the first zero column

dat_xyyaw[:,2] = np.atan2(np.sin(dat_xyyaw[:,2]), np.cos(dat_xyyaw[:,2]))

# %%

plt.figure(figsize=(8, 6))
# plt.plot(dat_xyyaw[:,0], dat_xyyaw[:,1], color='royalblue', linewidth=2, label='$^Ox_B$')
# plt.scatter(dat_xyyaw[0,0], dat_xyyaw[0,1], marker='o', s=100, color='red', label='Start')
# plt.scatter(dat_xyyaw[-1,0], dat_xyyaw[-1,1], marker='x', s=100, color='green', label='End')
plt.scatter(dat_ultrasound3[0,:], dat_ultrasound3[1,:], marker='.', s=100, color='black', label='Scan')
# plt.axis('equal')
plt.xlabel('$^x_B$ (m)', fontsize=12)
plt.ylabel('$^y_B$ (m)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()

# save
now = datetime.now()
filename = now.strftime("%y%m%d%H%M_Sensor_body_axis") + ".svg"
plt.savefig(filename, format='svg')
print(f"Plot saved successfully as '{filename}'")

plt.figure(figsize=(8, 6))
plt.scatter(dat_ultrasound3[0,:], dat_ultrasound3[1,:], marker='.', s=100, color='black', label='Body space reading')
plt.axis('equal')
plt.xlabel('$^x_B$ (m)', fontsize=12)
plt.ylabel('$^y_B$ (m)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()

now = datetime.now()
filename = now.strftime("%y%m%d%H%M_Sensor_body_XY") + ".svg"
plt.savefig(filename, format='svg')
print(f"Plot saved successfully as '{filename}'")

plt.figure(figsize=(8, 6))
plt.plot(dat_t, dat_ultrasound3[0,:], color='royalblue', linewidth=2, label='Body space reading')
plt.axis('equal')
plt.xlabel('$t$ (sec)', fontsize=12)
plt.ylabel('$x_B$ (m)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()

now = datetime.now()
filename = now.strftime("%y%m%d%H%M_Sensor_body_Xt") + ".svg"
plt.savefig(filename, format='svg')
print(f"Plot saved successfully as '{filename}'")

plt.figure(figsize=(8, 6))
plt.plot(dat_t, dat_ultrasound3[1,:], color='royalblue', linewidth=2, label='Body space reading')
plt.axis('equal')
plt.xlabel('$t$ (sec)', fontsize=12)
plt.ylabel('$y_B$ (m)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()

now = datetime.now()
filename = now.strftime("%y%m%d%H%M_Sensor_body_Yt") + ".svg"
plt.savefig(filename, format='svg')
print(f"Plot saved successfully as '{filename}'")
# %%
