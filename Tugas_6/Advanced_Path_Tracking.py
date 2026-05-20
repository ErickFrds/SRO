import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")

# helper function for tranformation matrix
def transformMat(alpha, beta, gamma, tx, ty, tz):
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
        [0, 0, 1]
    ])

    rot_total = np.matmul(rotx, roty)
    rot_total = np.matmul(rot_total, rotz)

    trans_vector = np.array([[tx], [ty], [tz]])
    R_t_3x4 = np.hstack((rot_total, trans_vector))
    homogeneous_row = np.array([[0, 0, 0, 1]])

    transform_matrix_4x4 = np.vstack((R_t_3x4, homogeneous_row))
    return transform_matrix_4x4


# 3. Object handles
sim.addLog(1, "Hello from Python!")

p3dx = sim.getObject("/PioneerP3DX")
p3dx_rw = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_lw = sim.getObject("/PioneerP3DX/leftMotor")

LH_Handle = sim.getObject("/LH")
perp_Handle = sim.getObject("/Perp")

path_Handle = []
for i in range(0, 57):
    path_Handle.append(sim.getObject(f"/p[{i}]"))

# Built-in PioneerP3DX ultrasonic sensors
front_sensor = sim.getObject("/PioneerP3DX/ultrasonicSensor[4]")
left_sensor  = sim.getObject("/PioneerP3DX/ultrasonicSensor[7]")
right_sensor = sim.getObject("/PioneerP3DX/ultrasonicSensor[0]")
back_sensor  = sim.getObject("/PioneerP3DX/ultrasonicSensor[11]")

sensors = [
    ("Front", front_sensor),
    ("Left", left_sensor),
    ("Right", right_sensor),
    ("Back", back_sensor)
]

rw = 0.195 / 2
rb = 0.318 / 2
d = 0.05

dt = 0.01
x_dot_int = 0.0
y_dot_int = 0.0
gamma_int = 0.0

LH_distance = 1.0

x_odom = []
y_odom = []

map_x = []
map_y = []


def detected_point_to_world(sensor_handle, detected_point):
    sensor_position = sim.getObjectPosition(sensor_handle, sim.handle_world)
    sensor_orientation = sim.getObjectOrientation(sensor_handle, sim.handle_world)

    T_world_sensor = transformMat(
        sensor_orientation[0],
        sensor_orientation[1],
        sensor_orientation[2],
        sensor_position[0],
        sensor_position[1],
        sensor_position[2]
    )

    detected_point_h = np.array([
        [detected_point[0]],
        [detected_point[1]],
        [detected_point[2]],
        [1]
    ])

    detected_world = T_world_sensor @ detected_point_h
    return detected_world[:3, :]


try:
    start_time = time.time()
    elapsed_prev = 0.0

    while (time.time() - start_time) < 105:

        # --- STUDENT CODE GOES HERE ---
        elapsed = time.time() - start_time

        # time difference
        dt = elapsed - elapsed_prev
        elapsed_prev = elapsed

        # Get Pose of p3dx
        p3dx_position = sim.getObjectPosition(p3dx, sim.handle_world)
        p3dx_orientation = sim.getObjectOrientation(p3dx, sim.handle_world)

        x_odom.append(p3dx_position[0])
        y_odom.append(p3dx_position[1])

        # Calculate LH position wrt the world
        LH_position_to_world = transformMat(
            0,
            0,
            p3dx_orientation[2],
            p3dx_position[0],
            p3dx_position[1],
            p3dx_position[2]
        ) @ np.array([[LH_distance], [0], [0], [1]])

        # delete 4th component
        LH_position_to_world = LH_position_to_world[:3, :]

        # Get path points positions
        path_points = []
        for i in range(len(path_Handle)):
            path_point_position = sim.getObjectPosition(path_Handle[i], sim.handle_world)
            path_points.append(path_point_position)

        # Create list of A->B vectors
        vec_AB = []
        for i in range(len(path_points) - 1):
            A = np.array(path_points[i]).reshape(3, 1)
            B = np.array(path_points[i + 1]).reshape(3, 1)
            vec_AB.append(B - A)

        # Close the loop
        A = np.array(path_points[-1]).reshape(3, 1)
        B = np.array(path_points[0]).reshape(3, 1)
        vec_AB.append(B - A)

        # Create list of A->LH vectors
        vec_ALH = []
        for i in range(len(path_points)):
            A = np.array(path_points[i]).reshape(3, 1)
            vec_ALH.append(LH_position_to_world - A)

        # Project ALH on AB to find scalar projection point
        scalar_proj_points = []
        for i in range(len(vec_AB)):
            AB = vec_AB[i]
            ALH = vec_ALH[i]

            denominator = np.dot(AB.flatten(), AB.flatten())

            if denominator == 0:
                scalar_proj = 0
            else:
                scalar_proj = np.dot(ALH.flatten(), AB.flatten()) / denominator

            if scalar_proj < 0:
                scalar_proj = 0
            elif scalar_proj > 1:
                scalar_proj = 1

            A = np.array(path_points[i]).reshape(3, 1)
            scalar_proj_point = A + scalar_proj * AB
            scalar_proj_points.append(scalar_proj_point)

        # Find closest scalar projection point to LH
        closest_index = 0
        min_distance = np.linalg.norm(scalar_proj_points[0] - LH_position_to_world)

        for i in range(1, len(scalar_proj_points)):
            distance = np.linalg.norm(scalar_proj_points[i] - LH_position_to_world)

            if distance < min_distance:
                min_distance = distance
                closest_index = i

        # Desired position
        desired_position = scalar_proj_points[closest_index]

        # Transformation matrix robot w.r.t world
        T_world_robot = transformMat(
            0,
            0,
            p3dx_orientation[2],
            p3dx_position[0],
            p3dx_position[1],
            p3dx_position[2]
        )

        # Desired position wrt robot
        desired_position_wrt_robot = np.linalg.inv(T_world_robot) @ np.append(
            desired_position,
            np.array([[1]]),
            axis=0
        )

        desired_position_wrt_robot = desired_position_wrt_robot[:3, :]

        # Error calculation
        ed = math.sqrt(
            desired_position_wrt_robot[0, 0] ** 2 +
            desired_position_wrt_robot[1, 0] ** 2
        )

        eh = math.atan2(
            desired_position_wrt_robot[1, 0],
            desired_position_wrt_robot[0, 0]
        )

        # calc body speed
        vx = 0.3 * ed
        wx = 0.9 * eh

        # speed limiter
        vx = max(min(vx, 0.5), -0.5)
        wx = max(min(wx, 1.5), -1.5)

        # calc wheel speeds
        wr_vel = (vx + (rb * wx)) / rw
        wl_vel = (vx - (rb * wx)) / rw

        # Actuate wheel speeds
        sim.setJointTargetVelocity(p3dx_rw, wr_vel)
        sim.setJointTargetVelocity(p3dx_lw, wl_vel)

        # Set position of LH and Perp point
        sim.setObjectPosition(
            LH_Handle,
            sim.handle_world,
            LH_position_to_world.flatten().tolist()
        )

        sim.setObjectPosition(
            perp_Handle,
            sim.handle_world,
            desired_position.flatten().tolist()
        )

        # Mapping using 4 ultrasonic sensors
        for sensor_name, sensor in sensors:
            result, distance, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.readProximitySensor(sensor)

            if result > 0:
                detected_world = detected_point_to_world(sensor, detectedPoint)

                map_x.append(detected_world[0, 0])
                map_y.append(detected_world[1, 0])

                print(f"{sensor_name} detected: {distance:.3f} m")

        time.sleep(0.01)


finally:
    sim.setJointTargetVelocity(p3dx_rw, 0)
    sim.setJointTargetVelocity(p3dx_lw, 0)

    sim.stopSimulation()
    print("\nSimulation Stopped")


# =========================
# PLOT RESULT MAP
# =========================

plt.figure(figsize=(10,7))

# flip horizontal
map_y_flip = [-y for y in map_y]
robot_y_flip = [-y for y in y_odom]

# map points
plt.scatter(
    map_y_flip,
    map_x,
    s=10,
    label="Detected Map Points"
)

# robot path
plt.plot(
    robot_y_flip,
    x_odom,
    linewidth=2,
    label="Robot Path"
)

plt.axis("equal")

plt.xlabel("Y Position")
plt.ylabel("X Position")

plt.title("Environment Mapping using 4 Ultrasonic Sensors")

plt.legend()
plt.grid(True)

plt.show()