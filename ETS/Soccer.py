#------------------------------#
# Autonomous Robot Soccer
# 3 Pioneer P3DX Robots
# Author: Erick Faisal Firdaus
# NRP   : 5022231180
#------------------------------#

import time
import math
import random
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require('sim')

# ==============================
# Robot Parameters
# ==============================
rw = 0.195 / 2
rb = 0.381 / 2

Kv = 0.6
Kw = 1.2

vx_max = 0.8
wx_max = 2.5

target_tolerance = 0.12

# Different speed for each role
Kv_striker = 1.0
Kw_striker = 1.3
vx_max_striker = 1.2

Kv_defender = 0.5
Kw_defender = 1.0
vx_max_defender = 0.65

Kv_keeper = 0.6
Kw_keeper = 1.2
vx_max_keeper = 0.8

# ==============================
# Field Boundary
# Adjust this if needed
# ==============================
xmin, xmax = -5.5, 5.5
ymin, ymax = -3.5, 3.5

teleport_margin = 1.0
last_teleport_time = 0
teleport_cooldown = 1.0

# ==============================
# Helper Functions
# ==============================
def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def stop_robot(right_motor, left_motor):
    sim.setJointTargetVelocity(right_motor, 0)
    sim.setJointTargetVelocity(left_motor, 0)

def move_to_target(robot, right_motor, left_motor, target_pos, Kv_local, Kw_local, vx_max_local):
    robot_pos = sim.getObjectPosition(robot, sim.handle_world)
    robot_ori = sim.getObjectOrientation(robot, sim.handle_world)

    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]

    distance = math.sqrt(dx**2 + dy**2)

    target_angle = math.atan2(dy, dx)
    robot_angle = robot_ori[2]

    angle_error = target_angle - robot_angle
    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

    if distance < target_tolerance:
        vx = 0
        wx = 0
    elif abs(angle_error) > 0.25:
        vx = 0
        wx = Kw_local * angle_error
    else:
        vx = Kv_local * distance
        wx = Kw_local * angle_error

    vx = clamp(vx, 0, vx_max_local)
    wx = clamp(wx, -wx_max, wx_max)

    wr = (vx + rb * wx) / rw
    wl = (vx - rb * wx) / rw

    sim.setJointTargetVelocity(right_motor, wr)
    sim.setJointTargetVelocity(left_motor, wl)

def teleport_ball_if_outside(ball_handle, ball_pos):
    global last_teleport_time

    outside_field = (
        ball_pos[0] < xmin or ball_pos[0] > xmax or
        ball_pos[1] < ymin or ball_pos[1] > ymax
    )

    if outside_field and time.time() - last_teleport_time > teleport_cooldown:
        new_x = 0.0
        new_y = 0.0

        sim.setObjectPosition(
            ball_handle,
            sim.handle_world,
            [new_x, new_y, ball_pos[2]]
        )

        last_teleport_time = time.time()
        print(f"\nBall teleported to ({new_x:.2f}, {new_y:.2f})")

# ==============================
# Get Objects
# ==============================
robot_pemain = sim.getObject('/Robot_Pemain')
robot_lawan_01 = sim.getObject('/Robot_Lawan_01')
robot_lawan_02 = sim.getObject('/Robot_Lawan_02')

pemain_R = sim.getObject('/Robot_Pemain/rightMotor')
pemain_L = sim.getObject('/Robot_Pemain/leftMotor')

lawan01_R = sim.getObject('/Robot_Lawan_01/rightMotor')
lawan01_L = sim.getObject('/Robot_Lawan_01/leftMotor')

lawan02_R = sim.getObject('/Robot_Lawan_02/rightMotor')
lawan02_L = sim.getObject('/Robot_Lawan_02/leftMotor')

bola_merah = sim.getObject('/Bola_Merah')
gawang_kuning = sim.getObject('/Gawang_Kuning')

# ==============================
# Start Simulation
# ==============================
sim.startSimulation()
print("Simulation Started")

try:
    start_time = time.time()

    while time.time() - start_time < 60*5:  # Run for 5 minutes
        ball_pos = sim.getObjectPosition(bola_merah, sim.handle_world)
        goal_pos = sim.getObjectPosition(gawang_kuning, sim.handle_world)

        # ==============================
        # Ball teleport if out of field
        # ==============================
        teleport_ball_if_outside(bola_merah, ball_pos)

        # update ball position after possible teleport
        ball_pos = sim.getObjectPosition(bola_merah, sim.handle_world)

        # ==============================
        # 1. STRIKER: smarter attacker
        # ==============================

        # direction from ball to goal
        dx_goal = goal_pos[0] - ball_pos[0]
        dy_goal = goal_pos[1] - ball_pos[1]
        dist_goal = math.sqrt(dx_goal**2 + dy_goal**2)

        if dist_goal < 1e-6:
            dist_goal = 1e-6

        ux = dx_goal / dist_goal
        uy = dy_goal / dist_goal

        striker_pos = sim.getObjectPosition(robot_pemain, sim.handle_world)

        dist_striker_ball = math.sqrt(
            (striker_pos[0] - ball_pos[0])**2 +
            (striker_pos[1] - ball_pos[1])**2
        )

        # point behind the ball, opposite from goal
        behind_distance = 0.65

        behind_ball_target = [
            ball_pos[0] - behind_distance * ux,
            ball_pos[1] - behind_distance * uy,
            ball_pos[2]
        ]

        # point beyond the ball toward goal
        push_distance = 3.0

        push_target = [
            ball_pos[0] + push_distance * ux,
            ball_pos[1] + push_distance * uy,
            ball_pos[2]
        ]

        # check if striker is already behind the ball
        striker_to_ball_x = ball_pos[0] - striker_pos[0]
        striker_to_ball_y = ball_pos[1] - striker_pos[1]

        dot = striker_to_ball_x * ux + striker_to_ball_y * uy

        # check alignment (VERY IMPORTANT)
        robot_angle = sim.getObjectOrientation(robot_pemain, sim.handle_world)[2]
        target_angle = math.atan2(dy_goal, dx_goal)

        angle_error = target_angle - robot_angle
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        aligned = abs(angle_error) < 0.25

        # decision logic
        if dot < 0 or dist_striker_ball > 0.75:
            # not behind ball yet
            striker_target = behind_ball_targ
        elif not aligned:
            striker_target = [
                ball_pos[0] + 1.2 * ux,
                ball_pos[1] + 1.2 * uy,
                ball_pos[2]
            ]

        else:
            # aligned + close → push
            striker_target = push_target

        # ==============================
        # 2. DEFENDER: wide X + wide Y
        # ==============================

        # X: follow ball a bit, but stay defensive
        defender_x = ball_pos[0]

        # clamp so it doesn't become striker
        defender_x = clamp(
            defender_x,
            goal_pos[0] - 3.5,   # can go forward more
            goal_pos[0] - 1.0    # not too close to goal
        )

        # Y: follow ball freely
        defender_y = ball_pos[1]

        defender_y = clamp(
            defender_y,
            goal_pos[1] - 2.0,
            goal_pos[1] + 5.0
        )

        defender_target = [
            defender_x,
            defender_y,
            ball_pos[2]
        ]

        # ==============================
        # 3. GOALKEEPER
        # Stay near goal, follow ball Y
        # ==============================
        keeper_x = goal_pos[0] - 0.5

        keeper_y = clamp(
            ball_pos[1],
            goal_pos[1] - 0.2,
            goal_pos[1] + 2.0
        )

        goalkeeper_target = [
            keeper_x,
            keeper_y,
            ball_pos[2]
        ]

        # ==============================
        # Move Robots
        # ==============================
        move_to_target(robot_pemain, pemain_R, pemain_L, striker_target,
                    Kv_striker, Kw_striker, vx_max_striker)

        move_to_target(robot_lawan_01, lawan01_R, lawan01_L, defender_target,
                    Kv_defender, Kw_defender, vx_max_defender)

        move_to_target(robot_lawan_02, lawan02_R, lawan02_L, goalkeeper_target,
                    Kv_keeper, Kw_keeper, vx_max_keeper)

        print(
            f"Ball: ({ball_pos[0]:.2f}, {ball_pos[1]:.2f}) | "
            f"Goal: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})",
            end="\r"
        )

        time.sleep(0.1)

finally:
    stop_robot(pemain_R, pemain_L)
    stop_robot(lawan01_R, lawan01_L)
    stop_robot(lawan02_R, lawan02_L)

    sim.stopSimulation()
    print("\nSimulation Stopped")