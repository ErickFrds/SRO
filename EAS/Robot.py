from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time, math, json, requests, re, heapq
import matplotlib.pyplot as plt

LM_STUDIO_URL = "http://localhost:1234/v1/chat/completions"
MODEL_NAME = "qwen/qwen3-vl-4b"

MAP_MIN = -6.0
MAP_MAX = 6.0
GRID_SIZE = 0.15
MAP_CELLS = int((MAP_MAX - MAP_MIN) / GRID_SIZE)

REPLAN_EVERY = 10
OBSTACLE_INFLATION = 1
MAX_SAVED_POINTS = 2500

SMOOTH_NEAR_TARGET_DISTANCE = 0.60
SMOOTH_FRONT_SAFE_DISTANCE = 0.80
A_STAR_FRONT_MIN_DISTANCE = 0.18

MAPPING_SENSOR_IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 15]

client = RemoteAPIClient()
sim = client.require("sim")

robot = sim.getObject("/PioneerP3DX")
left_motor = sim.getObject("/PioneerP3DX/leftMotor")
right_motor = sim.getObject("/PioneerP3DX/rightMotor")

ultrasonic_sensors = [
    sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]")
    for i in range(16)
]

saved_points = []
occupancy_grid = [[0 for _ in range(MAP_CELLS)] for _ in range(MAP_CELLS)]

plt.ion()
fig, ax = plt.subplots(figsize=(7, 7), constrained_layout=True)

try:
    fig.canvas.manager.set_window_title("P3DX Realtime Map")
except Exception:
    pass

PLOT_MARGIN = 0.75
PLOT_EXPAND_MARGIN = 0.60

plot_bounds_initialized = False
plot_xmin = MAP_MIN
plot_xmax = MAP_MAX
plot_ymin = MAP_MIN
plot_ymax = MAP_MAX


def limit(v, mn, mx):
    return max(min(v, mx), mn)


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def set_wheel_speed(left, right):
    sim.setJointTargetVelocity(left_motor, left)
    sim.setJointTargetVelocity(right_motor, right)


def stop_robot():
    set_wheel_speed(0.0, 0.0)


def apply_twist(linear, angular):
    left = limit(linear - angular, -3.0, 3.0)
    right = limit(linear + angular, -3.0, 3.0)
    set_wheel_speed(left, right)


def world_to_grid(x, y):
    gx = int((x - MAP_MIN) / GRID_SIZE)
    gy = int((y - MAP_MIN) / GRID_SIZE)
    return gx, gy


def grid_to_world(gx, gy):
    x = MAP_MIN + gx * GRID_SIZE + GRID_SIZE / 2
    y = MAP_MIN + gy * GRID_SIZE + GRID_SIZE / 2
    return x, y


def in_grid(gx, gy):
    return 0 <= gx < MAP_CELLS and 0 <= gy < MAP_CELLS


def mark_obstacle(gx, gy):
    for dx in range(-OBSTACLE_INFLATION, OBSTACLE_INFLATION + 1):
        for dy in range(-OBSTACLE_INFLATION, OBSTACLE_INFLATION + 1):
            nx, ny = gx + dx, gy + dy
            if in_grid(nx, ny):
                occupancy_grid[ny][nx] = 1


def clear_robot_area(robot_x, robot_y, radius_cell=2):
    gx, gy = world_to_grid(robot_x, robot_y)

    for dx in range(-radius_cell, radius_cell + 1):
        for dy in range(-radius_cell, radius_cell + 1):
            nx, ny = gx + dx, gy + dy
            if in_grid(nx, ny):
                occupancy_grid[ny][nx] = 0


def rebuild_occupancy_grid(points, robot_x, robot_y, target_x, target_y):
    global occupancy_grid

    occupancy_grid = [[0 for _ in range(MAP_CELLS)] for _ in range(MAP_CELLS)]

    for px, py in points:
        gx, gy = world_to_grid(px, py)
        if in_grid(gx, gy):
            mark_obstacle(gx, gy)

    clear_robot_area(robot_x, robot_y, radius_cell=2)
    clear_robot_area(target_x, target_y, radius_cell=2)


def safe_min(values, default=3.0):
    valid = [v for v in values if v is not None]
    return min(valid) if valid else default


def read_ultrasonic():
    distances = []
    current_points = []

    ignore_detected = [
        "pioneer", "floor", "ground", "terrain",
        "wall_visible", "respondable"
    ]

    for i, sensor in enumerate(ultrasonic_sensors):
        result, distance, detected_point, detected_object, *_ = sim.readProximitySensor(sensor)

        if result > 0:
            distances.append(distance)

            try:
                detected_alias = sim.getObjectAlias(detected_object).lower()
            except Exception:
                detected_alias = ""

            if any(word in detected_alias for word in ignore_detected):
                continue

            if i not in MAPPING_SENSOR_IDS:
                continue

            if distance < 0.12 or distance > 2.5:
                continue

            sensor_matrix = sim.getObjectMatrix(sensor, -1)
            world_point = sim.multiplyVector(sensor_matrix, detected_point)

            if world_point[2] < 0.05:
                continue

            current_points.append([world_point[0], world_point[1]])

        else:
            distances.append(None)

    return distances, current_points


def build_sensor_state(distances):
    return {
        "front": safe_min(distances[1:7]),
        "left": safe_min([distances[0], distances[15]]),
        "right": safe_min([distances[7], distances[8]]),
        "back": safe_min(distances[10:15]),
    }


def astar(start, goal):
    if not in_grid(start[0], start[1]) or not in_grid(goal[0], goal[1]):
        return []

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    neighbors = [
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1)
    ]

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = [current]

            while current in came_from:
                current = came_from[current]
                path.append(current)

            path.reverse()
            return path

        for dx, dy in neighbors:
            nx = current[0] + dx
            ny = current[1] + dy

            if not in_grid(nx, ny):
                continue

            if occupancy_grid[ny][nx] == 1:
                continue

            cost = math.hypot(dx, dy)
            new_g = g_score[current] + cost

            if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                g_score[(nx, ny)] = new_g
                f = new_g + h((nx, ny), goal)
                heapq.heappush(open_set, (f, (nx, ny)))
                came_from[(nx, ny)] = current

    return []


def extract_json(text):
    match = re.search(r"\{.*\}", text, re.DOTALL)

    if not match:
        raise ValueError("No JSON found")

    return json.loads(match.group(0))


def scan_targets():
    targets = {}

    exclude = [
        "pioneer", "motor", "ultrasonic", "sensor", "floor",
        "joint", "camera", "wheel", "body", "visible",
        "respondable", "left", "right", "connection",
        "graph", "default", "light", "path", "dummy",
        "cuboid", "box", "script", "caster_link"
    ]

    objects = sim.getObjectsInTree(sim.handle_scene)

    for obj in objects:
        alias = sim.getObjectAlias(obj)
        low = alias.lower()

        if any(e in low for e in exclude):
            continue

        try:
            pos = sim.getObjectPosition(obj, -1)

            targets[low] = {
                "name": alias,
                "handle": obj,
                "position": [
                    round(pos[0], 2),
                    round(pos[1], 2),
                    round(pos[2], 2)
                ]
            }

        except Exception:
            pass

    return targets


def ask_llm_target_sequence(user_mission, available_targets):
    prompt = f"""
You are a mission planner for a mobile robot.

User command:
"{user_mission}"

Available target objects:
{json.dumps(available_targets, indent=2)}

Return ONLY valid JSON.

Format:
{{
  "target_sequence": ["ObjectName1", "ObjectName2"]
}}

Rules:
1. Choose only from available target objects.
2. Preserve requested order.
3. If unclear, choose the most relevant target.
"""

    payload = {
        "model": MODEL_NAME,
        "messages": [{"role": "user", "content": prompt}],
        "temperature": 0.1,
        "max_tokens": 150,
    }

    r = requests.post(LM_STUDIO_URL, json=payload, timeout=15)
    r.raise_for_status()

    content = r.json()["choices"][0]["message"]["content"]
    data = extract_json(content)

    return data["target_sequence"], content


def ask_llm_navigation_mode(user_mission, llm_state):
    prompt = f"""
You are a high-level navigation supervisor for a Pioneer P3DX mobile robot.

Mission:
"{user_mission}"

Robot state:
{json.dumps(llm_state, indent=2)}

Return ONLY valid JSON.
Do not explain.

Format:
{{
  "mode": "A_STAR"
}}

Allowed modes:
- A_STAR: follow planned waypoint.
- SMOOTH: go directly to target with smooth controller.
- AVOID: prioritize obstacle avoidance.
- REPLAN: force new A* path planning.

Rules:
1. If obstacle is very close in front, choose AVOID.
2. If path_length > 0 and front is safe, choose A_STAR.
3. If path_length == 0, choose SMOOTH or REPLAN.
4. If robot seems confused or path does not progress, choose REPLAN.
5. Do not output linear/angular.
"""

    payload = {
        "model": MODEL_NAME,
        "messages": [{"role": "user", "content": prompt}],
        "temperature": 0.1,
        "max_tokens": 80,
    }

    r = requests.post(LM_STUDIO_URL, json=payload, timeout=10)
    r.raise_for_status()

    content = r.json()["choices"][0]["message"]["content"]
    data = extract_json(content)

    mode = data.get("mode", "A_STAR")

    if mode not in ["A_STAR", "SMOOTH", "AVOID", "REPLAN"]:
        mode = "A_STAR"

    return mode, content


def get_pose():
    pos = sim.getObjectPosition(robot, -1)
    ori = sim.getObjectOrientation(robot, -1)

    return pos[0], pos[1], ori[2]


def get_target_position(handle):
    pos = sim.getObjectPosition(handle, -1)

    return pos[0], pos[1]


def get_floor_bounds_from_scene():
    best_bounds = None
    best_area = 0.0

    try:
        objects = sim.getObjectsInTree(sim.handle_scene)
    except Exception:
        return MAP_MIN, MAP_MAX, MAP_MIN, MAP_MAX

    floor_keywords = ["floor", "ground", "terrain", "plane"]

    for obj in objects:
        try:
            alias = sim.getObjectAlias(obj).lower()
        except Exception:
            continue

        if not any(k in alias for k in floor_keywords):
            continue

        try:
            pos = sim.getObjectPosition(obj, -1)

            min_x = sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_min_x)
            max_x = sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_max_x)
            min_y = sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_min_y)
            max_y = sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_max_y)

            xmin = pos[0] + min_x - PLOT_MARGIN
            xmax = pos[0] + max_x + PLOT_MARGIN
            ymin = pos[1] + min_y - PLOT_MARGIN
            ymax = pos[1] + max_y + PLOT_MARGIN

            area = abs((xmax - xmin) * (ymax - ymin))

            if area > best_area:
                best_area = area
                best_bounds = (xmin, xmax, ymin, ymax)

        except Exception:
            continue

    if best_bounds is None:
        return MAP_MIN, MAP_MAX, MAP_MIN, MAP_MAX

    return best_bounds


def initialize_plot_bounds_once():
    global plot_bounds_initialized, plot_xmin, plot_xmax, plot_ymin, plot_ymax

    if plot_bounds_initialized:
        return

    plot_xmin, plot_xmax, plot_ymin, plot_ymax = get_floor_bounds_from_scene()
    plot_bounds_initialized = True


def expand_plot_bounds_if_needed(points):
    global plot_xmin, plot_xmax, plot_ymin, plot_ymax

    initialize_plot_bounds_once()

    if not points:
        return

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)

    if min_x < plot_xmin + PLOT_EXPAND_MARGIN:
        plot_xmin = min_x - PLOT_MARGIN

    if max_x > plot_xmax - PLOT_EXPAND_MARGIN:
        plot_xmax = max_x + PLOT_MARGIN

    if min_y < plot_ymin + PLOT_EXPAND_MARGIN:
        plot_ymin = min_y - PLOT_MARGIN

    if max_y > plot_ymax - PLOT_EXPAND_MARGIN:
        plot_ymax = max_y + PLOT_MARGIN


def follow_waypoint(robot_x, robot_y, yaw, wx, wy):
    dx = wx - robot_x
    dy = wy - robot_y

    distance = math.sqrt(dx * dx + dy * dy)
    desired_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(desired_heading - yaw)

    if distance < 0.18:
        return 0.0, 0.0, True

    angular = 1.2 * heading_error

    if abs(heading_error) > 0.7:
        linear = 0.2
    else:
        linear = 0.75

    return limit(linear, 0.0, 0.9), limit(angular, -1.4, 1.4), False


def smooth_controller(robot_x, robot_y, yaw, target_x, target_y, distances):
    dx = target_x - robot_x
    dy = target_y - robot_y

    distance_to_target = math.sqrt(dx * dx + dy * dy)
    desired_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(desired_heading - yaw)

    s = build_sensor_state(distances)

    front = s["front"]
    left = s["left"]
    right = s["right"]

    linear = 0.9
    angular = 0.9 * heading_error

    if front < 1.20:
        linear = 0.65

    if front < 0.75:
        linear = 0.40
        angular += -0.45 if right > left else 0.45

    if left < 0.40 and front > 0.45:
        linear = 0.55
        angular -= 0.30

    if right < 0.40 and front > 0.45:
        linear = 0.55
        angular += 0.30

    if front < 0.22:
        linear = -0.25
        angular = -0.7 if right > left else 0.7

    if distance_to_target < 0.45:
        linear = 0.30

    return limit(linear, -0.5, 0.9), limit(angular, -1.2, 1.2), distance_to_target, heading_error, s


def target_biased_controller(robot_x, robot_y, yaw, target_x, target_y, distances):
    dx = target_x - robot_x
    dy = target_y - robot_y

    distance_to_target = math.sqrt(dx * dx + dy * dy)
    desired_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(desired_heading - yaw)

    d = [3.0 if x is None else x for x in distances]

    sectors = [
        {"angle":  1.0, "dist": d[1]},
        {"angle":  0.6, "dist": d[2]},
        {"angle":  0.25, "dist": d[3]},
        {"angle": -0.25, "dist": d[4]},
        {"angle": -0.6, "dist": d[5]},
        {"angle": -1.0, "dist": d[6]},
    ]

    safe_distance = 0.28

    free_sectors = [
        s for s in sectors
        if s["dist"] > safe_distance
    ]

    if free_sectors:
        best = min(
            free_sectors,
            key=lambda s: abs(s["angle"] - heading_error)
        )

        linear = 0.55
        angular = 0.9 * best["angle"]

        if best["dist"] > 0.90:
            linear = 0.75

    else:
        sensor_state = build_sensor_state(distances)

        if sensor_state["right"] > sensor_state["left"]:
            linear = -0.20
            angular = -0.8
        else:
            linear = -0.20
            angular = 0.8

    if distance_to_target < 0.45:
        linear = 0.30

    return (
        limit(linear, -0.5, 0.8),
        limit(angular, -1.1, 1.1),
        distance_to_target,
        heading_error
    )


def emergency_avoid(distances):
    s = build_sensor_state(distances)

    if s["front"] > 0.18:
        return None

    if s["right"] > s["left"]:
        return -0.25, -0.9
    else:
        return -0.25, 0.9


def update_plot(robot_x, robot_y, yaw, target_x, target_y, current_points, path, waypoint_index):
    ax.clear()

    path_world = []
    if path:
        path_world = [grid_to_world(p[0], p[1]) for p in path]

    bounds_points = [
        [robot_x, robot_y],
        [target_x, target_y],
    ]

    bounds_points.extend(current_points)
    bounds_points.extend(path_world)

    if saved_points:
        bounds_points.extend(saved_points[-300:])

    expand_plot_bounds_if_needed(bounds_points)

    if saved_points:
        mx = [p[0] for p in saved_points]
        my = [p[1] for p in saved_points]
        ax.scatter(mx, my, s=7, alpha=0.25, label="Saved ultrasonic map")

    if current_points:
        cx = [p[0] for p in current_points]
        cy = [p[1] for p in current_points]
        ax.scatter(cx, cy, s=30, label="Current ultrasonic hits")

    if path_world:
        px = [p[0] for p in path_world]
        py = [p[1] for p in path_world]
        ax.plot(px, py, "g-", linewidth=2, label="A* path")

        if waypoint_index < len(path):
            wx, wy = grid_to_world(
                path[waypoint_index][0],
                path[waypoint_index][1]
            )
            ax.plot(wx, wy, "yo", markersize=8, label="Waypoint")

    ax.plot(robot_x, robot_y, "bo", markersize=8, label="Robot")

    ax.arrow(
        robot_x,
        robot_y,
        0.35 * math.cos(yaw),
        0.35 * math.sin(yaw),
        head_width=0.15,
        head_length=0.15
    )

    ax.plot(target_x, target_y, "ro", markersize=8, label="Target")

    ax.set_title("Ultrasonic Map + A* + LLM Mode Supervisor")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_xlim(plot_xmin, plot_xmax)
    ax.set_ylim(plot_ymin, plot_ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend(loc="upper right")

    fig.canvas.draw_idle()
    plt.pause(0.001)


# ==========================
# MAIN
# ==========================

user_mission = input("Masukkan perintah untuk robot: ")

targets = scan_targets()

print("\nObject target terdeteksi:")
for t in targets.values():
    print("-", t["name"], t["position"])

if not targets:
    raise Exception("Tidak ada target terdeteksi.")

available_names = [t["name"] for t in targets.values()]

manual_seq = []
mission_lower = user_mission.lower()

for target_name in available_names:
    if target_name.lower() in mission_lower:
        manual_seq.append(target_name)

if manual_seq:
    seq = manual_seq
    print("\nManual Mission Plan:")
    print(seq)
else:
    try:
        seq, raw_plan = ask_llm_target_sequence(user_mission, available_names)
        print("\nLLM Mission Plan:")
        print(raw_plan)
    except Exception as e:
        print("LLM planner gagal:", e)
        seq = [available_names[0]]

clean_sequence = []

for name in seq:
    key = name.lower()

    if key in targets:
        clean_sequence.append(key)

if not clean_sequence:
    clean_sequence = [available_names[0].lower()]

print("\nUrutan target final:")
for key in clean_sequence:
    print("-", targets[key]["name"])

sim.startSimulation()
time.sleep(1)
initialize_plot_bounds_once()

try:
    for target_key in clean_sequence:
        target = targets[target_key]
        target_name = target["name"]
        target_handle = target["handle"]

        print(f"\nMenuju {target_name}")

        path = []
        waypoint_index = 0
        counter = 0

        llm_mode = "A_STAR"
        llm_counter = 0

        last_dist = None
        stuck_counter = 0

        while True:
            robot_x, robot_y, yaw = get_pose()
            target_x, target_y = get_target_position(target_handle)

            distances, current_points = read_ultrasonic()
            sensor_state = build_sensor_state(distances)

            saved_points.extend(current_points)
            if len(saved_points) > MAX_SAVED_POINTS:
                del saved_points[:-MAX_SAVED_POINTS]

            dist_to_target = math.hypot(target_x - robot_x, target_y - robot_y)

            rebuild_occupancy_grid(saved_points, robot_x, robot_y, target_x, target_y)

            if dist_to_target < 0.35:
                print(f"{target_name} reached")
                stop_robot()
                time.sleep(1)
                break

            if last_dist is not None:
                if abs(last_dist - dist_to_target) < 0.01:
                    stuck_counter += 1
                else:
                    stuck_counter = 0

            last_dist = dist_to_target

            if llm_counter % 30 == 0:
                llm_state = {
                    "distance_to_target": round(dist_to_target, 2),
                    "front": round(sensor_state["front"], 2),
                    "left": round(sensor_state["left"], 2),
                    "right": round(sensor_state["right"], 2),
                    "path_length": len(path),
                    "waypoint_index": waypoint_index,
                    "stuck_counter": stuck_counter,
                }

                try:
                    stop_robot()
                    llm_mode, raw_mode = ask_llm_navigation_mode(user_mission, llm_state)
                    print("LLM MODE:", raw_mode)
                except Exception as e:
                    print("LLM mode failed:", e)
                    llm_mode = "A_STAR"

            llm_counter += 1

            emergency = emergency_avoid(distances)

            if emergency is not None:
                linear, angular = emergency
                mode = "EMERGENCY"

            else:
                force_replan = llm_mode == "REPLAN" or stuck_counter > 25

                if counter % REPLAN_EVERY == 0 or waypoint_index >= len(path) or force_replan:
                    start = world_to_grid(robot_x, robot_y)
                    goal = world_to_grid(target_x, target_y)
                    path = astar(start, goal)
                    waypoint_index = 0

                    if not path:
                        print("A* FAILED")
                        print("Robot world:", round(robot_x, 2), round(robot_y, 2))
                        print("Target world:", round(target_x, 2), round(target_y, 2))
                        print("Start grid:", start, "in_grid:", in_grid(start[0], start[1]))
                        print("Goal grid:", goal, "in_grid:", in_grid(goal[0], goal[1]))

                    if force_replan:
                        stuck_counter = 0

                if path:
                    while waypoint_index < len(path):
                        wx, wy = grid_to_world(
                            path[waypoint_index][0],
                            path[waypoint_index][1]
                        )

                        if math.hypot(wx - robot_x, wy - robot_y) < 0.35:
                            waypoint_index += 1
                        else:
                            break

                if path and waypoint_index < len(path) and sensor_state["front"] > A_STAR_FRONT_MIN_DISTANCE:
                    wx, wy = grid_to_world(
                        path[waypoint_index][0],
                        path[waypoint_index][1]
                    )
                    linear, angular, _ = follow_waypoint(robot_x, robot_y, yaw, wx, wy)
                    mode = "A_STAR"

                elif (
                    dist_to_target < SMOOTH_NEAR_TARGET_DISTANCE
                    and sensor_state["front"] > SMOOTH_FRONT_SAFE_DISTANCE
                ):
                    linear, angular, _, _, _ = smooth_controller(
                        robot_x, robot_y, yaw, target_x, target_y, distances
                    )
                    mode = "SMOOTH_NEAR_TARGET"

                elif sensor_state["front"] < 0.25:
                    linear, angular, _, _ = target_biased_controller(
                        robot_x, robot_y, yaw, target_x, target_y, distances
                    )
                    mode = "AVOID_CLOSE_OBSTACLE"

                else:
                    linear, angular, _, _ = target_biased_controller(
                        robot_x, robot_y, yaw, target_x, target_y, distances
                    )
                    mode = "TARGET_BIASED_FALLBACK"

            print(
                f"Target={target_name} | "
                f"Dist={dist_to_target:.2f} | "
                f"Front={sensor_state['front']:.2f} | "
                f"Left={sensor_state['left']:.2f} | "
                f"Right={sensor_state['right']:.2f} | "
                f"PathLen={len(path)} | "
                f"WP={waypoint_index} | "
                f"LLM={llm_mode} | "
                f"Mode={mode}"
            )

            apply_twist(linear, angular)

            if counter % 3 == 0:
                update_plot(
                    robot_x,
                    robot_y,
                    yaw,
                    target_x,
                    target_y,
                    current_points,
                    path,
                    waypoint_index
                )

            counter += 1
            time.sleep(0.05)

    print("\nAll targets reached")

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    stop_robot()
    time.sleep(0.5)
    sim.stopSimulation()
    plt.ioff()
    plt.show()