"""
Integrated pathfinding robot controller using Dijkstra's algorithm
with automatic waypoint generation and line-following drift correction.
"""
# -------------------------import--------------------
from controller import Robot, DistanceSensor, Motor
import numpy as np
import heapq


# -------------------------(Dijkstra's Algorithm)------------------------------
# Pathfinding Implementation

def create_grid():
    return np.array([
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0]
    ])


def create_costs():
    return np.ones((15, 17), dtype=int)


def dijkstra(grid, costs, start, goal):
    rows, cols = grid.shape
    visited = set()
    distance = {start: 0}
    prev = {}
    queue = [(0, start)]

    while queue:
        current_dist, current = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            break

        r, c = current
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if grid[nr, nc] == 0:
                    neighbor = (nr, nc)
                    cost = costs[nr, nc]
                    new_dist = current_dist + cost
                    if neighbor not in distance or new_dist < distance[neighbor]:
                        distance[neighbor] = new_dist
                        prev[neighbor] = current
                        heapq.heappush(queue, (new_dist, neighbor))

    # Reconstruct path
    path = []
    current = goal
    while current in prev:
        path.append(current)
        current = prev[current]
    path.append(start)
    path.reverse()
    return path


# -------------------------(grid to world)------------------------------

def grid_to_world(row, col):
    # Override coordinates for known mismatches
    manual_overrides = {
        (0, 2): (0.431347, -0.364056),
        (0, 4): (0.329743, -0.364056),
        (0, 6): (0.227713, -0.364056),
        (1, 2): (0.431976, -0.31445249999999997),
        (1, 4): (0.329976, -0.31445249999999997),
        (1, 6): (0.227976, -0.31445249999999997),
        (13, 10): (-0.15413, 0.2807895000000001),
        (13, 12): (-0.254551, 0.2807895000000001),
        (13, 14): (-0.358972, 0.2807895000000001),
        (14, 10): (-0.15413, 0.330393),
        (14, 12): (-0.254551, 0.330393),
        (14, 14): (-0.358972, 0.330393)
    }

    if (row, col) in manual_overrides:
        return manual_overrides[(row, col)]

    if row == 2:
        manual_row2 = {
            0: 0.531976, 1: 0.480661, 2: 0.431347, 3: 0.378032, 4: 0.329717,
            5: 0.275402, 6: 0.227087, 7: 0.172772, 8: 0.042291, 9: 0.000142,
            10: -0.032488, 11: -0.083803, 12: -0.135118, 13: -0.235118,
            14: -0.306433, 15: -0.387748, 16: -0.477393
        }
        x = manual_row2.get(col)
        y = -0.268549
        if x is not None:
            return (x, y)

    if row == 12:
        manual_row12 = {
            0: 0.531976, 1: 0.480661, 2: 0.429347, 3: 0.378032, 4: 0.326717,
            5: 0.275402, 6: 0.224087, 7: 0.172772, 8: 0.040142, 9: -0.04013,
            10: -0.15313, 11: -0.201445, 12: -0.255551, 13: -0.304075,
            14: -0.357972, 15: -0.406705, 16: -0.45802
        }
        x = manual_row12.get(col)
        y = 0.231986
        if x is not None:
            return (x, y)

    # Default mapping
    x_origin = 0.531976
    y_origin = -0.364056
    dx_per_col = -0.0622105625
    dy_per_row = 0.0496035

    x = x_origin + col * dx_per_col
    y = y_origin + row * dy_per_row
    return (x, y)

# -------------------------(world to grid)------------------------------

def world_to_grid(x, y):
    """Convert world coordinates back to grid coordinates,
    with special handling for rows 2, 12 and manual overrides."""

    # Manual override: reverse of custom grid_to_world patch
    manual_reverse = {
        (0.431347, -0.364056): (0, 2),
        (0.329743, -0.364056): (0, 4),
        (0.227713, -0.364056): (0, 6),
        (0.431976, -0.31445249999999997): (1, 2),
        (0.329976, -0.31445249999999997): (1, 4),
        (0.227976, -0.31445249999999997): (1, 6),
        (-0.15413, 0.2807895000000001): (13, 10),
        (-0.254551, 0.2807895000000001): (13, 12),
        (-0.358972, 0.2807895000000001): (13, 14),
        (-0.15413, 0.330393): (14, 10),
        (-0.254551, 0.330393): (14, 12),
        (-0.358972, 0.330393): (14, 14)
    }

    # Tolerance for matching float coordinates
    EPS = 1e-6
    for (mx, my), (r, c) in manual_reverse.items():
        if abs(mx - x) < EPS and abs(my - y) < EPS:
            return (r, c)

    # Manual mappings from grid_to_world:
    manual_row2 = {
        0: 0.531976, 1: 0.480661, 2: 0.429347, 3: 0.378032, 4: 0.326717,
        5: 0.275402, 6: 0.224087, 7: 0.172772, 8: 0.042291, 9: 0.000142,
        10: -0.032488, 11: -0.083803, 12: -0.135118, 13: -0.235118,
        14: -0.306433, 15: -0.387748, 16: -0.457393
    }
    manual_row12 = {
        0: 0.531976, 1: 0.480661, 2: 0.429347, 3: 0.378032, 4: 0.326717,
        5: 0.275402, 6: 0.224087, 7: 0.172772, 8: 0.040142, 9: -0.04013,
        10: -0.15013, 11: -0.201445, 12: -0.252551, 13: -0.304075,
        14: -0.354972, 15: -0.406705, 16: -0.45802
    }

    y_row2 = -0.267049
    y_row12 = 0.227986
    dy = 0.0496035 / 2.0
    dx = abs(-0.0622105625) / 2.0

    def find_manual_col(manual_dict):
        col, x_ref = min(manual_dict.items(), key=lambda kv: abs(kv[1] - x))
        if abs(x_ref - x) <= dx:
            return col
        else:
            return None

    if abs(y - y_row2) <= dy:
        col = find_manual_col(manual_row2)
        if col is not None:
            return (2, col)

    if abs(y - y_row12) <= dy:
        col = find_manual_col(manual_row12)
        if col is not None:
            return (12, col)

    # Default linear fallback
    x_origin = 0.531976
    y_origin = -0.364056
    dx_per_col = -0.0622105625
    dy_per_row = 0.0496035

    col = round((x - x_origin) / dx_per_col)
    row = round((y - y_origin) / dy_per_row)

    # Clamp
    row = max(0, min(14, row))
    col = max(0, min(16, col))

    return (row, col)

# -------------------------(waypoints)------------------------------

def generate_path_waypoints(start_pos, goal_pos, custom_grid=None):
    global main_grid
    grid = custom_grid if custom_grid is not None else main_grid
    costs = create_costs()

    # Convert world coordinates to grid coordinates
    start_grid = world_to_grid(*start_pos)
    goal_grid = world_to_grid(*goal_pos)

    print(f"Start grid: {start_grid}, Goal grid: {goal_grid}")

    # Find path using Dijkstra
    path = dijkstra(grid, costs, start_grid, goal_grid)

    # Convert grid path to world coordinates
    waypoints = []
    for step in path:
        world_coord = grid_to_world(*step)
        waypoints.append(world_coord)

        print(step, grid_to_world(*step))

    print(f"Generated {len(waypoints)} waypoints:")
    for i, wp in enumerate(waypoints):
        print(f"  {i + 1}: ({wp[0]:.3f}, {wp[1]:.3f})")
    force_line_following = False  # Disable override temporarily
    line_follow_start_time = robot.getTime()
    return waypoints


# ------------------------initializaion-------------------------------
# Robot Controller

MAX_SPEED = 6.28

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize the main grid
main_grid = create_grid()

# Initialize devices
ps = [robot.getDevice(f'ps{i}') for i in range(8)]
for sensor in ps:
    sensor.enable(timestep)

gs = [robot.getDevice(f'gs{i}') for i in range(3)]
for sensor in gs:
    sensor.enable(timestep)

encoder = [robot.getDevice(name) for name in ['left wheel sensor', 'right wheel sensor']]
for enc in encoder:
    enc.enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# -------------------------(Parameters)------------------------------

# Robot parameters
R = 0.0336    # radius of the wheels of the e-puck robot: 33.6mm
D = 0.097    # inside distance between the wheels of the e-puck robot: 97mm
delta_t = timestep / 1000.0  # 0.016

# Position tracking variables (odometry only)
x, y = 0.531976, -0.364056  # Initial position
phi = 1.57111

oldEncoderValues = [0, 0]
encoderValues = [0, 0]

# Define all box pickup and delivery locations
boxes = [
    {
        'pickup': (-0.46339299999999994, 0.330393),
        'delivery': (0.531976, -0.364056)  # Back to start for first box
    },
    {
        'pickup': (-0.358972, 0.330393),
        'delivery': (0.431347, -0.364056)
    },
    {
        'pickup': (-0.254551, 0.330393),
        'delivery': (0.329743, -0.364056)
    },
    {
        'pickup': (-0.15413, 0.330393),
        'delivery': (0.227713, -0.364056)
    }
]

# Mission control variables
current_box_index = 0
mission_phase = "pickup"  # "pickup" or "delivery"
waypoints = []
current_waypoint_index = 0
waypoint_reached_threshold = 0.045 #0.04 # Increased threshold for better reliability

# Generate initial path to first pickup location
start_position = (x, y)
goal_position = boxes[current_box_index]['pickup']
waypoints = generate_path_waypoints(start_position, goal_position)

# Control variables
e_acc = 0
e_prev = 0

# Line following state machine variables
line_following_states = ['forward', 'turn_right', 'turn_left']
line_following_state = line_following_states[0]
line_counter = 0
LINE_COUNTER_MAX = 2  # Reduced for more responsive line following
print_timer = 0.0  # Accumulator for debug printing
print_interval = 0.5  # Time in seconds between prints
last_replan_time = 0.0
replan_cooldown = 2.0  # seconds
force_line_following = False
line_follow_start_time = 0.0
line_follow_block_duration = 3.0  # seconds to disable force-follow
last_snap_time = 0.0
snap_cooldown = 0.45  # seconds 0.4
turn_direction = None  # Initialized somewhere appropriate
mission_complete = False

error = 0
PID_turn = False
PID_turn2 = False
# -------------------------functions------------------------------
# -------------position-----------

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t
    return wl, wr


def get_robot_speeds(wl, wr, R, D):
    u = R / 2.0 * (wr + wl)
    w = R / D * (wr - wl)
    return u, w


def get_robot_pose(u, w, x, y, phi, delta_t):
    delta_phi = w * delta_t
    phi += delta_phi

    # Wrap angle to [-π, π]
    if phi >= np.pi:
        phi = phi - 2 * np.pi
    elif phi < -np.pi:
        phi = phi + 2 * np.pi

    delta_x = u * np.cos(phi) * delta_t
    delta_y = u * np.sin(phi) * delta_t
    x += delta_x
    y += delta_y

    return x, y, phi


# -------------PID-----------

def get_pose_error(xd, yd, x, y, phi):
    x_err = xd - x
    y_err = yd - y
    dist_err = np.sqrt(x_err ** 2 + y_err ** 2)
    phi_d = np.arctan2(y_err, x_err)
    phi_err = np.arctan2(np.sin(phi_d - phi), np.cos(phi_d - phi))
    return dist_err, phi_err


def pid_controller(e, e_prev, e_acc, delta_t, kp=3.5, kd=0.2, ki=0.01):  # Increased gains
    P = kp * e
    I = e_acc + ki * e * delta_t
    D = kd * (e - e_prev) / delta_t
    output = P + I + D
    e_prev = e
    e_acc = I
    return output, e_prev, e_acc


def wheel_speed_commands(u_d, w_d, d, r):
    wr_d = float((2 * u_d + d * w_d) / (2 * r))
    wl_d = float((2 * u_d - d * w_d) / (2 * r))

    if np.abs(wl_d) > MAX_SPEED or np.abs(wr_d) > MAX_SPEED:
        speed_ratio = np.abs(wr_d) / np.abs(wl_d) if np.abs(wl_d) != 0 else 1
        if speed_ratio > 1:
            wr_d = np.sign(wr_d) * MAX_SPEED
            wl_d = np.sign(wl_d) * MAX_SPEED / speed_ratio
        else:
            wl_d = np.sign(wl_d) * MAX_SPEED
            wr_d = np.sign(wr_d) * MAX_SPEED * speed_ratio

    return wl_d, wr_d


# -------------adding waypoint -----------

def update_current_waypoint():
    global current_waypoint_index, e_acc, e_prev

    if current_waypoint_index < len(waypoints) - 1:
        current_waypoint_index += 1
        e_acc = 0
        e_prev = 0
        xd, yd = waypoints[current_waypoint_index]
        print(f"Moving to waypoint {current_waypoint_index + 1}/{len(waypoints)}: ({xd:.3f}, {yd:.3f})")
        return True
    else:
         return False

def advance_mission():
    global current_box_index, mission_phase, waypoints, current_waypoint_index, e_acc, e_prev
    global force_line_following, line_follow_start_time, mission_complete

    if mission_phase == "pickup":
        start_position = boxes[current_box_index]['pickup']
        # Switch to delivery phase
        mission_phase = "delivery"
        goal_position = boxes[current_box_index]['delivery']
        print(f"Box picked up! Delivering to {goal_position}")
    elif mission_phase == "delivery":
        start_position = boxes[current_box_index]['delivery']
        # Move to next box
        current_box_index += 1
        if current_box_index < len(boxes):
            mission_phase = "pickup"
            goal_position = boxes[current_box_index]['pickup']
            print(f"Box delivered! Moving to next pickup at {goal_position}")
        else:
            mission_complete = True
            print("All boxes delivered! Mission complete!")
            return False

    # Generate new path
    waypoints = generate_path_waypoints(start_position, goal_position)
    current_waypoint_index = 0
    e_acc = 0
    e_prev = 0
    force_line_following = False
    line_follow_start_time = robot.getTime()
    return True
# -------------Line_follower-----------

def line_following_control(gsValues, force_follow=False):
    """
    Enhanced line-following logic using ground sensors.
    Applies stronger corrections when off-center or explicitly forced.
    Returns (leftSpeed, rightSpeed).
    """
    global line_following_state, line_counter

    # Determine line visibility
    line_right = gsValues[0] > 600
    line_center = gsValues[1] > 600
    line_left = gsValues[2] > 600

    centered_on_line = (gsValues[0] > 600 and gsValues[1] < 400 and gsValues[2] > 600)

    # Stronger base speed for better control
    base_speed = MAX_SPEED * 0.6  # Increased from 0.5

    # Force robot to follow line even if orientation is off
    if force_follow or centered_on_line:
        line_following_state = 'forward'
        line_counter = 0

    # --- State machine ---
    if line_following_state == 'forward':
        leftSpeed = base_speed
        rightSpeed = base_speed

        if line_right and not line_left:
            line_following_state = 'turn_right'
            line_counter = 0
        elif line_left and not line_right:
            line_following_state = 'turn_left'
            line_counter = 0

    elif line_following_state == 'turn_right':
        leftSpeed = 1.2 * base_speed   # Increased turning strength
        rightSpeed = 0.1 * base_speed  # Reduced opposing wheel speed

        if line_counter >= LINE_COUNTER_MAX:
            line_following_state = 'forward'

    elif line_following_state == 'turn_left':
        leftSpeed = 0.1 * base_speed   # Reduced opposing wheel speed
        rightSpeed = 1.2 * base_speed  # Increased turning strength

        if line_counter >= LINE_COUNTER_MAX:
            line_following_state = 'forward'

    line_counter += 1
    return leftSpeed, rightSpeed


# --------------------position correction-----------------------

def correct_position_on_line(x, y, robot_time, last_snap_time):
    """
    If the robot is centered on a line, snap either x or y based on closest grid alignment.
    """
    horizontal_rows = {2, 5, 7, 9, 12}
    vertical_cols = {0, 2, 4, 6, 8, 10, 12, 14, 16}

    if robot_time - last_snap_time < snap_cooldown:
        return x, y, last_snap_time  # skip correction

    row, col = world_to_grid(x, y)
    corrected_x, corrected_y = grid_to_world(row, col)

    snap_x = col in vertical_cols
    snap_y = row in horizontal_rows

    dx = abs(x - corrected_x) if snap_x else float('inf')
    dy = abs(y - corrected_y) if snap_y else float('inf')

    if dy < dx:
        print(f"Snapping y from {y:.3f} → {corrected_y:.3f} (row {row})")
        return x, corrected_y, robot_time
    elif dx < dy:
        print(f"Snapping x from {x:.3f} → {corrected_x:.3f} (col {col})")
        return corrected_x, y, robot_time
    else:
        return x, y, last_snap_time  # no clear snap direction


# --------------------main loop-----------------------------------
#

print("Starting pathfinding robot controller with line following...")
print(f"Path has {len(waypoints)} waypoints")

while robot.step(timestep) != -1:
    # -------------position-----------
    if not force_line_following and (robot.getTime() - line_follow_start_time > line_follow_block_duration):  #
        force_line_following = True  #
        print("Line-follow override re-enabled.")  #
    oldEncoderValues = encoderValues
    encoderValues = [encoder[i].getValue() for i in range(2)]
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)

    # -------------Read sensors------------
    gsValues = [gs[i].getValue() for i in range(3)]

    psValues = [ps[i].getValue() for i in range(8)]

    # -------------obstacle detect-----------

    obstacle_detected = psValues[0] > 85.0 or psValues[7] > 85.0
    # print(f"obstacle: {obstacle_detected} {psValues[0]} {psValues[7]}")

    # -------------replann-----------

    if obstacle_detected and (robot.getTime() - last_replan_time) > replan_cooldown:
        last_replan_time = robot.getTime()
        print("Obstacle detected! Replanning...")

        # Block current waypoint on the grid
        blocked_cell = world_to_grid(*waypoints[current_waypoint_index])
        print(f"Marking grid cell {blocked_cell} as obstacle.")
        main_grid[blocked_cell[0], blocked_cell[1]] = 1  # Set as obstacle
        #
        # Replan path from current robot position
        waypoints = generate_path_waypoints((x, y), goal_position)
        current_waypoint_index = 0
        e_acc = 0
        e_prev = 0
        force_line_following = False  # Disable override temporarily
        line_follow_start_time = robot.getTime()
        # x, y = relocalize(waypoints, current_waypoint_index, gsValues)

        if len(waypoints) == 0:
            print("No alternative path found!")
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
        else:
            print(f"New path with {len(waypoints)} waypoints created.")
            continue

    # ------- Check if we have waypoints to follow-----------
    if len(waypoints) == 0:
        print("No valid path found!")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        continue

    # ---------current target waypoint---------
    if current_waypoint_index < len(waypoints):
        xd, yd = waypoints[current_waypoint_index]
    else:
        # -------All waypoints reached---------
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        continue
    # ----------correction----------
    snap_on_line = (
            (gsValues[0] > 500 and gsValues[1] < 400
            and gsValues[2] > 500) or (gsValues[0] < 400
            and gsValues[1] < 400) or (gsValues[1] < 400 and gsValues[2] < 400)
    )
    if snap_on_line:
        x, y, last_snap_time = correct_position_on_line(x, y, robot.getTime(), last_snap_time)

    # ----------errors----------
    position_err, orientation_err = get_pose_error(xd, yd, x, y, phi)

    # ------------Waypoint_PID and line_following-------------



    if position_err < waypoint_reached_threshold:
        print(f"Waypoint {current_waypoint_index + 1} reached!")
        if not update_current_waypoint():
            if not advance_mission():
                # Mission complete
                leftSpeed = 0
                rightSpeed = 0
        else:
            continue

    else:
        # Determine if robot is centered on the line: white–black–white
        centered_on_line = (
                gsValues[0] > 600 and
                gsValues[1] < 400 and
                gsValues[2] > 600
        )

        # Check if robot is on any line (for line following decision)
        on_line = (gsValues[0] > 600 or gsValues[1] > 600 or gsValues[2] > 600)

        if centered_on_line and force_line_following:
            # Force line-following if robot is centered on the line
            leftSpeed, rightSpeed = line_following_control(gsValues, force_follow=True)
            turn_direction = None
            error = 0
            PID_turn = False
            PID_turn2 = False

        elif abs(orientation_err) > 1.2 or PID_turn2:  # Increased threshold for major corrections
            # Use strong PID for very large errors
            print("PID Major Correction")
            w_d, e_prev, e_acc = pid_controller(orientation_err, e_prev, e_acc, delta_t, kp=4.0, kd=0.3)
            PID_turn2 = True
            PID_turn = False
            turn_direction = None

            u_d = 0.008  # Very slow forward movement for precise turning
            wl_d, wr_d = wheel_speed_commands(u_d, w_d, D, R)
            leftSpeed = wl_d
            rightSpeed = wr_d

            if abs(orientation_err) < 0.15:  # Exit condition
                PID_turn2 = False

        elif abs(orientation_err) > 0.6 or PID_turn:  # Medium corrections
            # Use moderate PID for medium errors
            print("PID Minor Correction")
            w_d, e_prev, e_acc = pid_controller(orientation_err, e_prev, e_acc, delta_t)
            PID_turn = True
            turn_direction = None

            u_d = 0.02  # Slow forward movement
            wl_d, wr_d = wheel_speed_commands(u_d, w_d, D, R)
            leftSpeed = wl_d
            rightSpeed = wr_d

            if abs(orientation_err) < 0.3:  # Exit condition
                PID_turn = False

        elif on_line:
            # Use line following when on a line
            leftSpeed, rightSpeed = line_following_control(gsValues, force_follow=False)
            turn_direction = None
            error = 0
            PID_turn = False
            PID_turn2 = False

        elif gsValues[0] > 700 and gsValues[1] > 700 and gsValues[2] > 700:
            # Fallback: All sensors see white - turn in direction of orientation error
            error += 1
            if error >= 8:  # Reduced from 12 for faster response
                if turn_direction is None:
                    if orientation_err > 0:
                        turn_direction = 'left'
                    else:
                        turn_direction = 'right'

                if turn_direction == 'left':
                    print("All sensors detect white - turning left to search for line")
                    leftSpeed = -0.5  # Reduced turning speed
                    rightSpeed = 0.5
                else:
                    print("All sensors detect white - turning right to search for line")
                    leftSpeed = 0.5
                    rightSpeed = -0.5

                if error >= 20:  # Reset after trying
                    error = 0
                    turn_direction = None

        else:
            # Default: move towards waypoint with basic control
            if orientation_err > 0.1:
                leftSpeed = MAX_SPEED * 0.3
                rightSpeed = MAX_SPEED * 0.6
            elif orientation_err < -0.1:
                leftSpeed = MAX_SPEED * 0.6
                rightSpeed = MAX_SPEED * 0.3
            else:
                leftSpeed = MAX_SPEED * 0.5
                rightSpeed = MAX_SPEED * 0.5

            turn_direction = None
            error = 0
            PID_turn = False
            PID_turn2 = False

    # Clamp speeds
    leftSpeed = max(-MAX_SPEED, min(MAX_SPEED, leftSpeed))
    rightSpeed = max(-MAX_SPEED, min(MAX_SPEED, rightSpeed))

    # Set motor speeds
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    # Debug output (reduce frequency for performance)
    print_timer += delta_t
    if print_timer >= print_interval:
        print(f"Position: ({x:.3f}, {y:.3f}), Heading: {phi:.2f}")
        print(f"Waypoint {current_waypoint_index + 1}/{len(waypoints)}: ({xd:.3f}, {yd:.3f})")
        print(f"Distance to waypoint: {position_err:.3f}")
        print(f"Line following state: {line_following_state}")
        print(f"Ground sensors: {gsValues}")
        print("---")
        print(F" hoekfout= {orientation_err}")
        print_timer = 0.0