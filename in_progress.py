import time
from machine import Pin, PWM, ADC
import heapq
import math

# initialization
# ---line_sensor----
sensor_pins = [32, 35, 34, 39, 36]
line_sensors = []

for pin in sensor_pins:
    adc = ADC(Pin(pin))
    adc.atten(ADC.ATTN_11DB)
    line_sensors.append(adc)

min_vals = [1300, 1300, 1600, 1160, 1000]
max_vals = [3060, 3440, 3650, 2720, 2060]

# -----motors----
# Motor 1 setup (left motor)
motor1_p1 = PWM(Pin(22), freq=100)
motor1_p2 = PWM(Pin(21), freq=100)

# Motor 2 setup (right motor)
motor2_p1 = PWM(Pin(14), freq=100)
motor2_p2 = PWM(Pin(13), freq=100)
# ---encoder----
# Encoder 1 setup (left encoder)
encoder1_a = Pin(19, Pin.IN, Pin.PULL_UP)
encoder1_b = Pin(18, Pin.IN, Pin.PULL_UP)

# Encoder 2 setup (right encoder)
encoder2_a = Pin(17, Pin.IN, Pin.PULL_UP)
encoder2_b = Pin(23, Pin.IN, Pin.PULL_UP)

line_following_state = 'forward'  # Current state of line-following state machine
line_counter = 0  # Counter used in turn states to time transitions
LINE_COUNTER_MAX = 5  # Maximum count before returning to forward state
MAX_SPEED = 100

waypoints = []  # List to hold computed world-coordinate waypoints
waypoints_generated = False  # Flag indicating whether waypoints have been generated
current_waypoint_index = 0  # Index of the current waypoint being pursued
x, y = 0, 0  # Initialize robot’s current world x, y position
phi = math.pi/2  # Initialize robot’s heading (radians)
start_position = (0, 0)  # Start world coordinates for pathfinding
goal_position = (-1.490000, 1.190000)  # Goal world coordinates for pathfinding
waypoint_reached_threshold = 0.05  # Distance threshold (meters) to consider waypoint reached

pulses_per_turn = 960
encoderValues = [0, 0]
oldEncoderValues = [0, 0]

x_old, y_old, phi_old = 0.0, 0.0, math.pi/2
R = 0.0336
D = 0.097

# Reset counters
ticks1 = 0
ticks2 = 0

start_time = time.time()
last_ticks1 = 0
last_ticks2 = 0

LINE_LOOP_DT   = 0.016   # 16 ms voor lijnvolgen
POSE_LOOP_DT   = 0.1     # 100 ms voor pose-update
delta_t  = POSE_LOOP_DT
last_pose_time = time.ticks_ms()

def create_grid():
    return [
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # Row 0: mixed free and blocked cells
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # Row 1: same pattern as row 0
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 2: all free cells
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 3: obstacles around a central corridor
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 4: same as row 3
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 5: obstacles then open corridor
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 6: obstacles with one gap at col 8
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 7: all free
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 8: similar obstacle layout as row 6
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 9: open until col 9 then blocked to col 15
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 10: same as row 8
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],  # Row 11: same as row 10
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 12: all free
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
        # Row 13: mostly blocked with some free at col 10, 12, 14, 16
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0]  # Row 14: same as row 13
    ]


def create_costs():
    return [[1 for _ in range(17)] for _ in range(15)]  # List comprehension to create 15 rows × 17 cols of cost=1


def dijkstra(grid, costs, start, goal):
    rows, cols = len(grid), len(grid[0])  # Determine grid dimensions
    visited = set()  # Set to track visited cells
    distance = {start: 0}  # Dictionary mapping cell → current shortest distance; initialize start at 0
    prev = {}  # Dictionary mapping neighbor → predecessor in shortest path
    queue = [(0, start)]  # Priority queue (min-heap) of (distance, cell)

    while queue:  # Loop until queue is empty
        current_dist, current = heapq.heappop(queue)  # Pop cell with smallest distance
        if current in visited:  # If already visited, skip
            continue
        visited.add(current)  # Mark current cell as visited
        if current == goal:  # If goal reached, break
            break
        r, c = current  # Unpack current cell row and col
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Explore neighbors (up, down, left, right)
            nr, nc = r + dr, c + dc  # Compute neighbor indices
            if 0 <= nr < rows and 0 <= nc < cols:  # Check bounds
                if grid[nr][nc] == 0:  # Only consider if neighbor is free cell
                    neighbor = (nr, nc)  # Tuple for neighbor cell
                    cost = costs[nr][nc]  # Cost to traverse neighbor
                    new_dist = current_dist + cost  # New tentative distance
                    if neighbor not in distance or new_dist < distance[neighbor]:
                        distance[neighbor] = new_dist  # Update shortest distance
                        prev[neighbor] = current  # Record predecessor
                        heapq.heappush(queue, (new_dist, neighbor))  # Add neighbor to queue

    path = []  # List to reconstruct path
    current = goal  # Start backtracking from goal
    while current in prev:  # While predecessor exists
        path.append(current)  # Add current to path
        current = prev[current]  # Step to predecessor
    path.append(start)  # Finally add start cell
    path.reverse()  # Reverse list so it goes start → goal
    return path  # Return list of (row, col) waypoints


# -------------------------(grid to world)-----


def grid_to_world(row, col):
    manual_coords = {
        0: {0: 0.000000, 2: 0.150000, 4: 0.300000, 6: 0.459000},
        1: {0: 0.000000, 2: 0.153000, 4: 0.306000, 6: 0.459000},
        2: {
            0: 0.000000, 1: -0.076500, 2: -0.153000, 3: -0.229500, 4: -0.306000,
            5: -0.382500, 6: -0.455000, 7: -0.604000, 8: -0.745000, 9: -0.838125,
            10: -0.931250, 11: -1.024375, 12: -1.117500, 13: -1.210625,
            14: -1.303750, 15: -1.396875, 16: -1.490000
        },
        12: {
            0: 0.000000, 1: -0.076500, 2: -0.153000, 3: -0.229500, 4: -0.306000,
            5: -0.382500, 6: -0.459000, 7: -0.535500, 8: -0.745000, 9: -0.890000,
            10: -1.031000, 11: -1.107500, 12: -1.184000, 13: -1.260500,
            14: -1.337000, 15: -1.413500, 16: -1.490000
        },
        13: {10: -1.031000, 12: -1.184000, 14: -1.337000, 16: -1.490000},
        14: {10: -1.031000, 12: -1.184000, 14: -1.337000, 16: -1.490000}
    }

    manual_ys = {
        0: 0.000000, 1: 0.115000, 2: 0.230000, 3: 0.310000,
        4: 0.375000, 5: 0.448000, 6: 0.520000, 7: 0.595000,
        8: 0.670000, 9: 0.750000, 10: 0.820000, 11: 0.900000,
        12: 0.960000, 13: 1.075000, 14: 1.190000
    }

    if row in manual_coords and col in manual_coords[row]:
        return (manual_coords[row][col], manual_ys[row])

    # Fallback: linear spacing for unmapped x
    dx_per_col = -0.093125
    x = col * dx_per_col
    y = manual_ys.get(row, row * 0.085)  # fallback y if missing
    return (x, y)


# -------------------------(world to grid)------------

def world_to_grid(x, y):
    manual_ys = {
        0: 0.000000, 1: 0.115000, 2: 0.230000, 3: 0.310000,
        4: 0.375000, 5: 0.448000, 6: 0.520000, 7: 0.595000,
        8: 0.670000, 9: 0.750000, 10: 0.820000, 11: 0.900000,
        12: 0.960000, 13: 1.075000, 14: 1.190000
    }

    manual_coords = {
        0: {0: 0.000000, 2: -0.150000, 4: -0.300000, 6: -0.459000},
        1: {0: 0.000000, 2: -0.153000, 4: -0.306000, 6: -0.459000},
        2: {
            0: 0.000000, 1: -0.076500, 2: -0.153000, 3: -0.229500, 4: -0.306000,
            5: -0.382500, 6: -0.455000, 7: -0.604000, 8: -0.745000, 9: -0.838125,
            10: -0.931250, 11: -1.024375, 12: -1.117500, 13: -1.210625,
            14: -1.303750, 15: -1.396875, 16: -1.490000
        },
        12: {
            0: 0.000000, 1: -0.076500, 2: -0.153000, 3: -0.229500, 4: -0.306000,
            5: -0.382500, 6: -0.459000, 7: -0.535500, 8: -0.745000, 9: -0.890000,
            10: -1.031000, 11: -1.107500, 12: -1.184000, 13: -1.260500,
            14: -1.337000, 15: -1.413500, 16: -1.490000
        },
        13: {10: -1.031000, 12: -1.184000, 14: -1.337000, 16: -1.490000},
        14: {10: -1.031000, 12: -1.184000, 14: -1.337000, 16: -1.490000}
    }

    dx = 0.0765 / 2
    dy = 0.0575  # slightly larger y-tolerance

    def find_manual_match():
        for row, y_ref in manual_ys.items():
            if abs(y - y_ref) <= dy:
                for col, x_ref in manual_coords.get(row, {}).items():
                    if abs(x - x_ref) <= dx:
                        return (row, col)
        return None

    result = find_manual_match()
    if result:
        return result

    # Fallback: approximate linear mapping
    dx_per_col = -0.093125
    x_origin = 0.000000
    col = round((x - x_origin) / dx_per_col)

    # Match closest y row from table
    row = min(manual_ys, key=lambda r: abs(y - manual_ys[r]))
    return (row, col)


# --------waypoints--------
def generate_path_waypoints(start_pos, goal_pos, custom_grid=None):
    grid = custom_grid if custom_grid else create_grid()  # Use custom grid if provided, else default
    costs = create_costs()  # Get uniform cost grid
    start_grid = world_to_grid(*start_pos)  # Convert start world coords to grid cell
    goal_grid = world_to_grid(*goal_pos)  # Convert goal world coords to grid cell
    path = dijkstra(grid, costs, start_grid, goal_grid)  # Compute grid-based path
    print(path)  # Debug: print list of (row, col) cells
    return [grid_to_world(*step) for step in path]  # Convert each grid step back to world coords


def update_current_waypoint():
    global current_waypoint_index  # Use and update global waypoint index
    current_waypoint_index += 1  # Advance to next waypoint index
    if current_waypoint_index >= len(waypoints):  # If index exceeds number of waypoints
        print("All waypoints reached! Mission complete.")  # Indicate mission complete
        return False  # Return False to signal no more waypoints
    return True  # Return True to indicate more waypoints remain


def encoder1_interrupt(pin):
    global ticks1
    if encoder1_b.value():
        ticks1 += 1
    else:
        ticks1 -= 1


def encoder2_interrupt(pin):
    global ticks2
    if encoder2_b.value():
        ticks2 += 1
    else:
        ticks2 -= 1


# Setup encoder interrupts
encoder1_a.irq(trigger=Pin.IRQ_RISING, handler=encoder1_interrupt)
encoder2_a.irq(trigger=Pin.IRQ_RISING, handler=encoder2_interrupt)


# ------------motors----------
def set_motor_speed(motor, speed):
    """Set motor speed: motor 1 or 2, speed -100 to 100"""
    speed = max(-100, min(100, speed))
    duty = int(abs(speed) * 10.23)  # Convert to 0-1023

    if motor == 1:
        if speed > 0:
            motor1_p1.duty(duty)
            motor1_p2.duty(0)
        elif speed < 0:
            motor1_p1.duty(0)
            motor1_p2.duty(duty)
        else:
            motor1_p1.duty(0)
            motor1_p2.duty(0)

    elif motor == 2:
        if speed > 0:
            motor2_p1.duty(duty)
            motor2_p2.duty(0)
        elif speed < 0:
            motor2_p1.duty(0)
            motor2_p2.duty(duty)
        else:
            motor2_p1.duty(0)
            motor2_p2.duty(0)


def stop_motors():
    """Stop both motors"""
    set_motor_speed(1, 0)
    set_motor_speed(2, 0)


# ---function_line_sensor------------
def normalize(val, min_val, max_val):
    if max_val - min_val == 0:
        return 0
    y = (val - min_val) * 1000 // (max_val - min_val)
    return max(0, min(1000, y))


def read_all_data(line_sensors):
    """
    Read data from all sensors, compute normalized readings, and calculate line position.
    Returns a dict with:
      - raw: list of raw ADC values
      - normalized: list of inverted normalized values (black line gives higher values)
      - position: weighted average position (0 = leftmost, 4000 = rightmost)
    """
    # Read raw sensor data
    raw = [s.read() for s in line_sensors]
    # Normalize and invert readings (line is dark, so invert to get higher values)
    norm = [1000 - normalize(raw[i], min_vals[i], max_vals[i]) for i in range(len(line_sensors))]

    return {
        'raw': raw,
        'normalized': norm,
    }


def line_following_control(normalized_values, force_follow=False):
    global line_following_state, line_counter  # Use and update global state variables

    # Determine line visibility based on thresholded sensor readings
    line_far_left = normalized_values[0] > 900  # Sensor 1 (links)
    line_left = normalized_values[1] > 900  # Sensor 2 (links-midden)
    line_center = normalized_values[2] > 900  # Sensor 3 (midden)
    line_right = normalized_values[3] > 900  # Sensor 4 (rechts-midden)
    line_far_right = normalized_values[4] > 900  # Sensor 5 (rechts)
    # True if front-center sensor off but sides detect line
    centered_on_line = (  # Determine if robot is centered on a line using ground sensors
            line_left and line_center and line_right
    )
    # Define a stronger base speed for line following (half of max)
    base_speed = MAX_SPEED * 0.8

    # If forced to follow or robot is centered, reset to forward state
    if force_follow or centered_on_line:
        line_following_state = 'forward'  # Set state to drive straight
        line_counter = 0  # Reset counter for turning durations

    # --- State machine logic ---
    if line_following_state == 'forward':  # If in forward state
        leftSpeed = base_speed  # Both wheels at base speed
        rightSpeed = base_speed
        if line_right and not line_left:  # If line detected more on right side
            line_following_state = 'turn_right'  # Switch to turning right state
            line_counter = 0
        elif line_left and not line_right:  # If line detected more on left side
            line_following_state = 'turn_left'  # Switch to turning left state
            line_counter = 0
        elif line_far_left and not line_center:
            line_following_state = 'turn_far_left'
            line_counter = 0
        elif line_far_right and not line_center:
            line_following_state = 'turn_far_right'
            line_counter = 0
    elif line_following_state == 'turn_right':  # If in turn right state
        leftSpeed = 1.0 * base_speed  # Left wheel faster
        rightSpeed = 0.65 * base_speed  # Right wheel slower
        if line_counter >= LINE_COUNTER_MAX:  # After a few iterations, return to forward
            line_following_state = 'forward'
    elif line_following_state == 'turn_left':  # If in turn left state
        leftSpeed = 0.65 * base_speed  # Left wheel slower
        rightSpeed = 1.0 * base_speed  # Right wheel faster
        if line_counter >= LINE_COUNTER_MAX:  # After enough counts, switch back
            line_following_state = 'forward'
    elif line_following_state == 'turn_far_left':
        leftSpeed = 0.65 * base_speed  # Left wheel slower
        rightSpeed = 1.1 * base_speed  # Right wheel faster
        if line_counter >= LINE_COUNTER_MAX:
            line_following_state = 'forward'
    elif line_following_state == 'turn_far_right':
        leftSpeed = 1.1 * base_speed
        rightSpeed = 0.65 * base_speed
        if line_counter >= LINE_COUNTER_MAX:
            line_following_state = 'forward'
    else:
        leftSpeed = 0
        rightSpeed = 0
    line_counter += 1  # Increment counter each call
    return leftSpeed, rightSpeed  # Return computed motor speeds


# Initialize - stop motors
stop_motors()

# print("80% Speed Test with Encoders")
# print("Motor 1: pins 5,4 | Encoder 1: pins 38,39")
# print("Motor 2: pins 6,7 | Encoder 2: pins 2,42")
# print()

# Reset counters
ticks1 = 0
ticks2 = 0

# print("Starting motors at 80% speed...")
# print("Time | Motor1_Ticks | Motor2_Ticks | Rate1 | Rate2")
# print("-" * 50)

start_time = time.time()
last_ticks1 = 0
last_ticks2 = 0


# -------------------- Wheel & Pose Estimation --------------------

def get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t):
    ang_diff_l = 2 * math.pi * (encoderValues[0] - oldEncoderValues[0]) / pulses_per_turn
    ang_diff_r = 2 * math.pi * (encoderValues[1] - oldEncoderValues[1]) / pulses_per_turn
    wl = ang_diff_l / delta_t
    wr = ang_diff_r / delta_t
    return wl, wr


def get_robot_speeds(wl, wr, R, D):
    u = R / 2.0 * (wr + wl)
    w = R / D * (wr - wl)
    return u, w


def get_robot_pose(u, w, x, y, phi, delta_t):
    delta_phi = w * delta_t
    phi += delta_phi
    if phi >= math.pi:
        phi -= 2 * math.pi
    elif phi < -math.pi:
        phi += 2 * math.pi
    delta_x = u * math.cos(phi) * delta_t
    delta_y = u * math.sin(phi) * delta_t
    x += delta_x
    y += delta_y
    return x, y, phi


try:
    while True:
        current_time = time.time()
        now = time.ticks_ms()

        line_data = read_all_data(line_sensors)
        line_norm = line_data['normalized']
        norm_str = "  ".join(f"N{i + 1}:{line_data['normalized'][i]}" for i in range(len(line_sensors)))
        # print(f"{norm_str}")

        if time.ticks_diff(now, last_pose_time) >= POSE_LOOP_DT * 1000:
            # lees encoders & update x,y,phi
            encoderValues[0] = ticks1
            encoderValues[1] = ticks2
            wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t)
            u, w = get_robot_speeds(wl, wr, R, D)
            x, y, phi = get_robot_pose(u, w, x_old, y_old, phi_old, delta_t)
            last_pose_time = now
            print(x, y)
            oldEncoderValues = encoderValues[:]
            x_old, y_old, phi_old = x, y, phi
        # print(x, y, phi)

        if not waypoints_generated:  # If waypoints have not yet been generated
            waypoints = generate_path_waypoints(start_position, goal_position)  # Compute initial path
            waypoints_generated = True  # Set flag to indicate waypoints are generated
            current_waypoint_index = 0  # Reset current waypoint index to first
            print("Generated waypoints:")
            for i, wp in enumerate(waypoints):  # Print each waypoint for debugging
                print(f"  {i}: {wp}")

        if len(waypoints) == 0:  # If no waypoints exist
            print("No valid path found!")
            stop_motors()
            continue
        elif current_waypoint_index < len(waypoints):  # If there are still waypoints to pursue
            # Get current target waypoint
            xd, yd = waypoints[current_waypoint_index]  # Extract x, y of current waypoint

        leftSpeed, rightSpeed = line_following_control(line_norm)

        set_motor_speed(1, leftSpeed)
        set_motor_speed(2, rightSpeed)

        elapsed = current_time - start_time

        # Calculate rates (ticks per second)
        rate1 = (ticks1 - last_ticks1) if elapsed > 0 else 0
        rate2 = (ticks2 - last_ticks2) if elapsed > 0 else 0

        print(f"{elapsed:4.0f}s |    {ticks1:6d}    |    {ticks2:6d}    | {rate1:4.0f}  | {rate2:4.0f}")

        last_ticks1 = ticks1
        last_ticks2 = ticks2

        time.sleep(LINE_LOOP_DT)


except KeyboardInterrupt:
    print("\nStopping motors...")
    stop_motors()
    print(f"Final results:")
    print(f"Motor 1 total ticks: {ticks1}")
    print(f"Motor 2 total ticks: {ticks2}")
    print("Test completed")
