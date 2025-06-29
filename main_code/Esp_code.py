 # -------------------------import--------------------
from machine import I2C, Pin, ADC, PWM  # Import Pin and UART classes from the machine module for hardware control
import time  # Import time module for sleep and time tracking
import heapq  # Import heapq module for priority queue in Dijkstra’s algorithm
import math  # Import math module for mathematical functions
import VL53L0X as VL
from Motor_control_encoders import MotorController
# -------------------------(hardware setup)-------------------------
encoderA_L = Pin(18, Pin.IN)
encoderB_L = Pin(19, Pin.IN)

encoderA_R = Pin(17, Pin.IN)
encoderB_R = Pin(23, Pin.IN)

i2c = I2C(0, scl=Pin(33), sda=Pin(25))

ToF_sensor = VL.VL53L0X(i2c)
distance_buffer = []

# Weights for the last 5 samples (newest to oldest)
ToF_weights = [0.393, 0.239, 0.146, 0.089, 0.055, 0.034, 0.020, 0.012, 0.007, 0.004]
total_ToF_weight = sum(ToF_weights)

line_sensor_pins = [36, 39, 34, 35, 32]
weights = [0, 1000, 2000, 3000, 4000]
min_vals = [1349, 1407, 1563, 1083, 810]
max_vals = [2143, 2274, 2559, 1892, 1454]

# Pin configuration for motors (adjust these to match your wiring)
IN1_PIN_A = 14  # left motor
IN2_PIN_A = 13

IN1_PIN_B = 21 #right motor
IN2_PIN_B = 22

# Create motor controller instance
motorA = MotorController(IN1_PIN_A, IN2_PIN_A)
motorB = MotorController(IN1_PIN_B, IN2_PIN_B)


def init_sensors(pins=None):
    """
    Initialize ADC sensors and return a list of sensor objects.
    Each sensor is configured for 11 dB attenuation (full input range).
    """
    if pins is None:
        pins = line_sensor_pins

    line_sensors = []
    for pin in pins:
        adc = ADC(Pin(pin))  # Create ADC object on the specified GPIO pin
        adc.atten(ADC.ATTN_11DB)  # Set attenuation for full voltage range (~0-3.3V)
        line_sensors.append(adc)

    return line_sensors


line_sensors = init_sensors()

# -------------------------(dijkstra)------------------------------

"""
    Purpose:
        Return a hardcoded grid representation of the environment, where 0 indicates free space and 1 indicates an obstacle.
    Inputs:
        None.
    Outputs:
        A 2D list of integers (15 rows × 17 columns) defining the static grid layout.
    """


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
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0], # Row 13: mostly blocked with some free at col 10, 12, 14, 16
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0]  # Row 14: same as row 13
    ]


"""
    Purpose:
        Generate a cost grid matching the dimensions of the static grid, assigning a uniform traversal cost of 1 to every cell.
    Inputs:
        None.
    Outputs:
        A 2D list of integers (15 rows × 17 columns) where each entry is 1.
    """


def create_costs():
    return [[1 for _ in range(17)] for _ in range(15)]  # List comprehension to create 15 rows × 17 cols of cost=1


"""
    Purpose:
        Perform Dijkstra’s shortest-path search on a grid to find the least-cost path from 'start' to 'goal'.
    Inputs:
        grid: 2D list of ints where 0 = free cell, 1 = obstacle.
        costs: 2D list of ints representing traversal cost for each corresponding grid cell.
        start: Tuple (row, col) indicating starting cell coordinates.
        goal: Tuple (row, col) indicating target cell coordinates.
    Outputs:
        A list of (row, col) tuples representing the sequence of grid cells from start to goal (inclusive). If no path is found, returns a list containing only the start cell.
    """


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

"""
    Purpose:
        Convert discrete grid coordinates (row, col) into continuous world coordinates (x, y) for the robot.
        Handles two rows (2 and 12) with manually defined mappings; all other rows use linear interpolation based on origin and spacing.
    Inputs:
        row: Integer row index (0 ≤ row < 15).
        col: Integer column index (0 ≤ col < 17).
    Outputs:
        Tuple (x, y) giving world coordinates corresponding to the center of the specified grid cell.
    """


def grid_to_world(row, col):
    manual_coords = {
        0: {0: 0.000000, 2: 0.150000, 4: 0.300000, 6: 0.459000},
        1: {0: 0.000000, 2: 0.153000, 4: 0.306000, 6: 0.459000},
        2: {
            0: 0.000000, 1: 0.076500, 2: 0.153000, 3: 0.229500, 4: 0.306000,
            5: 0.382500, 6: 0.455000, 7: 0.604000, 8: 0.745000, 9: 0.838125,
            10: 0.931250, 11: 1.024375, 12: 1.117500, 13: 1.210625,
            14: 1.303750, 15: 1.396875, 16: 1.490000
        },
        12: {
            0: 0.000000, 1: 0.076500, 2: 0.153000, 3: 0.229500, 4: 0.306000,
            5: 0.382500, 6: 0.459000, 7: 0.535500, 8: 0.745000, 9: 0.890000,
            10: 1.031000, 11: 1.107500, 12: 1.184000, 13: 1.260500,
            14: 1.337000, 15: 1.413500, 16: 1.490000
        },
        13: {10: 1.031000, 12: 1.184000, 14: 1.337000, 16: 1.490000},
        14: {10: 1.031000, 12: 1.184000, 14: 1.337000, 16: 1.490000}
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
    dx_per_col = 0.093125
    x = col * dx_per_col
    y = manual_ys.get(row, row * 0.085)  # fallback y if missing
    return (x, y)
# -------------------------(world to grid)------------

"""
    Purpose:
        Convert continuous world coordinates (x, y) back into discrete grid coordinates (row, col).
        Applies special handling for rows 2 and 12 using manual mappings; otherwise, inverts the linear transformation used in grid_to_world.
    Inputs:
        x: Float world x-coordinate.
        y: Float world y-coordinate.
    Outputs:
        Tuple (row, col) indicating the nearest grid cell indices.
    """


def world_to_grid(x, y):
    manual_ys = {
        0: 0.000000, 1: 0.115000, 2: 0.230000, 3: 0.310000,
        4: 0.375000, 5: 0.448000, 6: 0.520000, 7: 0.595000,
        8: 0.670000, 9: 0.750000, 10: 0.820000, 11: 0.900000,
        12: 0.960000, 13: 1.075000, 14: 1.190000
    }

    manual_coords = {
        0: {0: 0.000000, 2: 0.150000, 4: 0.300000, 6: 0.459000},
        1: {0: 0.000000, 2: 0.153000, 4: 0.306000, 6: 0.459000},
        2: {
            0: 0.000000, 1: 0.076500, 2: 0.153000, 3: 0.229500, 4: 0.306000,
            5: 0.382500, 6: 0.455000, 7: 0.604000, 8: 0.745000, 9: 0.838125,
            10: 0.931250, 11: 1.024375, 12: 1.117500, 13: 1.210625,
            14: 1.303750, 15: 1.396875, 16: 1.490000
        },
        12: {
            0: 0.000000, 1: 0.076500, 2: 0.153000, 3: 0.229500, 4: 0.306000,
            5: 0.382500, 6: 0.459000, 7: 0.535500, 8: 0.745000, 9: 0.890000,
            10: 1.031000, 11: 1.107500, 12: 1.184000, 13: 1.260500,
            14: 1.337000, 15: 1.413500, 16: 1.490000
        },
        13: {10: 1.031000, 12: 1.184000, 14: 1.337000, 16: 1.490000},
        14: {10: 1.031000, 12: 1.184000, 14: 1.337000, 16: 1.490000}
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
    dx_per_col = 0.093125
    x_origin = 0.000000
    col = round((x - x_origin) / dx_per_col)

    # Match closest y row from table
    row = min(manual_ys, key=lambda r: abs(y - manual_ys[r]))
    return (row, col)

# --------------------position correction-----------------------

"""
    Purpose:
        If the robot is centered on a grid line (horizontal or vertical), "snap" its x or y coordinate to the exact grid-aligned value.
    Inputs:
        x: Current float x-coordinate of the robot in world frame.
        y: Current float y-coordinate of the robot in world frame.
        robot_time: Float representing the current timestamp (seconds).
        last_snap_time: Float representing the timestamp of the last snap correction.
    Outputs:
        A tuple (corrected_x, corrected_y, new_last_snap_time):
          - corrected_x: Possibly adjusted x-coordinate (snapped if appropriate).
          - corrected_y: Possibly adjusted y-coordinate.
          - new_last_snap_time: Updated last snap timestamp (set to robot_time if a snap occurred; otherwise unchanged).
    Side Effects:
        Prints a message if a snap occurs, indicating the coordinate adjustment.
    """


def correct_position_on_line(x, y, robot_time, last_snap_time, snap_cooldown=1):
    horizontal_rows = {2, 5, 7, 9, 12}  # Set of rows considered “lines” for snapping y
    vertical_cols = {0, 8, 16}  # Set of columns considered “lines” for snapping x

    if robot_time - last_snap_time < snap_cooldown:  # Check if cooldown period has not elapsed
        return x, y, last_snap_time  # Skip snapping if too soon since last snap

    row, col = world_to_grid(x, y)  # Convert current world coords to nearest grid cell
    corrected_x, corrected_y = grid_to_world(row, col)  # Compute exact world coords of that cell center
    snap_x = col in vertical_cols  # Determine if column is a vertical line for snapping x
    snap_y = row in horizontal_rows  # Determine if row is a horizontal line for snapping y
    dx = abs(x - corrected_x) if snap_x else float('inf')  # Compute difference in x if snapping possible
    dy = abs(y - corrected_y) if snap_y else float('inf')  # Compute difference in y if snapping possible
    if dx < dy:  # If x difference is smaller, snap x
        print(f"Snapping x from {x:.3f} → {corrected_x:.3f} (col {col})")
        return corrected_x, y, robot_time  # Return snapped x, unchanged y, update snap time
    elif dy < dx:  # If y difference is smaller, snap y
        print(f"Snapping y from {y:.3f} → {corrected_y:.3f} (row {row})")
        return x, corrected_y, robot_time  # Return unchanged x, snapped y, update snap time
    else:
        return x, y, last_snap_time  # No snapping if differences equal or neither line


# --------------------generate path and waypoints-----------------------

"""
    Purpose:
        Compute a sequence of world-coordinate waypoints between a specified start and goal position using Dijkstra’s algorithm on a grid.
        If custom_grid is provided, use that grid to account for dynamic obstacles; otherwise, use the default static grid.
    Inputs:
        start_pos: Tuple (x, y) of the robot’s starting world coordinates.
        goal_pos: Tuple (x, y) of the robot’s goal world coordinates.
        custom_grid: Optional 2D list representing a grid with dynamic obstacles marked (1) and free space (0). If None, uses create_grid().
    Outputs:
        A list of (x, y) tuples representing the world-coordinate waypoints along the computed path, including start and goal.
    Side Effects:
        Prints the intermediate grid-based path (list of (row, col) tuples) via `print(path)`.
    """


def generate_path_waypoints(start_pos, goal_pos, custom_grid=None):
    grid = custom_grid if custom_grid else create_grid()  # Use custom grid if provided, else default
    costs = create_costs()  # Get uniform cost grid
    start_grid = world_to_grid(*start_pos)  # Convert start world coords to grid cell
    goal_grid = world_to_grid(*goal_pos)  # Convert goal world coords to grid cell
    path = dijkstra(grid, costs, start_grid, goal_grid)  # Compute grid-based path
    print(path)  # Debug: print list of (row, col) cells
    return [grid_to_world(*step) for step in path]  # Convert each grid step back to world coords


# -------------------------(obstacle detection and replanning)-------------------------

"""
    Purpose:
        Create a new grid identical to the static grid but with one additional cell marked as an obstacle.
    Inputs:
        blocked_cell: Tuple (row, col) indicating which grid cell to mark as blocked (value 1).
    Outputs:
        A 2D list (new grid) where the specified cell has been set to 1 (obstacle), and all other cells match create_grid().
    """


def create_dynamic_grid_with_obstacle(blocked_cell):
    dynamic_grid = [row[:] for row in create_grid()]  # Deep copy of static grid so we don’t modify original
    if 0 <= blocked_cell[0] < len(dynamic_grid) and 0 <= blocked_cell[1] < len(dynamic_grid[0]):  # Check bounds
        dynamic_grid[blocked_cell[0]][blocked_cell[1]] = 1  # Mark specified cell as obstacle
    return dynamic_grid  # Return new grid with dynamic obstacle


"""
    Purpose:
        Determine if there is an obstacle directly in front of the robot using proximity sensor readings,
        and compute the approximate grid cell location of that obstacle.
    Inputs:
        x: Current float x-coordinate of the robot in world frame.
        y: Current float y-coordinate of the robot in world frame.
        phi: Current robot heading (radians).

    Outputs:
        A tuple (obstacle_detected, obstacle_cell):
          - obstacle_detected: Boolean indicating if an obstacle is detected (True) or not (False).
          - obstacle_cell: Tuple (row, col) of the estimated obstacle location in grid coordinates if detected; otherwise None.
    """


def detect_obstacle_ahead(x, y, phi, distance):
    obstacle_threshold = 20.0  # Set threshold value for proximity detection
    # Check front-facing sensors (ps0 and ps7)
    obstacle_detected = distance < obstacle_threshold

    if obstacle_detected:  # If either front sensor reading exceeds threshold
        obstacle_distance = 20  # Approximate distance (centimeters) to assumed obstacle
        obstacle_x = x + obstacle_distance * math.cos(phi)  # Estimate obstacle x in world frame
        obstacle_y = y + obstacle_distance * math.sin(phi)  # Estimate obstacle y in world frame

        obstacle_cell = world_to_grid(obstacle_x, obstacle_y)  # Convert estimated world coords to grid cell
        return True, obstacle_cell  # Return that obstacle is detected and its grid cell

    return False, None  # No obstacle detected


# -------------------------(message processing function)-------------------------

"""
    Purpose:
        Parse an incoming UART message string, update global pose, ground sensor, or proximity sensor values accordingly.
        Messages are expected in the formats:
          - "POS:x,y,phi"
          - "GS:gs0,gs1,gs2"
          - "PS:ps0,ps1,...,ps7"
    Inputs:
        msg: String received (with newline stripped) containing one of the above prefixes and comma-separated values.
    Outputs:
        None (updates global variables x, y, phi, ,  as side effects).
    Side Effects:
        Prints warning messages if the message format is incorrect or contains non-float values.
    """
# -------------------------(parameters)-------------------------
# led_green.value(0)  # Set green led to off
waypoints = []  # List to hold computed world-coordinate waypoints
waypoints_generated = False  # Flag indicating whether waypoints have been generated
current_waypoint_index = 0  # Index of the current waypoint being pursued

# Robot motion state parameters
R = 0.0336    # radius of the wheels of the e-puck robot: 33.6mm
D = 0.097    # inside distance between the wheels of the e-puck robot: 97mm
delta_t = 0.016  # Loop time interval estimate (seconds)
x, y = 0, 0  # Initialize robot’s current world x, y position
phi = 0  # Initialize robot’s heading (radians)
MAX_SPEED = 80  # Maximum wheel angular speed (rad/s)
leftSpeed = 0
rightSpeed = 0
pulse_l = 0
last_A_l = encoderA_L.value()
pulse_r = 0
last_A_r = encoderA_R.value()
pulses_per_turn = 1939
old_l = pulse_l
old_r = pulse_r

filtered_ToF_distance = 0
# Ground sensor data initialization


# Proximity sensor data initialization (8 sensors)


# Pathfinding target positions
start_position = (0, 0)  # Start world coordinates for pathfinding
goal_position = (1.490000, 1.190000)  # Goal world coordinates for pathfinding
waypoint_reached_threshold = 0.05  # Distance threshold (meters) to consider waypoint reached

# PID control variables
e_acc = 0  # Accumulated integral of orientation error
e_prev = 0  # Previous orientation error

# Line following variables
line_following_state = 'forward'  # Current state of line-following state machine
line_counter = 0  # Counter used in turn states to time transitions
LINE_COUNTER_MAX = 5  # Maximum count before returning to forward state
force_line_following = True  # Flag to force line-follow control even if orientation is off
line_follow_block_duration = 2.0  # Seconds to block line following after using PID
line_follow_start_time = 0.0  # Timestamp when PID last used to block line following
current_time = 0.0  # Simulated current time (seconds) for control loop

# Position correction variables
snap_cooldown = 1.0  # Seconds to wait between successive snap corrections
last_snap_time = 0.0  # Timestamp of last snap

# Obstacle detection and replanning variables
last_replan_time = 0.0  # Timestamp of last replanning event
replan_cooldown = 3.0  # Minimum seconds between replanning attempts


#----------------------Line_sensor--------------------------
def determine_line_visibility(normalized_values):
    """
    Determine line visibility for each sensor position.
    Returns boolean values indicating if line is detected.
    Based on normalized sensor values (0-1000 scale).
    """
    # Line visibility thresholds - adjust these values based on your testing
    line_far_left   = normalized_values[0] > 900    # Sensor 1 (links)
    line_left       = normalized_values[1] > 900 # Sensor 2 (links-midden)
    line_center     = normalized_values[2] > 900 # Sensor 3 (midden)
    line_right      = normalized_values[3] > 900 # Sensor 4 (rechts-midden)
    line_far_right  = normalized_values[4] > 900 # Sensor 5 (rechts)

    return line_far_left, line_left, line_center, line_right, line_far_right

# -------------------------Line Following------------------------------

"""
    Purpose:
        Determine motor speed commands to follow a line based on ground sensor readings (gsValues).
        Applies stronger corrections when off-center or when explicitly forced to keep following.
    Inputs:
        gsValues: List of three floats [gs0, gs1, gs2] from left, center, and right ground sensors.
        force_follow: Boolean flag indicating whether to force line-following even if orientation is off.
    Outputs:
        A tuple (leftSpeed, rightSpeed) representing the wheel speed commands to correct toward the line.
    Side Effects:
        Modifies global 'line_following_state' and 'line_counter' to implement a simple state machine.
    """


def line_following_control(normalized_values, force_follow=False):
    global line_following_state, line_counter  # Use and update global state variables

    # Determine line visibility based on thresholded sensor readings
    line_far_left   = normalized_values[0] > 900    # Sensor 1 (links)
    line_left       = normalized_values[1] > 900 # Sensor 2 (links-midden)
    line_center     = normalized_values[2] > 900 # Sensor 3 (midden)
    line_right      = normalized_values[3] > 900 # Sensor 4 (rechts-midden)
    line_far_right  = normalized_values[4] > 900 # Sensor 5 (rechts)
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
        rightSpeed = 0.45 * base_speed  # Right wheel slower
        if line_counter >= LINE_COUNTER_MAX:  # After a few iterations, return to forward
            line_following_state = 'forward'
    elif line_following_state == 'turn_left':  # If in turn left state
        leftSpeed = 0.45 * base_speed  # Left wheel slower
        rightSpeed = 1.0 * base_speed  # Right wheel faster
        if line_counter >= LINE_COUNTER_MAX:  # After enough counts, switch back
            line_following_state = 'forward'
    elif line_following_state == 'turn_far_left':
        leftSpeed = 0.45 * base_speed  # Left wheel slower
        rightSpeed = 1.1 * base_speed  # Right wheel faster
        if line_counter >= LINE_COUNTER_MAX:
            line_following_state = 'forward'
    elif line_following_state == 'turn_far_right':
        leftSpeed = 1.1 * base_speed
        rightSpeed = 0.45 * base_speed
        if line_counter >= LINE_COUNTER_MAX:
            line_following_state = 'forward'

    line_counter += 1  # Increment counter each call
    return leftSpeed, rightSpeed  # Return computed motor speeds


# ------------all sensor values-----------------------
def normalize(val, min_val, max_val):
    """
    Normalize a raw ADC value to a 0-1000 scale.
    Values below min_val become 0, above max_val become 1000.
    """
    # Avoid division by zero
    if max_val - min_val == 0:
        return 0
    # Linear interpolation to 0..1000
    y = (val - min_val) * 1000 // (max_val - min_val)
    # Clamp result to [0, 1000]
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
    total = sum(norm)
    # Calculate line position as weighted average
    position = sum(weights[i] * norm[i] for i in range(len(line_sensors))) // total if total > 0 else -1

    return {
        'raw': raw,
        'normalized': norm,
        'position': position
    }


# ---------------------encode--------------
def handle_encoder_left(pin):
    global pulse_l, last_A_l
    A_l = encoderA_L.value()
    B_l = encoderB_L.value()
    if last_A_l == 0 and A_l == 1:
        if B_l == 0:
            pulse_l += 1
        else:
            pulse_l -= 1
    last_A_l = A_l


encoderA_L.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=handle_encoder_left)


def handle_encoder_right(pin):
    global pulse_r, last_A_r
    A_r = encoderA_R.value()
    B_r = encoderB_R.value()
    if last_A_r == 0 and A_r == 1:
        if B_r == 0:
            pulse_r += 1
        else:
            pulse_r -= 1
    last_A_r = A_r


encoderA_R.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=handle_encoder_right)


# -------------position------------------
def get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    # Calculate the change in angular position of the wheels:
    ang_diff_l = 2 * math.pi * (encoderValues[0] - oldEncoderValues[0]) / pulses_per_turn
    ang_diff_r = 2 * math.pi * (encoderValues[1] - oldEncoderValues[1]) / pulses_per_turn

    # Calculate the angular speeds:
    wl = ang_diff_l / delta_t
    wr = ang_diff_r / delta_t

    return wl, wr


def get_robot_speeds(wl, wr, R, D):
    u = R / 2.0 * (wr + wl)  # Linear velocity (average of both wheels)
    w = R / D * (wr - wl)  # Angular velocity (difference of wheel speeds)
    return u, w  # Return linear and angular speeds


def get_robot_pose(u, w, x, y, phi, delta_t):
    delta_phi = w * delta_t  # Change in orientation
    phi += delta_phi  # Update orientation

    # Wrap angle to [-π, π]
    if phi >= math.pi:
        phi = phi - 2 * math.pi
    elif phi < -math.pi:
        phi = phi + 2 * math.pi

    delta_x = u * math.cos(phi) * delta_t  # Change in x based on u, φ
    delta_y = u * math.sin(phi) * delta_t  # Change in y based on u, φ
    x += delta_x  # Update x
    y += delta_y  # Update y

    return x, y, phi  # Return updated pose


# -------------------------PID------------------------------

"""
    Purpose:
        Compute the distance and orientation error between the robot's current pose and a desired waypoint.
    Inputs:
        xd: Desired x-coordinate (float).
        yd: Desired y-coordinate (float).
        x: Current x-coordinate of the robot (float).
        y: Current y-coordinate of the robot (float).
        phi: Current orientation of the robot (radians).
    Outputs:
        Tuple (dist_err, phi_err):
          - dist_err: Euclidean distance between current position (x, y) and desired position (xd, yd).
          - phi_err: Signed angular difference between current heading 'phi' and the desired heading toward (xd, yd),
                     normalized to [-π, π].
    """


def get_pose_error(xd, yd, x, y, phi):
    x_err = xd - x  # Compute x error
    y_err = yd - y  # Compute y error
    dist_err = math.sqrt(x_err ** 2 + y_err ** 2)  # Euclidean distance error
    phi_d = math.atan2(y_err, x_err)  # Desired heading angle toward waypoint
    # Compute shortest angular difference: wrap between -π and π
    phi_err = math.atan2(math.sin(phi_d - phi), math.cos(phi_d - phi))
    return dist_err, phi_err  # Return distance and orientation errors


"""
    Purpose:
        Implement a basic PID controller to compute an angular velocity command given an orientation error.
    Inputs:
        e: Current orientation error (float).
        e_prev: Previous orientation error (float).
        e_acc: Accumulated integral of error (float).
        delta_t: Time step since last update (float, seconds).
        kp: Proportional gain constant (float, default 2.0).
        kd: Derivative gain constant (float, default 0.1).
        ki: Integral gain constant (float, default 0.00).
    Outputs:
        Tuple (output, new_e_prev, new_e_acc):
          - output: Control output (float) = P + I + D terms.
          - new_e_prev: Updated previous error (float), set to current error 'e'.
          - new_e_acc: Updated accumulated integral term (float).
    """


def pid_controller(e, e_prev, e_acc, delta_t, kp=2.0, kd=0.1, ki=0.00):
    P = kp * e  # Proportional term
    I = e_acc + ki * e * delta_t  # Integral term accumulation
    D = kd * (e - e_prev) / delta_t  # Derivative term
    output = P + I + D  # Sum PID terms
    e_prev = e  # Update previous error
    e_acc = I  # Update accumulated error
    return output, e_prev, e_acc  # Return control output and updated errors


"""
    Purpose:
        Convert desired linear velocity u_d and angular velocity w_d into individual wheel speed commands
        for a differential-drive robot, clamping to maximum wheel speed constraints.
    Inputs:
        u_d: Desired linear (forward) velocity (float).
        w_d: Desired angular (rotational) velocity (float).
        d: Distance between the wheels (float).
        r: Wheel radius (float).
    Outputs:
        Tuple (wl_d, wr_d):
          - wl_d: Angular speed (float) for the left wheel (rad/s), clamped to MAX_SPEED if necessary.
          - wr_d: Angular speed (float) for the right wheel (rad/s), clamped to MAX_SPEED if necessary.
    """


def wheel_speed_commands(u_d, w_d, d, r):
    wr_d = (2 * u_d + d * w_d) / (2 * r)  # Compute desired angular speed for right wheel
    wl_d = (2 * u_d - d * w_d) / (2 * r)  # Compute desired angular speed for left wheel

    abs_wr = abs(wr_d)  # Absolute value of right wheel speed
    abs_wl = abs(wl_d)  # Absolute value of left wheel speed

    if abs_wl > MAX_SPEED or abs_wr > MAX_SPEED:  # If either exceeds MAX_SPEED
        speed_ratio = abs_wr / abs_wl if abs_wl != 0 else 1  # Compute ratio to scale speeds
        if speed_ratio > 1:
            wr_d = math.copysign(MAX_SPEED, wr_d)  # Scale right wheel to MAX_SPEED
            wl_d = math.copysign(MAX_SPEED / speed_ratio, wl_d)  # Scale left wheel accordingly
        else:
            wl_d = math.copysign(MAX_SPEED, wl_d)  # Scale left wheel to MAX_SPEED
            wr_d = math.copysign(MAX_SPEED * speed_ratio, wr_d)  # Scale right wheel accordingly

    return wl_d, wr_d  # Return clamped wheel speeds


# -------------------------Update waypoints------------------------------

"""
    Purpose:
        Advance the global current_waypoint_index to the next waypoint. If the last waypoint is reached,
        signal completion.
    Inputs:
        None (relies on the global current_waypoint_index and waypoints list).
    Outputs:
        Boolean:
          - True if there is another waypoint to follow after increment.
          - False if all waypoints have been reached (mission complete).
    Side Effects:
        Increments the global 'current_waypoint_index'. Prints a completion message if no more waypoints remain.
    """


def update_current_waypoint():
    global current_waypoint_index  # Use and update global waypoint index
    current_waypoint_index += 1  # Advance to next waypoint index
    if current_waypoint_index >= len(waypoints):  # If index exceeds number of waypoints
        print("All waypoints reached! Mission complete.")  # Indicate mission complete
        return False  # Return False to signal no more waypoints
    return True  # Return True to indicate more waypoints remain


# -------------------------(loop)-------------------------
while True:  # Main control loop, runs indefinitely
    current_time += 0.02  # Increment simulated time by approximate loop interval

    # ----------- (Position) -----------
    new_l = pulse_l
    new_r = pulse_r

    wl, wr = get_wheels_speed([new_l, new_r], [old_l, old_r], pulses_per_turn, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)

    print("Position:", x, y, phi)

    # Save for next cycle
    old_l = new_l
    old_r = new_r

    # -----------ToF_sensor--------------
    raw_distance = ToF_sensor.read() - 25  # Apance = weighted_sum / ply offset correction - 25 mm
    distance_buffer.insert(0, raw_distance)  # Add new reading at the front
    # Limit buffer to last 10 readings
    if len(distance_buffer) > 10:
        distance_buffer.pop()
    # Compute weighted average if buffer has enough data
    if len(distance_buffer) == 10:
        weighted_sum = sum(distance_buffer[i] * ToF_weights[i] for i in range(10))
        filtered_ToF_distance = weighted_sum / total_ToF_weight
        print("Raw: {:>4} mm | Filtered (10-sample weighted): {:.2f} mm".format(raw_distance, filtered_ToF_distance))
    else:
        # Not enough data for full filter
        print("Raw: {:>4} mm | Filtered: N/A (collecting data...)".format(raw_distance))

        # ------------line_sensors------------------
    line_data = read_all_data(line_sensors)
    line_position = line_data['position']
    line_norm = line_data['normalized']
    norm_str = "  ".join(f"N{i + 1}:{line_data['normalized'][i]}" for i in range(len(line_sensors)))
    print(f"{norm_str}  |  Position: {line_data['position']}")

    line_far_left, line_left, line_center, line_right, line_far_right = determine_line_visibility(line_norm)

        # ----------- Position Correction -----------
    centered_on_line = (  # Determine if robot is centered on a line using ground sensors
            line_left and line_center and line_right
    )
    if centered_on_line:  # If robot is centered on line, attempt snapping
        old_x, old_y = x, y  # Save old coordinates for comparison
        x, y, last_snap_time = correct_position_on_line(x, y, current_time, last_snap_time)  # Snap if needed

        # Send corrected position to Webots if position was adjusted


    # ----------- Pathfinding -----------
    if not waypoints_generated:  # If waypoints have not yet been generated
        waypoints = generate_path_waypoints(start_position, goal_position)  # Compute initial path
        waypoints_generated = True  # Set flag to indicate waypoints are generated
        current_waypoint_index = 0  # Reset current waypoint index to first
        print("Generated waypoints:")
        for i, wp in enumerate(waypoints):  # Print each waypoint for debugging
            print(f"  {i}: {wp}")

    # ----------- Obstacle Detection and Replanning -----------
    if len(waypoints) > 0 and current_waypoint_index < len(waypoints) and len(
            distance_buffer) == 10:  # If there are remaining waypoints
        obstacle_detected, obstacle_cell = detect_obstacle_ahead(x, y, phi,
                                                                 filtered_ToF_distance)  # Check for obstacle ahead

        if obstacle_detected and (current_time - last_replan_time) > replan_cooldown:
            last_replan_time = current_time  # Update last replanning time
            print("Obstacle detected! Replanning...")

            # Block the detected obstacle cell on the grid
            print(f"Marking grid cell {obstacle_cell} as obstacle.")
            dynamic_grid = create_dynamic_grid_with_obstacle(obstacle_cell)  # Create grid with this obstacle

            # Replan path from current robot position
            new_waypoints = generate_path_waypoints((x, y), goal_position, custom_grid=dynamic_grid)  # Recompute path

            if len(new_waypoints) == 0:  # If no valid alternative path found
                print("No alternative path found!")
                motorA.stop()
                motorB.stop()
            else:
                waypoints = new_waypoints  # Update waypoints to new path
                current_waypoint_index = 0  # Reset waypoint index
                e_acc = 0  # Reset PID integral term
                e_prev = 0  # Reset PID previous error
                force_line_following = False  # Temporarily disable line-follow override
                line_follow_start_time = current_time  # Reset line-follow block timer
                print(f"New path with {len(waypoints)} waypoints created.")

    # ----------- Think -----------

    # Check if we have waypoints to follow
    if len(waypoints) == 0:  # If no waypoints exist
        print("No valid path found!")
        motorA.stop()
        motorB.stop()
    elif current_waypoint_index < len(waypoints):  # If there are still waypoints to pursue
        # Get current target waypoint
        xd, yd = waypoints[current_waypoint_index]  # Extract x, y of current waypoint

        # Calculate errors relative to current waypoint
        position_err, orientation_err = get_pose_error(xd, yd, x, y, phi)  # Compute distance and heading errors

        # Check if waypoint is reached within threshold
        if position_err < waypoint_reached_threshold:
            print(f"Waypoint {current_waypoint_index + 1} reached!")
            if not update_current_waypoint():  # Advance to next waypoint; if False returned, mission complete
                motorA.stop()
                motorB.stop()

            else:
                # Move to next waypoint, recalculate errors if any remain
                if current_waypoint_index < len(waypoints):
                    xd, yd = waypoints[current_waypoint_index]  # New waypoint coords
                    position_err, orientation_err = get_pose_error(xd, yd, x, y, phi)  # Recompute errors
                else:
                    motorA.stop()
                    motorB.stop()

        # Control decision logic for how to drive toward waypoint
        if current_waypoint_index < len(waypoints):
            # Re-enable line following if enough time has passed since last PID usage
            if not force_line_following and (current_time - line_follow_start_time > line_follow_block_duration):
                force_line_following = True  # Re-enable line-follow override
                print("Line-follow override re-enabled.")

            # Decision logic for control method
            if centered_on_line and force_line_following:
                # If centered on line and override is active, use line following
                leftSpeed, rightSpeed = line_following_control(line_norm, force_follow=True)

            elif abs(orientation_err) > 1.0 or (line_far_left and line_left and line_center and line_right and line_far_right):
                # If orientation error is large or all ground sensors detect off-line, use PID control
                u_d = 0.05  # Fixed small forward speed
                w_d, e_prev, e_acc = pid_controller(orientation_err, e_prev, e_acc, delta_t)  # Compute angular speed
                wl_d, wr_d = wheel_speed_commands(u_d, w_d, D, R)  # Convert to individual wheel speeds
                leftSpeed = wl_d  # Assign left wheel speed
                rightSpeed = wr_d  # Assign right wheel speed

                # Temporarily block line following after using PID
                force_line_following = False  # Disable line-follow override
                line_follow_start_time = current_time  # Reset block timer

            else:
                # Default: use line-following control
                leftSpeed, rightSpeed = line_following_control(line_norm)

    else:
        # All waypoints have been reached
        motorA.stop()
        motorB.stop()

    # ----------- Act (Send motor commands) -----------
    motorA.forward(leftSpeed)
    motorB.forward(rightSpeed)

    # Debug output at a reduced frequency (every 0.1 seconds)
    if current_waypoint_index < len(waypoints) and int(current_time * 10) % 10 == 0:
        print(f"Target: ({waypoints[current_waypoint_index][0]:.3f}, {waypoints[current_waypoint_index][1]:.3f})")
        print(f"Current: ({x:.3f}, {y:.3f}) | Speeds: L={leftSpeed:.3f}, R={rightSpeed:.3f}")

