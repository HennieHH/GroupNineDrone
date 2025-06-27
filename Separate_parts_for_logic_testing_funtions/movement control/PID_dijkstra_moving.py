import heapq
import math
import time
from machine import Pin, PWM, ADC
#--------------initialization---------------
#-----motors----
# Motor 1 setup (left motor)
motor1_p1 = PWM(Pin(22), freq=100)
motor1_p2 = PWM(Pin(21), freq=100)

# Motor 2 setup (right motor)
motor2_p1 = PWM(Pin(14), freq=100)
motor2_p2 = PWM(Pin(13), freq=100)
#---encoder----
# Encoder 1 setup (left encoder)
encoder1_a = Pin(19, Pin.IN, Pin.PULL_UP)
encoder1_b = Pin(18, Pin.IN, Pin.PULL_UP)

# Encoder 2 setup (right encoder)
encoder2_a = Pin(17, Pin.IN, Pin.PULL_UP)
encoder2_b = Pin(23, Pin.IN, Pin.PULL_UP)

#------------encoders-----------------
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

#------------motors----------
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
    leftSpeed = 0
    rightSpeed = 0
    return leftSpeed, rightSpeed

#-------------------grid--------------
def create_grid():
    return [
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # Row 0: mixed free and blocked cells
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # Row 1: same pattern as row 0
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Row 2: all free cells
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

#----------------dijkstra-----------------
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
            0: 0.000000, 1: -0.076500, 2: -0.153000, 3: -0.229500, 4:-0.306000,
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

#-----------------waypoints---------------------------

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

#-------------------pose------------

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

# -------------------------- PID Control --------------------------

def get_pose_error(xd, yd, x, y, phi):
    x_err = xd - x
    y_err = yd - y
    dist_err = math.sqrt(x_err ** 2 + y_err ** 2)
    phi_d = math.atan2(y_err, x_err)
    phi_err = math.atan2(math.sin(phi_d - phi), math.cos(phi_d - phi))
    return dist_err, phi_err

def pid_controller(e, e_prev, e_acc, delta_t, kp=2.0, kd=0.1, ki=0.00):
    P = kp * e
    I = e_acc + ki * e * delta_t
    D = kd * (e - e_prev) / delta_t
    output = P + I + D
    e_prev = e
    e_acc = I
    return output, e_prev, e_acc

def wheel_speed_commands(u_d, w_d, d, r):
    wr_d = (2 * u_d + d * w_d) / (2 * r)
    wl_d = (2 * u_d - d * w_d) / (2 * r)

    abs_wr = abs(wr_d)
    abs_wl = abs(wl_d)

    if abs_wl > MAX_SPEED or abs_wr > MAX_SPEED:
        speed_ratio = abs_wr / abs_wl if abs_wl != 0 else 1
        if speed_ratio > 1:
            wr_d = math.copysign(MAX_SPEED, wr_d)
            wl_d = math.copysign(MAX_SPEED / speed_ratio, wl_d)
        else:
            wl_d = math.copysign(MAX_SPEED, wl_d)
            wr_d = math.copysign(MAX_SPEED * speed_ratio, wr_d)

    return wl_d, wr_d

# -------------------- PID to Percent --------------------

def map_pid_to_percent(output, max_pid=6, min_percent=30, max_percent=80):
    output = max(-max_pid, min(max_pid, output))
    percent = ((output + max_pid) / (2 * max_pid)) * (max_percent - min_percent) + min_percent
    return percent

#---------------parameters-------------

waypoints = []  # List to hold computed world-coordinate waypoints
waypoints_generated = False  # Flag indicating whether waypoints have been generated
current_waypoint_index = 0  # Index of the current waypoint being pursued
x, y = 0, 0  # Initialize robot’s current world x, y position
phi = math.pi/2  # Initialize robot’s heading (radians)
start_position = (0, 0)  # Start world coordinates for pathfinding
goal_position = (-1.490000, 1.190000)  # Goal world coordinates for pathfinding
waypoint_reached_threshold = 0.05  # Distance threshold (meters) to consider waypoint reached
# Tick counters
ticks1 = 0
ticks2 = 0
MAX_SPEED = 100
leftSpeed, rightSpeed = stop_motors()
start_time = time.time()
last_ticks1 = 0
last_ticks2 = 0
pulses_per_turn = 960
delta_t = 0.16
encoderValues = [0, 0]
oldEncoderValues = [0, 0]

R = 0.0336
D = 0.097
u_d = 0

e_prev = 0
e_acc = 0

#-----------------loop----------------
try:
    while True:
        current_time = time.time()
        #------waypoints----------
        if not waypoints_generated:  # If waypoints have not yet been generated
            waypoints = generate_path_waypoints(start_position, goal_position)  # Compute initial path
            waypoints_generated = True  # Set flag to indicate waypoints are generated
            current_waypoint_index = 0  # Reset current waypoint index to first
            print("Generated waypoints:")
            for i, wp in enumerate(waypoints):  # Print each waypoint for debugging
                print(f"  {i}: {wp}")
#---position---
        encoderValues[0] = ticks1
        encoderValues[1] = ticks2
        wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t)
        u, w = get_robot_speeds(wl, wr, R, D)
        x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)
        oldEncoderValues = encoderValues
        print("encoder1:",encoderValues[0], "encoder2:",encoderValues[1])
#---waypoints_and_PID----
        if len(waypoints) == 0:  # If no waypoints exist
            print("No valid path found!")
            leftSpeed, rightSpeed = stop_motors()

        elif current_waypoint_index < len(waypoints):  # If there are still waypoints to pursue
        # Get current target waypoint
            xd, yd = waypoints[current_waypoint_index]  # Extract x, y of current waypoint
            position_err, orientation_err = get_pose_error(xd, yd, x, y, phi)

            if position_err < waypoint_reached_threshold:
                print(f"Waypoint {current_waypoint_index + 1} reached!")
                if not update_current_waypoint():  # Advance to next waypoint; if False returned, mission complete
                    leftSpeed, rightSpeed = stop_motors()

            w_d, e_prev, e_acc = pid_controller(orientation_err, e_prev, e_acc, delta_t)
            wl_d, wr_d = wheel_speed_commands(u_d, w_d, D, R)

            leftSpeed = map_pid_to_percent(wl_d)
            rightSpeed = map_pid_to_percent(wr_d)

        else:
            leftSpeed, rightSpeed = stop_motors()


        set_motor_speed(1, leftSpeed)
        set_motor_speed(2, rightSpeed)

except KeyboardInterrupt:
    print("\nStopping motors...")
    stop_motors()
    print(f"Final results:")
    print("Test completed")