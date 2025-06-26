import heapq

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
            0: 0.000000, 1: -0.076500, -2: 0.153000, -3: 0.229500, -4: 0.306000,
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


waypoints = []  # List to hold computed world-coordinate waypoints
waypoints_generated = False  # Flag indicating whether waypoints have been generated
current_waypoint_index = 0  # Index of the current waypoint being pursued
x, y = 0, 0  # Initialize robot’s current world x, y position
phi = 0  # Initialize robot’s heading (radians)
start_position = (0, 0)  # Start world coordinates for pathfinding
goal_position = (-1.490000, 1.190000)  # Goal world coordinates for pathfinding
waypoint_reached_threshold = 0.05  # Distance threshold (meters) to consider waypoint reached



while True:
    if not waypoints_generated:  # If waypoints have not yet been generated
        waypoints = generate_path_waypoints(start_position, goal_position)  # Compute initial path
        waypoints_generated = True  # Set flag to indicate waypoints are generated
        current_waypoint_index = 0  # Reset current waypoint index to first
        print("Generated waypoints:")
        for i, wp in enumerate(waypoints):  # Print each waypoint for debugging
            print(f"  {i}: {wp}")

    if len(waypoints) == 0:  # If no waypoints exist
        print("No valid path found!")
        # motorA.stop()
        # motorB.stop()
    elif current_waypoint_index < len(waypoints):  # If there are still waypoints to pursue
        # Get current target waypoint
        xd, yd = waypoints[current_waypoint_index]  # Extract x, y of current waypoint