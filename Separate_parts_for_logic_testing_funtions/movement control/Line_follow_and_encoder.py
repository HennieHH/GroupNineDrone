import time
from machine import Pin, PWM, ADC

#---line_sensor----
sensor_pins = [32, 35, 34, 39, 36]
line_sensors = []

for pin in sensor_pins:
    adc = ADC(Pin(pin))
    adc.atten(ADC.ATTN_11DB)
    line_sensors.append(adc)

min_vals = [1300, 1300, 1600, 1160, 1000]
max_vals = [3060, 3440, 3650, 2720, 2060]

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

# Tick counters
ticks1 = 0
ticks2 = 0

line_following_state = 'forward'  # Current state of line-following state machine
line_counter = 0  # Counter used in turn states to time transitions
LINE_COUNTER_MAX = 5  # Maximum count before returning to forward state
MAX_SPEED = 100
delta_t = 0.016

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
#---function_line_sensor------------
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


try:
    while True:
        current_time = time.time()

        line_data = read_all_data(line_sensors)
        line_norm = line_data['normalized']
        norm_str = "  ".join(f"N{i + 1}:{line_data['normalized'][i]}" for i in range(len(line_sensors)))
        print(f"{norm_str}")

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

        time.sleep(delta_t)

except KeyboardInterrupt:
    print("\nStopping motors...")
    stop_motors()
    print(f"Final results:")
    print(f"Motor 1 total ticks: {ticks1}")
    print(f"Motor 2 total ticks: {ticks2}")
    print("Test completed")
