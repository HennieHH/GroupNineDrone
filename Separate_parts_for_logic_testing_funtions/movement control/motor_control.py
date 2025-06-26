from machine import Pin, PWM

### this motorcontrol script is a derivative of the original initialiation.py, combined with own code for the L298N ###

"""
* PWM works with a minimal of 20% because of inertia
"""

class MotorController: # init the controller and each pin
    def __init__(self):
        """Simple dual motor controller for L298N"""
        # Motor 1 pins
        self.left_motor_forward = PWM(Pin(26), freq=100)
        self.left_motor_back = PWM(Pin(27), freq=100)
        
        # Motor 2 pins  
        self.right_motor_forward = PWM(Pin(14), freq=100)
        self.right_motor_back = PWM(Pin(12), freq=100)
        
        self.stop_all()
    
    def stop_all(self):
        """Stop all motors"""
        self.left_motor_forward.duty(0)
        self.left_motor_back.duty(0)
        self.right_motor_forward.duty(0)
        self.right_motor_back.duty(0)
    
    def set_motor_speeds(self, left_speed, right_speed):
        """Set motor speeds with direction control"""
        # Stop all motors first
        self.stop_all()
        
        # Left motor direction and speed
        if left_speed > 0:
            self.left_motor_forward.duty(abs(left_speed))
        elif left_speed < 0:
            self.left_motor_back.duty(abs(left_speed))
        
        # Right motor direction and speed  
        if right_speed > 0:
            self.right_motor_forward.duty(abs(right_speed))
        elif right_speed < 0:
            self.right_motor_back.duty(abs(right_speed))

motors = MotorController()
base_speed = 1023 # speed control from 0 - 1023

# Line following variables (you'll need to define these elsewhere)
# force_follow = False
# centered_on_line = False
# line_right = False
# line_left = False
# line_following_state = 'forward'
# line_counter = 0
# LINE_COUNTER_MAX = 10

def line_following_control(force_follow, centered_on_line, line_right, line_left, line_following_state, line_counter, LINE_COUNTER_MAX):
    """Line following control logic"""
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
        leftSpeed = int(1.0 * base_speed)
        rightSpeed = int(0.2 * base_speed)

        if line_counter >= LINE_COUNTER_MAX:
            line_following_state = 'forward'

    elif line_following_state == 'turn_left':
        leftSpeed = int(0.2 * base_speed)
        rightSpeed = int(1.0 * base_speed)

        if line_counter >= LINE_COUNTER_MAX:
            line_following_state = 'forward'

    line_counter += 1
    return leftSpeed, rightSpeed, line_following_state, line_counter

# Example usage:
# left_speed, right_speed, line_following_state, line_counter = line_following_control(
#     force_follow, centered_on_line, line_right, line_left, 
#     line_following_state, line_counter, LINE_COUNTER_MAX
# )
# motors.set_motor_speeds(left_speed, right_speed)