import time
from machine import Pin, PWM

# Motor 1 setup (left motor)
motor1_p1 = PWM(Pin(22), freq=100)
motor1_p2 = PWM(Pin(21), freq=100)

# Motor 2 setup (right motor)
motor2_p1 = PWM(Pin(14), freq=100)
motor2_p2 = PWM(Pin(13), freq=100)

# Encoder 1 setup (left encoder)
encoder1_a = Pin(19, Pin.IN, Pin.PULL_UP)
encoder1_b = Pin(18, Pin.IN, Pin.PULL_UP)

# Encoder 2 setup (right encoder)
encoder2_a = Pin(17, Pin.IN, Pin.PULL_UP)
encoder2_b = Pin(23, Pin.IN, Pin.PULL_UP)

# Tick counters
ticks1 = 0
ticks2 = 0


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
    set_motor_speed(1, 99)
    set_motor_speed(2, 99)


# Initialize - stop motors
stop_motors()

print("80% Speed Test with Encoders")
print("Motor 1: pins 5,4 | Encoder 1: pins 38,39")
print("Motor 2: pins 6,7 | Encoder 2: pins 2,42")
print()

# Reset counters
ticks1 = 0
ticks2 = 0

print("Starting motors at 80% speed...")
print("Time | Motor1_Ticks | Motor2_Ticks | Rate1 | Rate2")
print("-" * 50)

# Set both motors to 80% speed
set_motor_speed(1, 80)
set_motor_speed(2, 80)

start_time = time.time()
last_ticks1 = 0
last_ticks2 = 0

try:
    while True:
        current_time = time.time()
        elapsed = current_time - start_time

        # Calculate rates (ticks per second)
        rate1 = (ticks1 - last_ticks1) if elapsed > 0 else 0
        rate2 = (ticks2 - last_ticks2) if elapsed > 0 else 0

        print(f"{elapsed:4.0f}s |    {ticks1:6d}    |    {ticks2:6d}    | {rate1:4.0f}  | {rate2:4.0f}")

        last_ticks1 = ticks1
        last_ticks2 = ticks2

        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping motors...")
    stop_motors()
    print(f"Final results:")
    print(f"Motor 1 total ticks: {ticks1}")
    print(f"Motor 2 total ticks: {ticks2}")
    print("Test completed")
