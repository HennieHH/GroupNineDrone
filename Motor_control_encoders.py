from machine import Pin, PWM
import time


class MotorController:
    def __init__(self, in1_pin, in2_pin, freq=100, min_speed=1):
        """
        Initialize L298N motor controller without enable pin

        Args:
            in1_pin: GPIO pin for IN1 (PWM direction control)
            in2_pin: GPIO pin for IN2 (PWM direction control)
            freq: PWM frequency in Hz (default 30000Hz)
            min_speed: Minimum speed percentage to overcome motor friction
        """
        self.in1 = PWM(Pin(in1_pin), freq=freq)
        self.in2 = PWM(Pin(in2_pin), freq=freq)
        self.min_speed = min_speed

        # Stop motor initially
        self.stop()

    def forward(self, speed=100):
        """
        Rotate motor forward

        Args:
            speed: Speed percentage (0-100)
        """
        if speed <= 0:
            self.stop()
            return
        elif speed > 100:
            speed = 100

        # Apply minimum speed threshold
        if speed < self.min_speed:
            actual_speed = self.min_speed
        else:
            actual_speed = speed

        # Set PWM duty cycle (0-1023 for ESP32)
        duty = int(actual_speed * 1023 / 100)

        # Forward: IN1 = PWM, IN2 = LOW
        self.in1.duty(duty)
        self.in2.duty(0)

        print(f"Motor forward at {speed}% speed (PWM: {actual_speed}%, Duty: {duty})")

    def backward(self, speed=100):
        """
        Rotate motor backward

        Args:
            speed: Speed percentage (0-100)
        """
        if speed <= 0:
            self.stop()
            return
        elif speed > 100:
            speed = 100

        # Apply minimum speed threshold
        if speed < self.min_speed:
            actual_speed = self.min_speed
        else:
            actual_speed = speed

        # Set PWM duty cycle
        duty = int(actual_speed * 1023 / 100)

        # Backward: IN1 = LOW, IN2 = PWM
        self.in1.duty(0)
        self.in2.duty(duty)

        print(f"Motor backward at {speed}% speed (PWM: {actual_speed}%, Duty: {duty})")

    def stop(self):
        """Stop the motor"""
        self.in1.duty(0)
        self.in2.duty(0)
        print("Motor stopped")

    def brake(self):
        """Brake the motor (both direction pins high)"""
        self.in1.duty(1023)
        self.in2.duty(1023)
        print("Motor braking")


# Pin configuration (adjust these to match your wiring)
IN1_PIN = 26  # GPIO26 to L298N IN1
IN2_PIN = 27  # GPIO27 to L298N IN2
# No ENA pin needed - tie ENA to VCC (high) on the L298N board

# Create motor controller instance
motor = MotorController(IN1_PIN, IN2_PIN)


def demo_sequence():
    """Demonstration sequence showing various motor controls"""
    print("Starting motor control demo...")

    # Forward at different speeds with more noticeable differences
    print("\n--- Forward Direction ---")
    for speed in [40, 60, 80, 100]:
        motor.forward(speed)
        time.sleep(3)  # Longer time to observe differences

    motor.stop()
    time.sleep(2)

    # Backward at different speeds
    print("\n--- Backward Direction ---")
    for speed in [40, 60, 80, 100]:
        motor.backward(speed)
        time.sleep(3)

    motor.stop()
    time.sleep(2)

    # Speed ramping up (slower ramp for better observation)
    print("\n--- Speed Ramp Up ---")
    for speed in range(30, 101, 15):
        motor.forward(speed)
        time.sleep(1)

    # Speed ramping down
    print("\n--- Speed Ramp Down ---")
    for speed in range(100, 29, -15):
        motor.forward(speed)
        time.sleep(1)

    motor.stop()
    print("\nDemo complete!")


def interactive_control():
    """Interactive motor control via serial console"""
    print("\n=== Interactive Motor Control ===")
    print("Commands:")
    print("  f<speed> - Forward (e.g., f50 for 50% speed)")
    print("  b<speed> - Backward (e.g., b75 for 75% speed)")
    print("  s - Stop")
    print("  brake - Brake")
    print("  demo - Run demonstration")
    print("  quit - Exit")
    print()

    while True:
        try:
            cmd = input("Enter command: ").strip().lower()

            if cmd == 'quit':
                motor.stop()
                print("Goodbye!")
                break
            elif cmd == 's':
                motor.stop()
            elif cmd == 'brake':
                motor.brake()
                time.sleep(0.5)  # Brief brake
                motor.stop()
            elif cmd == 'demo':
                demo_sequence()
            elif cmd.startswith('f'):
                try:
                    speed = int(cmd[1:]) if len(cmd) > 1 else 100
                    motor.forward(speed)
                except ValueError:
                    print("Invalid speed value")
            elif cmd.startswith('b'):
                try:
                    speed = int(cmd[1:]) if len(cmd) > 1 else 100
                    motor.backward(speed)
                except ValueError:
                    print("Invalid speed value")
            else:
                print("Unknown command")

        except KeyboardInterrupt:
            motor.stop()
            print("\nMotor stopped. Exiting...")
            break
        except Exception as e:
            print(f"Error: {e}")


# Main execution
if __name__ == "__main__":
    print("L298N Motor Controller for ESP32 (No Enable Pin)")
    print("===============================================")
    print(f"Pins: IN1={IN1_PIN}, IN2={IN2_PIN}")
    print("Note: Connect ENA pin to VCC (high) on L298N board")
    print()

    # Uncomment one of these options:

    # Option 1: Run automatic demo
    # demo_sequence()

    # Option 2: Interactive control
    interactive_control()

    # Option 3: Simple test
    # motor.forward(50)
    # time.sleep(3)
    # motor.stop()