from machine import Pin, PWM
import time


class MotorController:
    def __init__(self, in1_pin, in2_pin, freq=50, min_speed=50):
        """
        Initialize L298N motor controller without enable pin

        Args:
            in1_pin: GPIO pin for IN1 (PWM direction control)
            in2_pin: GPIO pin for IN2 (PWM direction control)
            freq: PWM frequency in Hz (default 1000Hz)
            min_speed: Minimum speed percentage to overcome motor friction
        """
        self.in1 = PWM(Pin(in1_pin), freq=freq)
        self.in2 = PWM(Pin(in2_pin), freq=freq)
        self.min_speed = min_speed
        self.current_speed = 0  # Track current speed

        # Stop motor initially
        self.stop()

    def stop(self):
        """Stop the motor by setting both pins to LOW"""
        self.in1.duty(0)
        self.in2.duty(0)
        self.current_speed = 0
        print("Motor stopped")

    def forward(self, speed=100):
        """
        Rotate motor forward at specified speed percentage

        Args:
            speed: Speed percentage (0-100), default 100%
                  0 will stop the motor
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

        self.current_speed = speed
        print(f"Motor forward at {speed}% speed (Duty: {duty})")

print("Starting motor test in 2 seconds...")
time.sleep(2)

motora = MotorController(22, 21)
motorb = MotorController(13, 14)

print("Testing motor A at 30%")
motora.forward(40)
time.sleep(3)

print("Testing motor B a t 30%")
motorb.forward(40)
time.sleep(3)

print("Increasing both motors")
motora.forward(50)
motorb.forward(50)
time.sleep(3)

print("Stopping motors")
motora.stop()
motorb.stop()

