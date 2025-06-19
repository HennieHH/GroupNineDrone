from machine import Pin, PWM
import time


class MotorController:
    def __init__(self, in1_pin, in2_pin, freq=100, min_speed=1):
        """
        Initialize L298N motor controller without enable pin

        Args:
            in1_pin: GPIO pin for IN1 (PWM direction control)
            in2_pin: GPIO pin for IN2 (PWM direction control)
            freq: PWM frequency in Hz (default 100Hz)
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