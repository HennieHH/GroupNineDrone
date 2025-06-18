from machine import Pin
import time


class ObstacleDetector:
    def __init__(self):
        self.sensor = Pin(16, Pin.IN, Pin.PULL_UP)
        self.detection_start = None
        self.triggered = False
        self.clear_count = 0
        self.REQUIRED_CLEAR_CHECKS = 10  # Number of clear readings needed to reset

    def check_4_second_obstacle(self):
        current_state = not self.sensor.value()  # True when obstacle detected

        if current_state:
            self.clear_count = 0  # Reset clear counter

            if self.detection_start is None:
                self.detection_start = time.time()
            elif not self.triggered and (time.time() - self.detection_start >= 4):
                self.triggered = True
                return True
        else:
            self.clear_count += 1

            # Only reset after 10 consecutive clear readings
            if self.clear_count >= self.REQUIRED_CLEAR_CHECKS:
                self.detection_start = None
                self.triggered = False
                self.clear_count = 0
                print("No object detected ")

        return False


# Usage:
detector = ObstacleDetector()  # Use your actual pin number

while True:
    if detector.check_4_second_obstacle():
        print("Continuous obstacle detected for 4+ seconds!")
        # Add your response actions here

    # Rest of your program runs here
    time.sleep(0.1)  # 100ms between checks
