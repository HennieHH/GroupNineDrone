from machine import Pin
import time


class IR_Magnet:
    def __init__(self):
        # IR Sensor Setup
        self.sensor = Pin(16, Pin.IN, Pin.PULL_UP)
        # Magnet Setup
        self.magnet = Pin(26, Pin.OUT)
        self.magnet.off()  # Start with magnet off

        self.detection_start = None
        self.triggered = False
        self.clear_count = 0
        self.REQUIRED_CLEAR_CHECKS = 10  # Number of clear readings needed to reset

    def obstacle_detection(self):
        current_state = not self.sensor.value()  # True when obstacle detected

        if current_state:
            self.clear_count = 0  # Reset clear counter

            if self.detection_start is None:
                self.detection_start = time.time()
            elif not self.triggered and (time.time() - self.detection_start >= 1):  # 1 second detection time
                self.triggered = True
                self.magnet.on()
                print("Continuous obstacle detected for 1+ second! Magnet ON")
                return True
        else:
            self.clear_count += 1

            # Only reset after 10 consecutive clear readings
            if self.clear_count >= self.REQUIRED_CLEAR_CHECKS:
                self.detection_start = None
                self.triggered = False
                self.magnet.off()
                self.clear_count = 0
                print("No object detected, Magnet OFF")

        return False


# Usage:
detector = IR_Magnet()  # Create instance

while True:
    detector.obstacle_detection()  # Removed colon after this line
    time.sleep(0.1)  # 100ms between checks
