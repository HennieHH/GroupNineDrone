from machine import Pin
import time


class IR_Magnet:
    def __init__(self, goal_reached=False):
        self.sensor = Pin(16, Pin.IN, Pin.PULL_UP)
        self.magnet = Pin(26, Pin.OUT)
        self.magnet.off()

        self.detection_start = None
        self.triggered = False
        self.clear_count = 0
        self.REQUIRED_CLEAR_CHECKS = 10

        self.goal_reached = goal_reached
        self.drop_start_time = None
        self.drop_active = False

    def set_goal_reached(self, goal_reached):
        self.goal_reached = goal_reached

    def obstacle_detection(self):
        if self.drop_active:
            if time.time() - self.drop_start_time >= 10:
                self.drop_active = False
                self.drop_start_time = None
                self.goal_reached = False
                print("Magnet reactivated after 10s")
            return False

        current_state = not self.sensor.value()

        if current_state:
            self.clear_count = 0
            if self.detection_start is None:
                self.detection_start = time.time()
            elif not self.triggered and (time.time() - self.detection_start >= 1):
                self.triggered = True
                self.magnet.on()
                print("Magnet ON (object detected)")
                return True
        else:
            self.clear_count += 1
            if self.clear_count >= self.REQUIRED_CLEAR_CHECKS:
                self.detection_start = None
                self.triggered = False
                self.clear_count = 0
                print("No object detected")

        if self.goal_reached and not self.drop_active:
            self.magnet.off()
            self.drop_active = True
            self.drop_start_time = time.time()
            print("Goal reached! Magnet OFF for 10s")

        return False


# Initialize detector + button
detector = IR_Magnet(goal_reached=False)
button = Pin(34, Pin.IN, Pin.PULL_DOWN)  # Button to GND, GPIO 15 with pull-down

while True:
    detector.obstacle_detection()

    if button.value():  # Button pressed
        detector.set_goal_reached(True)
        print("Button pressed: Goal set to True")
        time.sleep(0.5)  # Debounce delay

    time.sleep(0.1)
