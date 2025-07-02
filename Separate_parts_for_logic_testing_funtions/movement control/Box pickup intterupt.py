from machine import Pin
import time


class IR_Magnet:
    def __init__(self, goal_reached=False):
        # Sensor and magnet init
        self.sensor = Pin(16, Pin.IN, Pin.PULL_UP)
        self.magnet = Pin(26, Pin.OUT)
        self.magnet.off()  # Start with magnet off

        # Set up an interrupt to call handle_detection when sensor changes
        self.sensor.irq(trigger=Pin.IRQ_RISING, handler=self.handle_detection)

        # Goal reached flag (for 10-second disable)
        self.goal_reached = goal_reached
        self.drop_start_time = None
        self.drop_active = False

    def set_goal_reached(self, goal_reached):
        self.goal_reached = goal_reached

    def handle_detection(self, pin):
        """Interrupt handler: called automatically when sensor changes state."""
        if self.drop_active:
            if time.time() - self.drop_start_time >= 10:
                self.drop_active = False
                self.drop_start_time = None
                self.goal_reached = False
                print("Magnet reactivated after 10s")
            return

        current_state = not self.sensor.value()  # True if object detected (pin high)

        if current_state:
            self.magnet.on()
            print("Magnet ON (object detected)")
        else:
            self.magnet.off()
            print("Magnet OFF (no object detected)")

        if self.goal_reached and not self.drop_active:
            self.magnet.off()
            self.drop_active = True
            self.drop_start_time = time.time()
            print("Goal reached! Magnet OFF for 10s")


# Usage:
#detector = IR_Magnet()  #
