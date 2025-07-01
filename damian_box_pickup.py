from machine import Pin
import time

class IR_Magnet:
    def __init__(self, goal_reached=False):
        # Sensor and magnet init
        
        """ 
        
        waarom input pin nodig?
        
        
        self.sensor = Pin(16, Pin.IN, Pin.PULL_UP) 
        
        
        """
        
        
        self.magnet = Pin(26, Pin.OUT)
        self.magnet.off()
        """

        is onnodig 

        # Motor control pins for turning 180 degrees
        self.left1 = Pin(14, Pin.OUT)   # Adjust based on your wiring
        self.left2 = Pin(12, Pin.OUT)
        self.right1 = Pin(27, Pin.OUT)
        self.right2 = Pin(25, Pin.OUT)


        """

        # Flags for time and object not detected
        self.detection_start = None
        self.triggered = False
        self.clear_count = 0
        self.REQUIRED_CLEAR_CHECKS = 10

        self.goal_reached = goal_reached
        self.drop_start_time = None
        self.drop_active = False

        # Rotation flags
        self.has_turned_after_pickup = False
        self.has_turned_after_dropoff = False

    def set_goal_reached(self, goal_reached):  # function used to set goal reached
        self.goal_reached = goal_reached


"""
    def rotate_180(self):
        print("Rotating 180 degrees...")

        # Spin in place: one wheel forward, one wheel backward
        self.left1.on()
        self.left2.off()
        self.right1.off()
        self.right2.on()

        time.sleep(1.0)  # Adjust this time until your robot turns 180°

        # Stop motors
        self.left1.off()
        self.left2.off()
        self.right1.off()
        self.right2.off()

"""

    def obstacle_detection(self):  # used for reading IR sensor and turning on magnet
        if self.drop_active:
            if time.time() - self.drop_start_time >= 10: # wat is deze logica?
                self.drop_active = False
                self.drop_start_time = None # waarom hou je dit bij?
                self.goal_reached = False
                print("Magnet reactivated after 10s")
            return False # er zit hier niks van de sensor in?

        current_state = not self.sensor.value() # inverted logica ?

        if current_state:
            self.clear_count = 0
            if self.detection_start is None: # none of false?
                self.detection_start = time.time() # miss timers weghalen
            elif not self.triggered and (time.time() - self.detection_start >= 1):
                self.triggered = True
                self.magnet.on()
                print("Magnet ON (object detected)")

                if not self.has_turned_after_pickup:
                    self.rotate_180()
                    self.has_turned_after_pickup = True
                    self.has_turned_after_dropoff = False

                return True
        else:
            self.clear_count += 1
            if self.clear_count >= self.REQUIRED_CLEAR_CHECKS: # bedoel je dat alle dozen weg zijn
                self.detection_start = None # ?
                self.triggered = False # ?
                self.clear_count = 0 
                self.magnet.off()
                print("No object detected - Magnet OFF")

        if self.goal_reached and not self.drop_active:
            self.magnet.off()
            self.drop_active = True
            self.drop_start_time = time.time()
            print("Goal reached! Magnet OFF for 10s")

            if not self.has_turned_after_dropoff:
                self.rotate_180()
                self.has_turned_after_dropoff = True
                self.has_turned_after_pickup = False

        return False


'''''
# NORMAL OPERATION =

detector = IR_Magnet(goal_reached=False)  # This is a way to import the class

while True:
    detector.obstacle_detection()  # calling the function for checking the IR sensor and activating the magnet

    detector.set_goal_reached(False)  # Used to set goal reached, turns off the Magnet for 10 seconds

    time.sleep(0.1)


# FOR BUTTON +

# Initialize detector + button
detector = IR_Magnet(goal_reached=False)
button = Pin(34, Pin.IN, Pin.PULL_DOWN)  # Button to GND, GPIO 34 with pull-down

while True:
    detector.obstacle_detection()

    if button.value():  # Button pressed
        detector.set_goal_reached(True)
        print("Button pressed: Goal set to True")
        time.sleep(0.5)  # Debounce delay

    time.sleep(0.1)
'''''
