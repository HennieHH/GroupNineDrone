from machine import I2C, Pin
import VL53L0X as VL
import time


class Repath:
    def __init__(self, detection_range=100, buffer_size=10):
        """Initialize the Repath distance sensor with weighted average filtering."""
        self.i2c = I2C(0, scl=Pin(25), sda=Pin(33))
        self.sensor = VL.VL53L0X(self.i2c)
        self.offset = 25
        self.buffer_size = buffer_size
        self.distance_buffer = []
        self.detection_range = detection_range

        # Range detection variables
        self.detection_counter = 0
        self.detection_flag = False
        self.detection_cycles_needed = 5
        self.last_flag_change_time = 0
        self.cooldown_period = 2  # 1 second cooldown after flag turns false

        # Weights for the samples
        self.weights = [0.393, 0.239, 0.146, 0.089, 0.055,
                        0.034, 0.020, 0.012, 0.007, 0.004][:buffer_size]
        self.total_weight = sum(self.weights)

        print(f"VL53L0X Distance Sensor - Detection Range: {detection_range} mm")

    def read_distance(self):
        """Read distance from the sensor and return weighted average."""
        raw_distance = self.sensor.read() - self.offset
        self.distance_buffer.insert(0, raw_distance)

        if len(self.distance_buffer) > self.buffer_size:
            self.distance_buffer.pop()

        if len(self.distance_buffer) == self.buffer_size:
            weighted_sum = sum(self.distance_buffer[i] * self.weights[i]
                               for i in range(self.buffer_size))
            filtered_distance = weighted_sum / self.total_weight
            self._check_range_detection(filtered_distance)
            return filtered_distance
        return None

    def _check_range_detection(self, distance):
        """Check range detection with cooldown period after flag turns false."""
        current_time = time.time()

        # Skip detection checks if we're in cooldown period
        if not self.detection_flag and (current_time - self.last_flag_change_time) < self.cooldown_period:
            return

        if distance <= self.detection_range:
            self.detection_counter += 1
            if self.detection_counter >= self.detection_cycles_needed:
                if not self.detection_flag:
                    self.detection_flag = True
                    self.last_flag_change_time = current_time
                    print(f"*** DETECTION FLAG SET - Object detected within {self.detection_range} mm ***")
        else:
            if self.detection_counter > 0:
                self.detection_counter -= 1

            if self.detection_flag and self.detection_counter <= 0:
                self.detection_flag = False
                self.last_flag_change_time = current_time
                print("*** DETECTION FLAG CLEARED - Cooldown period started ***")

    def get_detection_flag(self):
        """Get the current state of the detection flag."""
        return self.detection_flag

    def reset_detection_flag(self):
        """Manually reset the detection flag and counter."""
        self.detection_flag = False
        self.detection_counter = 0
        self.last_flag_change_time = time.time()
        print("Detection flag manually reset - Cooldown period started")


sensor = Repath(detection_range=200)
while True:
    distance = sensor.read_distance()
    if distance is not None:
        print(f"Distance: {distance:.1f} mm | Flag: {sensor.get_detection_flag()}")
    time.sleep(0.016)

## How to use
"""
sensor = Repath(detection_range=200) # Change detection range to needed range
if sensor.get_detection_flag():# IF TRUE OBJECT IS DETECTED # Warning it does spam true 
    Do a repath function


"""

