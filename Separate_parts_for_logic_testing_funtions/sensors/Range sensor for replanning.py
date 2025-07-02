from machine import I2C, Pin
import VL53L0X as VL
import time


class Repath:
    def __init__(self, detection_range=1000, buffer_size=10):
        """Initialize the Repath distance sensor with weighted average filtering.

        Args:
            scl_pin (int): Pin number for I2C SCL (default 25)
            sda_pin (int): Pin number for I2C SDA (default 33)
            offset (int): Distance offset correction (default 25)
            buffer_size (int): Size of the filtering buffer (default 10)
            detection_range (int): Distance range in mm for detection flag (default 100)
        """
        self.i2c = I2C(0, scl=Pin(25), sda=Pin(33))
        self.sensor = VL.VL53L0X(self.i2c)
        self.offset = 25
        self.buffer_size = buffer_size
        self.distance_buffer = []
        self.detection_range = detection_range

        # Range detection variables
        self.detection_counter = 0
        self.detection_flag = False
        self.detection_cycles_needed = 10

        # Weights for the samples (newest to oldest)
        self.weights = [0.393, 0.239, 0.146, 0.089, 0.055,
                        0.034, 0.020, 0.012, 0.007, 0.004][:buffer_size]
        self.total_weight = sum(self.weights)

        print("VL53L0X Distance Sensor - Detection Range: {} mm".format(detection_range))

    def read_distance(self):
        """Read distance from the sensor and return weighted average.
        Returns:
            float or None: Weighted average distance in mm, None if not enough data
        """
        raw_distance = self.sensor.read() - self.offset

        # Add new reading at the front
        self.distance_buffer.insert(0, raw_distance)

        # Limit buffer to specified size
        if len(self.distance_buffer) > self.buffer_size:
            self.distance_buffer.pop()

        # Compute filtered distance if enough data
        if len(self.distance_buffer) == self.buffer_size:
            weighted_sum = sum(self.distance_buffer[i] * self.weights[i]
                               for i in range(self.buffer_size))
            filtered_distance = weighted_sum / self.total_weight

            # Check for range detection
            self._check_range_detection(filtered_distance)

            return filtered_distance
        else:
            return None

    def _check_range_detection(self, distance):
        """Check if distance is within detection range and update flag.

        Args:
            distance (float): Current filtered distance in mm
        """
        if distance <= self.detection_range:
            self.detection_counter += 1
            if self.detection_counter >= self.detection_cycles_needed:
                if not self.detection_flag:
                    self.detection_flag = True
                    print("*** DETECTION FLAG SET - Object detected within {} mm for {} cycles ***".format(
                        self.detection_range, self.detection_cycles_needed))
        else:
            # Reset counter if object moves out of range
            if self.detection_counter > 0:
                self.detection_counter = 0
                if self.detection_flag:
                    self.detection_flag = False
                    print("*** DETECTION FLAG CLEARED - Object moved out of range ***")

    def get_detection_flag(self):
        """Get the current state of the detection flag.

        Returns:
            bool: True if object detected within range for required cycles
        """
        return self.detection_flag

    def reset_detection_flag(self):
        """Manually reset the detection flag and counter."""
        self.detection_flag = False
        self.detection_counter = 0
        print("Detection flag manually reset")

    def continuous_read(self, interval=0.016):
        """Continuously read and print weighted average distances.

        Args:
            interval (float): Time between readings in seconds (default 0.5)
        """
        while True:
            filtered_distance = self.read_distance()

            if filtered_distance is not None:
                print("Weighted Average: {:.2f} mm".format(filtered_distance))
            else:
                print("Collecting data... ({}/{} samples)".format(
                    len(self.distance_buffer), self.buffer_size))

            time.sleep(interval)


# Example usage:
if __name__ == "__main__":
    # Initialize sensor with 100mm detection range
    sensor = Repath(detection_range=300)

    # Start continuous reading
    sensor.continuous_read()

    # Alternative: Manual reading loop
    # while True:
    #     distance = sensor.read_distance()
    #     if distance is not None:
    #         print("Distance: {:.2f} mm | Flag: {}".format(distance, sensor.get_detection_flag()))
    #     time.sleep(0.5)
