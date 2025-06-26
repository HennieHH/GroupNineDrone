from machine import I2C, Pin
import VL53L0X as VL
import time

# Initialize I2C (SDA=21, SCL=22)
i2c = I2C(0, scl=Pin(22), sda=Pin(21))

# Initialize VL53L0X sensor
sensor = VL.VL53L0X(i2c)

print("VL53L0X Distance Sensor with Weighted Average Filter (Last 10 Samples)")

# Buffer to store the last 10 readings
distance_buffer = []

# Weights for the last 5 samples (newest to oldest)
weights = [0.393, 0.239, 0.146, 0.089, 0.055, 0.034, 0.020, 0.012, 0.007, 0.004]
total_weight = sum(weights)

while True:
    raw_distance = sensor.read() - 25  # Apply offset correction

    # Add new reading at the front
    distance_buffer.insert(0, raw_distance)

    # Limit buffer to last 10 readings
    if len(distance_buffer) > 10:
        distance_buffer.pop()

    # Compute weighted average if buffer has enough data
    if len(distance_buffer) == 10:
        weighted_sum = sum(distance_buffer[i] * weights[i] for i in range(10))
        filtered_distance = weighted_sum / total_weight
        print("Raw: {:>4} mm | Filtered (10-sample weighted): {:.2f} mm".format(raw_distance, filtered_distance))
    else:
        # Not enough data for full filter
        print("Raw: {:>4} mm | Filtered: N/A (collecting data...)".format(raw_distance))

    time.sleep(0.5)