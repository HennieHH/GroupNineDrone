from machine import ADC, Pin
from time import sleep

# Global configuration variables for sensors
# SENSOR_PINS: GPIO pins where the line sensors are connected
# WEIGHTS: weights used to calculate the line position (0 to 4000 scale)
# MIN_VALS and MAX_VALS: calibration values for each sensor (minimum and maximum raw ADC readings)
SENSOR_PINS = [27, 26, 25, 33, 32]
WEIGHTS = [0, 1000, 2000, 3000, 4000]
MIN_VALS = [1349, 1407, 1563, 1083, 810]
MAX_VALS = [2143, 2274, 2559, 1892, 1454]


def init_sensors(pins=None):
    """
    Initialize ADC sensors and return a list of sensor objects.
    Each sensor is configured for 11 dB attenuation (full input range).
    """
    if pins is None:
        pins = SENSOR_PINS

    sensors = []
    for pin in pins:
        adc = ADC(Pin(pin))  # Create ADC object on the specified GPIO pin
        adc.atten(ADC.ATTN_11DB)  # Set attenuation for full voltage range (~0-3.3V)
        sensors.append(adc)

    return sensors


def normalize(val, min_val, max_val):
    """
    Normalize a raw ADC value to a 0-1000 scale.
    Values below min_val become 0, above max_val become 1000.
    """
    # Avoid division by zero
    if max_val - min_val == 0:
        return 0
    # Linear interpolation to 0..1000
    y = (val - min_val) * 1000 // (max_val - min_val)
    # Clamp result to [0, 1000]
    return max(0, min(1000, y))


def read_all_data(sensors):
    """
    Read data from all sensors, compute normalized readings, and calculate line position.
    Returns a dict with:
      - raw: list of raw ADC values
      - normalized: list of inverted normalized values (black line gives higher values)
      - position: weighted average position (0 = leftmost, 4000 = rightmost)
    """
    # Read raw sensor data
    raw = [s.read() for s in sensors]
    # Normalize and invert readings (line is dark, so invert to get higher values)
    norm = [1000 - normalize(raw[i], MIN_VALS[i], MAX_VALS[i]) for i in range(len(sensors))]
    total = sum(norm)
    # Calculate line position as weighted average
    position = sum(WEIGHTS[i] * norm[i] for i in range(len(sensors))) // total if total > 0 else -1

    return {
        'raw': raw,
        'normalized': norm,
        'position': position
    }


def debug_print(sensors):
    """
    Print formatted debug information:
      - Raw ADC values (R1..R5)
      - Normalized values (N1..N5)
      - Current line position
    """
    data = read_all_data(sensors)
    raw_str = "  ".join(f"R{i + 1}:{data['raw'][i]}" for i in range(len(sensors)))
    norm_str = "  ".join(f"N{i + 1}:{data['normalized'][i]}" for i in range(len(sensors)))
    print(f"{raw_str}  |  {norm_str}  |  Position: {data['position']}")


def track():
    """
    Main tracking function. Initializes sensors and continuously reads values.
    Press Ctrl+C to stop.
    """
    sensors = init_sensors()
    print(">> Tracking with static calibration. Press Ctrl+C to stop.\n")
    try:
        while True:
            debug_print(sensors)
            sleep(0.1)  # Wait 100 ms between readings
    except KeyboardInterrupt:
        print("Tracking stopped.")


# If this script is run directly, start tracking
if __name__ == "__main__":
    track()

