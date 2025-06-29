from machine import ADC, Pin
from time import sleep

sensor_pins = [25, 26, 27, 32, 33]
sensors = []

for pin in sensor_pins:
    adc = ADC(Pin(pin))
    adc.atten(ADC.ATTN_11DB)
    sensors.append(adc)

min_vals = [1349, 1350, 1563, 1183, 1000]
max_vals = [1943, 2274, 2559, 1892, 1454]

def normalize(val, min_val, max_val):
    if max_val - min_val == 0:
        return 0
    y = (val - min_val) * 1000 // (max_val - min_val)
    return max(0, min(1000, y))

def track():
    print(">> Tracking without weighted average. Press Ctrl+C to stop.\n")
    while True:
        raw_values = [sensor.read() for sensor in sensors]
        norm_values = [
            1000 - normalize(raw_values[i], min_vals[i], max_vals[i])
            for i in range(5)
        ]

        sensor_output = "  ".join(f"R{i+1}:{raw_values[i]}" for i in range(5))
        norm_debug = "  ".join(f"N{i+1}:{val}" for i, val in enumerate(norm_values))
        print(f"{sensor_output}  |  {norm_debug}")

        sleep(0.1)

track()
