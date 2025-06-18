def determine_line_visibility(normalized_values):
    """
    Determine line visibility for each sensor position.
    Returns boolean values indicating if line is detected.
    Based on normalized sensor values (0-1000 scale).
    """
    # Line visibility thresholds - adjust these values based on your testing
    line_far_left   = normalized_values[0] > 0    # Sensor 1 (links)
    line_left       = normalized_values[1] > 1000 # Sensor 2 (links-midden)
    line_center     = normalized_values[2] > 2000 # Sensor 3 (midden)
    line_right      = normalized_values[3] > 3000 # Sensor 4 (rechts-midden)
    line_far_right  = normalized_values[4] > 4000 # Sensor 5 (rechts)

    return {
        'line_far_left':  line_far_left,
        'line_left':      line_left,
        'line_center':    line_center,
        'line_right':     line_right,
        'line_far_right': line_far_right
    }