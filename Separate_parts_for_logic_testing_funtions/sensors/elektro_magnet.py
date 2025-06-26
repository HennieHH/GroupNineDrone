from machine import Pin
import time


def initialize_robot_pin(pin_26_num=26):
    """
    Initialize robot control pin

    Args:
        pin_26_num: Pin number for magneet control (default: 26)

    Returns:
        Pin: pin_26 object for magneet control
    """
    pin_26 = Pin(pin_26_num, Pin.OUT)
    pin_26.off()  # Start met magneet uit

    print(f"Robot pin initialized - Pin {pin_26_num}: Magneet control")
    return pin_26


def robot_pickup_action(pin_26):
    """
    Execute pickup action - zet magneet aan (HIGH)

    Args:
        pin_26: Pin object for magneet control
    """
    print("Executing pickup action...")
    pin_26.on()
    print("Pin 26: HIGH - Magneet geactiveerd, box opgepakt")


def robot_drop_action(pin_26):
    """
    Execute drop action - zet magneet uit (LOW)

    Args:
        pin_26: Pin object for magneet control
    """
    print("Executing drop action...")
    pin_26.off()
    print("Pin 26: LOW - Magneet gedeactiveerd, box afgeleverd")


def toggle_magneet(pin_26):
    """
    Toggle magneet state

    Args:
        pin_26: Pin object for magneet control
    """
    current_state = pin_26.value()
    if current_state:
        pin_26.off()
        print("Pin 26: LOW - Magneet uit")
    else:
        pin_26.on()
        print("Pin 26: HIGH - Magneet aan")


def get_magneet_status(pin_26):
    """
    Check current magneet status

    Args:
        pin_26: Pin object for magneet control

    Returns:
        bool: True if magneet is on, False if off
    """
    status = pin_26.value()
    state_text = "AAN" if status else "UIT"
    print(f"Magneet status: {state_text}")
    return status


# Example usage in main code:
def main_robot_task():
    """
    Main function showing robot task execution
    """
    # Initialize pin
    pin_26 = initialize_robot_pin()

    # Simulate robot movement and tasks
    print("Robot moving to start position...")
    time.sleep(2)  # Simulate movement time

    print("Robot at pickup position")
    robot_pickup_action(pin_26)  # Magneet gaat AAN

    print("Robot moving to goal position...")
    time.sleep(3)  # Simulate movement time - magneet blijft AAN tijdens transport

    print("Robot at drop/goal position")
    robot_drop_action(pin_26)  # Magneet gaat UIT, box wordt afgeleverd

    print("Task completed!")


def continuous_pickup_drop_cycle(pin_26, cycle_delay=10):
    """
    Continuous pickup and drop cycle

    Args:
        pin_26: Pin object for magneet control
        cycle_delay: Time between pickup and drop (seconds)
    """
    print(f"Starting continuous pickup-drop cycle (delay: {cycle_delay}s)")
    print("Press Ctrl+C to stop")

    try:
        cycle_count = 0
        while True:
            cycle_count += 1
            print(f"\n--- Cycle {cycle_count} ---")

            print("Moving to pickup...")
            time.sleep(2)
            robot_pickup_action(pin_26)

            print("Transporting to goal...")
            time.sleep(cycle_delay)

            print("At goal position")
            robot_drop_action(pin_26)

            print("Returning to start...")
            time.sleep(3)

    except KeyboardInterrupt:
        print(f"\nStopping cycle after {cycle_count} cycles")
        pin_26.off()  # Zorg dat magneet uit is
        print("Magneet uitgeschakeld")


# If running this file directly
if __name__ == "__main__":
    main_robot_task()
