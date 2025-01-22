import time
import random

# Constants for robot dimensions and arena setup
ROBOT_SPEED = 0.5  # Speed in m/s
BLOCK_DETECTION_RANGE = 1.0  # Range of sensors in meters
ARM_EXTENSION_LIMIT = 1.2  # Maximum arm extension in meters
BASKET_POSITIONS = {'red': (5, 5), 'blue': (10, 5), 'yellow': (7, 10)}  # (x, y) positions

# Simulated hardware classes
class Motor:
    def __init__(self, name):
        self.name = name
        self.speed = 0

    def set_speed(self, speed):
        print(f"{self.name} motor speed set to {speed}")
        self.speed = speed

    def stop(self):
        print(f"{self.name} motor stopped")
        self.speed = 0

class Arm:
    def __init__(self):
        self.extension = 0

    def extend(self, distance):
        if distance <= ARM_EXTENSION_LIMIT:
            self.extension = distance
            print(f"Arm extended to {distance} meters")
        else:
            print("Extension limit exceeded!")

    def retract(self):
        self.extension = 0
        print("Arm retracted")

class ColorSensor:
    def detect_color(self):
        # Simulating random block detection
        return random.choice(['red', 'blue', 'yellow', None])

# Robot class for overall control
class Robot:
    def __init__(self):
        self.left_motor = Motor("Left")
        self.right_motor = Motor("Right")
        self.arm = Arm()
        self.sensor = ColorSensor()
        self.position = (0, 0)

    def move_to(self, x, y):
        print(f"Moving from {self.position} to ({x}, {y})")
        time.sleep(self.calculate_travel_time(self.position, (x, y)))
        self.position = (x, y)
        print(f"Arrived at ({x}, {y})")

    def calculate_travel_time(self, start, end):
        distance = ((end[0] - start[0])**2 + (end[1] - start[1])**2)**0.5
        return distance / ROBOT_SPEED

    def pick_up_block(self):
        print("Attempting to pick up block...")
        detected_color = self.sensor.detect_color()
        if detected_color:
            print(f"Block detected: {detected_color}")
            self.arm.extend(1.0)
            time.sleep(1)
            print("Block secured!")
            self.arm.retract()
            return detected_color
        else:
            print("No block detected!")
            return None

    def place_block(self, color):
        if color in BASKET_POSITIONS:
            target = BASKET_POSITIONS[color]
            print(f"Placing {color} block in basket at {target}")
            self.move_to(*target)
            self.arm.extend(1.0)
            time.sleep(1)
            print(f"{color} block placed in basket!")
            self.arm.retract()
        else:
            print(f"Unknown basket for color: {color}")

# Autonomous sequence for the competition
def autonomous_sequence(robot):
    print("Starting autonomous sequence")
    for _ in range(5):  # Attempt to process up to 5 blocks
        # Move to random position to simulate searching for blocks
        target_x = random.uniform(0, 15)
        target_y = random.uniform(0, 15)
        robot.move_to(target_x, target_y)

        # Attempt to pick up a block
        color = robot.pick_up_block()
        if color:
            robot.place_block(color)
        else:
            print("Continuing search for blocks...")

    print("Autonomous sequence completed")

# Teleoperated sequence for user control
def teleoperated_control(robot):
    print("Starting teleoperated control")
    while True:
        command = input("Enter command (move/pick/place/stop): ").strip().lower()
        if command == "move":
            x = float(input("Enter x-coordinate: "))
            y = float(input("Enter y-coordinate: "))
            robot.move_to(x, y)
        elif command == "pick":
            robot.pick_up_block()
        elif command == "place":
            color = input("Enter block color (red/blue/yellow): ").strip().lower()
            robot.place_block(color)
        elif command == "stop":
            print("Stopping teleoperated control")
            break
        else:
            print("Unknown command")

# Main program
if __name__ == "__main__":
    my_robot = Robot()

    # Choose mode
    mode = input("Enter mode (autonomous/teleoperated): ").strip().lower()
    if mode == "autonomous":
        autonomous_sequence(my_robot)
    elif mode == "teleoperated":
        teleoperated_control(my_robot)
    else:
        print("Invalid mode selected")
