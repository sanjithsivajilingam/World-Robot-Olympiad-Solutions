 #!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class WROrobot:
    # Define constants here

    BLACK = 6
    WHITE = 54
    # Blue and Red values that return from rgb sensor
    BLUE = 60
    RED = 30
    threshold = (BLACK + WHITE) / 2
    DRIVE_SPEED = 50
    Kp = 1.0
    Ki = 0.005
    Kd = 0.01

    #Junior Compeition Variables
    currentLeftTree = None
    currentRightTree = None
    currentRightDropOff = Color.RED
    currentLeftDropOff = Color.RED
    colors = [None, None, None, None, None, None] # Color Array

    #Senior
    clawHeight = 0
    clawGrab = 0

    def __init__(self, comp = "elementary", ev3 = None, motor_2 = None, motor_1 = None, left_wheel = None, right_wheel = None, driveBase = None, gyro = None, light_2 = None, light_1 = None, touch_2 = None, touch_1 = None, color_2 = None, color_1 = None, ultra_2 = None, ultra_1 = None):
        self.comp = comp
        self.ev3 = ev3
        self.motor_2 = motor_2
        self.motor_1 = motor_1
        self.left_wheel = left_wheel
        self.right_wheel = right_wheel
        self.driveBase = driveBase
        self.gyro = gyro
        self.light_2 = light_2
        self.light_1 = light_1
        self.touch_2 = touch_2
        self.touch_1 = touch_1
        self.color_2 = color_2
        self.color_1 = color_1
        self.PRECOLOR = None
        self.ultra_2 = ultra_2
        self.ultra_1 = ultra_1
        print("Created Robot")


def createRobot(robotParam):
    """Allows creation of different competition robots

    Args:
        robotParam (dict): robot dictionary used to create robot object

    Raises:
        TypeError: robotParam is not dict

    Returns:
        robotObject (WROrobot): Robot for competition
    """
    if not type(robotParam) in [dict]:
        raise TypeError("robotParam must be of type dict")

    if robotParam["comp"].lower() == "elementary":
        robotObject = WROrobot(robotParam["comp"], robotParam["ev3"], robotParam["motor_2"], robotParam["motor_1"], robotParam["left_wheel"], robotParam["right_wheel"], robotParam["driveBase"], None, robotParam["light_2"], robotParam["light_1"], None, None, None, robotParam["color_1"], None, None)
        return robotObject

    if robotParam["comp"].lower() == "junior":
        robotObject = WROrobot(robotParam["comp"], robotParam["ev3"], robotParam["motor_2"], robotParam["motor_1"], robotParam["left_wheel"], robotParam["right_wheel"], robotParam["driveBase"], None, robotParam["light_2"], robotParam["light_1"], None, None, robotParam["color_2"], robotParam["color_1"], None, None)
        return robotObject

    if robotParam["comp"].lower() == "senior":
        robotObject = WROrobot(robotParam["comp"], robotParam["ev3"], robotParam["motor_2"], robotParam["motor_1"], robotParam["left_wheel"], robotParam["right_wheel"], robotParam["driveBase"], robotParam["gyro"], robotParam["light_2"], robotParam["light_1"], robotParam["touch_2"], robotParam["touch_1"], robotParam["color_2"], robotParam["color_1"], robotParam["ultra_2"], robotParam["ultra_1"])
        return robotObject

def createParam():
    """Creates the param for the robot
    Args:
        None

    Raises:
        None
    Returns:
        dict of robot parameters
    """
    return {"comp": None, "ev3": None, "motor_2": None, "motor_1": None, "left_wheel": None, "right_wheel": None, "driveBase": None, "gyro": None, "light_2": None, "light_1": None, "touch_2": None, "touch_1": None, "color_2": None, "color_1": None, "ultra_2": None, "ultra_1": None}