#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from RobotBody import WROrobot
import Movement
import Detection

def initClawGrab(robot:WROrobot):
    """Initializes the claw grab

    Args:
        robot (WROrobot): Robot object used for competition

    Raises:
        None

    Returns:
        None
    """
    #This initalizes the grabber so that the gripper will be open,
    # first line makes the claw fully closed, third line opens it by 90 degrees
    robot.motor_1.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
    robot.motor_1.reset_angle(0)
    robot.motor_1.run_target(200, -90)

def initTwoPartClaw(robot:WROrobot):
    """Initializes the two part claw grab

    Args:
        robot (WROrobot): Robot object used for competition

    Raises:
        None

    Returns:
        None
    """
    robot.motor_1.run_until_stalled(500, Stop.COAST, 50)
    robot.motor_2.run_until_stalled(-500, Stop.COAST, 50)
    robot.motor_1.reset_angle(0)
    robot.motor_2.reset_angle(0)
    robot.motor_1.run_until_stalled(-500, then=Stop.COAST, duty_limit=90)
    robot.motor_2.run_until_stalled(500, then=Stop.COAST, duty_limit=90)
    robot.clawHeight = int(robot.motor_1.angle())
    robot.clawGrab = int(robot.motor_2.angle())
    twoPartLift(robot, 500, 0.5, 0, 0)

def clawGrab(robot:WROrobot, action):
    """Performs a claw grab

    Args:
        robot (WROrobot): Robot object used for competition
        action (str): Action robot will be taking (pick or release)
    Raises:
        None

    Returns:
        None
    """
    if not type(action) in [str]:
        raise TypeError("action must be of type str")

    if action == "pick":
        robot.motor_1.run_until_stalled(-200, Stop.HOLD, 40)

    if action == "release":
        #robot.motor_1.run_until_stalled(200, Stop.HOLD, 30)
        robot.motor_1.run_angle(200, 110, Stop.HOLD, True)

def twoPartLift(robot:WROrobot, speed, grabPer, heightPer, order = 0):
    """This function utilizes motor1 and motor2 found in the Senior competition design.
        The function takes in a speed  parameter which changes the speed at which these motors should run at.
        The function also takes in a grabPer parameter which specifies the percentage the claw of the robot will be open or closed.
        The heightPer parameter is similar as it changes the vertical position of the robot's claw based on the percentage entered.
        For example if the percentage entered is 1 for each of these parameters the robot's claw will be at its highest vertical position and will be completely open.
        This function does not return anything as it only changes the vertical position and angle of the claw attached to the Senior robot as specified by the inputted parameters.
        If the percentage is less than 0 or greater than 1 the respective motors will run until stalled in the corresponding direction.

    Args:
        robot (WROrobot): Robot object used for competition
        speed (int): Speed in which robit will drive
        grabPer (float): Indicates how open the claw is (0 being closed, 1 being open). Given as a percentage.
        heightPer (float): Indicates position of robot claw (0 being lowest, 1 being highest). Given as a percentage.
        order (int): Indicates order of operations of lift. If 1, vertical position changed, then grab angle changed. If 0, grab angle changed, then vertical position changed.

    Raises:
        None

    Returns:
        None
    """
    if order == 0:
        if grabPer >= 1:
            robot.motor_2.run_until_stalled(speed, Stop.HOLD, 95)
        elif grabPer <= 0:
            robot.motor_2.run_until_stalled(-speed, Stop.HOLD, 95)
        else:
            robot.motor_2.run_target(speed, robot.clawGrab * grabPer, Stop.HOLD, True)
        if heightPer >= 1:
            robot.motor_1.run_until_stalled(-speed, Stop.HOLD, 95)
        elif heightPer <= 0:
            robot.motor_1.run_until_stalled(speed, Stop.HOLD, 95)
        else:
            robot.motor_1.run_target(-speed, robot.clawHeight * heightPer, Stop.HOLD, True)
    elif order == 1:
        if heightPer >= 1:
            robot.motor_1.run_until_stalled(-speed, Stop.HOLD, 95)
        elif heightPer <= 0:
            robot.motor_1.run_until_stalled(speed, Stop.HOLD, 95)
        else:
            robot.motor_1.run_target(-speed, robot.clawHeight * heightPer, Stop.HOLD, True)
        if grabPer >= 1:
            robot.motor_2.run_until_stalled(speed, Stop.HOLD, 95)
        elif grabPer <= 0:
            robot.motor_2.run_until_stalled(-speed, Stop.HOLD, 95)
        else:
            robot.motor_2.run_target(speed, robot.clawGrab * grabPer, Stop.HOLD, True)

def treeDropOff(robot:WROrobot, dropNum):
    """Drops off a tree

    Args:
        robot (WROrobot): Robot object used for competition
        dropNum (int): Indicates the # of objects it should drop
    Raises:
        None

    Returns:
        None
    """
    if dropNum == 1:
        robot.motor_1.run_angle(1000, 205, Stop.HOLD, True)
        robot.driveBase.straight(-35)
        robot.driveBase.turn(10)
        clawGrab(robot, "pick")
        robot.currentRightDropOff = None

    if dropNum == 2:
        clawGrab(robot, "release")
        robot.currentRightDropOff = None
        robot.currentLeftDropOff = None

def nodeConstruction(robot:WROrobot, direction, constructionAreaColor, basesLeft):
    """Constructs a node. Node is the entire turbine. Form a construction of the full turbine at a node.

    Args:
        robot (WROrobot): Robot object used for competition
        direction (str): Direction robot will path during construction travel
        construcrtionAreaColor (Color): Color value that specifies the construction area the robot must travel to in order to construct a wind generator
    Raises:
        None

    Returns:
        None
    """
    basesDropped = 3 - basesLeft
    distance = 0
    if(constructionAreaColor != Color.RED):
        if(direction == "counterclockwise"):
            Movement.forwardMovement(robot, 90)
            Movement.turnUntilLine(robot, "RIGHT")
            twoPartLift(robot, 500, 0.5, 0, 0)
            distance = Detection.lineFollowUntilColorIntersection(robot, 300, 50)
            twoPartLift(robot, 500, 0, 0, 0)
            Movement.forwardMovement(robot, 94 + basesDropped * 60)
            Movement.backwardMovement(robot, basesDropped * 56)
            twoPartLift(robot, 500, 0, 1, 0)
            Movement.backwardMovement(robot, 100)
            twoPartLift(robot, 500, 0.5, 0.17, 1)
            Movement.backwardMovement(robot, 40)
            twoPartLift(robot, 500, 0.65, 0.2, 0)
            Movement.turnOnSpot(robot, -90)
            twoPartLift(robot, 500, 0.65, 0, 0)
            Movement.backwardMovement(robot, 100)
            Detection.stopOnLine(robot, 100)
        if(direction == "clockwise"):
            Movement.forwardMovement(robot, 90)
            Movement.turnUntilLine(robot, "LEFT")
            twoPartLift(robot, 500, 0.5, 0, 0)
            distance = Detection.lineFollowUntilColorIntersection(robot, 300, 50)
            twoPartLift(robot, 500, 0, 0, 0)
            Movement.forwardMovement(robot, 94 + basesDropped * 60)
            Movement.backwardMovement(robot, basesDropped * 56)
            twoPartLift(robot, 500, 0, 1, 0)
            Movement.backwardMovement(robot, 100)
            twoPartLift(robot, 500, 0.5, 0.17, 1)
            Movement.backwardMovement(robot, 40)
            twoPartLift(robot, 500, 0.65, 0.2, 0)
            Movement.turnOnSpot(robot, 90)
            twoPartLift(robot, 500, 0.65, 0, 0)
            Movement.backwardMovement(robot, 100)
            Detection.stopOnLine(robot, 100)
    else:
        if(direction == "counterclockwise"):
            Movement.forwardMovement(robot, 90)
            Movement.turnUntilLine(robot, "LEFT")
            twoPartLift(robot, 500, 0.5, 0, 0)
            distance = Detection.lineFollowUntilColorIntersection(robot, 300, 50)
            twoPartLift(robot, 500, 0, 0, 0)
            Movement.forwardMovement(robot, 94 + basesDropped * 60)
            Movement.backwardMovement(robot, basesDropped * 56)
            twoPartLift(robot, 500, 0, 1, 0)
            Movement.backwardMovement(robot, 100)
            twoPartLift(robot, 500, 0.5, 0.17, 1)
            Movement.backwardMovement(robot, 40)
            twoPartLift(robot, 500, 0.65, 0.2, 0)
            Movement.turnOnSpot(robot, 90)
            twoPartLift(robot, 500, 0.65, 0, 0)
            Movement.backwardMovement(robot, 100)
            Detection.stopOnLine(robot, 200)
        if(direction == "clockwise"):
            Movement.forwardMovement(robot, 90)
            Movement.turnUntilLine(robot, "RIGHT")
            twoPartLift(robot, 500, 0.5, 0, 0)
            distance = Detection.lineFollowUntilColorIntersection(robot, 300, 50)
            twoPartLift(robot, 500, 0, 0, 0)
            Movement.forwardMovement(robot, 94 + basesDropped * 60)
            Movement.backwardMovement(robot, basesDropped * 56)
            twoPartLift(robot, 500, 0, 1, 0)
            Movement.backwardMovement(robot, 100)
            twoPartLift(robot, 500, 0.5, 0.17, 1)
            Movement.backwardMovement(robot, 40)
            twoPartLift(robot, 500, 0.65, 0.2, 0)
            Movement.turnOnSpot(robot, -90)
            twoPartLift(robot, 500, 0.65, 0, 0)
            Movement.backwardMovement(robot, 100)
            Detection.stopOnLine(robot, 200)

def dropScientist(robot:WROrobot):
    """Drops scientist block

    Args:
        robot (WROrobot): Robot object used for competition
    Raises:
        None

    Returns:
        None
    """
    robot.motor_2.run_target(1000, 120, Stop.HOLD, True)
    robot.motor_2.run_until_stalled(-1000, Stop.COAST, 50)

def dropVisitor(robot:WROrobot):
    """Drops visitors block
    Args:
        robot (WROrobot): Robot object used for competition
    Raises:
        None

    Returns:
        None
    """
    robot.motor_1.run_target(1000, -120, Stop.HOLD, True)
    robot.motor_1.run_until_stalled(1000, Stop.COAST, 50)

def dropOff(robot:WROrobot, dropType):
    """Drops scientist block

    Args:
        robot (WROrobot): Robot object used for competition
        dropType (int): Takes in 0, 1, 2 to determine the type of drop off performed
    Raises:
        TypeError: Whhen dropType is not 0, 1, or 2

    Returns:
        None
    """

    if not type(dropType) in [int]:
        raise TypeError("dropType must be of type int")

    if(abs(dropType) == 0):
        dropVisitor(robot)
    elif(abs(dropType) == 1):
        dropScientist(robot)
        dropVisitor(robot)
    elif(abs(dropType) == 2):
        dropScientist(robot)
    else:
        raise TypeError("Option {} not found, please try again".format(dropType))

def initBlockCollect(robot:WROrobot):
    """This function first uses motor_1 to set the claw position of the robot to be raised so that the robot's claw does not hit the processing cubes to its side.
    The function then runs motor_2 to move the block collect mechanism on the robot to its outward position so that it can later be used to push processing cubes underneath the robot.


    Args:
        robot (WROrobot): Robot object used for competition
    Raises:
        None
    Returns:
        None
    """
    clawGrab(robot, "pick")
    robot.motor_1.run_angle(1000, 210, Stop.HOLD, True)
    robot.motor_2.run_until_stalled(600, Stop.HOLD, 90)

def collectBlock(robot:WROrobot):
    """This function first uses motor_1 to set the claw position of the robot to be raised so that the robot's claw does not hit the processing cubes to its side.
    The function then runs motor_2 to move the block collect mechanism on the robot to its outward position so that it can later be used to push processing cubes underneath the robot.


    Args:
        robot (WROrobot): Robot object used for competition
    Raises:
        None
    Returns:
        None
    """
    robot.motor_2.run_angle(500, -80, Stop.HOLD, True)
    robot.motor_2.run_until_stalled(300, Stop.HOLD, 90)

def blockHold(robot:WROrobot):
    """This function calls the claw grab function to raise the claw on motor_1 up and runs motor_2 by 1000 degrees to hold the process blocks in place by blocking the entrance to the storage compartment located under the robot.
    Args:
        robot (WROrobot): Robot object used for competition
    Raises:
        None
    Returns:
        None
    """
    clawGrab(robot, "pick")
    robot.motor_2.run_angle(1000, -80, Stop.HOLD, True)

def elevatorReset(robot:WROrobot):
    """Resets elevator to neutral/starting position

    Args:
        robot (WROrobot): Robot object used for competition

    Raises:
        None

    Returns:
        None
    """
    #This sets the elevator so the platforms carrying the blocks are upright and ready to be delivered,
    # first line resets elevator to base position
    robot.motor_2.run_until_stalled(-500, Stop.HOLD, 60)
    robot.motor_2.reset_angle(0)
    #robot.motor_2.run_angle(500, -90, Stop.HOLD, True)

def elevatorDrop(robot:WROrobot, blockNumber):
    """Moves the elevator a certain distance allowing a single block to drop

    Args:
        robot (WROrobot): Robot object used for competition

    Raises:
        None

    Returns:
        None
    """
    #This sets the elevator so the platforms carrying the blocks are upright and ready to be delivered,
    # first line resets elevator to base position
    robot.motor_2.run_angle(-500, -120*blockNumber, Stop.HOLD, True) #Second parameter(angle) needs to be tested with robot and changed accordingly

def setHolderPosition(robot:WROrobot, numRotations, command):
    """Moves the trash holder to a certain position

    Args:
        robot (WROrobot): Robot object used for competition
        numRotations: the number of spots to turn
        command: which direction the spinner should go

    Raises:
        None

    Returns:
        None
    """
    if (command == "turn"):
        robot.motor_2.run_angle(300, numRotations*90)
    elif (command == "reset"):
        robot.motor_2.run_angle(300, numRotations*-90)

def seniorClaw2016(robot:WROrobot, position, speed=200):
    """Moves the claw to the desired position

    Args:
        robot (WROrobot): Robot object used for competition
        position: which position the claw should go to
        speed: how fast the claw should move

    Raises:
        None

    Returns:
        None
    """
    if (position == "open"):
        robot.motor_1.run_target(speed, 120)
    elif (position == "close"):
        robot.motor_1.run_target(speed, 0)
    elif (position == "lift"):
        robot.motor_1.run_target(speed, -135)
    elif (position == "press"):
        robot.motor_1.run_until_stalled(-1*speed)
    elif (position == "init"):
        robot.motor_1.run_until_stalled(100)
        robot.motor_1.reset_angle(270)
        robot.motor_1.run_target(speed, 0)

