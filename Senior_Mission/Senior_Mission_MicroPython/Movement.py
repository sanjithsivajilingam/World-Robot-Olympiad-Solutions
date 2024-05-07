#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from RobotBody import WROrobot
import Lift
import Detection


def forwardMovement(robot: WROrobot, distanceToTravel: int):
    """
    Moves the robot forward a user given amount of distance

    Args:
        robot (robot object): A robot object
        distanceToTravel (int): A distance for the robot to travel in millimeters

    Raises:
        TypeError: distanceToTravel is not of type int

    Returns:
        None

    """

    if not type(distanceToTravel) in [int]:
        raise TypeError("distanceToTravel must be of type int")

    distance = 0

    startDistance = robot.driveBase.distance()

    while (distance < abs(distanceToTravel)):
        robot.driveBase.drive(robot.DRIVE_SPEED, 0)
        distance = robot.driveBase.distance() - startDistance
    robotStop(robot)
    # return None


def backwardMovement(robot: WROrobot, distanceToTravel: int):
    """
    Moves the robot backward a user given amount of distance

    Args:
        robot (robot object): A robot object
        distanceToTravel (int): A distance for the robot to travel in millimeters

    Raises:
        TypeError: distanceToTravel is not of type int

    Returns:
        None

    """
    if not type(distanceToTravel) in [int]:
        raise TypeError("distanceToTravel must be of type int")

    distance = 0

    startDistance = robot.driveBase.distance()

    while (distance < abs(distanceToTravel)):
        robot.driveBase.drive(-robot.DRIVE_SPEED, 0)
        distance = startDistance - robot.driveBase.distance()
    robotStop(robot)


def turnUntilLine(robot: WROrobot, direction):
    """Allows robot to turn in a specific direction until it reaches a black line then will stop the robot

    Args:
        robot (WROrobot): Robot object used for competition
        direction (int): direction in which the robot will turn

    Raises:
        TypeError: direction is not of type string

    Returns:
        None
    """
    if not type(direction) in [str]:
        raise TypeError("direction must be of type string")

    if(direction == "RIGHT"):
        robot.driveBase.drive(0,30)
        while(robot.light_1.color() != Color.BLACK):
            None
    elif(direction == "LEFT"):
        robot.driveBase.drive(0,-30)
        while(robot.light_2.color() != Color.BLACK):
            None
    robotStop(robot)


def arcTurn(robot: WROrobot, radius, wheelWidth, innerSpeed, direction, time):
    """Allows robot to perform and arcturn then commands the robot to stop

    This function utilizes the formula found below in order to determine what speed the outer wheel must run in order for the robot to perform an arc turn with the radius specified:

    $$outerSpeed = innerSpeed * (2*radius + wheelWidth)/(2*radius - wheelWidth)$$

    Args:
        robot (WROrobot): Robot object used for competition
        radius (int): radius of robot's wheels
        wheelWidth (int): width of the robot's wheels
        innerSpeed (int): speed of inner wheel
        time (int): time each wheel will move for
        direction (str): direction the robot will turn

    Raises:
        TypeError: radius is not of type int
        TypeError: wheelWidth is not of type int
        TypeError: innerSpeed is not of type int
        TypeError: time is not of type int
        TypeError: direction is not of type string

    Returns:
        None
    """

    if not type(radius) in [int]:
        raise TypeError("radius must be of type int")

    if not type(wheelWidth) in [int]:
        raise TypeError("wheelWidth is not of type int")

    if not type(innerSpeed) in [int]:
        raise TypeError("innerSpeed is not of type int")

    if not type(time) in [int]:
        raise TypeError("time is not of type int")

    if not type(direction) in [str]:
        raise TypeError("direction is not of type string")

    absRadius = abs(radius)
    abswheelWidth = abs(wheelWidth)
    absinnerSpeed = abs(innerSpeed)
    absTime = abs(time)

    outerSpeed = absinnerSpeed * \
        (2*absRadius + abswheelWidth)/(2*absRadius - abswheelWidth)
    if(direction == "RIGHT"):
        robot.left_wheel.run_time(outerSpeed, absTime, Stop.HOLD, False)
        robot.right_wheel.run_time(absinnerSpeed, absTime, Stop.HOLD, True)
    elif(direction == "LEFT"):
        robot.left_wheel.run_time(absinnerSpeed, absTime, Stop.HOLD, False)
        robot.right_wheel.run_time(outerSpeed, absTime, Stop.HOLD, True)
    robot.left_wheel.stop()
    robot.right_wheel.stop()

#----------Speed----------#


def setSpeed(robot: WROrobot, speed):
    """
    Sets the default Speed of the robot

    Args:
        robot (robot object): A robot object
        speed (int): A speed for the robot to travel in millimeters per second

    Raises:
        TypeError: speed is not of type int

    Returns:
        None

    """
    if not type(speed) in [int]:
        raise TypeError("speed must be of type int")

    robot.DRIVE_SPEED = abs(speed)


def setPIDSettings(robot: WROrobot, kp, ki, kd):
    """Arguments
    Robot - Takes in the predefined robot class created for this library, the robotParam["comp"] should be set to "junior" or "elementary" or "senior".

    This function sets the Kp, Ki and Kd values of the inputted WROrobot object. By default these values are set to 0.5, 0.005, 0.01. Whenever a function that utilizes PID controls for line following takes in a WROrobot instance it utilizes the Kp, Ki and Kd values of the inputted WROrobot instance."""
    robot.Kp = kp
    robot.Ki = ki
    robot.Kd = kd

#----------Turning----------#


def turnOnSpot(robot: WROrobot, angle):
    """Allows the robot to turn on the spot, then will stop the robot

    Args:
        robot (robot object): A robot object
        angle (int): angle of the desired turn

    Raises:
        TypeError: angle is not of type int

    Returns:
        None

    """
    if not type(angle) in [int]:
        raise TypeError("angle must be of type int")

    robot.driveBase.reset()
    if(angle > 0):
        robot.driveBase.drive(0, 80)
        while(robot.driveBase.angle() < angle):
            None
        robot.driveBase.stop()
    elif(angle < 0):
        robot.driveBase.drive(0, -80)
        while(robot.driveBase.angle() > angle):
            None
        robot.driveBase.stop()
    robotStop(robot)

def robotStop(robot: WROrobot):
    """Helper function to stop the robot

    Args:
        robot (robot object): A robot object

    Raises:
        None

    Returns:
        None

    """
    robot.driveBase.stop()
    # robot.left_wheel.brake()
    # robot.right_wheel.brake()


def PIDlineFollower(robot: WROrobot, distanceToTravel, speed, side):
    """Allows robot to follow the black line on the competition mats

    Args:
        robot (robot object): A robot object
        distanceToTravel (int): Indicates the distance robot should travel
        speed (int): Speed of the robot
        side (str): Indicates what side of the line robot is

    Raises:
        TypeError: distanceToTravel must be of type int
        TypeError: speed must be of type int
        TypeError: side must be of type str

    Returns:
        None

    """

    if not type(distanceToTravel) in [int]:
        raise TypeError("distanceToTravel must be of type int")

    if not type(speed) in [int]:
        raise TypeError("speed must be of type int")

    if not type(side) in [str]:
        raise TypeError("side must be of type str")

    Kp = robot.Kp
    Ki = robot.Ki
    Kd = robot.Kd
    light_2 = robot.light_2
    light_1 = robot.light_1
    startDistance = robot.driveBase.distance()
    distance = 0
    error = 0
    integral = 0
    lastError = 0
    derivative = 0

    # Continue to follow the line until the distance the robot has travelled is equal to the travel distance specified
    while ((distance < abs(distanceToTravel)) and (abs(distanceToTravel) >= 0)):
        if(side == "LEFT"):
            error = light_1.reflection() - robot.threshold
        elif(side == "RIGHT"):
            error = light_2.reflection() - robot.threshold
        integral = integral + error
        derivative = error - lastError
        turn_rate = Kp * error + Ki * integral + Kd * derivative
        robot.driveBase.drive(abs(speed), turn_rate *
                              ((-1 if side == "LEFT" else 1) * -1))
        lastError = error
        distance = robot.driveBase.distance() - startDistance


def dualSensorPIDlineFollower(robot: WROrobot, distanceToTravel, speed):
    """
    This function requires the robot to have two downward facing color sensors located at the front of it and should be called when the robot's color sensors are each located roughly over the edges of opposite sides of a line the robot is required to follow. The function will then use each color sensor's reflective value reading and compare the difference between the two to determine if the robot needs to turn right or left to stay following the line. If the robot begins to deviate to the left or right from the line this function will cause the robot to correct itself by making slight turns left or right depending on the magnitude of the previous error. The robot will continue to follow a line at the speed set in the speed parameter until it's traveled the distance specified in the distanceToTravel input parameter. This function utilizes the Ki, Kp & Kd values set in the instance variables of the WROrobot class passed in as the robot parameter.

    Arguments
        Robot - Takes in the predefined robot class created for this library, the robotParam["comp"] should be set to "junior" or "senior".
        distanceToTravel - Takes in an integer/ double which defines the distance in millimeters the robot will continue to travel following the line unless the robot detects an intersection
        Speed - Takes in an integer/ double which defines the speed the robot will travel in mm/s
   """

    Kp = robot.Kp
    Ki = robot.Ki
    Kd = robot.Kd
    light_2 = robot.light_2
    light_1 = robot.light_1
    startDistance = robot.driveBase.distance()
    distance = 0
    error = 0
    integral = 0
    lastError = 0
    derivative = 0

    # Continue to follow the line until the distance the robot has travelled is equal to the travel distance specified
    while ((distance < abs(distanceToTravel)) and (abs(distanceToTravel) >= 0)):
        error = light_1.reflection() - light_2.reflection()
        integral = integral + error
        derivative = error - lastError
        turn_rate = Kp * error + Ki * integral + Kd * derivative
        robot.driveBase.drive(abs(speed), turn_rate)
        lastError = error
        distance = robot.driveBase.distance() - startDistance

    robotStop(robot)


def nodeTraversal(robot: WROrobot, startingNode, finishNode, direction):
    """Based on the startingNode and finishNode input, this function computes the path the robot needs to travel in order to get from its current position (startingNode) to its final position (finishNode).
    After the robot is finished traveling to its destination this function returns a string representing the direction the robot is currently facing, either "clockwise" or "counterclockwise"
    described by the red arrow in Figure A in the documentation appendix.
    Args:
        robot (robot object): A robot object
        startingNode (int): Indicates node robot is starting at. Can be numbers 1 - 4.
        finishNode (int): Indicates node robot is ending at. Can be numbers 1-4.
        direction (str): Indicates the direction of travel robot will follow.
    Raises:
        None

    Returns:
        finishDirection (str): Direction robot is facing after node traversal
    """
    finishDirection = direction

    s = startingNode
    f = finishNode

    difference = f - s

    if(difference == 1 or difference == -3):
        if(direction == "counterclockwise"):
            forwardMovement(robot, 300)
            turnOnSpot(robot, -90)
            Detection.stopOnLine(robot, 475)
        else:
            backwardMovement(robot, 90)
            turnOnSpot(robot, 90)
            Detection.stopOnLine(robot, 200)
        finishDirection = "counterclockwise"

    if(difference == 1 or difference == -3):
        if(direction == "counterclockwise"):
            forwardMovement(robot, 300)
            turnOnSpot(robot, -90)
            Detection.stopOnLine(robot, 475)
        else:
            backwardMovement(robot, 90)
            turnOnSpot(robot, 90)
            Detection.stopOnLine(robot, 200)
        finishDirection = "counterclockwise"

    elif(difference == 3 or difference == -1):
        if(direction == "counterclockwise"):
            backwardMovement(robot, 80)
            turnOnSpot(robot, -90)
            Detection.stopOnLine(robot, 200)
        else:
            forwardMovement(robot, 270)
            turnOnSpot(robot, 90)
            Detection.stopOnLine(robot, 200)
        finishDirection = "clockwise"

    elif(difference == 2):
        directionPlaceholder = nodeTraversal(robot, 1, 2, direction)
        finishDirection = nodeTraversal(robot, 2, 3, directionPlaceholder)

    elif(difference == - 2):
        directionPlaceholder = nodeTraversal(robot, 3, 2, direction)
        finishDirection = nodeTraversal(robot, 2, 1, directionPlaceholder)

    return finishDirection


def findConstructionPath(robot: WROrobot, constructionAreas):
    """Based on the inputted constructionAreas array this function calculates and returns an array of size 3 which contains integers that represent the optimal node path the robot should travel to in order to construct three wind generators at the appropriate construction areas.
    The array returned has index 0 representing the first node the robot should travel to, index 1 representing the second node the robot should travel to and index 2 being the last node the robot should travel to.
    The nodes are described in Figure A of the appendix where an integer value from 1-4 represents all the node locations on the game mat.
    Args:
        robot (robot object): A robot object
        constructionAreas (list): Array of Color values used to indicated the construction area robot must travel to in order to build the wind generators
    Raises:
        None

    Returns:
        node (list): List of nodes robot has to go to
    """
    connections = {1: {Color.BLACK, Color.RED}, 2: {Color.BLUE, Color.RED}, 3: {
        Color.YELLOW, Color.RED}, 4: {Color.GREEN, Color.RED}}
    nodes = [0, 0, 0]
    i = 0
    while i != 3:
        for item in connections.items():
            if constructionAreas[i] in item[1]:
                nodes[i] = item[0]
                if constructionAreas[i] == Color.RED and i != 2:
                    for item2 in connections.items():
                        if constructionAreas[i + 1] in item2[1]:
                            nodes[i] = item2[0]
                            i = i + 1
                            nodes[i] = item2[0]
        i = i + 1
    return nodes


def startNodeFinder(robot: WROrobot, nodes):
    """The inputted array should contain the integer values of each node the robot will be traveling to in the correct order meaning the robot should first travel to the node described by nodes[0] then nodes[1] and then nodes[2].
    Based on the first node the robot is going to travel to, this function determines the most efficient node for the robot to start at to traverse its path.
    This function should be called when the robot is roughly at the starting point described and labeled by Figure A in the appendix.
    The function first moves the robot forward until it hits a line and then turns to either Node 4 or Node 1 described in Figure A in the appendix depending on the optimal path computed.
    This function returns an integer representing the node the robot has moved to, 4 or 1.
    If the robot moves to node 4 it will be facing in the clockwise direction, otherwise if the robot moves to node 1 it will be facing the counterclockwise direction as described in Figure A in the appendix.

    Args:
        robot (robot object): A robot object
        nodes (list): Indicates the nodes robot has to go to
    Raises:
        None

    Returns:
        startNode (int): The node the robot has to start at
    """
    startNode = 0
    if (nodes[0] == 4 or nodes[0] == 3):
        Detection.stopOnLine(robot, 200)
        forwardMovement(robot, 100)
        turnOnSpot(robot, -45)
        Detection.stopOnLine(robot, 200)
        startNode = 4
    if (nodes[0] == 1 or nodes[0] == 2):
        Detection.stopOnLine(robot, 200)
        forwardMovement(robot, 80)
        turnOnSpot(robot, 45)
        Detection.stopOnLine(robot, 200)
        startNode = 1
    return startNode


def branchTraversal(robot: WROrobot, speed):
    """Goes to branch and decides which branch to go to depending on where the wall is (will go left or right)"""

    turbineBase = None
    technologyDecider = None
    TurbineBaseTechnologyDecider = []
    Detection.lineFollowUntilIntersection(robot, 10000, speed)
    forwardMovement(robot, 110)

    if(robot.color_2.color() != None):
        turbineBase = robot.color_2.color()
        if turbineBase == Color.BLACK or turbineBase == Color.GREEN:
            if robot.color_2.rgb()[2] > robot.color_2.rgb()[1] and robot.color_2.rgb()[2] > 5:
                turbineBase = Color.BLUE
        if turbineBase == Color.BROWN:
            turbineBase = Color.YELLOW
        backwardMovement(robot, 10)
        turnUntilLine(robot, "LEFT")
        Detection.lineFollowUntilIntersection(robot, 1000, speed)
        Lift.twoPartLift(robot, 500, 0.0, 0.0, 0)
        backwardMovement(robot, 100)
        turnOnSpot(robot, 45)
        turnUntilLine(robot, "RIGHT")
        Lift.twoPartLift(robot, 500, 0.0, 0.0, 0)
        Detection.lineFollowUntilIntersection(robot, 10000, 100)
        Lift.twoPartLift(robot, 1000, 0.0, 1, 1)
        forwardMovement(robot, 45)
        robot.left_wheel.run_angle(50, 90, Stop.HOLD, True)
        technologyDecider = robot.color_1.color()
        robot.left_wheel.run_angle(-50, 80, Stop.HOLD, True)
        Lift.twoPartLift(robot, 500, 0.35, 0.0, 1)
        Lift.twoPartLift(robot, 500, 0.0, 0.0, 0)
        backwardMovement(robot, 170)
        turnOnSpot(robot, -45)
    elif(robot.color_2.color() == None):
        backwardMovement(robot, 10)
        turnUntilLine(robot, "RIGHT")
        Detection.lineFollowUntilIntersection(robot, 1000, speed)
        Lift.twoPartLift(robot, 500, 0.0, 0.0, 0)
        turbineBase = robot.color_1.color()
        if turbineBase == Color.BLACK or turbineBase == Color.GREEN:
            if robot.color_1.rgb()[2] > robot.color_1.rgb()[1] and robot.color_1.rgb()[2] > 5:
                turbineBase = Color.BLUE
        backwardMovement(robot, 100)
        turnOnSpot(robot, -45)
        turnUntilLine(robot, "LEFT")
        Lift.twoPartLift(robot, 500, 0.0, 0.0, 0)
        Detection.lineFollowUntilIntersection(robot, 10000, 100)
        Lift.twoPartLift(robot, 1000, 0.0, 1, 1)
        forwardMovement(robot, 45)
        robot.left_wheel.run_angle(50, 90, Stop.HOLD, True)
        technologyDecider = robot.color_1.color()
        robot.left_wheel.run_angle(-50, 80, Stop.HOLD, True)
        Lift.twoPartLift(robot, 500, 0.35, 0.0, 1)
        Lift.twoPartLift(robot, 500, 0.0, 0.0, 0)
        backwardMovement(robot, 170)
        turnOnSpot(robot, 45)

    TurbineBaseTechnologyDecider.append(turbineBase)
    TurbineBaseTechnologyDecider.append(technologyDecider)

    return TurbineBaseTechnologyDecider

def nodeTraversal2016(robot, startingNode, endingNode):
    """Allows the robot to move from any node on the senior mat to any other node in stage 1. or move to stage 2 when nessesary. 
    Given the starting and ending node it calculates the moves nessesary to park the claw in line with the block and right in front of it

    Args:
        robot (robot object): A robot object
        startingNode: Where the robot currently is
        endingNode: where the robot needs to go

    Raises:
        None

    Returns:
        None
    """
    # replaces node colors with value, lower means closer to the start
    nodeLookup = {Color.GREEN:1, Color.RED:2, Color.BLUE:3, Color.YELLOW:4}
    rightAngle = 97
    negativeRightAngle = -100
    greenBackupAmmount = 0
    redBackupAmmount = 80           
    blueBackupAmmount = 220
    yellowBackupAmmount = 280

    # if it has a longer starting spot then move until you are at the main bus line
    if (startingNode == Color.BLUE or startingNode == Color.YELLOW):
        Detection.stopOnLine(robot, robot.DRIVE_SPEED)
        # forwardMovement(robot, 30)

    # if we need to go to stage 2 just move down the mat
    if (endingNode == "stage2"):
        forwardMovement(robot, 110)
        if (startingNode == Color.GREEN or startingNode == Color.BLUE):
            turnOnSpot(robot, 50)
            turnUntilLine(robot, "RIGHT")
            turnOnSpot(robot, -10)
            forwardMovement(robot, 30)
        else:
            turnOnSpot(robot, -50)
            turnUntilLine(robot, "LEFT")
            turnOnSpot(robot, 10)
            forwardMovement(robot, 30)
        Lift.seniorClaw2016(robot, "close")
        Detection.lineFollowUntilLineIntersection(robot, 1000, robot.DRIVE_SPEED)
        return

    # compare the nodes to see what needs to be done to get the robot lined up with the main bus in the right direction
    if (nodeLookup[startingNode] < nodeLookup[endingNode]):
        if (startingNode == Color.GREEN or startingNode == Color.BLUE):
            print("Case that starting Node is GREEN or BLUE")
            forwardMovement(robot, 80)
            turnOnSpot(robot, 50)
            turnUntilLine(robot, "RIGHT")
            turnOnSpot(robot, -10)
            forwardMovement(robot, 30)
        else:
            forwardMovement(robot, 130)
            turnOnSpot(robot, -50)
            turnUntilLine(robot, "LEFT")
            backwardMovement(robot, 200)
            turnOnSpot(robot, 10)
    else :
        if (startingNode == Color.YELLOW or startingNode == Color.RED):
            print("Case that starting Node is YELLOW or RED")
            forwardMovement(robot, 80)
            turnOnSpot(robot, 50)
            turnUntilLine(robot, "RIGHT")
            turnOnSpot(robot, -10)
            forwardMovement(robot, 30)
        else:
            print("Else condition")
            forwardMovement(robot, 130)
            turnOnSpot(robot, -50)
            turnUntilLine(robot, "LEFT")
            #backwardMovement(robot, 200)
            turnOnSpot(robot, 10)
    Lift.seniorClaw2016(robot, "lift")
    # if the difference in traversal is 3 stops its a max range move, yellow to green or vica versa
    print(abs(nodeLookup[startingNode]-nodeLookup[endingNode]))
    if (abs(nodeLookup[startingNode]-nodeLookup[endingNode]) == 3):
        Detection.PIDlineFollowUntilTurn(robot, 250, robot.DRIVE_SPEED, "LEFT")
        forwardMovement(robot, 50)
        Detection.PIDlineFollowUntilTurn(robot, 1000, robot.DRIVE_SPEED, "LEFT")
        PIDlineFollower(robot, 225, robot.DRIVE_SPEED, "RIGHT")
        turnOnSpot(robot, 102)
        if (startingNode == Color.GREEN):
            backwardAmount = 170
        else:
            backwardAmount = 50
    # if its only 2 then more options available, find the correct move
    elif (abs(nodeLookup[startingNode]-nodeLookup[endingNode]) == 2):
        if (startingNode == Color.YELLOW or startingNode == Color.GREEN):
            Detection.PIDlineFollowUntilTurn(robot, 1000, robot.DRIVE_SPEED, "RIGHT")
            turnOnSpot(robot, negativeRightAngle)
            if (startingNode == Color.YELLOW):
                backwardAmount = 0
            else:
                backwardAmount = 125
        else:
            print("Going to green")
            Detection.PIDlineFollowUntilTurn(robot, 1000, robot.DRIVE_SPEED, "LEFT")
            PIDlineFollower(robot, 220, robot.DRIVE_SPEED, "RIGHT")
            turnOnSpot(robot, rightAngle)
            if (startingNode == Color.RED):
                backwardAmount = 170
            else:
                backwardAmount = 50
    # one move, even more options, find correct movement again
    elif (abs(nodeLookup[startingNode]-nodeLookup[endingNode]) == 1):
        if (nodeLookup[startingNode] < nodeLookup[endingNode]):
            if (startingNode == Color.GREEN or startingNode == Color.BLUE):
                Detection.PIDlineFollowUntilTurn(robot, 1000, robot.DRIVE_SPEED, "LEFT")
                PIDlineFollower(robot, 220, robot.DRIVE_SPEED, "RIGHT")
                turnOnSpot(robot, rightAngle)
                if (startingNode == Color.BLUE):
                    backwardAmount = 160
                else:
                    backwardAmount = 55
            else:
                Detection.PIDlineFollowUntilTurn(robot, 1000, robot.DRIVE_SPEED, "RIGHT")
                turnOnSpot(robot, negativeRightAngle)
                backwardAmount = 135
        else:
            if (startingNode == Color.YELLOW or startingNode == Color.RED):
                Detection.PIDlineFollowUntilTurn(robot, 1000, robot.DRIVE_SPEED, "LEFT")
                PIDlineFollower(robot, 225, robot.DRIVE_SPEED, "RIGHT")
                turnOnSpot(robot, rightAngle)
                if (startingNode == Color.YELLOW):
                    backwardAmount = 160
                else:
                    backwardAmount = 50
            else:
                Detection.PIDlineFollowUntilTurn(robot, 1000, robot.DRIVE_SPEED, "RIGHT")
                turnOnSpot(robot, negativeRightAngle)
                backwardAmount = 0
    # open the claw when you arrive and move back to the block
    Lift.seniorClaw2016(robot, "open")
    
    #Detection.stopOnLine(robot, -robot.DRIVE_SPEED)
    
    if endingNode == Color.GREEN:
        backwardAmount = greenBackupAmmount
    if endingNode == Color.RED:
        backwardAmount = redBackupAmmount
    if endingNode == Color.BLUE:
        backwardAmount = blueBackupAmmount
    if endingNode == Color.YELLOW:
        backwardAmount = yellowBackupAmmount
        print("Yellow")
    print("Moving backwards by set backup amount")
    backwardMovement(robot, backwardAmount)

def hexagonFollower2016(robot: WROrobot):
    """Allows the robot to move around the hexagon and detect any position of tanks. Only moves foward if its on the line, if not it
    moves back and to the left to make sure nothing is missed

    Args:
        robot (robot object): A robot object

    Raises:
        None

    Returns:
        The currect color of the tank it stopped at
    """
    colorFound = None
    # loop until block is found
    while(colorFound == None):
        # if its off the line move back and the the left until it finds the line again
        if (robot.color_1.reflection() > 12):
            robot.driveBase.drive(-50,-35)
        else:
            # move forward and slightly right to keep on outside edge
            robot.driveBase.drive(robot.DRIVE_SPEED, 10)
        # detect a block if one exists
        currentColor = Detection.detectBlockReflection2016(robot, "stage2")
        if (currentColor is not None):
                colorFound = currentColor
    # once a color is found stop, move back, and approach slowly to properly line up the tank so the block can be dropped accurately
    backwardMovement(robot, 50)
    turnOnSpot(robot, -6)
    print(colorFound)
    print("Entering second while loop")
    while (Detection.detectBlockReflection2016(robot, "stage2") is not colorFound):
        robot.driveBase.drive(50, 0)
    robot.driveBase.stop()
    forwardMovement(robot, 75)
    print(colorFound)
    return colorFound