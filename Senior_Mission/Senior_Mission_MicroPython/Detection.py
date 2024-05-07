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
import Movement

def colorStore(robot:WROrobot):
    """Stores 6 Colours in a row with a minimum separation distance of 75 mm

    Args:
        robot (WROrobot): Robot object used for competition

    Raises:
        None

    Returns:
        colors (list): array of colours stored
    """
    colors = [None,None,None,None,None,None]
    i = 0
    distanceTravelled = 0
    Lift.initBlockCollect(robot)
    robot.driveBase.drive(100, 5)
    robot.driveBase.reset()
    while(True):
        if((robot.color_2.color() == Color.RED and robot.color_2.reflection() > 16) or (robot.color_2.color() == Color.YELLOW and robot.color_2.reflection() > 43) or robot.color_2.color() == Color.GREEN or robot.color_2.color() == Color.WHITE):
            robot.driveBase.stop()
            if(robot.color_2.color() != Color.WHITE):
                colors[i] = robot.color_2.color()
                distanceTravelled = robot.driveBase.distance()
                robot.driveBase.drive(100, 5)
                while(robot.driveBase.distance() - distanceTravelled < 105):
                    None
                robot.driveBase.stop()
                Lift.collectBlock(robot)
            else:
                distanceTravelled = robot.driveBase.distance()
                robot.driveBase.drive(100, 5)
                while(robot.driveBase.distance() - distanceTravelled < 105):
                    None
            if(i == 5):
                robot.driveBase.stop()
                break
            i = i + 1
            robot.driveBase.drive(100, 5)
    robot.driveBase.stop()
    Lift.blockHold(robot)
    robot.colors = colors
    return colors


def push(robot:WROrobot, distanceToTravel):
    """Pushes object a certain distance
    Args:
        robot (WROrobot): Robot object used for competition
        distanceToTravel (int): Distance robot has to travel
    Raises:
        TypeError: If distanceToTravel is not of type int
    Returns:
        pushFlag (int): indicates if robot is pushing an object (1 for pushing, 0 for not pushing)
    """

    if not type(distanceToTravel) in [int]:
        raise TypeError("distanceToTravel must be of type int")

    distance = 0
    pushFlag1 = 0
    pushFlag2 = 0
    startDistance = robot.driveBase.distance()

    while (distance < abs(distanceToTravel)):
        robot.driveBase.drive(robot.DRIVE_SPEED, 0)
        distance = robot.driveBase.distance() - startDistance
        if(robot.touch_2.pressed()):
           pushFlag1 = 1
        if(robot.touch_1.pressed()):
            pushFlag2 = 1
    Movement.robotStop(robot)
    return (pushFlag1 + pushFlag2)

def lineFollowerScanTreeAndPickup(robot:WROrobot, distanceToTravel, speed):
    """Line follows then picks up a tree
    Args:
        robot (WROrobot): Robot object used for competition
        distanceToTravel (int): Distance robot has to travel
    Raises:
        None
    Returns:
        None
    """

    Lift.clawGrab(robot, "pick")
    robot.motor_1.run_angle(100, 105, Stop.HOLD, True)
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
    #Continue to follow the line until the distance the robot has travelled is equal to the travel distance specified
    while ((distance < abs(distanceToTravel)) and (abs(distanceToTravel) >= 0)):
        error = light_1.reflection() - light_2.reflection()
        integral = integral + error
        derivative = error - lastError
        turn_rate = Kp * error + Ki * integral + Kd * derivative
        robot.driveBase.drive(abs(speed), turn_rate)
        lastError = error
        distance = robot.driveBase.distance() - startDistance
        if(robot.color_1.color() != None and robot.color_2.color() != None and robot.color_1.color() in robot.colors and robot.color_2.color() in robot.colors):
            robot.driveBase.stop()
            ##leftcolor and rightcolor are global variables defined in the robot class instance
            robot.currentLeftTree = robot.color_1.color()
            robot.currentRightTree = robot.color_2.color()
            break
    Movement.backwardMovement(robot, 100)
    Lift.clawGrab(robot, "release")
    Movement.forwardMovement(robot, 90)
    Lift.clawGrab(robot, "pick")
    Movement.robotStop(robot)

def panelPickup(robot:WROrobot, distanceToTravel, speed):
    """Picks up a panel
    Args:
        robot (WROrobot): Robot object used for competition
        distanceToTravel (int): Distance robot has to travel
        speed (int): Speed in which robot will drive
    Raises:
        None
    Returns:
        None
    """
    Lift.clawGrab(robot, "pick")
    robot.motor_1.run_angle(100, 105, Stop.HOLD, True)
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

    #Continue to follow the line until the distance the robot has travelled is equal to the travel distance specified
    while ((distance < abs(distanceToTravel)) and (abs(distanceToTravel) >= 0)):
        error = light_1.reflection() - light_2.reflection()
        integral = integral + error
        derivative = error - lastError
        turn_rate = Kp * error + Ki * integral + Kd * derivative
        robot.driveBase.drive(abs(speed), turn_rate)
        lastError = error
        distance = robot.driveBase.distance() - startDistance
        if(robot.color_1.color() != None and robot.color_2.color() != None):
            robot.driveBase.stop()
            ##leftcolor and rightcolor are global variables defined in the robot class instance

            break
    Movement.backwardMovement(robot, 80)
    Lift.clawGrab(robot, "release")
    Movement.forwardMovement(robot, 75)
    Lift.clawGrab(robot, "pick")
    Movement.robotStop(robot)

def treeDropOffSpotLocator(robot:WROrobot):
    """Determines where the drop off location for a tree is
    Args:
        robot (WROrobot): Robot object used for competition
    Raises:
        None
    Returns:
        None
    """
    iteratorValueRight = 0
    for i in robot.colors:
        if(i == robot.currentRightTree):
            robot.colors[iteratorValueRight] = None
            break
        iteratorValueRight = iteratorValueRight+1
    if(iteratorValueRight >= 0 and iteratorValueRight <= 1):
        robot.currentRightDropOff = Color.RED
    elif(iteratorValueRight >= 2 and iteratorValueRight <= 3):
        robot.currentRightDropOff = Color.WHITE
    elif(iteratorValueRight >= 4 and iteratorValueRight <= 5):
        robot.currentRightDropOff = Color.BLUE


    iteratorValueLeft = 0
    for i in robot.colors:
        if(i == robot.currentLeftTree):
            robot.colors[iteratorValueLeft] = None
            break
        iteratorValueLeft = iteratorValueLeft+1
    if(iteratorValueLeft >= 0 and iteratorValueLeft <= 1):
        robot.currentLeftDropOff = Color.RED
    elif(iteratorValueLeft >= 2 and iteratorValueLeft <= 3):
        robot.currentLeftDropOff = Color.WHITE
    elif(iteratorValueLeft >= 4 and iteratorValueLeft <= 5):
        robot.currentLeftDropOff = Color.BLUE


def lineFollowUntilLineIntersection(robot: WROrobot, distanceToTravel, speed):
    """Follows line until robot reaches a line intersection on the competition mat

    Args:
        robot (robot object): A robot object
        distanceToTravel (int): Indicates the distance robot should travel
        speed (int): Speed of the robot

    Raises:
        None

    Returns:
        None
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
        if(robot.light_1.reflection() <= 15 and robot.light_2.reflection() <= 15):
            print("Stopping")
            robot.driveBase.stop()
            break
    Movement.robotStop(robot)
    return robot.driveBase.distance() - startDistance


def lineFollowUntilColorIntersection(robot: WROrobot, distanceToTravel, speed):
    """Follows line until robot reaches an intersection of any color except for white

    Args:
        robot (robot object): A robot object
        distanceToTravel (int): Indicates the distance robot should travel
        speed (int): Speed of the robot

    Raises:
        None

    Returns:
        None
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
        if((robot.light_1.reflection() >= 30 and robot.light_2.reflection() >= 30) or (robot.light_1.reflection() <= 15 and robot.light_2.reflection() <= 15)):
            robot.driveBase.stop()
            break
    Movement.robotStop(robot)
    return robot.driveBase.distance() - startDistance

def stopOnLine(robot: WROrobot, speed):
    """Allows robot to stop on the black line

    Args:
        robot (WROrobot): Robot object used for competition
        speed (int): Takes in an integer/ double which defines the speed the medium motors will rotate in mm/s.

    Raises:
        TypeError: speed is not of type int

    Returns:
        None
    """

    if not type(speed) in [int]:
        raise TypeError("speed must be of type int")

    robot.driveBase.drive(speed, 0)
    while(True):
        if(robot.light_2.color() == Color.BLACK and robot.light_1.color() == Color.BLACK):
            Movement.robotStop(robot)
            break

#Depreicated
def collectTurbinesOntoBase(robot:WROrobot, turbineDeciders, speed):
    """Collects turbines onto their designated base
    Args:
        robot (WROrobot): Robot object used for competition
        turbineDeciders (Color array): Colors of turbines that need to be put on the bases
        speed (int): Speed of the robit
    Raises:
        None
    Returns:
        Color that is being read
    """
    Lift.twoPartLift(robot, 500, 0.65, 0, 0)
    turbinesRequired = 0

    for i in range(3):
        if (turbineDeciders[i] != None):
            turbinesRequired = turbinesRequired + 1

    turbinesPickedUp = 0
    turbinesChecked = 0

    if (turbinesRequired == 0):
        return turbineDeciders

    Movement.dualSensorPIDlineFollower(robot, 45, 100)

    while(True):
        #Pickup Turbine and place into correct base or flick turbine out of the path
        turbineColor = averageColorDetect(robot.color_1)

        turbineBaseNumber = -1
        turbinesChecked = turbinesChecked + 1
        for i in range(3):
            if (turbineDeciders[i] == turbineColor):
                turbineBaseNumber = i

        if(turbineBaseNumber != -1):
            turbinesPickedUp = turbinesPickedUp + 1
            turbineDeciders[turbineBaseNumber] = None
            #Pickup turbine
            Lift.twoPartLift(robot, 500, 0.0, 0.8 if turbineBaseNumber == 2 and turbinesPickedUp > 1 else 2, 0)
            #Move backwards to place turbine in turbine base
            Movement.backwardMovement(robot, (turbineBaseNumber * 56) + 90)
            #Release turbine
            Lift.twoPartLift(robot, 500, 0.5, 0.48, 1)
            Lift.twoPartLift(robot, 500, 0.5, 1, 1)
            Movement.forwardMovement(robot, (turbineBaseNumber * 56) + 110)
            #Reinitialize grabber claw
            Lift.twoPartLift(robot, 500, 0.65, 0, 0)
        else:
            # Flick turbine out of the path
            Movement.backwardMovement(robot, 20)
            Lift.twoPartLift(robot, 1000, 2, 0, 0)
            Lift.twoPartLift(robot, 500, 0.65, 0, 0)
            Movement.forwardMovement(robot, 40)

        if(turbinesChecked == 4 or turbinesPickedUp == turbinesRequired):
            Movement.forwardMovement(robot, 30)
            Movement.turnOnSpot(robot, 160)
            Movement.turnUntilLine(robot, "RIGHT")
            lineFollowUntilIntersection(robot, 1300, 100)
            return turbineDeciders

        #Continue to follow the line until the distance the robot has travelled is equal to the travel distance specified
        Movement.dualSensorPIDlineFollower(robot, 87.5, 50)

#Depreicated
def averageColorDetect(colorSensor):
    """Determines the average colour that the sensor can detect
    Args:
       colorSensor: Colour sensor of the robot
    Raises:
        None
    Returns:
        Color that is being read
    """
    arrayOfColors = [0, 0, 0, 0, 0, 0, 0]
    for each in range(30):
        arrayOfColorsDetected = []
        modeOfArrayOfColorsDetected = None
        black = 0
        blue = 0
        green = 0
        yellow = 0
        red = 0
        white = 0
        brown = 0
        none = 0

        colorDetectCountArray = [0, 0, 0, 0, 0, 0, 0, 0]

        for i in range(30):

            colorDetected = colorSensor.color()
            arrayOfColorsDetected.append(colorDetected)

        for i in range(30):

            if(arrayOfColorsDetected[i] == Color.BLACK):
                black = black + 1
                colorDetectCountArray[0] = black
            if(arrayOfColorsDetected[i] == Color.BLUE):
                blue = blue + 1
                colorDetectCountArray[1] = blue
            if(arrayOfColorsDetected[i] == Color.GREEN):
                green = green + 1
                colorDetectCountArray[2] = green
            if(arrayOfColorsDetected[i] == Color.YELLOW):
                yellow = yellow + 1
                colorDetectCountArray[3] = yellow
            if(arrayOfColorsDetected[i] == Color.RED):
                red = red + 1
                colorDetectCountArray[4] = red
            if(arrayOfColorsDetected[i] == Color.WHITE):
                white = white + 1
                colorDetectCountArray[5] = white
            if(arrayOfColorsDetected[i] == Color.BROWN):
                brown = brown + 1
                colorDetectCountArray[6] = brown
            if(arrayOfColorsDetected[i] == None):
                none = none + 1
                colorDetectCountArray[7] = none

        max_value = max(colorDetectCountArray)
        index_max_value = colorDetectCountArray.index(max_value)

        if(colorDetectCountArray[4] > 20):
            #RED
            arrayOfColors[4] = arrayOfColors[4] + 1

        if(colorDetectCountArray[2] > 15 and colorSensor.rgb()[1] > colorSensor.rgb()[2]):
            #GREEN
            arrayOfColors[2] = arrayOfColors[2] + 1

        if(colorDetectCountArray[2] > 5 and colorDetectCountArray[0] >= 0 and colorSensor.rgb()[1] < colorSensor.rgb()[2]):
            #BLUE
            arrayOfColors[1] = arrayOfColors[1] + 1

        if (colorDetectCountArray[7] > 8 and colorSensor.rgb()[0] <= 1 and colorSensor.rgb()[1] <= 1 and colorSensor.rgb()[2] <= 1):
            #BLACK
            arrayOfColors[0] = arrayOfColors[0] + 1

        if(colorDetectCountArray[0] > 15 and colorSensor.rgb()[1] >= 4):
            #WHITE
            arrayOfColors[5] = arrayOfColors[5] + 1

        if(colorDetectCountArray[0] > 15 and colorSensor.rgb()[1] < 4):
            #YELLOW
            arrayOfColors[3] = arrayOfColors[3] + 1

        if(index_max_value == 6):
            #BROWN
            arrayOfColors[6] = arrayOfColors[6] + 1

    index_avg_value = arrayOfColors.index(max(arrayOfColors))

    if(index_avg_value == 0):
        return Color.BLACK
    if(index_avg_value == 1):
        return Color.BLUE
    if(index_avg_value == 2):
        return Color.GREEN
    if(index_avg_value == 3):
        return Color.YELLOW
    if(index_avg_value == 4):
        return Color.RED
    if(index_avg_value == 5):
        return Color.WHITE
    if(index_avg_value == 6):
        return Color.BROWN

def detectBlockColor(robot:WROrobot):
    """Measures the color of a surface and returns a color

    Args:
        robot (WROrobot): Robot object used for competition

    Raises:
        None

    Returns:
        Current Block Color
    """
    currentColor = robot.color_1.color()
    return currentColor

def PIDlineFollowerUntilBlock(robot: WROrobot, distanceToTravel, speed, side):
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
    lastColor = Color.BLACK
    consecutiveColor = 0

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
        color = robot.color_1.rgb()
        if (color[0] >= robot.RED or color[2] >= robot.BLUE):
            break
    Movement.robotStop(robot)
    return robot.driveBase.distance() - startDistance

def lineFollowUntilBlock(robot: WROrobot, distanceToTravel, speed):
    """Follows line until robot reaches a block on the competition mat

    Args:
        robot (robot object): A robot object
        distanceToTravel (int): Indicates the distance robot should travel
        speed (int): Speed of the robot

    Raises:
        None

    Returns:
        None
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
        color = robot.color_1.rgb()
        if (color[0] >= robot.RED or color[2] >= robot.BLUE):
            break
    Movement.robotStop(robot)
    return robot.driveBase.distance() - startDistance

def forwardMovementUntilSolidColor(robot: WROrobot, distanceToTravel, speed):
    """Forward movement until a solid color is seen

    Args:
        robot (robot object): A robot object
        distanceToTravel (int): Indicates the distance robot should travel
        speed (int): Speed of the robot

    Raises:
        None

    Returns:
        distance travelled
    """
    startDistance = robot.driveBase.distance()
    distance = 0
    while ((distance < abs(distanceToTravel)) and (abs(distanceToTravel) >= 0)):
        robot.driveBase.drive(abs(speed), 0)
        distance = robot.driveBase.distance() - startDistance
        if(robot.light_1.color() != None and robot.light_2.color() != None):
            print("Stopping")
            robot.driveBase.stop()
            break
    Movement.robotStop(robot)
    return robot.driveBase.distance() - startDistance

def lineFollowUntilTurn(robot: WROrobot, distanceToTravel, speed, side):
    """Follows line until robot reaches a turn on the competition mat

    Args:
        robot (robot object): A robot object
        distanceToTravel (int): Indicates the distance robot should travel
        speed (int): Speed of the robot

    Raises:
        None

    Returns:
        None
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
    threshold = 10

    # Continue to follow the line until the distance the robot has travelled is equal to the travel distance specified
    while ((distance < abs(distanceToTravel)) and (abs(distanceToTravel) >= 0)):
        error = light_1.reflection() - light_2.reflection()
        integral = integral + error
        derivative = error - lastError
        turn_rate = Kp * error + Ki * integral + Kd * derivative
        robot.driveBase.drive(abs(speed), turn_rate)
        lastError = error
        distance = robot.driveBase.distance() - startDistance
        if (side == "left"):
            if (light_1.reflection() < threshold):
                Movement.robotStop(robot)
                return robot.driveBase.distance() - startDistance
        else :
            if (light_2.reflection() < threshold):
                Movement.robotStop(robot)
                return robot.driveBase.distance() - startDistance
    Movement.robotStop(robot)
    return robot.driveBase.distance() - startDistance

def PIDlineFollowUntilTurn(robot: WROrobot, distanceToTravel, speed, side):
    """Allows robot to follow the black line on the competition mats until a turn is seen on the specified side

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
        Distance traveled

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
    threshold = 10

    # Continue to follow the line until the distance the robot has travelled is equal to the travel distance specified
    while ((distance < abs(distanceToTravel)) and (abs(distanceToTravel) >= 0)):
        if(side == "LEFT"):
            error = light_2.reflection() - robot.threshold
            if (light_1.reflection() < threshold):
                Movement.robotStop(robot)
                return robot.driveBase.distance() - startDistance
        elif(side == "RIGHT"):
            error = light_1.reflection() - robot.threshold
            if (light_2.reflection() < threshold):
                Movement.robotStop(robot)
                return robot.driveBase.distance() - startDistance
        integral = integral + error
        derivative = error - lastError
        turn_rate = Kp * error + Ki * integral + Kd * derivative
        robot.driveBase.drive(abs(speed), turn_rate *
                              ((-1 if side == "RIGHT" else 1) * -1))
        lastError = error
        distance = robot.driveBase.distance() - startDistance
    Movement.robotStop(robot)
    return robot.driveBase.distance() - startDistance

def detectBlockColor2016(robot: WROrobot, item):
    """Returns the color seen by the container or tank sensor

    Args:
        robot (robot object): A robot object
        item: Indicates which sensor should be checked

    Raises:
        None

    Returns:
        The color the sensor sees

    """
    if (item == "container"):
        print(robot.color_2.color())
        return robot.color_2.color()
    elif (item == "tank"):
        return robot.color_1.color()

def getBlockReflection2016(robot, item):
    """Returns the color seen by the container or tank sensor in rgb values

    Args:
        robot (robot object): A robot object
        item: Indicates which sensor should be checked

    Raises:
        None

    Returns:
        The color the sensor sees in rgb values

    """
    if (item == "container"):
        return robot.color_2.rgb()
    elif (item == "tank"):
        return robot.color_1.rgb()

def detectBlockReflection2016(robot: WROrobot, item):
    """Returns the color seen by the container or tank sensor using rgb values and value logic

    Args:
        robot (robot object): A robot object
        item: The current stage, which sensor to use 

    Raises:
        None

    Returns:
        The color the sensor sees

    """
    if (item == "stage1"):
        colorSensor = robot.color_2
    elif (item == "stage2"):
        colorSensor = robot.color_1

    red = colorSensor.rgb()[0]
    green = colorSensor.rgb()[1]
    blue = colorSensor.rgb()[2]
    threshold = 3
    threshold2 = 6
    thresholdGreen = 2
    
    
   
    if (item == "stage1"):
        print("R G B:")
        print(red, green, blue)
        if ((blue >= red) & (blue >= green)):
            return Color.BLUE
        elif ((red>green and red > blue and blue >= green)):
            return Color.RED
        elif ((green>=red) and (green>=blue)):
            return Color.GREEN
        elif ((red-green) <= 5):
            return Color.YELLOW
        print("R G B:")
        print(red, green, blue)
        print("Returns None")
        return None
    elif (item == "stage2"):
        print(red, green, blue)
        if ((red >= 30) and (green >= 20)):
            return Color.YELLOW
        elif (blue >= 40):
            return Color.BLUE
        elif (green >= 20):
            return Color.GREEN
        elif (red >= 25):
            return Color.RED
        else:
            return None
        # alternative method of checking color
        # if (red in range(15,28)) and (green in range(15, 28)) and (blue in range(5, 15)):
        #     return Color.YELLOW
        # elif (red in range(2,7)) and (green in range(10, 15)) and (blue in range(39, 44)):
        #     return Color.BLUE
        # elif (red in range(2,7)) and (green in range(19, 24)) and (blue in range(4, 9)):
        #     return Color.GREEN
        # elif (red in range(34,39)) and (green in range(3, 8)) and (blue in range(2, 7)):
        #     return Color.RED
        # else:
        #     return None


def getNextUncheckedNode(nodeDict):
    """Returns the next node that has not been checked

    Args:
        nodeDict: The dictionary with the nodes and their checked state

    Raises:
        None

    Returns:
        The first node that has a value the doent match its key, none if all match their keys

    """
    for node in nodeDict:
        if (node != nodeDict[node]):
            return node
    return None

def PIDlineFollowerUntilBlock2016(robot: WROrobot, distanceToTravel, speed, side):
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
    light_2 = robot.color_1
    light_1 = robot.color_1
    startDistance = robot.driveBase.distance()
    distance = 0
    error = 0
    integral = 0
    lastError = 0
    derivative = 0
    lastColor = Color.BLACK
    consecutiveColor = 0

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
        color = detectTankColor2016(robot, "tank")
        if ((color == Color.RED) or (color == Color.GREEN) or (color == Color.BLUE) or (color == Color.YELLOW)):
            break
    Movement.robotStop(robot)
    return robot.driveBase.distance() - startDistance