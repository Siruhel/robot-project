"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot
import math

from controller import Compass

def getDirectionInDegrees(compass):
  north = compass.getValues()
  rad = math.atan2(north[0], north[2]);
  direction = (rad - 1.5708) / math.pi * 180.0;

  if direction < 0.0:
    direction = direction + 360.0

  return direction;

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

compass = robot.getCompass("compass")

compass.enable(timeStep)

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 180

# Get left and right wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

# Get frontal distance sensors.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity =  maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
    
    direction = getDirectionInDegrees(compass)
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant

    # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
    leftMotor.setVelocity(initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 2)
    rightMotor.setVelocity(initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 2 - centralSensorValue)

    if outerLeftSensorValue == 0.0 and centralLeftSensorValue == 0.0 and centralSensorValue == 0.0 and centralRightSensorValue == 0.0 and outerRightSensorValue == 0.0:
        leftS = leftMotor.getVelocity()
        rightS = rightMotor.getVelocity()

        if direction < 85:
            leftMotor.setVelocity(leftS + 0.6)
            rightMotor.setVelocity(rightS - 0.6)

        if direction > 95:
            leftMotor.setVelocity(leftS - 0.6)
            rightMotor.setVelocity(rightS + 0.6)

        if direction > 88.5 and direction < 90.5 and leftS < 20 and rightS < 20:
            leftMotor.setVelocity(leftS + 0.1)
            rightMotor.setVelocity(rightS + 0.1)