"""thymio_labyrinth controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
  
# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 180

# Get left and right wheel motors.
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

# Get frontal distance sensors.
leftSensor = robot.getDevice("ds1")
outerLeftSensor = robot.getDevice("ds2")
centralLeftSensor = robot.getDevice("ds3")
centralRightSensor = robot.getDevice("ds4")
outerRightSensor = robot.getDevice("ds5")
rightSensor = robot.getDevice("ds6")

# Enable distance sensors.
leftSensor.enable(timestep)
outerLeftSensor.enable(timestep)
centralLeftSensor.enable(timestep)
centralRightSensor.enable(timestep)
outerRightSensor.enable(timestep)
rightSensor.enable(timestep)


# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity =  maxMotorVelocity

def getDistance(sensor):
    """
    Return the distance of an obstacle for a sensor.

    The value returned by the getValue() method of the distance sensors
    corresponds to a physical value (here we have a sonar, so it is the
    strength of the sonar ray). This function makes a conversion to a
    distance value in meters.
    """
    return (round(sensor.getValue() / 100 * 5, 2) )


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    leftMotor.setVelocity(initialVelocity)
    rightMotor.setVelocity(initialVelocity)
    #print("front: " + str(getDistance(centralLeftSensor)))
    print("side: " + str(getDistance(outerLeftSensor)))
    #print("side: " + str(getDistance(outerLeftSensor)))

# Enter here exit cleanup code.