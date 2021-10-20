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
leftMotor = robot.getDevice("motor.left")
rightMotor = robot.getDevice("motor.right")

# Get frontal distance sensors.
outerLeftSensor = robot.getDevice("prox.horizontal.0")
centralLeftSensor = robot.getDevice("prox.horizontal.1")
centralSensor = robot.getDevice("prox.horizontal.2")
centralRightSensor = robot.getDevice("prox.horizontal.3")
outerRightSensor = robot.getDevice("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timestep)
centralLeftSensor.enable(timestep)
centralSensor.enable(timestep)
centralRightSensor.enable(timestep)
outerRightSensor.enable(timestep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity =  maxMotorVelocity

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    leftMotor.setVelocity(initialVelocity)
    rightMotor.setVelocity(initialVelocity)
    print("front: " + str(centralSensor.getValue()))
    print("side: " + str(outerLeftSensor.getValue()))
    print("side: " + str(outerRightSensor.getValue()))

# Enter here exit cleanup code.
