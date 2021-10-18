"""thymio_labyrinth controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

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


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
   
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)
print("front: " + centralSensor.getValue)
print("side: " + outerLeftSensor.getValue)
# Enter here exit cleanup code.
