"""thymio_labyrinth controller."""

from controller import Robot
import time

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
  
MAX_SPEED = 6.27

camera = robot.getDevice("camera")

camera.enable(timestep)

# Get left and right wheel motors.
leftWheel = robot.getDevice("left wheel motor")
rightWheel = robot.getDevice("right wheel motor")

frontRightS= robot.getDevice("ps0")
leftS= robot.getDevice("ps5")
centralLeftS= robot.getDevice("ps6")
frontLeftS= robot.getDevice("ps7")

# Enable distance sensors.
frontRightS.enable(timestep)
leftS.enable(timestep)
centralLeftS.enable(timestep)
frontLeftS.enable(timestep)

# Disable motor PID control mode.
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

# convert the value from the distance sensors into values that are easier to compare. 
# values under 100 are ignored as white noise
def getDistance(sensor):
    if (sensor.getValue() > 100):
        return round(sensor.getValue()/50, 2)
    else:
        return 0

leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(MAX_SPEED)

# robot will first check if there is a wall on its left and find one if not
if (getDistance(leftS) < 8.8):
    if(getDistance(frontLeftS) < 1): 
        leftWheel.setVelocity(MAX_SPEED)
        rightWheel.setVelocity(MAX_SPEED)
    else:
        rightWheel.setVelocity(0)
        leftWheel.setVelocity(MAX_SPEED)
        
while robot.step(timestep) != -1:
        print("finding wall")
        if(getDistance(frontRightS) > 1):
            break;

# Main loop:
# robot will then proceed to follow the wall on its left side 
while robot.step(timestep) != -1:
    
    # if robot finds a wall straight in front of it, it will turn right
    if(getDistance(frontRightS) > 1):
        rightWheel.setVelocity(0)
        leftWheel.setVelocity(MAX_SPEED)        
    
    elif(getDistance(leftS) < 8.8):
        # if robot senses it is going too much towards the wall it will attempt to turn parallel to the wall
        if(getDistance(centralLeftS) > 4.5):
            if(getDistance(frontLeftS) > 2):
                rightWheel.setVelocity(MAX_SPEED * 0.2)
                leftWheel.setVelocity(MAX_SPEED)
            else:
                rightWheel.setVelocity(MAX_SPEED * 0.4)
                leftWheel.setVelocity(MAX_SPEED)
        # if there is no wall on the left side, the robot will turn left 
        else:
            leftWheel.setVelocity(MAX_SPEED*0.2)
            rightWheel.setVelocity(MAX_SPEED)
            
    else:
        if(getDistance(centralLeftS) > 4.5):
            rightWheel.setVelocity(MAX_SPEED * 0.6)
            leftWheel.setVelocity(MAX_SPEED)
        else:
            leftWheel.setVelocity(MAX_SPEED)
            rightWheel.setVelocity(MAX_SPEED)
