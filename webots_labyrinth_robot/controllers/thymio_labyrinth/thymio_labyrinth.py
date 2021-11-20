"""thymio_labyrinth controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
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
centralRightS= robot.getDevice("ps1")
rightS= robot.getDevice("ps2")
rightBackS= robot.getDevice("ps3")
leftBackS= robot.getDevice("ps4")
leftS= robot.getDevice("ps5")
centralLeftS= robot.getDevice("ps6")
frontLeftS= robot.getDevice("ps7")

# Enable distance sensors.
frontRightS.enable(timestep)
centralRightS.enable(timestep)
rightS.enable(timestep)
rightBackS.enable(timestep)
leftBackS.enable(timestep)
leftS.enable(timestep)
centralLeftS.enable(timestep)
frontLeftS.enable(timestep)


# Disable motor PID control mode.
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity =  MAX_SPEED



def getDistance(sensor):
    """
    Return the distance of an obstacle for a sensor.

    The value returned by the getValue() method of the distance sensors
    corresponds to a physical value (here we have a sonar, so it is the
    strength of the sonar ray). This function makes a conversion to a
    distance value in meters.
    """
    #print(sensor.getValue())
    if (sensor.getValue() > 100):
        return round(sensor.getValue()/50, 2)
    else:
        return 0



leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(MAX_SPEED)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
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


while robot.step(timestep) != -1:
    if(getDistance(frontRightS) > 1):
        rightWheel.setVelocity(0)
        leftWheel.setVelocity(MAX_SPEED)
        print("there wall ahead")
        
        
    elif(getDistance(leftS) < 8.8):
        if(getDistance(centralLeftS) > 4.5):
            if(getDistance(frontLeftS) > 2):
                rightWheel.setVelocity(MAX_SPEED * 0.2)
                leftWheel.setVelocity(MAX_SPEED)
                print("really going at the wall")
            else:
                print("kinda going at the wall")
                rightWheel.setVelocity(MAX_SPEED * 0.4)
                leftWheel.setVelocity(MAX_SPEED)
        else:
            leftWheel.setVelocity(MAX_SPEED*0.2)
            rightWheel.setVelocity(MAX_SPEED)
            
            print("no wall on left must do turn")
            
    
    
    else:
        if(getDistance(centralLeftS) > 4.5):
            rightWheel.setVelocity(MAX_SPEED * 0.6)
            leftWheel.setVelocity(MAX_SPEED)
            print("too close to wall")
        else:
            print("just go straight all good")
            leftWheel.setVelocity(MAX_SPEED)
            rightWheel.setVelocity(MAX_SPEED)


# Enter here exit cleanup code.

# leftsensor value should be around 10 
# centralleftsensor value should be around 4 

# need to fix: must check if value from centralleftsensor is too big outside the lop of checking 
# for leftsensor value: robot goes to the wall because it doesnät care of the value if left is ok 