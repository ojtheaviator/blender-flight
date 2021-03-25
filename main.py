import random
import bpy


#  START-SIMULATION-CODE---------------------
maxFrame = 250
fps = 24

mass = 1500
timeChange = 1 / fps
g = 9.81
maxThrust = 1.5 * mass * g

#  Higher gives smoother thrust, usually greater than 1
thrustVariationParameter = 2

zMin = 1

#  Initialize output lists for coordinates
xValues = []
yValues = []
zValues = []
outputString = ''

#  Initialize variables for location and velocity in previous round (or initially)
xPrevious = 0
xVelocityPrevious = 0

yPrevious = 0
yVelocityPrevious = 0

zPrevious = zMin
zVelocityPrevious = 0
#  ---------------------


#  START-CONTROL-CODE---------------------
#  PID tuning parameters
Kp = 4
Ki = 0.6
Kd = 3

targetHeight = 3

#  Initialize a variable for a random force offset (for reaction)
forceOffset = 0
zForce = 0
integralError = 0
#  ---------------------



for curFrame in range(0, maxFrame):    
    #  START-CONTROL-CODE----------------
    if (curFrame % 3 == 0):
        forceOffset = mass * g * random.randint(-100,100) / (100*thrustVariationParameter)
    error = targetHeight - zPrevious
    integralError += error
    
    proportionalTerm = Kp * error
    integralTerm = Ki * integralError
    derivativeTerm = Kd * (-1) * zVelocityPrevious
    
    zForce = ((mass * g) + forceOffset) * (proportionalTerm + integralTerm + derivativeTerm)
    if (zForce > maxThrust):
        zForce = maxThrust
    
    xForce = random.randint(-50,50)
    yForce = random.randint(-50,50)
    #  ----------------------------------
    
    
    #  START-SIMULATION-CODE------------
    #  Z axis
    zVelocity = zVelocityPrevious + (timeChange * ((zForce / mass) - g))
    z = ((timeChange**2 / 2) * ((zForce / mass) - g)) + (zVelocityPrevious * timeChange) + zPrevious
    if (z < zMin):
        z = zMin
        zVelocity = 0
    zValues.append(z)

    zVelocityPrevious = zVelocity
    zPrevious = z 
    
    
    #  X axis
    xVelocity = xVelocityPrevious + (timeChange * xForce / mass)
    x = ((timeChange**2 / 2) * xForce / mass) + (xVelocityPrevious * timeChange) + xPrevious
    
    xValues.append(x)

    xVelocityPrevious = xVelocity
    xPrevious = x 
    
    
    #  Y axis
    yVelocity = yVelocityPrevious + (timeChange * yForce / mass)
    y = ((timeChange**2 / 2) * yForce / mass) + (yVelocityPrevious * timeChange) + yPrevious
    
    yValues.append(y)

    yVelocityPrevious = yVelocity
    yPrevious = y
    
    bpy.data.objects["Cube"].location = x, y, z
    bpy.data.objects["Cube"].keyframe_insert(data_path="location", frame=curFrame)
