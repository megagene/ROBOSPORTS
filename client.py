#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import threading
import math
import time

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.

ev3=EV3Brick()

#initialize the driving motors
left_motor=Motor(Port.A)
right_motor=Motor(Port.D)
shoving_motor=Motor(Port.C, positive_direction=Direction.CLOCKWISE,gears=None)
ultrasonic_sensor = UltrasonicSensor(Port.S4)

wheel_diameter=56
axle_track=99.5

#movement parameters
s_speed=200
s_acceleration=200

turn_speed=200 
turn_acceleration=200

#setup the robot drivebase
robot=DriveBase(left_motor,right_motor,wheel_diameter, axle_track)
robot.settings(s_speed, s_acceleration, turn_speed, turn_acceleration)

#beginning the process
robot.reset()

def move(distance):
    neg_distance=distance*-1

    robot.straight(distance)
    robot.stop()
    left_motor.brake()
    right_motor.brake()
    #Shoving motor code here for moving when the mother bot reaches the top
    shoving_motor.run_time(3000,480, then=Stop.HOLD, wait=True)
    shoving_motor.run_time(-3000,450, then=Stop.HOLD, wait=True) # Changed from 480 to 450
    wait(2000)
    shoving_motor.run_time(3000,480, then=Stop.HOLD, wait=True)
    shoving_motor.run_time(-3000,450, then=Stop.HOLD, wait=True) # Changed from 480 to 450
    #Now moving back to where it came from.


while True:

    while ultrasonic_sensor.distance()>70:
        pass
    print(ultrasonic_sensor.distance())
    wait(15000)
    move(-1025)
    while True :
        shoving_motor.run_time(3000,480, then=Stop.HOLD, wait=True)
        shoving_motor.run_time(-3000,450, then=Stop.HOLD, wait=True) # Changed from 480 to 450
        wait(2000)
        shoving_motor.run_time(3000,480, then=Stop.HOLD, wait=True)
        shoving_motor.run_time(-3000,450, then=Stop.HOLD, wait=True) # Changed from 480 to 450

