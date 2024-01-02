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
from pybricks.messaging import BluetoothMailboxServer, TextMailbox

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

'''This code is for the bot that moves to collect all the balls.
        it is currently called clientbot'''

# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
picking_motor = Motor(Port.D)
kicking_motor = Motor(Port.C)
wheel_diameter = 55
axle_track = 183
robot = DriveBase(left_motor, right_motor, wheel_diameter,axle_track)

#Changing the ultrasonic sensor to the touch sensor
#ultrasonic_sensor1 = UltrasonicSensor(Port.S3)
touch_sensor1=TouchSensor(Port.S3)

x_pos = 0
y_pos = 0
angle_sum = 0
stopwatch = StopWatch()
mat_centre = (590.5, 285.75)
robot.settings(600, 400, 200, 200)
robot.reset() #Start measuring the distance
# Write your program here.

#The angle is the angle of rotation of the tyre
def turn(angle):
     # 2nd argument is the turn rate in degrees per second
    robot.turn(angle)
    robot.stop()
    left_motor.brake()
    right_motor.brake()

def move_distance(distance):
    robot.straight(distance)
def move():
    robot.drive(270, 0)
    robot.reset()
def picking_mechanism():
    picking_motor.run(-1000)
def picking_reverse():
    picking_motor.run_time(1000, 6600, then=Stop.HOLD, wait=True)
def move_until_wall_hit():
    while not touch_sensor1.pressed():
        move()
    move_distance(100)
    move_distance(-100)
    tunr(90)


def to_be_looped():
    while not touch_sensor1.pressed(): #Bot moves to side barrier at lower part of the half until it hits it
        move()
    move_distance(200)
    move_distance(-50)# Bot reverses
    turn(109) #Turns at angle towards the ramp in the first half
    move_distance(865) #Moves towards the ramp
    move_distance(-20) # Reverse
    turn(-109) #Makes a turn towards the sideline barrier
    while not touch_sensor1.pressed(): #Moves to side barrier at upper part of the half
        move()
    move_distance(-30) #Bot reverses
    turn(-107) # Turns an abgle towards midway barrier

    while not touch_sensor1.pressed(): #Moves toward midway barrier until touch sensor pressed
        move()
                        
    move_distance(-74) # Reverses
    turn(-114) #Turns toward the second half
    while not touch_sensor1.pressed(): #Move into the side barrier of the second half
        move()
    move_distance(-30) #Reverse
    turn(180) # Turn at an angle toward to midway barrier
    while not touch_sensor1.pressed():
        move()
    move_distance(-320) # Moves toward sideline barrier in second half
    move_distance(50) #Moves forward
    turn(-90) # Turns toward the client bot
    while not touch_sensor1.pressed(): #Moves to hit the wall of the client bot
        move()
    move_distance(-130) # Backs up
    turn(90) #Makes a turn perpendicular to the position of the client bot
    move_distance(215) # Moves toward the client bot (Might have to increase the distance)
    turn(-90) # Turns at an angle toward the client bot
    # REVERSE FEEDING BEGINS
    t2 = threading.Thread(target=picking_reverse)
    t2.start()
    kicking_motor.run_time(-400, 400, then=Stop.HOLD, wait=True)
    kicking_motor.run_time(400, 400, then=Stop.HOLD, wait=True)
    kicking_motor.run_time(-400, 400, then=Stop.HOLD, wait=True)
    kicking_motor.run_time(400, 400, then=Stop.HOLD, wait=True)
    kicking_motor.run_time(-400, 400, then=Stop.HOLD, wait=True)
    kicking_motor.run_time(400, 400, then=Stop.HOLD, wait=True)
    # REVERSE FEEDING ENDS
    move_distance(-140) # Backs up
    turn(50) # Turns toward first half
    move_distance(390) # Moves into first half
    turn(45) # Turns at angle toward side barrier

    #PICKING BEGINS
    t1 = threading.Thread(target=picking_mechanism)
    t1.start()

#turn(-3)
#wait(1000)
t1 = threading.Thread(target=picking_mechanism)
t1.start()




for i in range(0, 2):
    move_until_wall_hit()
turn(-112) #Turn towards the second half

#SECOND HALF OF THE BOARD
move_distance(340) #Move forward close to the barrrier
turn(-59)   # Turn bot to straighten
move_distance(325)  #Move forward toward the ramp
turn(95) # Turn to the right
move_distance(225) # Move across the width
turn(78)    # Turn towards the base
robot.settings(1000, 400, 200, 200)
move_distance(330)  #Move towars the base
turn(88)    # Turn towards the second bot
move_distance(225)
turn(-90)
move_distance(60) # Move forward to feed the balls to client

#BALL TRANSFER FROM SERVER TO CLIENT BEGINS
t2 = threading.Thread(target=picking_reverse)
t2.start()
for i in range(0 ,4):
    kicking_motor.run_time(-400, 400, then=Stop.HOLD, wait=True)
    kicking_motor.run_time(400, 400, then=Stop.HOLD, wait=True)
wait(2000)
#BALL TRANSFER TO CLIENT ENDS

move_distance(-150)#Moves away from client
turn(48) #Turns towards the barrier
move_distance(390)#Moves forward to first half
turn(45)#Turns toward sideline barrier
#Restart picking mechanism
t1 = threading.Thread(target=picking_mechanism)
t1.start()

while not touch_sensor1.pressed(): #Bot moves to side barrier at lower part of the half until it hits it
    move()
move_distance(200)
move_distance(-50)# Bot reverses
turn(109) #Turns at angle towards the ramp in the first half
move_distance(865) #Moves towards the ramp
move_distance(-20) # Reverse
turn(-109) #Makes a turn towards the sideline barrier
while True:
    while not touch_sensor1.pressed(): #Moves to side barrier at upper part of the half
        move()
    move_distance(-250)



"""

server = BluetoothMailboxServer()
mbox = TextMailbox('greeting', server)

# The server must be started before the client!
print('waiting for connection...')
server.wait_for_connection()
print('connected!')

# In this program, the server waits for the client to send the first message
# and then sends a reply.
#mbox.wait()
mbox.send('Payload Reached')
print(mbox.read())


move_distance(100)
move_distance(-70)
turn(150)

for i in range(0, 2):
    while ultrasonic_sensor1.distance(silent = False) > 120:
        move()
    turn(90)
    move_distance()


    move_distance(-20)
    turn(75)


def picking_mecahnism():
    picking_motor.run(40)
def shooting_mechanism():
    shooting_motor.run(20)
def measure_distance1():
    ultrasonic_sensor1.distance(silent = False)
def measure_distance2():
    ultrasonic_sensor2.distance(silent = False)

def ball_detect():

    while measure_distance1() > 50 and measure_distance2() > 50:
        move_distance(222)
        move_distance(-222)

    if measure_distance1() < 50:
        turn(90)
        move_straight(measure_distance1())
    elif measure_distance2()<50:
        turn(-90)
        move_straight(measure_distance1())






#Starting move
run = True
while run:
    move_distance(443)
    turn(90)
    move_distance(220)
    turn(90)
    move_distance(443)
    turn(180)
    move_distance(100)
    turn(-90)
    move_distance(220)
    turn(-90)
    wait(15)
    shooting_mechanism()
    run = False
while True:
    ball_detect()

        


t1 = threading.Thread(target=picking_mechanism)
t1.start()
#UltraSonic Sensor Detection
while True:
    ball_detect()
    

print(robot.distance())
"""
