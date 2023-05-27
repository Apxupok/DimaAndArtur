#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

motor_left = Motor(Port.A)
motor_right = Motor(Port.B)
motor_C = Motor(Port.C)
motor_V = Motor(Port.D)

ultra = UltrasonicSensor(Port.S1)
colorSensor = ColorSensor(Port.S2)
gyro = GyroSensor(Prort.S3)


motor_V.run_target(0,100)
while True:
    
    motor_C.run_target(30,100)
    
    if ultra.distance() <= 200:
        while gyro.angle(30):
            motor_left.run(450)
            motor_right.run(-450)

        if colorSensor.color() = Color.RED:
            motor_V.run(960)
            wait(3000)

    motor_C.run_target(-30,100)

    if ultra.distance() <= 200:
        while gyro.angle(-30):
            motor_left.run(-450)
            motor_right.run(450)

        if colorSensor.color() = Color.RED:
            motor_V.run(960)
            wait(3000)
