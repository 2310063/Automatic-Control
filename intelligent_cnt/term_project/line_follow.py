#!/usr/bin/python3
import RPi.GPIO as GPIO
import time
from motor import Robot

robot = Robot()

def run(s1, s2, s3, s4):
    robot.run(s1, s2, s3, s4)

sensor_freq = 0.05
ts = time.monotonic()
while True:
    print("running")
    try:
        # print("sensor value 1 - 4: ", GPIO.input(robot.sensor1), " " , GPIO.input(robot.sensor2), " ", GPIO.input(robot.sensor3), " ", GPIO.input(robot.sensor4)) 
        s1, s2, s3, s4 = robot.update_sensor_values(GPIO.input(robot.sensor1), GPIO.input(robot.sensor2), GPIO.input(robot.sensor3), GPIO.input(robot.sensor4))
        ps = time.monotonic()
        print("time : ",ps - ts,"sensor value 1 - 4: ", s1, " " , s2, " ", s3, " ", s4)
        run(s1, s2, s3, s4)
        time.sleep(sensor_freq)
    except KeyboardInterrupt:
        GPIO.cleanup()
        # robot.edge_logger()
        print("breaking")
        break
