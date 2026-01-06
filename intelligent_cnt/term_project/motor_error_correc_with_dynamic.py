import time
import RPi.GPIO as GPIO
import json
import numpy as np
from collections import deque

class Robot:
    def __init__(self):
        """This is the constructor. All setup happens here automatically."""

        # -------- Pin Definitions --------
        self.SW1 = 5
        self.SW2 = 6
        # -------- Motor Pins --------
        self.PWMA = 18
        self.AIN1 = 22
        self.AIN2 = 27
        
        self.PWMB = 23
        self.BIN1 = 25
        self.BIN2 = 24
        # -------- Sensor Pins --------
        self.sensor1 = 6
        self.sensor2 = 19
        self.sensor3 = 5
        self.sensor4 = 13

        # -------- PID parameters --------
        # self.Kp = 75
        # self.Ki = 10
        # self.Kd = 35

        self.Kp = 75  # WAS 75.
        self.Ki = 5   # WAS 10.
        self.Kd = 60  # WAS 35.

        # -------- Define PID variables --------
        self.base_speed = 40
        self.last_error = 0
        self.integral = 0

        self.left_speed = self.base_speed
        self.right_speed = self.base_speed

        # -------- Set sensor counters --------
        self.s1_cnt = 0
        self.s2_cnt = 0
        self.s3_cnt = 0
        self.s4_cnt = 0

        self.edge_cnt = 0

        self.low_speed_cnd = False

        self.path_history = deque(maxlen=100)
        self.lost_history = deque(maxlen = 2)
        self.lost_path_timer = 0
        self.is_lost = False
        self.lost_path_duration = 4

        # -------- Set time counters --------
        # min, max time for edge detection
        self.time_min = 1.3
        
        self.forward = 0.6

        self.turn_min = 0.1
        self.turn_max = 4

        self.rush_time = 2
        self.max_rush_time = 4

        self.rush_speed = 80

        self.bs_time = 0
        self.max_time_slow = 10
        # -------- internal time counter --------
        self.time_cnt = 0
        self.motor_cnt = 1
        self.motor_sensor_ratio = 2
        
        # -------- control frequency --------
        self.ctr_freq = 0.1

        # --- GPIO Setup ---
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.SW1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.SW2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # ... setup for all other pins ...
        GPIO.setup(self.PWMA, GPIO.OUT)
        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.AIN2, GPIO.OUT)

        GPIO.setup(self.PWMB, GPIO.OUT)
        GPIO.setup(self.BIN1, GPIO.OUT)
        GPIO.setup(self.BIN2, GPIO.OUT)
        
        # setup sensors
        GPIO.setup(self.sensor1,GPIO.IN)
        GPIO.setup(self.sensor2,GPIO.IN)
        GPIO.setup(self.sensor3,GPIO.IN)
        GPIO.setup(self.sensor4,GPIO.IN)

        # Motor objects also become attributes
        self.L_Motor = GPIO.PWM(self.PWMA, 500)
        self.L_Motor.start(0)

        self.R_Motor = GPIO.PWM(self.PWMB, 500)
        self.R_Motor.start(0)
        print("Robot initialized successfully!")

        # Initialize sensor value log
        self.sensor_values = np.array([1,1,1,1])

        self.sensor_number = 20

        self.s1_cond = np.array([])
        self.s2_cond = np.array([])
        self.s3_cond = np.array([])
        self.s4_cond = np.array([])

        self.error_list = []

    def motor_stop(self, ):
        GPIO.output(self.AIN1,0)
        GPIO.output(self.AIN2,1)
        self.L_Motor.ChangeDutyCycle(0)
        GPIO.output(self.BIN1,0)
        GPIO.output(self.BIN2,1)
        self.R_Motor.ChangeDutyCycle(0)

    def set_base_speed(self):

        if self.low_speed_cnd == True:
            print("------- Slow speed has triggerd for : ", self.max_time_slow)
            self.bs_time += self.ctr_freq

            if self.bs_time <= self.max_time_slow:
                self.base_speed = 40
            else:
                print("--------- Slow speed has ended. ---------")
                self.low_speed_cnd = False
                self.bs_time = 0
                self.base_speed = 70
        else:
            self.low_speed_cnd = False
            self.base_speed = 70
            self.bs_time = 0


    def PID_speed(self, s1, s2, s3, s4):
        error = self.calc_error(s1, s2, s3, s4)

        # 3. PID CALCULATION
        proportional = error
        self.integral = self.integral_calculator() * 0.5
        self.integral = np.clip(self.integral, -5, 5)  # Anti-windup
        derivative = error - self.last_error
        # This is the final value that will adjust the motor speeds
        
        correction = (self.Kp * proportional) + (self.Ki * self.integral) + (self.Kd * derivative)
        print("Integral, Derivative, Correction, Error: ")
        print(self.integral,"  ", derivative,"  ", "{0:2f}".format(correction),"  ",error,"  ",)
        
        self.last_error = error

        # 4. MOTOR CONTROL
        self.left_speed  = self.base_speed - correction
        self.right_speed = self.base_speed + correction

    def rush(self, s1, s2, s3, s4):
        self.time_cnt += self.ctr_freq

        if self.time_cnt < self.rush_time and (s1 == 1 and s4 == 1):
            self.base_speed = 40

            if s1 == 0 or s4 == 0:
                self.time_cnt = 0

        elif self.time_cnt >= self.rush_time and (s1 == 1 and s4 == 1):
            print("----- rushing ------ speed : ", self.rush_speed)
            print("------- rushing time : ", self.time_cnt)
            self.base_speed = self.rush_speed
            
            if s1 == 0 or s4 == 0:
                self.base_speed = 40
                self.time_cnt = 0

            elif self.time_cnt >= self.max_rush_time:
                self.base_speed = 40
                self.time_cnt = 0
                print("----- rush ended -----", self.time_cnt)

    def run_motor_speeds_undo(self, left_speed, right_speed):
        """
        runs motor for a short duration based on the current left and right speed attributes
        """

        max_speed = 100
        min_speed = 10

        print(f"motor speeds - Left: {left_speed}, Right: {right_speed}")
        print(f"base speeds  : {self.base_speed}")
        

        if left_speed > 0:
            left_speed = np.clip(left_speed, min_speed, max_speed)
            GPIO.output(self.AIN1,0)
            GPIO.output(self.AIN2,1)
            self.L_Motor.ChangeDutyCycle(left_speed)

        elif left_speed < 0:
            left_speed = np.clip(-left_speed, min_speed, max_speed)
            GPIO.output(self.AIN1,1)
            GPIO.output(self.AIN2,0)
            self.L_Motor.ChangeDutyCycle(left_speed)

        if right_speed > 0:
            right_speed = np.clip(right_speed, min_speed, max_speed)
            GPIO.output(self.BIN1,0)
            GPIO.output(self.BIN2,1)
            self.R_Motor.ChangeDutyCycle(right_speed)

        elif right_speed < 0:
            right_speed = np.clip(-right_speed, min_speed, max_speed)
            GPIO.output(self.BIN1,1)
            GPIO.output(self.BIN2,0)
            self.R_Motor.ChangeDutyCycle(right_speed)


    def run_motor_speeds(self):
        """
        runs motor for a short duration based on the current left and right speed attributes
        """
        if not self.is_lost:
             self.path_history.append((self.left_speed, self.right_speed, self.ctr_freq))

        max_speed = 100
        min_speed = 10

        print(f"motor speeds - Left: {self.left_speed}, Right: {self.right_speed}")
        print(f"base speeds  : {self.base_speed}")
        

        if self.left_speed > 0:
            self.left_speed = np.clip(self.left_speed, min_speed, max_speed)
            GPIO.output(self.AIN1,0)
            GPIO.output(self.AIN2,1)
            self.L_Motor.ChangeDutyCycle(self.left_speed)

        elif self.left_speed < 0:
            self.left_speed = np.clip(-self.left_speed, min_speed, max_speed)
            GPIO.output(self.AIN1,1)
            GPIO.output(self.AIN2,0)
            self.L_Motor.ChangeDutyCycle(self.left_speed)

        if self.right_speed > 0:
            self.right_speed = np.clip(self.right_speed, min_speed, max_speed)
            GPIO.output(self.BIN1,0)
            GPIO.output(self.BIN2,1)
            self.R_Motor.ChangeDutyCycle(self.right_speed)

        elif self.right_speed < 0:
            self.right_speed = np.clip(-self.right_speed, min_speed, max_speed)
            GPIO.output(self.BIN1,1)
            GPIO.output(self.BIN2,0)
            self.R_Motor.ChangeDutyCycle(self.right_speed)

    def edge_logger(self, corner = 'c1'):
        with open("term_project/edge_log.json", "r") as f:
            data = json.load(f)
        print(data)
        print(self.sensor_values)
        data_ap = {corner : self.sensor_values}
        data.append(data_ap)
        
        with open("term_project/edge_log.json", "w") as f:
            json.dump(data, f, indent=4)

    def logger(self, corner = 'c1'):
        with open("term_project/edge_log.json", "r") as f:
            data = json.load(f)
        print(data)
        print(self.sensor_values)
        data_ap = {corner : self.sensor_values}
        data.append(data_ap)
        
        with open("term_project/edge_log.json", "w") as f:
            json.dump(data, f, indent=4)

    def undo_path(self):
        """
        Retraces the robot's path for the duration it was lost.
        """
        print("\n--- UNDO PATH ACTIVATED ---")
        print(f"Retracing path for {self.lost_path_duration} seconds.")

        self.low_speed_cnd = True

        self.motor_stop()
        time.sleep(0.2) # A brief pause before reversing

        # Calculate how many steps to undo
        # Logic runs at 0.1s intervals, so steps = duration / 0.1
        steps_to_undo = int(self.lost_path_duration / self.ctr_freq)
        
        # Ensure we don't try to undo more steps than we have in history
        steps_to_undo = min(steps_to_undo, len(self.path_history))
        print(f"Reversing last {steps_to_undo} steps...")

        for _ in range(steps_to_undo):
            if not self.path_history:
                print("History empty, cannot undo further.")
                break
                
            last_left, last_right, duration = self.path_history.pop()
            
            # Reverse the speeds
            self.left_speed = -last_left
            self.right_speed = -last_right
            
            print(f"Reversing move: L={self.left_speed:.2f}, R={self.right_speed:.2f} for {duration}s")
            self.run_motor_speeds_undo(self.left_speed, self.right_speed) # Use run_motor_speeds to execute the reversed move
            time.sleep(duration) # Wait for the duration of the reversed move
            
        print("--- UNDO PATH COMPLETE ---")
        self.motor_stop()
        time.sleep(0.5) # Pause after undoing to stabilize before resuming
        
        # Reset lost state
        self.is_lost = False
        self.lost_path_timer = 0
        self.integral = 0 # Reset integral to prevent sudden jerks
        self.last_error = 0 # Reset last error

    def update_sensor_values(self, s1, s2, s3, s4):
        """
        Stores sensor values up to sensor_number.
        And eliminates undesired 0 inputs to sensor.
        """
        error = self.calc_error(s1, s2, s3, s4)
        
        if len(self.sensor_values) < self.sensor_number:
            self.sensor_values = np.vstack((self.sensor_values, [s1, s2, s3, s4]))
            self.error_list.append(error)
            return s1, s2, s3, s4

        else:
            self.sensor_values = self.sensor_values[1:]
            self.sensor_values = np.vstack((self.sensor_values, [s1, s2, s3, s4]))

            self.error_list.pop(0)
            self.error_list.append(error)
            return np.logical_or(self.sensor_values[-2], self.sensor_values[-1])

    def end_check(self, s1, s2, s3, s4):
        # self.update_sensor_values(s1, s2, s3, s4)
        if  sum(self.error_list) == 0 and len(self.error_list) == self.sensor_number:
            self.motor_stop()
            print("End of line detected. Stopping the robot.")
            exit(1)
            return True
        
        return False

    def run(self, s1, s2, s3, s4):

        if s2 == 1 and s3 == 1:
            print("time for undo path", self.lost_path_timer)
            self.lost_path_timer += self.ctr_freq
            if self.lost_path_timer >= self.lost_path_duration:
                self.is_lost = True
        else:
            # If sensors detect the line again, reset the timer.
            self.lost_path_timer = 0
            self.is_lost = False

        # --- State Machine: Normal, Lost, or Edge ---
        if self.is_lost:
            # If the path is lost, execute the undo maneuver
            self.undo_path()
            return # Skip the rest of the logic for this cycle
        
        cnd = self.cnd_checker(s1, s2, s3, s4)
        self.set_base_speed()
        self.end_check(s1, s2, s3, s4)

        # this code runs at every 2 iterations

        if self.motor_cnt < self.motor_sensor_ratio:
            self.motor_cnt += 1
            
        else:
            if self.edge_cnt == 0:

                if cnd == "s1" or self.s1_cnt == 1:
                    print("--------- left corner ---------")
                    self.s1_cnt = 1
                    self.edge_runner(s1, s2, s3, s4, time_duration=self.ctr_freq)

                elif cnd == "s4" or self.s4_cnt == 1:
                    print("--------- right corner ---------")
                    self.s4_cnt = 1
                    self.edge_runner(s1, s2, s3, s4, time_duration=self.ctr_freq)

                else:
                    print("--------- PID control ---------")
                    # self.rush(s1, s2, s3, s4)
                    self.PID_speed(s1, s2, s3, s4)
                    self.run_motor_speeds()

            elif self.edge_cnt == 1:
                print("edge counter not working for ", self.time_min, " seconds")

                if self.time_cnt < self.time_min:
                    self.time_cnt += self.ctr_freq

                    self.PID_speed(s1, s2, s3, s4)
                    self.run_motor_speeds()

                else:
                    self.time_cnt = 0
                    self.edge_cnt = 0

            self.motor_cnt = 1
        
    def cnd_checker(self, s1, s2, s3, s4):

        if len(self.s1_cond) < 3:
            self.s1_cond = np.append(self.s1_cond, s1)
            self.s4_cond = np.append(self.s4_cond, s4)
        else:
            self.s1_cond = self.s1_cond[1:]
            self.s4_cond = self.s4_cond[1:]
            self.s1_cond = np.append(self.s1_cond, s1)
            self.s4_cond = np.append(self.s4_cond, s4)
            
            if sum(self.s1_cond) == 0 or (s1 == 0 and s2 == 0):
                return "s1"
            elif sum(self.s4_cond) == 0 or (s3 == 0 and s4 == 0):
                return "s4"
            else:
                return "none"

    def edge_runner(self, s1, s2, s3, s4, time_duration = 0.1):
        self.time_cnt += self.ctr_freq

        if self.s1_cnt == 1:

            if self.time_cnt < self.forward:
                print("------- left edge -------")
                print("operating time : ", self.time_cnt)
                self.right_speed = 100
                self.left_speed  = 100
                self.run_motor_speeds()

            elif self.time_cnt < self.forward + self.turn_min:
                print("------- turning -------")
                print("operating time : ", self.time_cnt)
                self.right_speed = 100
                self.left_speed  = -100
                self.run_motor_speeds()

            elif self.time_cnt < self.forward + self.turn_max:
                print("------- min time over -------")
                print("operating time : ", self.time_cnt)
                self.right_speed = 55
                self.left_speed  = -55
                self.run_motor_speeds()

                if (s3 == 1 or s4 == 1) and s2 == 0:
                    print("------- left turn ended -------")
                    print("operating time : ", self.time_cnt)
                    self.time_cnt = 0 
                    self.s1_cnt = 0
                    self.edge_cnt = 1

            else:
                print("------- max time over -------")
                print("operating time : ", self.time_cnt)
                self.time_cnt = 0 
                self.s1_cnt = 0
                self.edge_cnt = 1
        
        elif self.s4_cnt == 1:

            if self.time_cnt < self.forward:
                print("------- right edge -------")
                print("operating time : ", self.time_cnt)
                self.right_speed = 100
                self.left_speed  = 100
                self.run_motor_speeds()

            elif self.time_cnt < self.forward + self.turn_min:
                print("------- turning -------")
                print("operating time : ", self.time_cnt)
                self.right_speed = -100
                self.left_speed  = 100
                self.run_motor_speeds()

            elif self.time_cnt < self.forward + self.turn_max:
                print("------- min time over -------")
                print("operating time : ", self.time_cnt)
                self.right_speed = -55
                self.left_speed  = 55
                self.run_motor_speeds()

                if (s1 == 1 or s2 == 1) and s3 == 0:
                    print("------- right turn ended -------")
                    print("operating time : ", self.time_cnt)
                    self.time_cnt = 0 
                    self.s4_cnt = 0
                    self.edge_cnt = 1
            else:
                print("------- max time over -------")
                print("operating time : ", self.time_cnt)
                self.time_cnt = 0 
                self.s4_cnt = 0
                self.edge_cnt = 1

    def integral_calculator(self):
        return sum(self.error_list)

    def calc_error(self, s1, s2, s3, s4):
        return (s1 * -3 + s2 * -1 + s3 * 1 + s4 * 3) / 4
    
    def cleanup(self):
        """A method to clean up GPIO pins properly."""
        GPIO.cleanup()