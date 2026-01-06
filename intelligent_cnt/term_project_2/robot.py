import time
import RPi.GPIO as GPIO
import numpy as np
from collections import deque
import cv2
import numpy as np
import tensorflow as tf
from infer_utils import load_interpreter, run_inference
from camera import cam
import datetime

class Robot:
    def __init__(self,
                 args,
                 model_name = 'ccw_cor_cnt_0.93.tflite',
                 ):
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

        # -------- PID parameters --------

        self.Kp = 75  # WAS 75. This is a much safer starting point.
        self.Ki = 5   # WAS 10.
        self.Kd = 60  # WAS 35.

        # -------- Define PID variables --------
        # self.base_speed = 60 * 1.0
        self.base_speed = 50

        self.left_speed = self.base_speed
        self.right_speed = self.base_speed

        self.direction = args.direction


        # -------- Set sensor counters --------
        self.low_speed_cnd = False

        self.path_history = deque(maxlen=100)
        self.lost_history = deque(maxlen = 5)
        self.images_log = deque(maxlen = 30)
        self.steer_log = deque(maxlen = 30)
        self.lost_path_timer = 0
        self.is_lost = False
        self.lost_path_duration = 7

        # -------- Set time counters --------
        # min, max time for edge detection
        self.time_min = 1.3
        
        self.forward = 0.6

        self.turn_min = 0.1
        self.turn_max = 3.8

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

        # Motor objects also become attributes
        self.L_Motor = GPIO.PWM(self.PWMA, 500)
        self.L_Motor.start(0)

        self.R_Motor = GPIO.PWM(self.PWMB, 500)
        self.R_Motor.start(0)

        self.cam = cam()

        model_path = f'model_tflite/{model_name}'

        self.interpreter, self.in_det, self.out_det = load_interpreter(model_path)

        print("Robot initialized successfully!")
        print("Using model : ", model_name)

        # Initialize sensor value
        self.steer_list = deque(maxlen=10)

        self.left_cnt = 0
        self.right_cnt = 0

    def motor_stop(self, ):
        GPIO.output(self.AIN1,0)
        GPIO.output(self.AIN2,1)
        self.L_Motor.ChangeDutyCycle(0)

        GPIO.output(self.BIN1,0)
        GPIO.output(self.BIN2,1)
        self.R_Motor.ChangeDutyCycle(0)

    def process_steer(self, steer):
        if steer == 45:
            self.left_cnt += 1
            self.left_cnt = np.clip(self.left_cnt, 0, 10)
            self.right_cnt = 0
            
            self.left_speed  = - 2 * self.left_cnt - self.base_speed
            self.right_speed = self.base_speed

        elif steer == 135:
            self.right_cnt += 1
            self.right_cnt = np.clip(self.right_cnt, 0, 10)
            self.left_cnt   = 0

            self.left_speed  = self.base_speed
            self.right_speed = - 2 * self.right_cnt - self.base_speed

        else:
            self.left_cnt  = 0
            self.right_cnt = 0
        
            self.left_speed  = self.base_speed
            self.right_speed = self.base_speed

            print("Set to normal speed")
        
    def run_motor_speeds(self):
        """
        runs motor for a short duration based on the current left and right speed attributes
        """
        if not self.is_lost:
            self.path_history.append((self.left_speed, self.right_speed, self.ctr_freq))

        max_speed = 100
        min_speed = 0

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

    def logger(self, ):
        self.steer_log
        for i, img in enumerate(self.images_log):
            filepath = "/home/user/Documents/corner_data5/"
            # steer = self.steer_log[i] / 45 - 2
            cv2.imwrite("%s_%05d_%03d.png" % (filepath, i, 1), img)
            print(f"{filepath}{i}, {1} has been wrote")

    def lost_checker(self, contour):
        print("----------------", np.sum(contour), "----------------")
        if np.sum(contour) < 400000:
            self.lost_history.append(1)
            print("--------- Appending lost history ---------")

        else:
            self.lost_history.append(0)

        if sum(self.lost_history) >= len(self.lost_history) - 1:
            print("--------- Path lost detected. ---------")
            return self.is_lost == True

    def run(self, ):
        t0 = datetime.datetime.now()

        try:
            while True:
                t1 = datetime.datetime.now()
                image, pro, contour = self.cam.get_image()
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                self.show_img(image, pro, contour)
                
                
                probs, y = run_inference(self.interpreter, self.in_det, self.out_det, contour)
                t2 = datetime.datetime.now()
                carState = "go"
                print("inference speed : ", t2 - t1)

                steering_angle = int(np.argmax(probs))
                steering_angle += 1 # 1, 2, 3
                steering_angle *= 45 # 45, 90, 135
                print("predicted angle : ",steering_angle)
                self.process_steer(steering_angle)

                self.images_log.appendleft(contour)
                self.steer_log.appendleft(steering_angle)

                if self.lost_checker(contour):
                    self.is_lost = True

                    if self.direction == "ccw":
                        print("Path lost! Turning CCW to find path.")
                        self.left_speed = self.base_speed
                        self.right_speed = self.base_speed
                        self.run_motor_speeds()
                        time.sleep(1)

                        self.left_speed = -30
                        self.right_speed = 30
                        self.run_motor_speeds()
                        time.sleep(0.5)
                        t4 = datetime.datetime.now()
                        print("Time stamp : ", t4 - t0)

                    elif self.direction == "cw":
                        print("Path lost! Turning CW to find path.")

                        self.left_speed = self.base_speed
                        self.right_speed = self.base_speed
                        self.run_motor_speeds()
                        time.sleep(1)

                        self.left_speed = 30
                        self.right_speed = -30
                        self.run_motor_speeds()
                        time.sleep(0.5)
                        t4 = datetime.datetime.now()
                        print("Time stamp : ", t4 - t0)

                if carState == "go":
                    if steering_angle == 45:
                        print("left")
                        self.run_motor_speeds()
                        # time.sleep(0.05)
                        t4 = datetime.datetime.now()
                        print("Time stamp : ", t4 - t0)

                    elif steering_angle == 90:
                        print("go")
                        self.run_motor_speeds()
                        # time.sleep(0.05)
                        t4 = datetime.datetime.now()
                        print("Time stamp : ", t4 - t0)

                    elif steering_angle == 135:
                        print("right")
                        self.run_motor_speeds()
                        # time.sleep(0.05)
                        t4 = datetime.datetime.now()
                        print("Time stamp : ", t4 - t0)

                elif carState == "stop":
                    self.motor_stop()
                
        except KeyboardInterrupt:
            self.logger()
            self.cleanup()

    def show_img(self, image, processed, contour):
        cv2.imshow("original image", image)
        cv2.imshow("Processed Mask (200x66)", processed)
        cv2.imshow('contours', contour)
    
    def cleanup(self):
        """A method to clean up GPIO pins properly."""
        cv2.destroyAllWindows()
        GPIO.cleanup()