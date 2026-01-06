import threading
import time
import cv2
import RPi.GPIO as GPIO
import numpy as np
import tensorflow as tf
from infer_utils import load_interpreter, run_inference
from camera import cam

PWMA = 18
AIN1 = 22
AIN2 = 27

PWMB = 23
BIN1 = 25
BIN2 = 24

def motor_back(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2,False)#AIN2
    GPIO.output(AIN1,True) #AIN1
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,False)#BIN2
    GPIO.output(BIN1,True) #BIN1
    
def motor_go(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2,True)#AIN2
    GPIO.output(AIN1,False) #AIN1
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,True)#BIN2
    GPIO.output(BIN1,False) #BIN1

def motor_stop():
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN2,False)#AIN2
    GPIO.output(AIN1,False) #AIN1
    R_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN2,False)#BIN2
    GPIO.output(BIN1,False) #BIN1
    
def motor_right(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2,True)#AIN2
    GPIO.output(AIN1,False) #AIN1
    R_Motor.ChangeDutyCycle(speed/3)
    GPIO.output(BIN2,False)#BIN2
    GPIO.output(BIN1,True) #BIN1
    
def motor_left(speed):
    L_Motor.ChangeDutyCycle(speed/3)
    GPIO.output(AIN2,False)#AIN2
    GPIO.output(AIN1,True) #AIN1
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,True)#BIN2
    GPIO.output(BIN1,False) #BIN1
        
def motor_speed_setter(memory : list):

    left = 1
    right = 1
    return left, right

GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN2,GPIO.OUT)
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)

GPIO.setup(BIN1,GPIO.OUT)
GPIO.setup(BIN2,GPIO.OUT)
GPIO.setup(PWMB,GPIO.OUT)

L_Motor= GPIO.PWM(PWMA,100)
L_Motor.start(0)

R_Motor = GPIO.PWM(PWMB,100)
R_Motor.start(0)


def main():
    camera = cam()
    folder_path = 'model_tflite'
    model_path = f'{folder_path}/ccw_cor_cnt_aug_0.89.tflite'
    # model = load_model(model_path)
    interpreter, in_det, out_det = load_interpreter(model_path)
    print("interpreter : ",interpreter)
    classification = True
    carState = "stop"
    
    try:
        while True:
            image, pro, cnt = camera.get_image()
            
            cv2.imshow("original image", image)
            cv2.imshow('contours', cnt)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            probs, y = run_inference(interpreter, in_det, out_det, cnt)
            carState = "go"

            if classification:

                steering_angle = int(np.argmax(probs))
                steering_angle += 1 # 1, 2, 3
                steering_angle *= 45 # 45, 90, 135
                print("predict class:", int(np.argmax(probs)))
                
            else:
                steering_angle = probs[0]
                print("predict angle:", steering_angle)
                
            if carState == "go":
                if steering_angle >= 70 and steering_angle <= 110:
                    print("go")
                    speedSet = 50
                    motor_go(speedSet)
                    time.sleep(0.1)
                elif steering_angle > 111:
                    print("right")
                    speedSet = 40
                    motor_right(speedSet)
                    time.sleep(0.1)
                elif steering_angle < 71:
                    print("left")
                    speedSet = 40
                    motor_left(speedSet)
                    time.sleep(0.1)
            elif carState == "stop":
                motor_stop()
            
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
