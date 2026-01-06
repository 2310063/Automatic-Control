import time
import cv2
import RPi.GPIO as GPIO
from camera import cam

PWMA = 18
AIN1 = 22
AIN2 = 27

PWMB = 23
BIN1 = 25
BIN2 = 24

def motor_back(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN1,True) #AIN1
    GPIO.output(AIN2,False)#AIN2
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN1,True) #BIN1
    GPIO.output(BIN2,False)#BIN2
    
def motor_go(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2,True) #AIN2
    GPIO.output(AIN1,False) #AIN1
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,True) #BIN2
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

    R_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN2,False)#BIN2
    GPIO.output(BIN1,True) #BIN1
    
def motor_left(speed):
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN2,False)#AIN2
    GPIO.output(AIN1,True) #AIN1

    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,True)#BIN2
    GPIO.output(BIN1,False) #BIN1
        
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

speedSet = 50

def main():
    camera = cam()
    filepath = "/home/user/Documents/data_ccw/"
    i = 0
    
    try:
        while True:
            carState = "stop"
            keyValue = cv2.waitKey(1)
        
            if keyValue == ord('q'):
                motor_stop()
                break

            elif keyValue == 82:
                print("go")
                carState = "go"
                motor_go(speedSet)
                time.sleep(0.3)
                motor_stop()

            elif keyValue == 84:
                print("stop")
                carState = "stop"
                motor_stop()
                time.sleep(0.2)

            elif keyValue == 81:
                print("left")
                carState = "left"
                motor_left(speedSet)
                time.sleep(0.2)
                motor_stop()

            elif keyValue == 83:
                print("right")
                carState = "right"
                motor_right(speedSet)
                time.sleep(0.2)
                motor_stop()
            
            image, pro, contour = camera.get_image()
            
            if carState == "left":
                cv2.imwrite("%s_%05d_%03d.png" % (filepath, i, -1), contour)
                print(f"{filepath}{i}, {-1} has been wrote")
                i += 1
            elif carState == "go":
                cv2.imwrite("%s_%05d_%03d.png" % (filepath, i, 0), contour)
                print(f"{filepath}{i}, {0} has been wrote")
                i += 1
            elif carState == "right":
                cv2.imwrite("%s_%05d_%03d.png" % (filepath, i, 1), contour)
                print(f"{filepath}{i}, {1} has been wrote")
                i += 1
            
            cv2.imshow('Original', image)
            cv2.imshow('Processed', contour)
            
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
    GPIO.cleanup()
