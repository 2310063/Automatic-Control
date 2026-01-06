import threading
import time
import cv2
# import RPi.GPIO as GPIO
import numpy as np
import tensorflow as tf
# from tensorflow.keras.models import load_model
from intelligent_cnt.term_project_2.infer_utils import load_interpreter, run_inference
import os
import glob
# ccw_model_baseline_class.tflite is really good

# this sucks
# model = "no_gen_class.tflite"

model = "cor_ccw_class_aug_0.21.tflite"
# model = "ccw_class.tflite"
data_path = "video_ccw_cor"


interpreter, in_det, out_det = load_interpreter(model)

search_path = os.path.join(data_path, 'train_*.png')
file_list = glob.glob(search_path)

for file_path in file_list:
        try:
            filename = os.path.basename(file_path)
            parts = filename.split('_')
            angle_str = parts[-1].split('.png')[0]
            angle = float(angle_str)

            # --- Read the image ---
            img = cv2.imread(file_path)

            if img is None:
                print(f"Warning: Could not read image {file_path}. Skipping.")
                continue
            
            # Convert BGR (OpenCV default) to RGB
            # img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
            probs, y = run_inference(interpreter, in_det, out_det, img)
            cls = np.argmax(probs)
            direction = "L" if cls == 0 else ("F" if cls == 1 else "R")
            conf = float(probs[cls])
            cv2.imshow(f"Predicted class : {direction}, prob : {conf}", img)
            key = cv2.waitKey(0)
            if key == ord('n'):  # Press 'n' to go to the next image
                continue
            elif key == ord('q'): # Press 'q' to quit
                break
            
        except Exception as e:
            print(f"Warning: Error processing file {file_path}. Error: {e}. Skipping.")