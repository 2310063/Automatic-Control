# Soruce code for 2nd projects on Intelligent Control !
Below is explanaitons for codes.

## model_tflite.
This folder contains trained networks for classfy line.
Quantized with tflite framework, trained with different datasets.

## utils
This folder contains utilities for collecting datas.
edge_data_collect is specifically for collecting edge datasets to fine-tune models.

## car_runner.py
Base code for running car.
Futher developed in roboto.py

## camera.py
Camera module for preprocessing.
Uses simple threshold function in cv2.
Tried Otsu's binarization, and adaptive threshold, but not working fine

## infer_utils.py
Utility for classification.

## robot.py
Main running programs.

## run.py
Wrapper code.

## test.py
Testing classifiers.
