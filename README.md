### Soruce code for 1st projects on Intelligent Control !

File consists of 3 main codes,
1. motor.py
2. motor_error_correct.py
3. motor_error_correct_with_dynamic.py

and following operating codes: line_follow.py, line_follow_error_correct.py, line_follow_error_correct_with_dy.py

## motor.py is basic code contains main movements.
PID, condition checker for edges, and rushing robot.

## motor_error_correct.py has more sophiscated path returning codes.
If it fails to detect errors for 4s, robot returns to its previous position that 4 seconds before.

## motor_error_correct_with_dynamic.py has advanced motor controlling system.
Untill it loses its path, runs at high PWM, then lower PWM after lost path so that sensor error can not be rejected.
