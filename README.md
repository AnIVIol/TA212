# 2-DOF Gimbal with Inverse Kinematics and PID
---
This was part of a course project in **TA212 (Manufacturing processes II)** to add a twist to a bare metal creation, nothing fancy. We used a pre trained HAAR cascade to detect faces and send centroid coordinates of the bounding box over serial to an **Arduino UNO**. 

---
## Overview

The code is designed to:
1. Receive **X, Y** coordinates over the serial port.
2. Calculate the necessary angles for the servos using inverse kinematics.
3. Use a PID controller to achieve the desired angles smoothly and accurately.

---


### Variables and Constants

- `PWM_LOWER`, `PWM_UPPER`: PWM pins for the servos.
- `L1`, `L2`: Lengths of the arm segments.
- `x`, `y`: Target X, Y coordinates.
- `prevX`, `prevY`: Previous X, Y coordinates.
- `setpoint1`, `input1`, `output1`: PID variables for the first servo.
- `setpoint2`, `input2`, `output2`: PID variables for the second servo.
- `Kp`, `Ki`, `Kd`: PID constants.

---
### Functions

- `setup()`: Initializes serial communication and attaches servos.
- `Pos()`: Updates speed variables based on position changes.
- `loop()`: Reads X, Y coordinates from the serial port, computes the target angles using inverse kinematics, and adjusts the servo positions using PID control.

---
### Inverse Kinematics

The angles for the servos are calculated using the following equations:

\[ \theta_2 = \arccos\left(\frac{x^2 + y^2 - L1^2 - L2^2}{2 \cdot L1 \cdot L2}\right) \]
\[ \theta_1 = \arctan2(y, x) - \arctan2\left(L2 \cdot \sin(\theta_2), L1 + L2 \cdot \cos(\theta_2)\right) \]

---
### PID Control

The PID controller ensures that the servo angles reach the desired positions smoothly and accurately. The PID library is used to compute the control signals.

---
## Face Detection Program

The X, Y coordinates are generated by a face detection program using a pre-trained Haar cascade. The face detection program sends the coordinates over serial communication to the Arduino, which then adjusts the gimbal's position accordingly.


