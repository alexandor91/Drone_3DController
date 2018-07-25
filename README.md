# Robo_nd Follow_me
This repository is established for the project 3 of Udacity "Flying car" nanodegree program
---
## Project: 3DController

---

## C++ Environment Setup

---
1. Download or clone the [project repository](https://github.com/udacity/FCND-Controls-CPP) onto the local pc.  
2. Use any C++ editor to fire the project, I used Visual Studio as editor, load the "sln" project and retarget the solution for new version SDK, then compile and run the program.
---

## Code Implementation
All the codes are included in the "QuadControl.cpp" file.
### 1. Bodyrate Controller
The code corresponding to body control is from line 119 to line 128.\
'''\
float p_error = pqrCmd[0] - pqr[0];\
float u_bar_p = Ixx * kpPQR[0] * p_error;

float q_error = pqrCmd[1] - pqr[1];\
float u_bar_q = Iyy * kpPQR[1] * q_error;

float r_error = pqrCmd[2] - pqr[2];\
float u_bar_r = Izz * kpPQR[2] * r_error;

V3F momentCmd(u_bar_p, u_bar_q, u_bar_r);\
'''\
Fron the code snippet above, it is obvious that the commanded moments are proportional to the difference of body rates, the intertia od the drone about each axis is also taken into account.
### 2. Roll Pitch Controller
The code corresponding to roll pitch control is from 157 to 173.\
'''\
 float c_d = collThrustCmd / mass;\
  if (collThrustCmd > 0.0){\
    float target_R13 = -CONSTRAIN(accelCmd[0] / c_d, -maxTiltAngle, maxTiltAngle);\
    float target_R23 = -CONSTRAIN(accelCmd[1] / c_d, -maxTiltAngle, maxTiltAngle);\
    pqrCmd[0] = (1 / R(2, 2)) * (-R(1, 0) * kpBank * (R(0, 2) - target_R13) + R(0, 0) * kpBank * (R(1, 2) - target_R23));\
    pqrCmd[1] = (1 / R(2, 2)) * (-R(1, 1) * kpBank * (R(0, 2) - target_R13) + R(0, 1) * kpBank * (R(1, 2) - target_R23));\
  }
  else {\
  pqrCmd[0] = 0.0;\
  pqrCmd[1] = 0.0;\
  collThrustCmd = 0.0;\
  }\
'''\
To calculate the target angle, drone mass is accounted for. The element in the non-linear rotation matrix is responsible for the non-linear transform from local acceleration to body rate. This pitch and roll controller is cascaded to the body rate controller, so the output is desired body rate.
### 3. Altitude Controller
The code of this controller part is from line 203 to line 211.\
'''\
float error = posZCmd - posZ;\
float h_dot_cmd = kpPosZ * error + velZCmd;\
h_dot_cmd = CONSTRAIN(h_dot_cmd, -maxAscentRate, maxDescentRate);\
integratedAltitudeError += error * dt;\
float acceleration_cmd = accelZCmd + kpVelZ * (h_dot_cmd - velZ) + KiPosZ * integratedAltitudeError;\

float R33 = R(2,2);\
thrust = mass * (9.81f - acceleration_cmd) / R33;\
thrust = CONSTRAIN(thrust, 4 * minMotorThrust, 4 * maxMotorThrust);\
'''\
The bottom two lines convert the desired acceleration to collective thrust, also includes the non-linear effects from non-zero roll or pitch angles, the two p controllers, cascaded together form the second order control system, one for velocity control, the other for position control. Aside from the normal p term, the integrator is adopted to handle the shift of weight presented in one of the drones of scenario 4. 
### 4. Lateral Position Controller
Line 247 to line 263 is the code for horizontal position control.\
'''\
velCmd.x = kpPosXY * (posCmd.x - pos.x);\
velCmd.y = kpPosXY * (posCmd.y - pos.y);\
float vel_norm = sqrt(velCmd.x * velCmd.x + velCmd.y * velCmd.y);\
 
if (vel_norm > maxSpeedXY) {\
    velCmd.x = CONSTRAIN(velCmd.x, -velCmd.x * maxSpeedXY / vel_norm, velCmd.x * maxSpeedXY / vel_norm);\
    velCmd.y = CONSTRAIN(velCmd.y, -velCmd.y * maxSpeedXY / vel_norm, velCmd.y * maxSpeedXY / vel_norm);\
  }\
accelCmd.x += kpPosXY * (posCmd.x - pos.x) + kpVelXY * (velCmd.x - vel.x);\
accelCmd.y += kpPosXY * (posCmd.y - pos.y) + kpVelXY * (velCmd.y - vel.y);\

float accel_norm = sqrt(accelCmd.x * accelCmd.x + accelCmd.y * accelCmd.y);\

if (accel_norm > maxAccelXY) {\
	  accelCmd.x = CONSTRAIN(accelCmd.x, -accelCmd.x * maxAccelXY / accel_norm, accelCmd.x * maxAccelXY / accel_norm);\
	  accelCmd.y = CONSTRAIN(accelCmd.y, -accelCmd.y * maxAccelXY / accel_norm, accelCmd.y * maxAccelXY / accel_norm);\
  }\
'''\
The local NE position and velocity  are used to generate a commanded local acceleration, here the desired velocity and the desired accceleration are constrained to the bounding range to ensure safe operation in reality.
### 5. Yaw Controller
Line 284 to line 291 for yaw control.\
'''\
yawCmd = fmodf(yawCmd, 2.0 * 3.1416f);\
float yaw_error = yawCmd - yaw;\
if (yaw_error > 3.1416f)
	  yaw_error = yaw_error - (2.0f * 3.1416f);\
else if (yaw_error < -3.1416f)
	  yaw_error = yaw_error + (2.0f * 3.1416f);\
  
yawRateCmd = kpYaw * yaw_error;\
'''\
The code snippet above is very simple, the controller output is proportional  to the difference between disired yaw and actual yaw.
### 6. Motor Commands
Line 73 to line 92 for four motor commands given the moment and collective thrust commands.\
'''\
float l = L / sqrt(2);\
float term_A = collThrustCmd;\
float term_B = momentCmd[0]/ l;\
float term_C = momentCmd[1] / l;\
float term_D = -momentCmd[2] /kappa;\

float F0 = (term_A + term_B + term_C + term_D) / 4.0f; //front left\
float F1 = (term_A + term_C - term_B - term_D) / 4.0f; //front right\
float F2 = (term_A + term_B - term_C - term_D) / 4.0f; //rear left\
float F3 = (term_A + term_D - term_B - term_C) / 4.0f; //rear right\

//printf("F0 force %f/n", term_A);\
//printf("F1 force %f/n", term_B);\
//printf("F2 force %f/n", term_C);\
//printf("F3 force %f/n", term_D);\

cmd.desiredThrustsN[0] = CONSTRAIN(F0, minMotorThrust, maxMotorThrust);\
cmd.desiredThrustsN[1] = CONSTRAIN(F1, minMotorThrust, maxMotorThrust);\
cmd.desiredThrustsN[2] = CONSTRAIN(F2, minMotorThrust, maxMotorThrust);\
cmd.desiredThrustsN[3] = CONSTRAIN(F3, minMotorThrust, maxMotorThrust);\
'''
The output here is the individual thrust force generated by each motor, the three moments about respective axis along with the collective force are mapped accordingly, to note is the moment about z axis, in local NED all the upward thrust are negative.

## Results
The final results for different scenarios are presented below.
1 
![Metric IOU for merged training dataset](img/attitude-scenario.jpg)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Metric IOU for merged training dataset<br />
The snapshot above shows the average IoU value for the hero detection, approximately 42% is obtained with the tuned hyperparameters.

![Plot of cost](plot_cost.png)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Plot of cost vs epoch<br />
This is the plot corresponding to the training with merged dataset with about 5000 images.

This last one is the plot corresponding to the new dataset. The cross function for training decreases rapidly after the first epoch and then slowly with more epochs, but it proves the outcome is still improving, while the cost for the validation is almost unchanged from 1st epoch, the order of magnitude stay around one decimal. Even the overfitting occurs after 9th epoch.
