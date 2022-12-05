# GPS-position-prediction-and-correction-using-Kalman-Filter

## Introduction

The low-grade GPS module receiver often tends to lose the signal coming from satellites due to factors such as electromagnetic interference and multipath effect.
Kalman Filter is an iterative mathematical process that quickly estimates one’s true position by using a set of equations and consecutive data inputs. In order to reduce errors made by a GPS in prediction of a vehicle’s position, linear Kalman Filter and non-linear Kalman Filter algorithms are used. 
In addition to this u-Blox Neo6M GPS module is used to measure the vehicle’s position. Data from a triaxial accelerometer-ADXL345, triaxial magnetometer-QMC5833L and triaxial gyroscope-MPU6050 are used in correcting the vehicle's position. Kalman FIlter is a three stage process and it provides low cost and accurate estimation of the object's position. The model was tested in real time on Raspberry Pi 3 and the predicted results were plotted on Google Maps

## Proposed System

The proposed system consists of a triaxial Accelerometer, Gyroscope and Magnetometer along with position and velocity measured by the GPS module. Raspberry Pi 3 is used to receive the sensor readings and perform real time analysis. 
An adaptive filter called Kalman Filter is used to predict and correct the state vector. Initially, the diagonal elements in the covariance matrix used in the Kalman Filter are populated with the standard deviation values of the state variables. As the number of iterations increases, the filter becomes more confident about its’ prediction and the standard deviation is reduced.

![image](https://user-images.githubusercontent.com/43513525/205542692-fb07c4f6-81e8-4b49-ba3e-4fe56edd4682.png)

## Linear Kalman Filter 

The Kalman Filtering process involves 3 major steps: Predict, Correct and Update.   
                      ![image](https://user-images.githubusercontent.com/43513525/205558091-32e7974d-96b2-480b-871f-4f422c145574.png)   
                           Above is the flow diagram of a typical Kalman Filter   
It’s a discrete-time process. In the predict stage, the algorithm makes the best possible prediction of the next state with the knowledge of physics.
All the variables that are used in the Linear Kalman Filter exhibit linear relation. To increase the accuracy of the system, we take the orientation of the body into consideration. This requires modification in the algorithm to handle the nonlinearities.  
Below is the flow diagram of the the equations involved in the Kalman Filter process.  
![image](https://user-images.githubusercontent.com/43513525/205558898-55b82c30-2686-4c46-9cf9-464c3f89800c.png)

## Non-Linear Kalman Filter

The proposed non-linear Kalman Filter system consists of two main stages. In the first stage, the orientation of the object is estimated and in the second stage, the Kalman Filter uses the estimated angles from the first step to estimate the position.  
![image](https://user-images.githubusercontent.com/43513525/205559483-0e0255ef-1225-4496-851d-42d3ac734747.png)
In the first stage, we estimate the Euler angles, roll(θ), pitch(φ) and yaw(Ψ), to estimate the orientation of the object. These angles are used to transform vectors from body frame to the navigation frame.
The Gyroscope gives the angular speed which can be integrated to obtain Euler angles (g_r, g_p, g_y) roll(![image](https://user-images.githubusercontent.com/43513525/205559607-61fec4a3-2ad9-43c8-92f8-b54dfbf0c002.png)
), pitch( ) and yaw( ). The Magnetometer and Accelerometer’s readings are fused together by sensor fusion [6] to obtain Euler angles (f_r, f_p, f_y)  roll( ), pitch( ) and yaw( ).
Set 1 : Fusion Euler angles ( ,   ,   ) : 


