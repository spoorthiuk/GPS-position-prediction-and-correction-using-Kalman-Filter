# GPS-position-prediction-and-correction-using-Kalman-Filter

## Introduction

The low-grade GPS module receiver often tends to lose the signal coming from satellites due to factors such as electromagnetic interference and multipath effect.
Kalman Filter is an iterative mathematical process that quickly estimates one’s true position by using a set of equations and consecutive data inputs. In order to reduce errors made by a GPS in prediction of a vehicle’s position, linear Kalman Filter and non-linear Kalman Filter algorithms are used. 
In addition to this u-Blox Neo6M GPS module is used to measure the vehicle’s position. Data from a triaxial accelerometer-ADXL345, triaxial magnetometer-QMC5833L and triaxial gyroscope-MPU6050 are used in correcting the vehicle's position. Kalman FIlter is a three stage process and it provides low cost and accurate estimation of the object's position. The model was tested in real time on Raspberry Pi 3 and the predicted results were plotted on Google Maps

## Proposed System

The proposed system consists of a triaxial Accelerometer, Gyroscope and Magnetometer along with position and velocity measured by the GPS module. Raspberry Pi 3 is used to receive the sensor readings and perform real time analysis. 
An adaptive filter called Kalman Filter is used to predict and correct the state vector. Initially, the diagonal elements in the covariance matrix used in the Kalman Filter are populated with the standard deviation values of the state variables. As the number of iterations increases, the filter becomes more confident about its’ prediction and the standard deviation is reduced.

![image](https://user-images.githubusercontent.com/43513525/205542692-fb07c4f6-81e8-4b49-ba3e-4fe56edd4682.png)

