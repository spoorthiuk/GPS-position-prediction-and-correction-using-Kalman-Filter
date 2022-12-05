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
The Gyroscope gives the angular speed which can be integrated to obtain Euler angles (g_r, g_p, g_y) roll(Θg), pitch(Φg) and yaw(Ψg). The Magnetometer and Accelerometer’s readings are fused together by sensor fusion [6] to obtain Euler angles (f_r, f_p, f_y)  roll(Θf), pitch(Φf) and yaw(Ψf).
### Set 1 : Fusion Euler angles (Θf, Φf, Ψf) : 
![image](https://user-images.githubusercontent.com/43513525/205560071-ff1dc231-0cba-41ee-abf4-8f71eb62c5df.png)  

### Set 2 :  Gyroscope Euler angles (Θg, Φg, Ψg) :
![image](https://user-images.githubusercontent.com/43513525/205560249-ff920183-ef06-43e6-a5a0-5a87b56a69c6.png)  

These values are fed to a Kalman filter by considering fusion angles as the observed 
values and gyroscope angles as the measured values. 
The equations are modified as  
![image](https://user-images.githubusercontent.com/43513525/205560628-42c0eedb-73ff-45ae-83cb-f7064b68a223.png)  

The estimated Euler angels are used to construct rotation matrices  
![image](https://user-images.githubusercontent.com/43513525/205560760-626bf091-dcae-4efe-96a9-56a73a2668b1.png)

From theses matrices, we can find Direction Cosine Matrix (DCM) by matrix 
multiplication.  
![image](https://user-images.githubusercontent.com/43513525/205560836-cfd05160-3d94-4e9b-8584-65c8a0e9baaa.png)  

In the second stage, we estimate the position and velocity of the object. The 
prediction stage is modified as  
![image](https://user-images.githubusercontent.com/43513525/205560900-a118b112-f4b5-4f8f-ac8f-83917e757f8b.png)  

And the correct and update stage is modified as  
![image](https://user-images.githubusercontent.com/43513525/205560966-f201781b-4a5a-4fd4-8fe8-952950bf4aec.png)

## Results

We first tested the algorithm on pre-recorded test data. After obtaining the prediction, we plotted it on google earth to compare the results.
![image](https://user-images.githubusercontent.com/43513525/205561334-c5b3e083-29fd-4a70-8bab-9d034f1ba73e.png)  
 
The above figure is the position plot using the test data on Google earth. the red dots are the faulty GPS measurements, the white dots are the predictions made by the linear Kalman Filter and the yellow dots are the predictions made by our proposed system.

![image](https://user-images.githubusercontent.com/43513525/205562024-fa1c5832-184e-46fa-a99e-f7d665bd04dc.png)  
![image](https://user-images.githubusercontent.com/43513525/205562215-12883547-8302-4b03-9446-b8d745a72674.png)  

In the above plots, the Blue line indicates Kalman filter prediction, the Orange line indicates Measured values and the Green line indicates current Corrected values.
Following are the Euler angle plots obtained while predicting the position using  the proposed system.
![image](https://user-images.githubusercontent.com/43513525/205563661-139168f0-05a6-487e-83e5-152e0b888d23.png)    
![image](https://user-images.githubusercontent.com/43513525/205563724-69fab9c1-f0c0-4a5d-85d5-4c894527d6c3.png)   
![image](https://user-images.githubusercontent.com/43513525/205562484-b338f492-6007-4133-a67a-f4e91154cccf.png)    

In the above plots, the Blue line indicates Kalman filter prediction, the Orange line indicates Measured values and the Green line indicates current Corrected values. We can observe that the Kalman Filter predicted line closely follows the Fusion angles which is more gradual and smooth compared to the Gyroscope angles.
After testing the algorithm on test data, we tested it in real time. The sensor readings and the GPS readings were read, processed and plotted on Google maps in real time.  
![image](https://user-images.githubusercontent.com/43513525/205562572-3f673c67-e8f2-4637-b5ce-026869ccc42e.png)  

In the above plot, the green line is the predicted position by our algorithm and the red line the measured position by the GPS module. After ending the program, the data was saved in a csv file for further analysis. 
Following is the position plot.  
![image](https://user-images.githubusercontent.com/43513525/205562669-35d8c56b-4bf6-4ee4-9aa1-c38b052ed1c9.png)    

The Orange line indicates the GPS measured data and the Blue line indicates the Kalman Filter predicted data. We can observe some spikes in the GPS measured data. This is rectified by considering the previously predicted value as current value if the error is greater than 50 m.
Following are the velocity plots obtained during real time prediction.  
![image](https://user-images.githubusercontent.com/43513525/205563834-6a3c5a06-714d-4a29-b1c1-b43747ff5f21.png) 
![image](https://user-images.githubusercontent.com/43513525/205562849-3d3a7d3d-8227-41b5-a540-756f0882bec8.png)  
![image](https://user-images.githubusercontent.com/43513525/205562870-553bf25e-4d59-4476-ba31-8370b9b4ab58.png)  

## Conclusion

Kalman filters is a very powerful tool and is used in a variety of state estimation systems. They’re able to make accurate predictions and is still computationally cheap. Methods such as lane tracking, and traffic sign localization together with map matching can be used along with sensor fusion to make more accurate prediction.
The GPS devices used in systems are of high grade and expensive. It is possible to arrive at a low cost solution using a cheaper GPS module and IMU measurements with the help of the Kalman Filters. 

