import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
#This script tests the linear kalman filter algorithm for a small test data
obs=np.array([[4000,5000,280,245],[4260,5204,282,250],[4550,5408,285,252],[4860,5722,286,254],[5110,6101,290,259]])
obs=np.array(obs)
ax=[[2],[2]]
ax=np.array(ax)
Qk=0
Wk=0
Zk=0
t=1
xd=20
vd=5
yd=20
xd_obs=25
vd_obs=6
yd_obs=25

A=np.array([[1,0,t,0],[0,1,0,t],[0,0,1,0],[0,0,0,1]])
B=np.array([[0.5*(t*t),0],[0,0.5*(t*t)],[t,0],[t,0]])

i=0
xkp=np.zeros((5,4))
x_curr=np.zeros((5,4))
x_curr[i]=obs[0]
xkp[i]=obs[0]
xkp[i+1]=np.dot(A,np.transpose(obs[i]))+np.transpose(np.dot(B,ax))+Wk
P_ik=np.array([[xd*xd,0,0,0],[0,yd*yd,0,0],[0,0,vd*vd,0],[0,0,0,vd*vd]])
Pkp=np.dot(np.dot(A,P_ik),np.transpose(A))+Qk
Pkp[0][2]=Pkp[1][3]=Pkp[2][0]=Pkp[3][1]=0
H=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
R=np.array([[xd_obs*xd_obs,0,0,0],[0,yd_obs*yd_obs,0,0],[0,0,vd_obs*vd_obs,0],[0,0,0,vd_obs*vd_obs]])
K=np.dot(np.dot(Pkp,(np.transpose(H))),np.linalg.inv(np.dot(np.dot(H,Pkp),np.transpose(H))+R))
C=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
yk=np.dot(C,np.transpose(obs[i+1]))+Zk
x_curr[i+1]=np.transpose(xkp[i+1])+np.dot(K,(yk-(np.dot(H,np.transpose(xkp[i+1])))))
I=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
Pk=np.dot((I-np.dot(K,H)),Pkp)

for i in range(1,4):
    xkp[i+1]=(np.dot(A, np.transpose(obs[i])) + np.transpose(np.dot(B, ax)) + Wk)
    P_ik = np.array([[xd * xd, 0, 0, 0], [0, yd * yd, 0, 0], [0, 0, vd * vd, 0], [0, 0, 0, vd * vd]])
    Pkp = np.dot(np.dot(A, P_ik), np.transpose(A)) + Qk
    Pkp[0][2] = Pkp[1][3] = Pkp[2][0] = Pkp[3][1] = 0
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    R = np.array([[xd_obs * xd_obs, 0, 0, 0], [0, yd_obs * yd_obs, 0, 0], [0, 0, vd_obs * vd_obs, 0],
                  [0, 0, 0, vd_obs * vd_obs]])
    K = np.dot(np.dot(Pkp, (np.transpose(H))), np.linalg.inv(np.dot(np.dot(H, Pkp), np.transpose(H)) + R))
    C = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    yk = np.dot(C, np.transpose(obs[i + 1])) + Zk
    x_curr[i+1]=(xkp[i+1]) + np.dot(K, (yk - (np.dot(H, np.transpose(xkp[i+1])))))
    I = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    Pk = np.dot((I - np.dot(K, H)), Pkp)
T=[0,1,2,3,4]
plt.subplot(2,2,1)
plt.plot(T,np.transpose(x_curr)[0],'-o')
plt.plot(T,np.transpose(obs)[0],'-*')
plt.plot(T,np.transpose(xkp)[0],'-s')
plt.title('x position plot')
plt.xlabel('Time')
plt.ylabel('Position')
plt.subplot(2,2,2)
plt.plot(T,np.transpose(x_curr)[1],'-o')
plt.plot(T,np.transpose(obs)[1],'-*')
plt.plot(T,np.transpose(xkp)[1],'-s')
plt.title('y position plot')
plt.xlabel('Time')
plt.ylabel('Position')
plt.subplot(2,2,3)
plt.plot(T,np.transpose(x_curr)[2],'-o')
plt.plot(T,np.transpose(obs)[2],'-*')
plt.plot(T,np.transpose(xkp)[2],'-s')
plt.title('x velocity plot')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.subplot(2,2,4)
plt.plot(T,np.transpose(x_curr)[3],'-o')
plt.plot(T,np.transpose(obs)[3],'-*')
plt.plot(T,np.transpose(xkp)[3],'-s')
plt.figlegend(['Kalman filter(Predicted)','Measured','Current'])
plt.title('y velocity plot')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.figure(2)
plt.plot(np.transpose(x_curr)[0],np.transpose(x_curr)[1],'-o')
plt.plot(np.transpose(obs)[0],np.transpose(obs)[1],'-*')
plt.plot(np.transpose(xkp)[0],np.transpose(xkp)[1],'-s')
plt.title('Object position plot')
plt.xlabel('x co-ordinate')
plt.ylabel('y co-ordinate')
plt.legend(['Kalman filter(Predicted)','Measured','Current'])
plt.show()