import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math as m

#################FUNCTIONS#######################
######Conversion from Latitude and Longitude to x and y########################
Re= 6378137
eq_circum = 2 * m.pi * Re
init_res= eq_circum / 256.0
origin_shift= eq_circum / 2.0

def latlontoPixels(lat, lon, zoom):
     mx = (lon * origin_shift) / 180.0
     my = m.log(m.tan((90 + lat) * m.pi / 360.0)) / (m.pi / 180)
     my = (my * origin_shift) / 180.0
     res = init_res / (2 ** zoom)
     px = (mx + origin_shift) / res
     py = (my + origin_shift) / res
     return px, py

def pixelstoLatLon(px, py, zoom):
     res = init_res/ (2 ** zoom)
     mx = px * res - origin_shift
     my = py * res - origin_shift
     lat = (my / origin_shift) * 180.0
     lat = (180 / m.pi * (2 * m.atan(m.exp(lat * m.pi / 180.0)) - m.pi / 2.0))
     lon = (mx / origin_shift) * 180.0
     return lat, lon
###############Euler angles calculations###########
def gyro_euler(g,dt):
    G_roll=g[0]*dt
    G_pitch=g[1]*dt
    G_yaw=g[2]*dt
    return [G_roll,G_pitch,G_yaw]

def fusion_euler(a,mag,dt):
    A_roll=(a[1]/a[2])
    A_pitch=m.atan(-a[0]/m.sqrt((a[1]**2)+(a[2]**2)))
    Mx=mag[0]*m.cos(A_pitch)+mag[2]*m.sin(A_pitch)
    My=mag[0]*m.sin(A_roll)*m.sin(A_pitch)+mag[1]*m.cos(A_roll)-mag[2]*m.sin(A_roll)*m.cos(A_pitch)
    A_yaw=m.atan2(-My,Mx)
    return [A_roll,A_pitch,A_yaw]
###########Body frame to navigation frame########
def dcm(phi,theta,psi):
    cbn=[[m.cos(theta)*m.cos(psi),-m.cos(phi)*m.sin(psi)+m.sin(phi)*m.sin(theta)*m.cos(psi),m.sin(phi)*m.sin(psi)+m.cos(phi)*m.sin(theta)*m.cos(psi)],
         [m.cos(theta)*m.sin(psi),m.cos(phi)*m.cos(psi)+m.cos(phi)*m.sin(theta)*m.sin(psi),-m.sin(phi)*m.cos(psi)+m.cos(phi)*m.sin(theta)*m.sin(psi)],
         [-m.sin(theta),m.sin(phi)*m.cos(theta),m.cos(phi)*m.cos(theta)]]
    return cbn
###########Kalman Filters########################
def filter_init():
    sig_r = 0.1
    sig_p = 0.1
    sig_y = 0.1
    sig_gr = 0.5
    sig_gp = 0.5
    sig_gy = 0.2
    dt = 0.1
    xd = 1000000
    vd = 0.5
    yd = 1000000
    xd_obs = 1000000
    vd_obs = 0.5
    yd_obs = 1000000
    sigma_x=1000000
    sigma_y=1000000
    Pkp_fusion = [[sig_r, 0, 0],
                  [0, sig_p, 0],
                  [0, 0, sig_y]]
    Rk = np.array([[xd_obs * xd_obs, 0, 0, 0, 0, 0],
                  [0, yd_obs * yd_obs, 0, 0, 0, 0],
                  [0, 0, yd_obs * yd_obs, 0, 0, 0],
                  [0, 0, 0, vd_obs * vd_obs, 0, 0],
                  [0, 0, 0, 0, vd_obs * vd_obs, 0],
                  [0, 0, 0, 0, 0, vd_obs * vd_obs]])
    Pkp = np.array([[xd * xd, 0, 0, 0, 0, 0],
                    [0, yd * yd, 0, 0, 0, 0],
                    [0, 0, yd * yd, 0, 0, 0],
                    [0, 0, 0, vd * vd, 0, 0],
                    [0, 0, 0, 0, vd * vd, 0],
                    [0, 0, 0, 0, 0, vd * vd]])
    Rf=np.array([[sig_gr,0,0],
                 [0,sig_gp,0],
                 [0,0,sig_gy]])
    return [Pkp, Pkp_fusion, Rk, Rf, sigma_x,sigma_y]

def kalman_euler(acc,mag,gyro,dt,Rf,Pkp):
    A = np.identity(3)
    H = np.identity(3)
    f_euler=fusion_euler(acc,mag,dt)
    g_euler=gyro_euler(gyro,dt)
    x_kp=np.dot(A,np.transpose([f_euler[0],f_euler[1],f_euler[2]]))
    Pkp=np.dot(A,np.dot(Pkp,np.transpose(A)))
    K=np.dot(np.dot(Pkp,np.transpose(H)),np.linalg.inv(np.dot(H,np.dot(Pkp,np.transpose(H)))+Rf))
    delta_x=np.dot(K,(np.transpose([g_euler[0]*dt,g_euler[1]*dt,g_euler[2]*dt])-np.dot(H,np.transpose([f_euler[0],f_euler[1],f_euler[2]]))))
    x_cor = x_kp + delta_x
    Pkp = np.dot((np.identity(3) - np.dot(H, Pkp)), Pkp)
    return [x_cor,Pkp]

def kalman_filter(x,y,h,v,x_1,y_1,h_1,v_1,f,acc,dt,sig_x,sig_y,R,Pkp):
    A = np.array([[1, 0, 0, dt, 0, 0],
                  [0, 1, 0, 0, dt, 0],
                  [0, 0, 1, 0, 0, dt],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]])
    B = np.array([[0.5 * (dt * dt), 0, 0],
                  [0, 0.5 * (dt * dt), 0],
                  [0, 0, 0.5 * (dt * dt)],
                  [dt, 0, 0],
                  [0, dt, 0],
                  [0, 0, dt]])
    H = np.identity(6)
    C = np.identity(6)
    I = np.identity(6)
    Qk = np.zeros((6, 6))
    Qk[0][0] = sig_x ** 2
    Qk[1][1] = sig_y ** 2
    Wk = 0
    Zk = 0
    obs=np.reshape(np.concatenate(([x,y,h],v)),[6,1])
    obs_1=np.reshape(np.concatenate(([x_1,y_1,h_1],v_1)),[6,1])
    x_curr=np.reshape(np.concatenate(([x,y,h],v)),[6,1])
    cbn=dcm(f[0],f[1],f[2])
    xkp=np.dot(A,obs)+np.dot(B,np.dot(cbn,np.reshape([acc[0],acc[1],acc[2]],[3,1])))+Wk
    Pkp = np.dot(np.dot(A, Pkp), np.transpose(A)) + Qk
    K = np.dot(np.dot(Pkp, (np.transpose(H))), np.linalg.inv(np.dot(np.dot(H, Pkp), np.transpose(H)) + R))
    yk = np.dot(C, obs_1) + Zk
    x_curr= (xkp) + np.dot(K, (yk - (np.dot(H,xkp))))
    Pkp = np.dot((I - np.dot(K, H)), Pkp)
    return [xkp,x_curr,Pkp]