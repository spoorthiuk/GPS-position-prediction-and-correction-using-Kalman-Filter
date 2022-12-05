##############LIBRARIES##########################
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
     lat = 180 / m.pi * (2 * m.atan(m.exp(lat * m.pi / 180.0)) - m.pi / 2.0)
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
def kalman_euler(acc,mag,gyro,dt,g_d,Pkp,i):
    A = np.identity(3)
    H = np.identity(3)
    R = [[g_d[0], 0, 0],
         [0, g_d[1], 0],
         [0, 0, g_d[2]]]
    f_euler=fusion_euler(acc,mag,dt)
    g_euler=gyro_euler(gyro,dt)
    x_kp=np.dot(A,np.transpose([f_euler[0],f_euler[1],f_euler[2]]))
    Pkp=np.dot(A,np.dot(Pkp,np.transpose(A)))
    K=np.dot(np.dot(Pkp,np.transpose(H)),np.linalg.inv(np.dot(H,np.dot(Pkp,np.transpose(H)))+R))
    delta_x=np.dot(K,(np.transpose([g_euler[0]*dt,g_euler[1]*dt,g_euler[2]*dt])-np.dot(H,np.transpose([f_euler[0],f_euler[1],f_euler[2]]))))
    x_cor = np.transpose([f_euler[0],f_euler[1],f_euler[2]]) + delta_x
    Pkp = np.dot((np.identity(3) - np.dot(H, Pkp)), Pkp)
    return [x_cor,Pkp]

def kalman_filter(i,x,y,h,v,x_1,y_1,h_1,v_1,f,acc,dt,sig_x,sig_y,R,Pkp):
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
    if(i==355):
        print('Qk',Qk)
        print('xkp',xkp)
        print('Pkp',Pkp)
        print('K',K)
        print('x_curr',x_curr)
    Pkp = np.dot((I - np.dot(K, H)), Pkp)
    if(i==355):
        print(Pkp)
    return [xkp,x_curr,Pkp]

#################################################
##################MAIN###########################
Sensors=pd.read_csv(r'RawSensors.csv')
GPS=pd.read_csv(r'RawGNSS.csv')
prev=pd.read_csv(r'predicted_to_gpx.csv')

[ax,ay,az]=[Sensors['Accelerometer X'],Sensors['Accelerometer Y'],Sensors['Accelerometer Z']]
[mx,my,mz]=[Sensors['Magnetometer X'],Sensors['Magnetometer Y'],Sensors['Magnetometer Z']]
[gx,gy,gz]=[Sensors['Gyroscope X'],Sensors['Gyroscope Y'],Sensors['Gyroscope Z']]
[vx,vy,vz]=[GPS['Velocity North'],GPS['Velocity East'],GPS['Velocity Down']]

lon=GPS['Longitude']
lat=GPS['Latitude']
h=GPS['Height']

sig_lat=GPS['Latitude Standard Deviation']
sig_lon=GPS['Longitude Standard Deviation']

lat_p=prev['latitude']
lon_p=prev['longitude']

x=[]
y=[]
for i in range(0,len(lon)):
     a=latlontoPixels(lat[i],lon[i],15)
     x.append(a[0])
     y.append(a[1])

sigma_x=[]
sigma_y=[]
for i in range(0,len(lon)):
     a=latlontoPixels(sig_lat[i],sig_lon[i],15)
     sigma_x.append(a[0])
     sigma_y.append(a[1])

sig_r = 0.001
sig_p = 0.001
sig_y = 0.1
sig_gr = 0.005
sig_gp = 0.005
sig_gy = 0.2
dt=0.1
fusion=[]
xd=100
vd=0.5
yd=100
xd_obs=100
vd_obs=0.5
yd_obs=100
Pkp_fusion=[[sig_r,0,0],
    [0,sig_p,0],
    [0,0,sig_y]]
R=np.array([[xd_obs*xd_obs,0,0,0,0,0],
            [0,yd_obs*yd_obs,0,0,0,0],
            [0,0,yd_obs*yd_obs,0,0,0],
            [0,0,0,vd_obs*vd_obs,0,0],
            [0,0,0,0,vd_obs*vd_obs,0],
            [0,0,0,0,0,vd_obs*vd_obs]])
Pkp=np.array([[xd*xd,0,0,0,0,0],
               [0,yd*yd,0,0,0,0],
               [0,0,yd*yd,0,0,0],
               [0,0,0,vd*vd,0,0],
               [0,0,0,0,vd*vd,0],
               [0,0,0,0,0,vd*vd]])
x_kp=[]
x_current=[]
lat_pred=[]
lon_pred=[]
lat_e=[]
lon_e=[]
pd_lat=[]
pd_lon=[]
g_att=[]
f_att=[]
i=0
[x_fusion,Pkp_fusion]=kalman_euler([ax[i],ay[i],az[i]],[mx[i],my[i],mz[i]],[gx[i],gy[i],gz[i]]
                             ,dt,[sig_gr,sig_gp,sig_gy],Pkp_fusion,i)
fusion.append(x_fusion)
g_att.append(gyro_euler([gx[i],gy[i],gz[i]],dt))
f_att.append(fusion_euler([ax[i],ay[i],az[i]],[mx[i],my[i],mz[i]],dt))
    #final.csv
    #[xkp,x_curr,Pkp]=kalman_filter(x[i],y[i],h[i],[vx[i],vy[i],vz[i]],x[i+1],y[i+1],h[i+1],[vx[i+1],vy[i+1],vz[i+1]],x_fusion,[ax[i],ay[i],az[i]],dt,sigma_x[i],sigma_y[i],R,Pkp)
    #final2.csv
[xkp,x_curr,Pkp]=kalman_filter(i,x[i],y[i],h[i],np.dot([vx[i],vy[i],vz[i]],dcm(x_fusion[0],x_fusion[1],x_fusion[2])),x[i+1],y[i+1],h[i+1],np.dot([vx[i+1],vy[i+1],vz[i+1]],dcm(x_fusion[0],x_fusion[1],x_fusion[2])),x_fusion,[ax[i],ay[i],az[i]],dt,sigma_x[i],sigma_y[i],R,Pkp)
    #conclusion=identical
x_kp.append(xkp)
x_current.append(x_curr)
a = pixelstoLatLon(x_curr[0][0], x_curr[1][0], 15)
lat_pred.append(a[0])
lon_pred.append(a[1])
lat_e.append(a[0]-lat[i])
lon_e.append(a[1]-lon[i])
pd_lat.append(a[0]-lat_p[i])
pd_lon.append(a[1]-lon_p[i])

for i in range(0,len(x)-1):
    [x_fusion,Pkp_fusion]=kalman_euler([ax[i],ay[i],az[i]],[mx[i],my[i],mz[i]],[gx[i],gy[i],gz[i]]
                             ,dt,[sig_gr,sig_gp,sig_gy],Pkp_fusion,i)
    fusion.append(x_fusion)
    g_att.append(gyro_euler([gx[i],gy[i],gz[i]],dt))
    f_att.append(fusion_euler([ax[i],ay[i],az[i]],[mx[i],my[i],mz[i]],dt))
    #final.csv
    #[xkp,x_curr,Pkp]=kalman_filter(x[i],y[i],h[i],[vx[i],vy[i],vz[i]],x[i+1],y[i+1],h[i+1],[vx[i+1],vy[i+1],vz[i+1]],x_fusion,[ax[i],ay[i],az[i]],dt,sigma_x[i],sigma_y[i],R,Pkp)
    #final2.csv

    [xkp,x_curr,Pkp]=kalman_filter(i,x[i],y[i],h[i],np.dot([vx[i],vy[i],vz[i]],dcm(x_fusion[0],x_fusion[1],x_fusion[2])),x[i+1],y[i+1],h[i+1],np.dot([vx[i+1],vy[i+1],vz[i+1]],dcm(x_fusion[0],x_fusion[1],x_fusion[2])),x_fusion,[ax[i],ay[i],az[i]],dt,sigma_x[i],sigma_y[i],R,Pkp)
    #conclusion=identical
    x_kp.append(xkp)
    x_current.append(x_curr)
    a = pixelstoLatLon(x_curr[0][0], x_curr[1][0], 15)
    if(i==355):
        print(a)
    lat_pred.append(a[0])
    lon_pred.append(a[1])
    lat_e.append(a[0] - lat[i])
    lon_e.append(a[1] - lon[i])
    pd_lat.append(a[0] - lat_p[i])
    pd_lon.append(a[1] - lon_p[i])
fusion=np.transpose(fusion)
g_att=np.transpose(g_att)
f_att=np.transpose(f_att)
x_current=np.transpose(x_current)
ans=np.transpose(np.array([lat_pred,lon_pred,lat_e,lon_e,pd_lat,pd_lon,
                           fusion[0]*(180/m.pi),fusion[1]*(180/m.pi),fusion[2]*(180/m.pi),g_att[0]*(180/m.pi),g_att[1]*(180/m.pi),g_att[2]*(180/m.pi),f_att[0]*(180/m.pi),f_att[1]*(180/m.pi),f_att[2]*(180/m.pi),
                           x_current[0][3],x_current[0][4],x_current[0][5]]))
conv_pred=pd.DataFrame(ans,columns=['latitude','longitude','latitude error','longitude error','Prev Predicted Latitude','Previous Predicted Longitude',
                                    'Roll','Pitch','Heading','Gyro Roll','Gyro Pitch','Gyro Heading','Fusion Roll','Fusion Pitch','Fusion Heading',
                                    'Velocity X','Velocity Y','Velocity Z'])
conv_pred.to_csv('final.csv')
x_kp=np.transpose(x_kp)
t=GPS['Unix Time']
plt.plot(lat_pred,lon_pred)
plt.plot(lat,lon)
plt.legend(['Kalman Filter','Measured'])
plt.title('Position')
plt.figure()
plt.plot(t,fusion[0])
plt.plot(t,g_att[0])
plt.plot(t,f_att[0])
plt.legend(['Kalman Filter','Gyroscope','Fusion'])
plt.title('Roll')
plt.figure()
plt.plot(t,fusion[1])
plt.plot(t,g_att[1])
plt.plot(t,f_att[1])
plt.legend(['Kalman Filter','Gyroscope','Fusion'])
plt.title('Pitch')
plt.figure()
plt.plot(t,fusion[2])
plt.plot(t,g_att[2])
plt.plot(t,f_att[2])
plt.legend(['Kalman Filter','Gyroscope','Fusion'])
plt.title('Heading')
plt.figure()
plt.plot(t,lat_e)
plt.plot(t,lon_e)
plt.legend(['Latitude','Longitude'])
plt.title('Fusion Error')
plt.figure()
plt.plot(t,lat_p)
plt.plot(t,lon_p)
plt.legend(['Latitude','Longitude'])
plt.title('Linear Error')
plt.figure()
plt.plot(t,x_current[0][3])
plt.plot(t,vx)
plt.plot(t,(x_kp)[0][4])
plt.title('x velocity plot')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend(['Kalman filter(Predicted)','Measured','Current'])
plt.figure()
plt.plot(t,x_current[0][4])
plt.plot(t,vy)
plt.plot(t,(x_kp)[0][4])
plt.title('y velocity plot')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend(['Kalman filter(Predicted)','Measured','Current'])
plt.figure()
plt.plot(t,x_current[0][5])
plt.plot(t,vz)
plt.plot(t,(x_kp)[0][5])
plt.title('z velocity plot')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend(['Kalman filter(Predicted)','Measured','Current'])
plt.show()
