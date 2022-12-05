clear all;close all;clc;
obs=[4000,280;4260,282;4550,285;4860,286;5110,290];%[x vx]
ax=2;
Qk=0;
Wk=0;
Zk=0;
%new predicted state
t=1;%delta t
A=[1,t;0,1];
B=[0.5*(t^2);t];
i=1
xkp(i,:)=A*(obs(i,:)')+B*ax+Wk

%initial process covariance
xd=20;%delta x
vd=5;%delta v

%P_ik=[xd^2,xd*vd;xd*vd,vd^2];

P_ik=[xd^2,0;0,vd^2];
%in our case cross diagonal=0

%Predicted process covariance matrix
Pkp=A*P_ik*(A')+Qk;
%in our case cross diagonal=0
Pkp(1,2)=0;
Pkp(2,1)=0;

%Kalman Gain
H=[1,0;0,1];%transformation matrix
xd_obs=25;%observation errors
vd_obs=6;
R=[xd_obs^2,0;0,vd_obs^2];
K=(Pkp*(H'))/((H*Pkp*(H'))+R);

%New observation
C=[1,0;0,1];
yk=C*(obs(i+1,:)')+Zk;

%Calculating the current state
x_curr(i,:)=xkp(i,:)' +(K*(yk-(H*(xkp(i,:)'))))

%Updating covariance matrix
I=[1,0;0,1];
Pk=(I-K*H)*Pkp;

for i=2:4
    xkp(i,:)=A*(obs(i,:)')+B*ax+Wk

    %Predicted process covariance matrix
    Pkp=A*Pk*(A')+Qk;
    %in our case cross diagonal=0
    Pkp(1,2)=0;
    Pkp(2,1)=0;

    %Kalman Gain
    H=[1,0;0,1];%transformation matrix
    xd_obs=25;%observation errors
    vd_obs=6;
    R=[xd_obs^2,0;0,vd_obs^2];
    K=(Pkp*(H'))/((H*Pkp*(H'))+R);

    %New observation
    C=[1,0;0,1];
    yk=C*(obs(i+1,:)')+Zk;

    %Calculating the current state
    x_curr(i,:)=xkp(i,:)' +(K*(yk-(H*(xkp(i,:)'))))

    %Updating covariance matrix
    I=[1,0;0,1];
    Pk=(I-K*H)*Pkp;
end
x_curr=[obs(1,:);x_curr]
xkp=[obs(1,:);xkp]
%x_pos plot
T=1:5;
plot(x_curr(:,1),T);
hold on
plot(obs(:,1),T);
hold on
plot(xkp(:,1),T);
legend('Kalman filter','Measured','Predicted')

%v plot
figure
plot(x_curr(:,2),T);
hold on
plot(obs(:,2),T);
hold on
plot(xkp(:,2),T);
legend('Kalman filter','Measured','Predicted')

obs_x=obs;
figure

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

obs=[5000,245;5205,250;5408,252;5722,254;6101,259];%[y vy]
ay=1.5;
Qk=0;
Wk=0;
Zk=0;
%new predicted state
t=1;%delta t
A=[1,t;0,1];
B=[0.5*(t^2);t];
i=1
ykp(i,:)=A*(obs(i,:)')+B*ay+Wk

%initial process covariance
yd=20;%delta x
vd=5;%delta v

%P_ik=[xd^2,xd*vd;xd*vd,vd^2];

P_ik=[yd^2,0;0,vd^2];
%in our case cross diagonal=0

%Predicted process covariance matrix
Pkp=A*P_ik*(A')+Qk;
%in our case cross diagonal=0
Pkp(1,2)=0;
Pkp(2,1)=0;

%Kalman Gain
H=[1,0;0,1];%transformation matrix
yd_obs=25;%observation errors
vd_obs=6;
R=[yd_obs^2,0;0,yd_obs^2];
K=(Pkp*(H'))/((H*Pkp*(H'))+R);

%New observation
C=[1,0;0,1];
yk=C*(obs(i+1,:)')+Zk;

%Calculating the current state
y_curr(i,:)=ykp(i,:)' +(K*(yk-(H*(ykp(i,:)'))))

%Updating covariance matrix
I=[1,0;0,1];
Pk=(I-K*H)*Pkp;

for i=2:4
    ykp(i,:)=A*(obs(i,:)')+B*ay+Wk;
    %Predicted process covariance matrix
    Pkp=A*Pk*(A')+Qk;
    %in our case cross diagonal=0
    Pkp(1,2)=0;
    Pkp(2,1)=0;

    %Kalman Gain
    H=[1,0;0,1];%transformation matrix
    K=(Pkp*(H'))/((H*Pkp*(H'))+R);

    %New observation
    C=[1,0;0,1];
    yk=C*(obs(i+1,:)')+Zk;

    %Calculating the current state
    y_curr(i,:)=ykp(i,:)' +(K*(yk-(H*(ykp(i,:)'))));
    %j=x_curr(i,:)
    %Updating covariance matrix
    I=[1,0;0,1];
    Pk=(I-K*H)*Pkp;
end
y_curr=[obs(1,:);y_curr]
ykp=[obs(1,:);ykp]
%x_pos plot
T=1:5;
plot(y_curr(:,1),T);
hold on
plot(obs(:,1),T);
hold on
plot(ykp(:,1),T);
legend('Kalman filter','Measured','Predicted')

%v plot
figure
plot(y_curr(:,2),T);
hold on
plot(obs(:,2),T);
hold on
plot(ykp(:,2),T);
legend('Kalman filter','Measured','Predicted')

figure
plot(x_curr(:,1),y_curr(:,1));
hold on
plot(obs_x(:,1),obs(:,1));
hold on
plot(xkp(:,1),ykp(:,1));
legend('Kalman filter(Predicted)','Measured','Current')

