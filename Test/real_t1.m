clear all;close all;clc;
obs=[4000,5000,280,245;4000,5000,282,250;4000,5000,285,252;4000,5000,286,254;4000,5000,290,259];%[x  y vx vy]
ax=[2;2];
Qk=0;
Wk=0;
Zk=0;
%new predicted state
t=1;%delta t
A=[1,0,t,0;0,1,0,t;0,0,1,0;0,0,0,1];
B=[0.5*(t^2),0;0,0.5*(t^2);t,0;t,0];
i=1;
xkp(i,:)=A*(obs(i,:)')+B*ax+Wk;

%initial process covariance
xd=20;%delta x
vd=5;%delta v
yd=20;
%P_ik=[xd^2,xd*vd;xd*vd,vd^2];

P_ik=[xd^2,0,0,0;0,yd^2,0,0;0,0,vd^2,0;0,0,0,vd^2];
%in our case cross diagonal=0

%Predicted process covariance matrix
Pkp=A*P_ik*(A')+Qk;
%in our case cross diagonal=0
Pkp(1,2)=0;Pkp(3,1)=0;Pkp(2,4)=0;
Pkp(2,1)=0;Pkp(1,3)=0;Pkp(4,2)=0;

%Kalman Gain
H=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];%transformation matrix
xd_obs=25;%observation errors
vd_obs=6;
yd_obs=25;
R=[xd_obs^2,0,0,0;0,yd_obs^2,0,0;0,0,vd_obs^2,0;0,0,0,vd_obs^2];
K=(Pkp*(H'))/((H*Pkp*(H'))+R);

%New observation
C=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
yk=C*(obs(i+1,:)')+Zk;

%Calculating the current state
l=00
x_curr(i,:)=xkp(i,:)' +(K*(yk-(H*(xkp(i,:)'))));

%Updating covariance matrix
I=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
Pk=(I-K*H)*Pkp;

for i=2:4
    xkp(i,:)=A*(obs(i,:)')+B*ax+Wk;
    P_ik=[xd^2,0,0,0;0,yd^2,0,0;0,0,vd^2,0;0,0,0,vd^2];
    %in our case cross diagonal=0

    %Predicted process covariance matrix
    Pkp=A*P_ik*(A')+Qk
    %in our case cross diagonal=0
    Pkp(1,2)=0;Pkp(3,1)=0;Pkp(2,4)=0;
    Pkp(2,1)=0;Pkp(1,3)=0;Pkp(4,2)=0;

    %Kalman Gain
    H=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];%transformation matrix
    xd_obs=25;%observation errors
    vd_obs=6;
    yd_obs=25;
    R=[xd_obs^2,0,0,0;0,yd_obs^2,0,0;0,0,vd_obs^2,0;0,0,0,vd_obs^2];
    K=(Pkp*(H'))/((H*Pkp*(H'))+R)

    %New observation
    C=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
    yk=C*(obs(i+1,:)')+Zk;

    %Calculating the current state
    x_curr(i,:)=xkp(i,:)' +(K*(yk-(H*(xkp(i,:)'))));

    %Updating covariance matrix
    I=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
    Pk=(I-K*H)*Pkp;
end
x_curr=[obs(1,:);x_curr]
xkp=[obs(1,:);xkp]
%x_pos plot
T=1:5;
subplot(2,2,1)
plot(T,x_curr(:,1),'-*');
hold on
plot(T,obs(:,1),'-o');
hold on
plot(T,xkp(:,1),'-s');
title('x position plot')
xlabel('Time')
ylabel('Position')
legend('Kalman filter','Measured','Predicted')

%v plot
subplot(2,2,2)
plot(T,x_curr(:,3),'-*');
hold on
plot(T,obs(:,3),'-o');
hold on
plot(T,xkp(:,3),'-s');
title('x velocity plot')
xlabel('Time')
ylabel('Velocity')
legend('Kalman filter','Measured','Predicted')

%y_pos plot
subplot(2,2,3)
T=1:5;
plot(T,x_curr(:,2),'-*');
hold on
plot(T,obs(:,2),'-o');
hold on
plot(T,xkp(:,2),'-s');
title('y position plot')
xlabel('Time')
ylabel('Position')
legend('Kalman filter','Measured','Predicted')

%v plot
T=1:5;
subplot(2,2,4)
plot(T,x_curr(:,4),'-*');
hold on
plot(T,obs(:,4),'-o');
hold on
plot(T,xkp(:,4),'-s');
title('y velocity plot')
xlabel('Time')
ylabel('Velocity')
legend('Kalman filter','Measured','Predicted')

figure
plot(x_curr(:,1),x_curr(:,2),'-*');
hold on
plot(obs(:,1),obs(:,2),'-o');
hold on
plot(xkp(:,1),xkp(:,2),'-s');
title('Object position plot')
xlabel('x co-ordinate')
ylabel('y co-ordinate')
legend('Kalman filter(Predicted)','Measured','Current')