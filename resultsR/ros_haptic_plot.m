function [ ] = ros_haptic_plot( )
%% Load 
clc
clear all
close all

disp('Loading ...')

Pos = load('Test_2_1/dirdyn_q.res');
Vit = load('Test_2_1/dirdyn_qd.res');
Torque = load('Test_2/dirdyn_Qc.res'); 

Volant_q = Pos(:,51);
Volant_qd = Vit(:,51);
TorqueFeedback = Torque(:,51);
Time = Pos(:,1);
Time_2 = Torque(:,1);

Ros_qq = load('Test_2_1/qq.txt');
q_ros = Ros_qq(:,2);
qq_ros = Ros_qq(:,3);
Time_ros_qq = Ros_qq(:,1);

Ros_t = load('Test_2/torque.txt');
T_ros = Ros_t(:,2);
Time_ros_T = Ros_t(:,1);

disp('Loading done !')

%% Plot qq
figure

plot((Time_ros_qq-Time_ros_qq(1))/(10^9),qq_ros)
hold on
plot(Time,Volant_qd)
legend('ros','rob')


%% Plot Torque
figure

plot((Time_ros_T-Time_ros_T(1))/(10^9),-T_ros)
hold on
plot(Time_2,TorqueFeedback)
end

