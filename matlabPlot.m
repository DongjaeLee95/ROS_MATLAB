
clc;
clear;
close all;

%% NOTE
%{

1. To run this code, "ROS Custom Message Support" should be 
dounloaded beforehand. Also, mavros_msgs should be imported through "ROS 
Custom Message Support".

2. matlab function quat2eul: outputs are [yaw pitch roll]. In the following
code, the order is reversed as [roll pitch yaw].

3. ROS quaternion convention and matlab quaternion convention are different
in the way that ROS: [q1 q2 q3 q0] ,whereas MATLAB: [q0 q1 q2 q3].
Therefore, the ROS quaternion order is revised as [q0 q1 q2 q3].

%}

%% matlabPlot for experiments

cd '~/bagfiles'
bag = rosbag('./0725_exp01/exp02.bag');

% check available topics
bag.AvailableTopics

%% topics to be used

bag_pos_sp = select(bag,'Topic','/commander/setpoint_raw/position');
bag_att_sp = select(bag,'Topic','/mavros/setpoint_raw/attitude');

bag_pose = select(bag,'Topic','/mavros/local_position/pose');
bag_odom = select(bag,'Topic','/mavros/local_position/odom');

bag_vicon = select(bag,'Topic','/mavros/vision_pose/pose');

%% timeseries

% type can be found by "rosmsg show mavros_msgs/PositionTarget"
ts_pos_sp = timeseries(bag_pos_sp,'Position.X','Position.Y','Position.Z');
ts_att_sp = timeseries(bag_att_sp,'Orientation.X','Orientation.Y','Orientation.Z','Orientation.W');

ts_pose_pos = timeseries(bag_pose,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
ts_pose_att = timeseries(bag_pose,'Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z','Pose.Orientation.W');

ts_odom_linvel = timeseries(bag_odom,'Twist.Twist.Linear.X','Twist.Twist.Linear.Y','Twist.Twist.Linear.Z');
ts_odom_rotvel = timeseries(bag_odom,'Twist.Twist.Angular.X','Twist.Twist.Angular.Y','Twist.Twist.Angular.Z');

ts_vicon_pos = timeseries(bag_vicon,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
ts_vicon_att = timeseries(bag_vicon,'Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z','Pose.Orientation.W');

%% SOLVED - NEED TO SOLVE PROBLEM OF ATTITUDE fluctuating between 180 and -180
% need to watch the video taken... why the yaw angle is 180 deg?

% Ans. ROS quaternion convention is [q1 q2 q3 q0] whereas matlab quaternion
% convention is [q0 q1 q2 q3].
% --> NEED TO CHANGE THE ORDER OF QUATERNION.

%% data post-processing


% time axis
    % Since time stamps are all different among data, find one with the
    % longest time stamp which is usually ts_pos_sp.
    % Therfore, set the StartTime of bag_pos_sp as the reference start
    % time, and subtract its value from every ts.Time to get reasonable
    % time axis values.
    
T0 = bag_pose.StartTime;

T_pos_sp = ts_pos_sp.Time - T0;
T_att_sp = ts_att_sp.Time - T0;

T_pose = ts_pose_pos.Time - T0;
T_odom = ts_odom_linvel.Time -T0;

T_vicon = ts_vicon_pos.Time - T0;


pos_sp = ts_pos_sp.Data;
att_sp_quat = ts_att_sp.Data;
att_sp_quat = [att_sp_quat(:,4) att_sp_quat(:,1:3)];

pos = ts_pose_pos.Data;
att_quat = ts_pose_att.Data;
att_quat = [att_quat(:,4) att_quat(:,1:3)];

linvel = ts_odom_linvel.Data;
rotvel = ts_odom_rotvel.Data;

vicon_pos = ts_vicon_pos.Data;
vicon_att_quat = ts_vicon_att.Data;
vicon_att_quat = [vicon_att_quat(:,4) vicon_att_quat(:,1:3)];

att_sp = 180/pi*quat2eul(att_sp_quat,'ZYX');  % [psi theta phi]
att_sp = [att_sp(:,3) att_sp(:,2) att_sp(:,1)];     % [phi theta psi]   

att = 180/pi*quat2eul(att_quat,'ZYX');  % [psi theta phi]
att = [att(:,3) att(:,2) att(:,1)];     % [phi theta psi]   

vicon_att = 180/pi*quat2eul(vicon_att_quat,'ZYX');  % [psi theta phi]
vicon_att = [vicon_att(:,3) vicon_att(:,2) vicon_att(:,1)];     % [phi theta psi]   

%%

figure
% state & setpoint
subplot(2,2,1)
% position setpoint & position
plot(T_pos_sp, pos_sp,'--','Linewidth', 2);
hold on;
plot(T_pose, pos, 'Linewidth', 2);
legend('X_{sp}','Y_{sp}','Z_{sp}','X','Y','Z')
title('X, Y, Z')

subplot(2,2,2)
% attitude setpoint & attitude
plot(T_att_sp, att_sp,'--','Linewidth', 2);
hold on;
plot(T_pose, att, 'Linewidth', 2);
legend('\phi_{sp}','\theta_{sp}','\psi_{sp}','\phi','\theta','\psi')
title('\phi, \theta, \psi')

subplot(2,2,3)
% linear velocity
plot(T_odom,linvel,'LineWidth',2);
legend('V_x','V_y','V_z')
title('V_x, V_y, V_z')

subplot(2,2,4)
% rotational velocity
plot(T_odom,rotvel,'LineWidth',2);
legend('\omega_x','\omega_y','\omega_z')
title('\omega_x, \omega_y, \omega_z')


figure
% /mavros/local_position/pose vs. /mavros/vison_pose/pose
% vicon + imu vs. vicon
subplot(2,1,1)
% position
plot(T_pose,pos,'LineWidth',2);
hold on;
plot(T_vicon,vicon_pos,'--','LineWidth',2);
legend('X','Y','Z','X_v','Y_v','Z_v')
title('vicon vs. ekf2 fusion');

subplot(2,1,2)
% position
plot(T_pose,att,'LineWidth',2);
hold on;
plot(T_vicon,vicon_att,'--','LineWidth',2);
legend('\phi','\theta','\psi','\phi_v','\theta_v','\psi_v')
title('vicon vs. ekf2 fusion');

