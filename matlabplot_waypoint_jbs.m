
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
bag = rosbag('./0731_waypoint/waypoint01.bag');

% check available topics
bag.AvailableTopics

%% topics to be used

bag_ctr_pose = select(bag,'Topic','/traj_gen_ui/control_pose');
% bag_traj = select(bag,'Topic','/traj_gen_ui/trajectory');

% bag_pose = select(bag,'Topic','/mavros/local_position/pose');
% bag_odom = select(bag,'Topic','/mavros/local_position/odom');

% bag_vicon = select(bag,'Topic','/mavros/vision_pose/pose');

%% timeseries

% type can be found by "rosmsg show mavros_msgs/PositionTarget"
ts_ctr_pose_pos = timeseries(bag_ctr_pose,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
ts_ctr_pose_att = timeseries(bag_ctr_pose,'Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z','Pose.Orientation.W');

% ts_traj_pos = timeseries(bag_traj,'Poses.Pose.Position.X','Poses.Pose.Position.Y','Poses.Pose.Position.Z');
% ts_traj_att = timeseries(bag_traj,'Poses.Pose.Orientation.X','Poses.Pose.Orientation.Y','Poses.Pose.Orientation.Z','Poses.Pose.Orientation.W');

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
    
T0 = bag_ctr_pose.StartTime;

T_ctr_pose_pos = ts_ctr_pose_pos.Time - T0;
T_ctr_pose_att = ts_ctr_pose_att.Time - T0;

ctr_pose_pos = ts_ctr_pose_pos.Data;
% ctr_pose_att = ts_ctr_pose_att.Data;
% ctr_pose_att_quat = [ctr_pose_att(:,4) ctr_pose_att(:,1:3)];
% 
% ctr_pose_att = 180/pi*quat2eul(ctr_pose_att_quat,'ZYX');  % [psi theta phi]
% ctr_pose_att = [ctr_pose_att(:,3) ctr_pose_att(:,2) ctr_pose_att(:,1)];     % [phi theta psi]   

%%

figure
% state & setpoint
subplot(2,1,1)
% position setpoint & position
plot(T_ctr_pose_pos, ctr_pose_pos,'--','Linewidth', 2);
legend('X_{ctr}','Y_{ctr}','Z_{ctr}')
title('X, Y, Z')

subplot(2,1,2)
% attitude setpoint & attitude
plot(T_ctr_pose_att, ctr_pose_att,'--','Linewidth', 2);
legend('\phi_{ctr}','\theta_{ctr}','\psi_{ctr}')
title('\phi, \theta, \psi')

figure
plot3(ctr_pose_pos(:,1),ctr_pose_pos(:,2),ctr_pose_pos(:,3))
hold on;
plot3(ctr_pose_pos(1,1),ctr_pose_pos(1,2),ctr_pose_pos(1,3),'r*');
plot3(ctr_pose_pos(end,1),ctr_pose_pos(end,2),ctr_pose_pos(end,3),'b*');
legend('trajectory','start','end')
axis equal