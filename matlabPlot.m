
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

cd '~/bagfiles2019'
% bag = rosbag('./0725_exp01/exp02.bag');
figName_att = './0808/0908-exp-att.fig';
bag = rosbag('./0808/0908-exp.bag');


% check available topics
bag.AvailableTopics


% gain
%{
    %%%%%%%%%%%%%% 1.
    - title: exp01-1728-dobtuning.bag, exp01-1803-dobtuning.bag

    - rate_controller - dob
        mass: 2.0 max_thrust: 90.0
        k_r_x: 3.0 k_r_y: 3.0 k_r_z: 5.0
        k_v_x: 4.0 k_v_y: 4.0 k_v_z: 3.0
        k_R_x: 6.0 k_R_y: 6.0 k_R_z: 3.0
        a0: 1.0 a1: 2.0 eps: 0.1

    - pixhawk controller
        MC_ROLLRATE_P:  0.180     MC_ROLLRATE_I:   DEFAULT    MC_ROLLRATE_D:   0.0040
        MC_PITCHRATE_P: 0.180    MC_PITCHRATE_I:   DEFAULT   MC_PITCHRATE_D:   0.0040
        MC_YAWRATE_P:   DEFAULT    MC_YAWRATE_I:   DEFAULT     MC_YAWRATE_D:  DEFAULT

    %%%%%%%%%%%%%% 2.
    - title: exp01-1742-dobtuning.bag

    - rate_controller - dob
        mass: 2.0 max_thrust: 90.0
        k_r_x: 3.0 k_r_y: 3.0 k_r_z: 5.0
        k_v_x: 4.0 k_v_y: 4.0 k_v_z: 3.0
        k_R_x: 6.0 k_R_y: 6.0 k_R_z: 3.0
        a0: 1.0 a1: 2.0 eps: 0.1

    - pixhawk controller
        MC_ROLLRATE_P:  0.180     MC_ROLLRATE_I:   DEFAULT    MC_ROLLRATE_D:   0.0040
        MC_PITCHRATE_P: 0.180    MC_PITCHRATE_I:   DEFAULT   MC_PITCHRATE_D:   0.0040
        MC_YAWRATE_P:   DEFAULT    MC_YAWRATE_I:   DEFAULT     MC_YAWRATE_D:     0.01

    %%%%%%%%%%%%%% 3.
    - title: exp01-1817-dobtuning.bag

    - rate_controller - dob
        mass: 2.0 max_thrust: 90.0
        k_r_x: 3.0 k_r_y: 3.0 k_r_z: 5.0
        k_v_x: 4.0 k_v_y: 4.0 k_v_z: 3.0
        k_R_x: 6.0 k_R_y: 6.0 k_R_z: 3.0
        a0: 1.0 a1: 2.0 eps: 0.1

    - pixhawk controller
        MC_ROLLRATE_P:  0.120     MC_ROLLRATE_I:   DEFAULT    MC_ROLLRATE_D:   0.0040
        MC_PITCHRATE_P: 0.120    MC_PITCHRATE_I:   DEFAULT   MC_PITCHRATE_D:   0.0040
        MC_YAWRATE_P:   DEFAULT    MC_YAWRATE_I:   DEFAULT     MC_YAWRATE_D:  DEFAULT

    %%%%%%%%%%%%%% 4.
    - title: exp01-1822-dobtuning.bag

    - rate_controller - dob
        mass: 2.0 max_thrust: 90.0
        k_r_x: 3.0 k_r_y: 3.0 k_r_z: 5.0
        k_v_x: 4.0 k_v_y: 4.0 k_v_z: 3.0
        k_R_x: 6.0 k_R_y: 6.0 k_R_z: 3.0
        a0: 1.0 a1: 2.0 eps: 0.1

    - pixhawk controller
        MC_ROLLRATE_P:  0.180     MC_ROLLRATE_I:   DEFAULT    MC_ROLLRATE_D:   0.0060
        MC_PITCHRATE_P: 0.180    MC_PITCHRATE_I:   DEFAULT   MC_PITCHRATE_D:   0.0060
        MC_YAWRATE_P:   DEFAULT    MC_YAWRATE_I:   DEFAULT     MC_YAWRATE_D:  DEFAULT
%}

%% topics to be used

bag_pos_sp = select(bag,'Topic','/commander/setpoint_raw/position');
bag_att_sp = select(bag,'Topic','/mavros/setpoint_raw/attitude');

bag_pose = select(bag,'Topic','/mavros/local_position/pose');
bag_odom = select(bag,'Topic','/mavros/local_position/odom');

bag_vicon = select(bag,'Topic','/mavros/vision_pose/pose');

% bag_rcout = select(bag,'Topic','mavros/rc/out');
%% timeseries

% type can be found by "rosmsg show mavros_msgs/PositionTarget"
ts_pos_sp = timeseries(bag_pos_sp,'Position.X','Position.Y','Position.Z');
ts_att_sp = timeseries(bag_att_sp,'Orientation.X','Orientation.Y','Orientation.Z','Orientation.W');
ts_rotvel_sp = timeseries(bag_att_sp,'BodyRate.X','BodyRate.Y','BodyRate.Z');

ts_pose_pos = timeseries(bag_pose,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
ts_pose_att = timeseries(bag_pose,'Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z','Pose.Orientation.W');

ts_odom_linvel = timeseries(bag_odom,'Twist.Twist.Linear.X','Twist.Twist.Linear.Y','Twist.Twist.Linear.Z');
ts_odom_rotvel = timeseries(bag_odom,'Twist.Twist.Angular.X','Twist.Twist.Angular.Y','Twist.Twist.Angular.Z');

ts_vicon_pos = timeseries(bag_vicon,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
ts_vicon_att = timeseries(bag_vicon,'Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z','Pose.Orientation.W');

%% RCOut

% rcout_msg = readMessages(bag_rcout);
% T_rcout = [];
% RCOut = [];
% for i=1:size(rcout_msg,1)
%    T_rcout = [T_rcout;rcout_msg{i}.Header.Stamp.Sec];
%    RCOut = [RCOut;double(rcout_msg{i}.Channels(1:6,1)).'];
% end
% 
% T_rcout = T_rcout - T_rcout(1);
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
% T0 = bag_pos_sp.StartTime;
T_pos_sp = ts_pos_sp.Time - T0;
T_att_sp = ts_att_sp.Time - T0;
T_rotvel_sp = ts_rotvel_sp.Time - T0;

T_pose = ts_pose_pos.Time - T0;
T_odom = ts_odom_linvel.Time -T0;

T_vicon = ts_vicon_pos.Time - T0;

% setpoints
pos_sp = ts_pos_sp.Data;
att_sp = quatData2Eul(ts_att_sp.Data);
rotvel_sp = ts_rotvel_sp.Data;

% states
pos = ts_pose_pos.Data;
att = quatData2Eul(ts_pose_att.Data);
linvel = ts_odom_linvel.Data;
rotvel = ts_odom_rotvel.Data;

% vicon
vicon_pos = ts_vicon_pos.Data;
vicon_att = quatData2Eul(ts_vicon_att.Data);

% vicon & ekf2 data time difference
ekf2_Tdiff = zeros(length(T_pose)-1,1);
for i=1:length(ekf2_Tdiff)
   ekf2_Tdiff(i,1) = T_pose(i+1,1)-T_pose(i,1); 
end
vicon_Tdiff = zeros(length(T_vicon)-1,1);
for i=1:length(vicon_Tdiff)
   vicon_Tdiff(i,1) = T_vicon(i+1,1)-T_vicon(i,1); 
end
 
%% main figure

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


%%
figure
subplot(3,1,1)
plot(T_odom,linvel(:,1),'.');
subplot(3,1,2)
plot(T_odom,linvel(:,2),'.');
subplot(3,1,3)
plot(T_odom,linvel(:,3),'.');
% legend('V_x','V_y','V_z')
% title('V_x, V_y, V_z')
%% position each axis
figure
sgtitle('P');
% state & setpoint
subplot(3,1,1)
title('X')
% position setpoint & position
plot(T_pos_sp, pos_sp(:,1),'.');
hold on;
plot(T_pose, pos(:,1),'k.');
legend('X_{sp}','X')
axis tight
subplot(3,1,2)
title('Y')
% position setpoint & position
plot(T_pos_sp, pos_sp(:,2),'.');
hold on;
plot(T_pose, pos(:,2),'k.');
legend('Y_{sp}','Y')
axis tight
subplot(3,1,3)
title('Z')
% position setpoint & position
plot(T_pos_sp, pos_sp(:,3),'.');
hold on;
plot(T_pose, pos(:,3),'k.');
legend('Z_{sp}','Z')
axis tight

%% atttidue each axis
% vicon log file - what happens 
% rosbag file run in NUC -> move with scp 


figure
sgtitle('\Phi, \omega');
% attitude setpoint & attitude
subplot(3,2,1)
hold on;
h = plot(T_att_sp, att_sp(:,1),'.');
plot(T_pose, att(:,1),'k.');
legend('sp','\phi');
title('\phi')
axis tight
subplot(3,2,3)
hold on;
plot(T_att_sp, att_sp(:,2),'.');
plot(T_pose, att(:,2),'k.');
title('\theta')
legend('sp','\theta');
axis tight
subplot(3,2,5)
hold on;
plot(T_att_sp, att_sp(:,3),'.');
plot(T_pose, att(:,3),'k.');
legend('sp','\psi');
title('\psi')
axis tight

% figure
% rotational velocity setpoint & rotational velocity
subplot(3,2,2)
hold on;
plot(T_rotvel_sp, rotvel_sp(:,1),'.');
plot(T_odom, rotvel(:,1),'k.');
legend('sp','\omega_x');
title('\omega_x')
axis tight
subplot(3,2,4)
hold on;
plot(T_rotvel_sp, rotvel_sp(:,2),'.');
plot(T_odom, rotvel(:,2),'k.');
title('\omega_y')
legend('sp','\omega_y');
axis tight
subplot(3,2,6)
hold on;
plot(T_rotvel_sp, rotvel_sp(:,3),'.');
plot(T_odom, rotvel(:,3),'k.');
title('\omega_z')
legend('sp','\omega_z');
axis tight

savefig(figName_att);
%% Vicon vs. ekf2 fusion
% /mavros/local_position/pose vs. /mavros/vison_pose/pose
% vicon + imu vs. viconfigure
figure
% position
% sgtitle('position');
% subplot(3,1,1)
subplot(3,2,1)
hold on;
plot(T_vicon,vicon_pos(:,1),'.');
plot(T_pose,pos(:,1),'k.');
legend('vicon','ekf')
title('X_{ekf} vs. X_{vicon}')
axis tight
% subplot(3,1,2)
subplot(3,2,3)
hold on;
plot(T_vicon,vicon_pos(:,2),'.');
plot(T_pose,pos(:,2),'k.');
legend('vicon','ekf')
title('Y_{ekf} vs. Y_{vicon}')
axis tight
% subplot(3,1,3)
subplot(3,2,5)
hold on;
plot(T_vicon,vicon_pos(:,3),'.');
plot(T_pose,pos(:,3),'k.');
legend('vicon','ekf')
title('Z_{ekf} vs. Z_{vicon}')
axis tight
% figure
% attitude
% sgtitle('attitude');
% subplot(3,1,1)
subplot(3,2,2)
hold on;
plot(T_vicon,vicon_att(:,1),'.');
plot(T_pose,att(:,1),'k.');
legend('vicon','ekf')
title('\phi_{ekf} vs. \phi_{vicon}')
axis tight
% subplot(3,1,2)
subplot(3,2,4)
hold on;
plot(T_vicon,vicon_att(:,2),'.');
plot(T_pose,att(:,2),'k.');
legend('vicon','ekf')
title('\theta_{ekf} vs. \theta_{vicon}')
axis tight
% subplot(3,1,3)
subplot(3,2,6)
hold on;
plot(T_vicon,vicon_att(:,3),'.');
plot(T_pose,att(:,3),'k.');
legend('vicon','ekf')
title('\psi_{ekf} vs. \psi_{vicon}')
axis tight

figure
subplot(2,1,1)
plot(T_vicon(1:end-1,1),vicon_Tdiff,'.');
axis tight
title('vicon time difference') 
xlabel('time (s)');
ylabel('\DeltaT');
subplot(2,1,2)
plot(T_pose(1:end-1,1),ekf2_Tdiff,'k.');
axis tight
title('ekf2 time difference') 
xlabel('time (s)');
ylabel('\DeltaT');

%% RCOut figure

% figure
% subplot(4,1,1)
% hold on;
% plot(T_rcout,RCOut(:,3),'Linewidth', 2);
% plot(T_rcout,RCOut(:,5),'Linewidth', 2);
% plot(T_rcout,RCOut(:,4),'--','Linewidth', 2);
% plot(T_rcout,RCOut(:,6),'--','Linewidth', 2);
% title('Forward, Backward');
% legend('3-F','5-F','4-B','6-B')
% subplot(4,1,2)
% hold on;
% plot(T_rcout,RCOut(:,2),'Linewidth', 2);
% plot(T_rcout,RCOut(:,3),'Linewidth', 2);
% plot(T_rcout,RCOut(:,6),'Linewidth', 2);
% plot(T_rcout,RCOut(:,1),'--','Linewidth', 2);
% plot(T_rcout,RCOut(:,4),'--','Linewidth', 2);
% plot(T_rcout,RCOut(:,5),'--','Linewidth', 2);
% title('Left, Right');
% legend('2-L','3-L','6-L','1-R','4-R','5-R')
% subplot(4,1,3)
% plot(T_rcout,RCOut,'LineWidth',2);
% title('All');
% legend('1','2','3','4','5','6');
% subplot(4,1,4)
% plot(T_pos_sp, pos_sp,'--','Linewidth', 2);
% legend('X_{sp}','Y_{sp}','Z_{sp}')
% title('setpoint');

%% functions
function att = quatData2Eul(data)
    % quaternion data from ROS package to ZYX euler angle
    data = [data(:,4) data(:,1:3)];
    att = 180/pi*quat2eul(data,'ZYX');      % [psi theta phi]
    att = [att(:,3) att(:,2) att(:,1)];     % [phi theta psi]
end