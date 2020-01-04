%% CHECK_vicon.m
clc;
clear;

cd '~/bagfiles2019'
bag = rosbag('./0801/1105-CHECKvicon.bag');

% check available topics
bag.AvailableTopics

bag_pose = select(bag,'Topic','/mavros/local_position/pose');
bag_vicon = select(bag,'Topic','/mavros/vision_pose/pose');

ts_pose_pos = timeseries(bag_pose,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
ts_pose_att = timeseries(bag_pose,'Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z','Pose.Orientation.W');
ts_vicon_pos = timeseries(bag_vicon,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
ts_vicon_att = timeseries(bag_vicon,'Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z','Pose.Orientation.W');

%% data post-processing

T0 = bag_pose.StartTime;
T_pose = ts_pose_pos.Time - T0;
T_vicon = ts_vicon_pos.Time - T0;

pos = ts_pose_pos.Data;
att = quatData2Eul(ts_pose_att.Data);

vicon_pos = ts_vicon_pos.Data;  
vicon_att = quatData2Eul(ts_vicon_att.Data);
vicon_Tdiff = zeros(length(T_vicon)-1,1);
for i=1:length(vicon_Tdiff)
   vicon_Tdiff(i,1) = T_vicon(i+1,1)-T_vicon(i,1); 
end
  
%% figure

% /mavros/local_position/pose vs. /mavros/vison_pose/pose
% vicon + imu vs. viconfigure
figure
% position
sgtitle('position & attitude');
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
plot(T_vicon(1:end-1,1),vicon_Tdiff,'k.','MarkerSize',10);
axis tight
title('vicon time difference') 
xlabel('time (s)');
ylabel('\DeltaT');

%% 
for i=1:length(vicon_Tdiff)
   if (vicon_Tdiff(i) - 2) > 0
       disp(i)
   end
end
%% functions
function att = quatData2Eul(data)
    % quaternion data from ROS package to ZYX euler angle
    data = [data(:,4) data(:,1:3)];
    att = 180/pi*quat2eul(data,'ZYX');      % [psi theta phi]
    att = [att(:,3) att(:,2) att(:,1)];     % [phi theta psi]
end