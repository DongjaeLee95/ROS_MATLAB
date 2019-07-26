%% Custom ROS package - mavros_msg 

%% copying mavros_msgs to the userFolder
% path: /opt/ros/kinetic/share/mavros_msgs
clc;
clear;

package = '/opt/ros/kinetic/share/mavros_msgs';

userFolder_gen1 = '~/MATLAB_ROS_custom_msgs';

userFolder_gen2 = '~/MATLAB_ROS_custom_msgs/mavros_msgs';

% copyfile(package,userFolder_gen2);


%%
folderpath = userFolder_gen1;
rosgenmsg(folderpath);


