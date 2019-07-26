clear;
clc;

bag = rosbag('ex_multiple_topics.bag');

bagselect1 = select(bag, 'Topic', '/odom');

start = bag.StartTime;
bagselect2 = select(bag, 'Time', [start start+30], 'Topic', '/odom');

bagselect3 = select(bagselect2, 'Time', [205 206]);
% MessageType: nav_msgs/Odometry

ts = timeseries(bagselect3, 'Pose.Pose.Position.X', 'Twist.Twist.Angular.Z');

% where does these ('Pose.Pose.Position.X', 'Twist.Twist.Angular.Z') come from?
% Ans: elements in the message type "nav_msgs/Odometry"

ts.Data

ts.mean

% figure
% plot(ts, 'Linewidth', 3)

figure
plot(ts.Time, ts.Data, 'LineWidth', 3)