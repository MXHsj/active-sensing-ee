%% record pose data from franka & distance data from sensor
clc; clear; close all
rosshutdown
rosinit('localhost')

franka_pos_sub = rossubscriber('franka_state_custom', 'std_msgs/Float64MultiArray');
ch101_dist_sub = rossubscriber('ch101','std_msgs/Float64MultiArray');
franka_pose = zeros(4);
ch101_dist = zeros(1,4);

%% collect data
freq = 1;
rate = rateControl(freq);
z_rec = [];     
d0_rec = [];     % port0 dist
d1_rec = [];     % port1 dist
d2_rec = [];     % port2 dist
d3_rec = [];     % port3 dist

while true
    franka_pose_msg = receive(franka_pos_sub);
    franka_pose = reshape([franka_pose_msg.Data],4,4)';
    ch101_dist_msg = receive(ch101_dist_sub);
    ch101_dist = ch101_dist_msg.Data;
    z_rec = [z_rec, franka_pose(3,end)];
    d0_rec = [d0_rec, ch101_dist(1)];
    d1_rec = [d1_rec, ch101_dist(2)];
    d2_rec = [d2_rec, ch101_dist(3)];
    d3_rec = [d3_rec, ch101_dist(4)];
    fprintf('s0: %.3f, s1: %.3f, s2: %.3f, s3: %.3f\n', ...
        ch101_dist(1), ch101_dist(2), ch101_dist(3), ch101_dist(4))
    waitfor(rate);
end

%% vis
figure('Position',[1920/3,1080/3,640,480])
plot(1:length(z_rec),z_rec.*1000,'LineWidth',1.8);
hold on
plot(1:length(d0_rec),d0_rec,'LineWidth',1.8);
plot(1:length(d1_rec),d1_rec,'LineWidth',1.8);
plot(1:length(d2_rec),d2_rec,'LineWidth',1.8);
plot(1:length(d3_rec),d3_rec,'LineWidth',1.8);
grid on
xlabel('time [sec]')
ylabel('z distance [mm]')
legend('franka z','sensor0','sensor1','sensor2','sensor3')
