%% 
clc; clear; close all

% franka_state = csvread('data/franka_state.csv');
% franka_state = csvread('data/03-21-2022_franka_state_stat.csv');
% franka_state = csvread('data/03-21-2022_franka_state_dyna.csv');
% franka_state = csvread('data/03-21-2022_franka_state_stat2.csv');
franka_state = csvread('data/03-21-2022_franka_state_dyna2.csv');

Fz = franka_state(:,end);

figure('Position',[1920/4,1080/4,800,400])
plot((1:numel(Fz))/30, Fz)
hold on
plot((1:numel(Fz))/30,3.5*ones(numel(Fz),1),'r','LineWidth',1.5)
legend('measured','desired');
axis tight; 
xlabel('time [sec]'); ylabel('z force [N]')

mean(Fz)
std(Fz)
