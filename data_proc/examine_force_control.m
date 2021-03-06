%% force control experiment with active-sensing-ee
clc; clear; close all

file_dir = './data/franka_state/';
% franka_state = csvread([dir,'03-21-2022_franka_state_stat.csv']);
% franka_state = csvread([dir,'03-21-2022_franka_state_dyna.csv']);
% franka_state = csvread([dir,'03-21-2022_franka_state_stat2.csv']);
% franka_state = csvread([dir,'03-21-2022_franka_state_dyna2.csv']);
franka_state = csvread([file_dir,'03-31-2022_franka_state{force_test}.csv']);

%% plot force vs. time
Fz = franka_state(:,end);
Fz_desired = 3.5;
fps = 30; ts = 106; tf = numel(Fz);
figure('Position',[1920/4,1080/4,900,300])
plot((ts:tf)/fps, Fz(ts:tf), 'k', 'LineWidth', 1)
hold on
plot((ts:tf)/fps,Fz_desired*ones(tf-ts+1,1),'--r','LineWidth',2.0)
% legend('measured','desired','Location','southeast');
axis tight; 
xlabel('time [sec]'); ylabel('z force [N]'); ylim([0 5])
fprintf('mean force: %f [N], std: %f\n', mean(Fz), std(Fz));
ax = gca(); ax.LineWidth = 1.5; ax.YGrid = 'on';

%% plot force vs. robot pose
ts = 150;                   % start time
tf = length(franka_state);  % end time
pose = zeros(4,4,length(franka_state));
for i = 1:length(franka_state)
    pose(:,:,i) = reshape(franka_state(i,1:16)',4,4)';
end

x = squeeze(pose(1,end,ts:tf))*1e3;     % [mm] 
y = squeeze(pose(2,end,ts:tf))*1e3;     % [mm]
z = squeeze(pose(3,end,ts:tf))*1e3;     % [mm]
cmap = Fz_desired - Fz(ts:tf);

figure('Position',[1920/3, 1080/3, 800, 400])
scatter3(x,y,z,10,-cmap,'filled','LineWidth',2);
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
axis equal off;
% grid off
ax = gca; ax.LineWidth = 1.5;
colormap(ax, 'jet'); 
cb = colorbar(); cb.Label.String = 'force error [N]';
% plot approach vector
isShowVec = false;
if isShowVec
    hold on
    interval = 12;
    len = 8;
    for i = 1:interval:tf-ts
        vec_z = -len*pose(1:3,3,i);
        line([x(i), x(i)+vec_z(1)],[y(i), y(i)+vec_z(2)],[z(i), z(i)+vec_z(3)], ...
            'Color','red','LineWidth',1);
    end
end
% view(90,0)      % Y-Z
% view(0, 90)     % X-Y
view(60,30)     % isotropic

