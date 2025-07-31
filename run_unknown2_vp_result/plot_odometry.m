% plot_odometry_vs_gps.m
clear; clc; close all;

% 1. 加载数据
S = load('data/VictoriaPark.mat', 'gps', 'timeGps', 'ut', 'timeUt');
if ~isfield(S, 'gps') || ~isfield(S, 'ut')
    error('缺少必要变量，请确认 VictoriaPark.mat 包含 gps 和 ut');
end
gps     = S.gps;       % 2 x M
timeGps = S.timeGps(:)';  % 1 x M
ut      = S.ut;        % 3 x T
timeUt  = S.timeUt(:)';  % 1 x T

% 2. 定义 wrapToPi（角度归一化）
wrapToPi = @(ang) mod(ang + pi, 2*pi) - pi;

% 3. 初始位姿：用 GPS 第一个位置 + 前两点估算初始朝向
if size(gps,2) >= 2
    dx = gps(1,2) - gps(1,1);
    dy = gps(2,2) - gps(2,1);
    theta0 = atan2(dy, dx);
else
    theta0 = 0;
end

% 4. dead-reckoning 积分（运动模型 same as try_data）
dt = 0.025;  % 控制量缩放
N = size(ut,2);
x = zeros(3, N);
x(:,1) = [gps(1,1); gps(2,1); theta0];  % 起点设置为 GPS 起点

for i = 2:N
    u = ut(:,i) * dt;  % 3x1
    prev = x(:, i-1);
    theta = prev(3);
    new_x = prev(1) + u(1) * cos(theta + u(2));
    new_y = prev(2) + u(1) * sin(theta + u(2));
    new_theta = wrapToPi(theta + u(2) + u(3));
    x(:, i) = [new_x; new_y; new_theta];
end

% 5. 绘图：里程计轨迹 vs GPS
figure('Name','Odometry vs GPS','NumberTitle','off');
plot(x(1,:), x(2,:), '-','LineWidth',1, 'Color',[0.85 0.2 0.2]); hold on; %  接近红色的线
plot(gps(1,:), gps(2,:), '.','Color','b', 'MarkerSize',2);       % 蓝色点
axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
title('Odometry dead-reckoning vs GPS'); 
legend('Odometry','GPS True','Location','best');
