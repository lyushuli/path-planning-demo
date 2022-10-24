clear; clc;

%% 初始化
% 引力尺度因子
xi = 1;
% 斥力尺度因子
eta = 10;
% 障碍物影响半径
rho0 = 4;
% 步长
robot_step = 0.01;
% 最大迭代次数
count_max = 3000;
% 初始位置
start = [1, 5]; target = [9, 5];
% start = [1, 1]; target = [9, 9];
% start = [9, 1]; target = [1, 9];
obs = [
        3, 4, 0.6;
        6, 7, 0.8;
        7.5, 2, 0.4;
        ];
hold on;
axis([0, 10, 0, 10]);


[result, count] = APF([0, 10, 0, 10], start, target, obs, robot_step, count_max, xi, eta, rho0);



cla;
for i = 1: size(obs, 1)
    rectangle('Position', [obs(i,1)-obs(i,3), obs(i,2)-obs(i,3), obs(i,3) * 2, obs(i,3) * 2], 'Curvature', [1 1]);
end
plot(start(1), start(2), '.', 'MarkerSize', 15, 'Color', 'green');
plot(target(1), target(2), '.', 'MarkerSize', 15, 'Color', 'green');
for i = 1: size(result, 1)
    plot(result(i, 1), result(i, 2), '.', 'MarkerSize', 5, 'Color', 'red');
end
hold off;
pause(0);



