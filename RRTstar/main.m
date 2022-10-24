%% 初始化
% 最大迭代次数
count_max = 30000;
% 地图范围
mapLimit = [0, 100, 0, 100];
% 步长
growStep = 1;
% 起始点、目标点
% start = [5, 05]; target = [95, 95];
% start = [95, 05]; target = [05, 95];
% start = [10, 30]; target = [80, 80];
start = [30, 60]; target = [60, 10];


% 障碍物 x y r
obs = [30, 45, 5;
       60, 70, 6;
       80, 20, 5;
       50, 30, 7;
       20, 80, 3;
       50, 50, 5
    ];

axis(mapLimit);
hold on;

result = RRTstar(mapLimit, start, target, obs, growStep, count_max);

cla;
for i = 1: size(obs, 1)
    rectangle('Position', [obs(i,1)-obs(i,3), obs(i,2)-obs(i,3), obs(i,3) * 2, obs(i,3) * 2], 'Curvature', [1 1]);
end

for i = 2: size(result, 1)
    plot(result(i-1:i,1), result(i-1:i, 2), '-', color='blue');
end

plot(start(1), start(2), '.', markersize=30, color='red');
plot(target(1), target(2), '.', markersize=30, color='green');