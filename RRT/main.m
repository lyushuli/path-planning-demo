%% 初始化
% 最大迭代次数
count = 1;
count_max = 3000;
% 地图范围
minX = 0; maxX = 10;
minY = 0; maxY = 10;
% 记录所有节点，及其父节点
points = zeros(count_max, 2);
parent = zeros(count_max, 1);
% 步长
growStep = 0.1;
% 起始点、目标点
% start = [0.5, 0.5]; target = [9.5, 9.5];
% start = [9.5, 0.5]; target = [0.5, 9.5];
% start = [1.0, 3.0]; target = [8.0, 8.0];
start = [3.0, 6.0]; target = [6.0, 1.0];

points(1, :) = start; 
currentIndex = 1;
% 障碍物 x y r
obs = [3, 4, 0.5;
       6, 7, 1;
       8, 2, 0.5;
       5, 2, 1;
    ];

axis([0, 10, 0, 10]);
hold on;
for i = 1: size(obs, 1)
    rectangle('Position', [obs(i,1)-obs(i,3), obs(i,2)-obs(i,3), obs(i,3) * 2, obs(i,3) * 2], 'Curvature', [1 1]);
end

%% 主循环
while count < count_max
    count = count + 1;
    % 生成随机点
    if mod(count, 5) == 0
        randPoint = target;
    else
        randPoint = NewRandPoint(minX, maxX, minY, maxY);
    end
    plot(randPoint(1), randPoint(2), '.', markersize=3, color='blue');

    % 寻找最近节点为新节点的父节点
    currentParent= FindNearestPoint(points, currentIndex, randPoint);
    % 从父节点向随机点生长
    newPoint = Grow(points(currentParent, :), randPoint, growStep);
    % 检测是否发生碰撞
    if (~Collisionless(obs, newPoint, points(currentParent, :))) 
        continue;
    end
    % 将新节点添加
    currentIndex = currentIndex + 1;
    points(currentIndex, :) = newPoint;
    parent(currentIndex) = currentParent;

    plot([newPoint(1), points(currentParent, 1)], [newPoint(2), points(currentParent, 2)], '-', color='black');
    plot(start(1), start(2), '.', markersize=30, color='red');
    plot(target(1), target(2), '.', markersize=30, color='green');

    pause(0)
    if norm(target - newPoint) < growStep
        points(currentIndex + 1, :) = target;
        parent(currentIndex + 1) = currentIndex;
        break;
    end
end


hold off;