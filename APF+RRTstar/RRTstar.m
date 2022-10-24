%% 主函数
function result = RRTstar(mapLimit, start, target, obs, step, countMax)
%RRTSTAR RRTstar路径规划
%   args: 
%       mapLimit 地图大小[minx, maxx, miny, maxy], 
%       start 起始点[x, y], 
%       target 目标点[x, y], 
%       obs 圆形障碍物 [x1, y1, r1; x2, y2, r2; ....; xn, yn, rn], 
%       step 步长, 
%       countMax 最大迭代次数 

%% 初始化参数
    % 最大迭代次数
    count = 1;
    count_max = countMax;
    % 地图范围
    minX = mapLimit(1); 
    maxX = mapLimit(2);
    minY = mapLimit(3);
    maxY = mapLimit(4);
    % 记录所有节点，及其父节点的矩阵
    points = zeros(count_max, 2);
    parent = zeros(count_max, 1);
    % 步长
    growStep = step;
    % 添加根节点
    points(1, :) = start; 
    currentIndex = 1;

%% 主循环
    over = false;
    while count < count_max && ~over
        count = count + 1;
        if norm(target - points(currentIndex, :)) < growStep
            % 如果上一个节点到目标点距离小于步长，可以直接到达目标点，结束
            newPoint = target;
            over = true;
        else
            % 不能直接到达
            if mod(count, 5) == 0
                randPoint = target;
            else
                randPoint = NewRandPoint(minX, maxX, minY, maxY);
            end
            % 寻找最近节点为新节点的父节点
            currentParent= FindNearestPoint(points, currentIndex, randPoint);
            % 从父节点向随机点生长
            newPoint = Grow(points(currentParent, :), randPoint, growStep);
            % 检测是否发生碰撞
            if (~Collisionless(obs, newPoint, points(currentParent, :))) 
                continue;
            end
        end

        % 寻找更优父节点
        while currentParent ~= 1 && Collisionless(obs, newPoint, points(parent(currentParent), :))
            currentParent = parent(currentParent);
        end
        % 将新节点添加
        currentIndex = currentIndex + 1;
        points(currentIndex, :) = newPoint;
        parent(currentIndex) = currentParent;
%         for i = 1: size(obs, 1)
%             rectangle('Position', [obs(i,1)-obs(i,3), obs(i,2)-obs(i,3), obs(i,3) * 2, obs(i,3) * 2], 'Curvature', [1 1]);
%         end
%         plot([newPoint(1), points(currentParent, 1)], [newPoint(2), points(currentParent, 2)], '-', color='#aaaaaa');
%         plot(start(1), start(2), '.', markersize=30, color='red');
%         plot(target(1), target(2), '.', markersize=30, color='green');
%         pause(0);
    end

%% 返回结果
    % 未找到路径返回空
    if points(currentIndex, :) ~= target
        result = [];
        return;
    end
    % 计算结果大小
    cur = currentIndex;
    count = 1;
    while parent(cur) ~= 0
        cur = parent(cur);
        count = count + 1;
    end
    % 记录结果
    cur = currentIndex;
    result = zeros(count, 2);
    for i = count : -1 : 1
        result(i, :) = points(cur, :);
        cur = parent(cur);
    end
end


%% 产生随机节点
function randPoint = NewRandPoint(minX, maxX, minY, maxY)
%RANDPOINT 根据范围随机产生新节点
    randPoint = [minX + (maxX - minX) * rand, minY + (maxY - minY) * rand];    
end
%% 根据父节点和目标节点和步长生成新节点
function newPoint = Grow(src, tar, step)
%GROW 根据源节点、目标节点和步长生成新节点
    newPoint = src +  (tar - src) / norm(tar - src) * step;
end
%% 找到最进节点
function index = FindNearestPoint(points, size, point)
%FINDNEARESTPOINT 寻找距离最近的点
% points 待查找数组, size 待查找数组有用大小，point 目标点
    minDistance = norm(points(1, :) - point);
    index = 1;
    for i = 1 : size
        if norm(points(i, :) - point) < minDistance
            minDistance = norm(points(i, :) - point);
            index = i;
        end
    end
end

%% 判断是否碰到障碍物
function reval = Collisionless(obs, p1, p2)
%COLLISIONLESS 判断是否碰撞返回布尔值
%   输入障碍物中心点和半径、待检测点
    for i = 1: size(obs, 1)
        center = obs(i, 1:2);
        radio = obs(i, 3);
        % 点在圆内
        if norm(center - p1) < radio || norm(center - p2) < radio
            reval = false;
            return;
        end
        % 直线 Ax + By + C = 0;  (y1 - y2) x + (x2 - x1) y + x1y2 - y1x2 = 0;
        if p1(1) == p2(1)
            A = 1;
            B = 0;
            C = -p1(1);
        elseif p1(2) == p2(2)
            A = 0;
            B = 1;
            C = -p1(2);
        else
            A = p1(2) - p2(2);
            B = p2(1) - p1(1);
            C = p1(1) * p2(2) - p1(2) * p2(1);
        end
        dist1 = (A * center(1) + B * center(2) + C) ^ 2;
        dist2 = (A * A + B * B) * radio * radio;
        if dist1 > dist2
            continue;
        end
        angle1 = (center(1) - p1(1)) * (p2(1) - p1(1)) + (center(2) - p1(2)) * (p2(2) - p1(2));
        angle2 = (center(1) - p2(1)) * (p1(1) - p2(1)) + (center(2) - p2(2)) * (p1(2) - p2(2));
        if angle1 > 0 && angle2 > 0
            reval = false;
            return;
        end
    end
    reval = true;
end


