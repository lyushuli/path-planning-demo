function result = PRM(mapLimit, start, target, obs, countMax)
%PRM 根据地图大小和障碍物生成相应无碰撞的无向有权图的邻接矩阵
    pointList = RandPointList(start, target, mapLimit, countMax, obs);
    adjacentMat = CreateAdjacentMat(pointList, obs);
    [indexs, ~] = Dijkstra(adjacentMat, 1, 2);
    if isempty(indexs)
        result = [];
        return;
    end
    result = pointList(indexs(:), :);
end



%% 记录起始点、目标点，并随机生成无碰撞采样点，含起始点1、目标点2共countMax个
function pointList = RandPointList(start, target, mapLimit, countMax, obs)
    pointList = zeros(countMax, 2);
    pointList(1,:) = start;
    pointList(2,:) = target;
    count = 3;
    while count <= countMax
        [x, y] = RandPoint(mapLimit);
        if Collision1([x, y], obs)
            continue;
        end
        pointList(count, :) = [x, y]; 
        count = count + 1;
    end
end




%% 连接无碰撞的点，构建无向有权图(邻接矩阵)，权值即两点直线距离
function adjacentMat = CreateAdjacentMat(pointList, obs)
    n = size(pointList, 1);
    adjacentMat = zeros(n, n);
    for i = 1 : n
        for j = i + 1 : n
            p1 = pointList(i, :);
            p2 = pointList(j, :);
            if ~Collision2(p1, p2, obs)
                dist = norm(p1 - p2);
                adjacentMat(i, j) = dist;
                adjacentMat(j, i) = dist;
            end
        end
    end
end

%% 在地图内生成随机点
function [x, y] =  RandPoint(mapLimit) 
    x = rand * (mapLimit(2) - mapLimit(1)) + mapLimit(1);
    y = rand * (mapLimit(4) - mapLimit(3)) + mapLimit(3);
%     plot(x, y, '.', markersize=15, color='black');
end
%% 圆形障碍物碰撞检测
function reval = Collision1(p, obs)
    for i = 1: size(obs, 1)
        if norm(p - obs(i, 1:2)) < obs(i, 3)
            reval = true;
            return;
        end
    end
    reval = false;
end

function reval = Collision2(p1, p2, obs) 
%COLLISIONLESS 判断是否碰撞返回布尔值
%   输入障碍物中心点和半径、待检测点
    for i = 1: size(obs, 1)
        center = obs(i, 1:2);
        radio = obs(i, 3);
        % 点在圆内
        if norm(center - p1) < radio || norm(center - p2) < radio
            reval = true;
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
            reval = true;
            return;
        end
    end
    reval = false;
end