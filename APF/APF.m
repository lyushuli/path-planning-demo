function [result, count] = APF(mapLimit, start, target, obs, step, countMax, kAttr, kRep, kObs)
%APF APF路径规划算法
    result = zeros(countMax, 2);
    count = 1;
    rob = start;
    while norm(rob - target) > step && count < countMax
        count = count + 1;
        % 计算引力
        Fattr = AttractiveForce(target, rob, kAttr);
        % 计算斥力
        Frep = [0, 0];
        for i = 1 : size(obs, 1)
            Frep = Frep + RepulsiveForce(obs(i, 1:2), rob, kRep, obs(i, 3) * kObs);
        end
        % 计算合力
        F = Fattr + Frep;
        % 机器人移动
        result(count, :) = rob;
        move = step * F / norm(F);
        rob = rob + move;
        cla;
        for i = 1: size(obs, 1)
            rectangle('Position', [obs(i,1)-obs(i,3), obs(i,2)-obs(i,3), obs(i,3) * 2, obs(i,3) * 2], 'Curvature', [1 1]);
        end
        plot(start(1), start(2), '.', 'MarkerSize', 15, 'Color', 'green');
        plot(target(1), target(2), '.', 'MarkerSize', 15, 'Color', 'green');
        plot(rob(1), rob(2), '.', markersize=30, color='red');
        plot([rob(1), rob(1)+Fattr(1)], [rob(2), rob(2)+Fattr(2)], '-', color='#aaaaaa');
        plot([rob(1), rob(1)+Frep(1)], [rob(2), rob(2)+Frep(2)], '-', color='#aaaaaa');
        plot([rob(1), rob(1)+F(1)], [rob(2), rob(2)+F(2)], '-', color='black');
        pause(0)
    end
    result = result(1:count, :);
end

%% 计算引力函数
function force = AttractiveForce(target, point, xi)
%REPULSIVEFORCE 计算所受引力（向量）
%   输入目标坐标、待计算点坐标和引力尺度因子，返回力矢量
    force =  xi * (target - point);
end

%% 计算斥力函数
function force = RepulsiveForce(obstracle, point, eta, rho_0)
%REPULSIVEFORCE 计算所受斥力（向量）
%   输入障碍物坐标、待计算点坐标，斥力尺度因子和影响半径，返回力矢量
    rho = norm(obstracle - point);
    
    if rho > rho_0
        force = [0, 0];
        return
    end
    k = eta * ((1/rho - 1/rho_0)) * (1/rho^2);
    force = k * (point - obstracle) / rho;
end

