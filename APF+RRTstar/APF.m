function [result, success, current_position, count, newObs] = APF(mapLimit, global_start, global_target, start, target, obs, step, countMax, kAttr, kRep, kObs)
%APF APF路径规划算法
    result = zeros(countMax, 2);
    count = 1;
    rob = start;
    move_record = [];
    success = true;
    while norm(rob - target) > step && count < countMax
        count = count + 1;
        cla;
        for i = 1: size(obs, 1)
            rectangle('Position', [obs(i,1)-obs(i,3), obs(i,2)-obs(i,3), obs(i,3) * 2, obs(i,3) * 2], 'Curvature', [1 1]);
        end
%         plot(global_start(1), global_start(2), '.', markersize=30, color='red');
        plot(global_target(1), global_target(2), '.', markersize=30, color='green');

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
        if ~isempty(move_record) 
            if abs(move + move_record) < 0.00001
                success = false;
                break;
            end
        end
        rob = rob + move;
        move_record = move;
        plot(rob(1), rob(2), '.', markersize=30, color='red')
        pause(0)
    end
    result = result(1:count, :);
    current_position = rob;
    newObs = obs;
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

