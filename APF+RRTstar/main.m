

%% 初始化
% 最大迭代次数
RRTCountMax = 30000;
APFCountMax = 30000;
% 地图范围
mapLimit = [0, 10, 0, 10];
% 步长
RRTstep = 0.1;
APFstep = 0.007;
% 起始点、目标点
% select = 5;
starts = [1, 5; 1, 1; 1, 9; 1, 3; 4,4];
targets = [9, 4; 9,9; 9, 1; 5, 9; 9,8];

select = 1;
start = starts(select, :);
target = targets(select, :);

% 障碍物 x y r
obs = [
       3.5, 3.1, 0.3;
       2.5, 5.5, 0.5;
       5.2, 6.6, 0.4;
       6.8, 4.5, 0.7;
       7.4, 7.1, 0.5;
       5.1, 4.8, 0.3;
       3.2, 8.8, 0.5;
       6.7, 8.9, 0.3;
       6.2, 1.8, 0.2;
       9.1, 5.6, 0.3
    ];


% kAttr, kRep
kAttr = 1;
kRep = 5;
kObs = 3;


axis(mapLimit);
hold on;
cla;
for i = 1: size(obs, 1)
    rectangle('Position', [obs(i,1)-obs(i,3), obs(i,2)-obs(i,3), obs(i,3) * 2, obs(i,3) * 2], 'Curvature', [1 1]);
end
plot(start(1), start(2), '.', markersize=30, color='red');
plot(target(1), target(2), '.', markersize=30, color='green');

% 
ok = false;
result = [];
while ~ok
    ok = true;
    rrt_result = RRTstar(mapLimit, start, target, obs, RRTstep, RRTCountMax);
    if isempty(rrt_result)
        disp("rrt star cannot find path")
        return
    end
    
    if size(rrt_result, 1) == 1
        disp('start == target')
        return
    end

    plot(rrt_result(:, 1), rrt_result(:, 2), '-', color='blue');
    
    for i = 2: size(rrt_result, 1)
        apf_start = rrt_result(i - 1, :);
        apf_target = rrt_result(i, :);
        [apf_result, success, newStart, count, obs] = APF(mapLimit, start, target,apf_start, apf_target, obs, APFstep, APFCountMax, kAttr, kRep, kObs);
        result = [result; apf_result];
        if (success == false)
            ok = false;
            start = newStart;
            break;
        end
    end
end


plot(result(:, 1), result(:, 2), '.', color='red');





