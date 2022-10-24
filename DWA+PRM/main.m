%% 初始化
% function main(start_point, tar)
clear; clc;
start_point = [1, 1, 0]; tar = [9, 9, 0.5];

PRM_max = 50;
DWA_max = 100;
robot_state = [start_point(1), start_point(2), start_point(3), 0, 0]';
dt = 0.05;
kinematic_limit = [0.7, pi / 3, 0.3, pi / 3];
fortime = 1.5;

obs = [
       3.5, 3.1, rand * 0.4 + 0.3;
       2.5, 5.5, rand * 0.4 + 0.3;
       5.2, 6.6, rand * 0.4 + 0.3;
       6.8, 4.5, rand * 0.4 + 0.3;
       7.4, 7.1, rand * 0.4 + 0.3;
       5.1, 4.8, rand * 0.4 + 0.3;
       3.2, 8.8, rand * 0.4 + 0.3;
       6.7, 8.9, rand * 0.4 + 0.3;
       6.2, 1.8, rand * 0.4 + 0.3;
       9.1, 5.6, rand * 0.4 + 0.3;
    ];

obsv = [
        rand * 0.2 - 0.1, rand * 0.2 - 0.1, 0;
        rand * 0.2 - 0.1, rand * 0.2 - 0.1, 0;
        rand * 0.2 - 0.1, rand * 0.2 - 0.1, 0;
        rand * 0.2 - 0.1, rand * 0.2 - 0.1, 0;
        rand * 0.2 - 0.1, rand * 0.2 - 0.1, 0;
        rand * 0.2 - 0.1, rand * 0.2 - 0.1, 0;
        rand * 0.2 - 0.1, rand * 0.2 - 0.1, 0;
        rand * 0.2 - 0.1, rand * 0.2 - 0.1, 0;
        rand * 0.2 - 0.1, rand * 0.2 - 0.1, 0;
        rand * 0.2 - 0.1, rand * 0.2 - 0.1, 0;
];




map_limit = [0, 10, 0, 10];
axis(map_limit);
hold on;


%% 主循环
ok = false;
while ~ok
    ok = true;
    cla;
    prm_result = PRM(map_limit, [robot_state(1), robot_state(2)], tar(1:2), obs, PRM_max);
    if isempty(prm_result)
        prm_result = [robot_state(1), robot_state(2); tar(1), tar(2)];
    end

    for i = 2: size(prm_result, 1)
        PRM_tar = prm_result(i, :);
        [~, robot_state, success, obs] = DWA(robot_state, kinematic_limit, [PRM_tar, 1], obs, obsv, DWA_max, dt, fortime, tar, prm_result, i);
        if success == false
            ok = false;
            break;
        end
    end
end







