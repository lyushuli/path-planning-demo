%% 初始化
clear; clc;

start_point = [1, 1, 0];
tar = [9, 9, 0.5];

RRTstar_max = 10000;
DWA_max = 100;
RRTstep = 0.5;
robot_state = [start_point(1), start_point(2), start_point(3), 0, 0]';
dt = 0.05;
kinematic_limit = [0.7, pi / 3, 0.3, pi / 3];
fortime = 1;

% obs = [
%        3.5, 3.1, rand * 0.4 + 0.3;
%        2.5, 5.5, rand * 0.4 + 0.3;
%        5.2, 6.6, rand * 0.4 + 0.3;
%        6.8, 4.5, rand * 0.4 + 0.3;
%        7.4, 7.1, rand * 0.4 + 0.3;
%        5.1, 4.8, rand * 0.4 + 0.3;
%        3.2, 8.8, rand * 0.4 + 0.3;
%        6.7, 8.9, rand * 0.4 + 0.3;
%        6.2, 1.8, rand * 0.4 + 0.3;
%        9.1, 5.6, rand * 0.4 + 0.3;
%     ];

obs = [
       rand * 8 + 1, rand * 8 + 1, rand * 0.4 + 0.3;
       rand * 8 + 1, rand * 8 + 1, rand * 0.4 + 0.3;
       rand * 8 + 1, rand * 8 + 1, rand * 0.4 + 0.3;
       rand * 8 + 1, rand * 8 + 1, rand * 0.4 + 0.3;
       rand * 8 + 1, rand * 8 + 1, rand * 0.4 + 0.3;
       rand * 8 + 1, rand * 8 + 1, rand * 0.4 + 0.3;
       rand * 8 + 1, rand * 8 + 1, rand * 0.4 + 0.3;
       rand * 8 + 1, rand * 8 + 1, rand * 0.4 + 0.3;
       rand * 8 + 1, rand * 8 + 1, rand * 0.4 + 0.3;
       rand * 8 + 1, rand * 8 + 1, rand * 0.4 + 0.3;
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
    rrt_result = RRTstar(map_limit, [robot_state(1), robot_state(2)], tar(1:2), obs, RRTstep, RRTstar_max);
    if isempty(rrt_result)
        rrt_result = [robot_state(1), robot_state(2); tar(1), tar(2)];
    end

    for i = 2: size(rrt_result, 1)
        RRT_tar = rrt_result(i, :);
        [~, robot_state, success, obs] = DWA(robot_state, kinematic_limit, [RRT_tar, 1], obs, obsv, DWA_max, dt, fortime, tar, rrt_result, i);
        if success == false
            ok = false;
            break;
        end
    end
end







