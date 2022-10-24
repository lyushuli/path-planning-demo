%% DWA主函数
function [result, robot_state, success] = DWA(robot_state, kinematic_limit, tar, obs, count_max, dt, fortime)
    result = zeros(5, count_max);
    v_step = 0.01;
    w_step = pi/180;
    %% 主循环
    for count = 1: count_max
        if norm([robot_state(1), robot_state(2)] - tar(1:2)) < tar(3)
            success = true;
            return;
        end
        [v_min, v_max, w_min, w_max] = DynamicWindow(robot_state, kinematic_limit, dt);
        fit_min = inf;
        best_v = 0;
        best_w = 0;
        for v = v_min : v_step : v_max
            for w = w_min: w_step: w_max
                sta = [robot_state(1); robot_state(2); robot_state(3); v; w];
                [state, trajs] = GetTrajectory(sta, fortime, dt);
                if Collision(state, obs, kinematic_limit(3), dt)
                    continue;
                end
                fit = Fitness(state, tar, obs, kinematic_limit);
                if fit < fit_min
                    fit_min = fit;
                    best_v = v;
                    best_w = w;
                end
            end
        end
    
        robot_state(4) = best_v;
        robot_state(5) = best_w;
        robot_state = NextState(robot_state, dt);
        result(:, count) = robot_state;
    
        cla;
        hold on;
        plot(tar(1), tar(2), '.', markersize=30, color='blue');
        for k = 1: size(obs, 1)
            ob = obs(k, :);
            rectangle('Position', [ob(1) - ob(3), ob(2)-ob(3), ob(3) * 2, ob(3) * 2], 'Curvature', [1 1],  'FaceColor', '#aaaaaa');
        end
        plot(trajs(1, :), trajs(2, :), '--', color='blue', LineWidth=1);
        plot(robot_state(1), robot_state(2), '.', markersize=30, color='red');
        plot([robot_state(1), robot_state(1) + cos(robot_state(3))], [robot_state(2), robot_state(2) + sin(robot_state(3))], '-', color='red', LineWidth=1);
        pause(0);
    end
    success = false;
end



%% 计算制动距离
function dist = BrakeDistance(v, a, dt)
    dist=0;
    while v > 0 
        dist = dist + v*dt;
        v = v - a*dt;
    end
end

%% 检测是否碰撞
function bool = Collision(state, obs, a, dt)
    for i = 1 : size(obs, 1) 
        dist = norm(state(1:2)- obs(i, 1:2)) - obs(i, 3);
        brakedist =  BrakeDistance(state(4), a, dt);
        if dist < brakedist
            bool = true;
            return;
        end
    end
    bool = false;
end

%% 获取动态窗口即 速度和角速度范围
function [Vmin, Vmax, Wmin, Wmax] = DynamicWindow(robot_state, kinematic_limit, dt)
    v_min = 0;
    v_max = kinematic_limit(1);
    w_min = -kinematic_limit(2);
    w_max = kinematic_limit(2);
    va_max = kinematic_limit(3);
    wa_max = kinematic_limit(4);

    Vmin = max(v_min, robot_state(4) - va_max * dt);
    Vmax = min(v_max, robot_state(4) + va_max * dt);
    Wmin = max(w_min, robot_state(5) - wa_max * dt);
    Wmax = min(w_max, robot_state(5) + wa_max * dt);
end

%% 评判位置的适应函数
function val = Fitness(state, tar, obs, kinematic_limit)
    k_att = 10;
    k_rep = 1;
    k_rho = 1.2;
    k_head = 0;
    k_vel = 0;

    f_val = Fatt(state(1), state(2), tar, k_att);
    for i = 1 : size(obs, 1)
        f_val = f_val + Frep(state(1), state(2), obs(i, :), k_rep, k_rho);
    end
    h_val = k_head * Head(state(1), state(2), state(3), tar);
    v_val = k_vel * Velocity(state(4), kinematic_limit(1));
    val = h_val + f_val + v_val;
end

function fatt = Fatt(x, y, tar, k_att)
    fatt = 1 / 2 * k_att * norm([x,y] - tar(1:2)) ^ 2;
end

function frep = Frep(x, y, obs, k_rep, k_rho) 
    rho = norm([x,y] - obs(1:2)) - obs(3);
    if rho > k_rho * obs(3)
        frep = 0;
        return;
    end
    frep = 1 / 2 * k_rep * (1 / rho - 1/ k_rho * obs(3)) ^ 2;
end

function head = Head(x, y, theta, tar)
    tar = tar(1:2);
    px = x + cos(theta);
    py = y + sin(theta);
    tar = tar / norm(tar - [x,y]);
    dist = norm([px,py] - tar);
    head = dist ^ 2;
end

function vel = Velocity(v, v_max)
    vel = (v_max - v) / v;
end

%% 获得当前状态到 time_max 时间后的状态的和相应的轨迹
function [robot_state, traj] = GetTrajectory(robot_state, time_max, dt)
    traj = zeros(5, time_max / dt);
    for i = 1 : time_max / dt
        robot_state = NextState(robot_state, dt);
        traj(:, i) = robot_state; 
    end
end

%% 计算 dt 时间后的下一个状态
function x = NextState(x, dt)
    F = [1 0 0 0 0
         0 1 0 0 0
         0 0 1 0 0
         0 0 0 0 0
         0 0 0 0 0];

    B = [dt*cos(x(3)) 0
        dt*sin(x(3)) 0
        0 dt
        1 0
        0 1];
    
    u = [x(4); x(5)];

    x= F*x+B*u;  
end

%% 判断机器人停留原地即陷入局部最优
% function bool = Stoped(result, index)
%     count = 0;
%     while index > 1
%         if (result(1:2, index) == result(1:2, index - 1))
%             count = count + 1;
%             if count > 30
%                 bool = true;
%                 return;
%             end
%         else
%             bool = false;
%             return;
%         end
%     end
%     bool = false;
%     return;
% end






