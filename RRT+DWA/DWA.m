%% DWA主函数
function [result, robot_state, success, obs] = DWA(robot_state, kinematic_limit, tar, obs, obsv, count_max, dt, fortime, global_tar, rrt_result, rrt_i)
    result = zeros(5, count_max);
    v_step = 0.01;
    w_step = pi/180;
    %% 主循环
    count = 1;
    while count < count_max || robot_state(4) > 0.5 * kinematic_limit(1)
        count = count + 1;
        if tar(1:2) == global_tar(1:2)
            tar = global_tar;
        end
        if norm([robot_state(1), robot_state(2)] - tar(1:2)) < tar(3)
            success = true;
            return;
        end
        obs = obs + obsv * dt;
        [v_min, v_max, w_min, w_max] = DynamicWindow(robot_state, kinematic_limit, dt);
        fit_min = inf;
        best_v = 0;
        best_w = 0;
        trajs = zeros(5, fortime / dt, round((v_max - v_min) / v_step * (w_max - w_min) * w_step));
        index = 0;
        for v = v_min : v_step : v_max
            for w = w_min: w_step: w_max
                index = index + 1;
                sta = [robot_state(1); robot_state(2); robot_state(3); v; w];

                if Collision(sta, obs, obsv, fortime, dt)
                    continue;
                end
                fit = 0;
                newobs = obs + obsv * fortime;
                for t = dt: dt: fortime
                    [state, ~] = GetTrajectory(sta, t, dt);
                    fit = fit + Fitness(state, tar, newobs, kinematic_limit);
                end
                [~, traj] = GetTrajectory(sta, t, dt);
                if fit < fit_min
                    fit_min = fit;
                    best_v = v;
                    best_w = w;
                    best_traj = traj;
                end
            end
        end
    
        robot_state(4) = best_v;
        robot_state(5) = best_w;
        robot_state = NextState(robot_state, dt);
        result(:, count) = robot_state;
    
        %% 显示
        cla;
        hold on;
        for k = 1: size(obs, 1)
            ob = obs(k, :);
            rectangle('Position', [ob(1) - ob(3), ob(2)-ob(3), ob(3) * 2, ob(3) * 2], 'Curvature', [1 1],  'FaceColor', '#dddddd');
        end
%         for k = 1: size(newobs, 1)
%             ob = newobs(k, :);
%             rectangle('Position', [ob(1) - ob(3), ob(2)-ob(3), ob(3) * 2, ob(3) * 2], 'Curvature', [1 1],  LineStyle='--', EdgeColor='#999999');
%         end
        plot(global_tar(1), global_tar(2), '.', markersize=30, color='green');
        rectangle('Position', [global_tar(1) - global_tar(3), global_tar(2)-global_tar(3), global_tar(3) * 2, global_tar(3) * 2], 'Curvature', [1 1], 'EdgeColor', '#aaaaaa', 'LineStyle', '--');
        plot(tar(1), tar(2), '.', markersize=30, color='blue');
        rectangle('Position', [tar(1) - tar(3), tar(2)-tar(3), tar(3) * 2, tar(3) * 2], 'Curvature', [1 1], 'EdgeColor', '#aaaaaa', 'LineStyle', '--');
        plot(rrt_result(rrt_i:end, 1), rrt_result(rrt_i:end, 2), '--', color='black');
        plot([robot_state(1), tar(1)], [robot_state(2), tar(2)], '--', color='#aaaaaa');
        for i = 1: size(trajs, 3)
            traj = trajs(:, :, i);
            plot(traj(1, :), traj(2, :), '-', color='blue', LineWidth=0.5);
        end
        if ~isempty(best_traj)
            plot(best_traj(1, :), best_traj(2, :), '-', color='blue');
        end
        
%         plot([robot_state(1), robot_state(1) + cos(robot_state(3))* robot_state(4)], [robot_state(2), robot_state(2) + sin(robot_state(3)) * robot_state(4)], '-', color='red');
        plot(robot_state(1), robot_state(2), '.', markersize=30, color='red');
        pause(0);
    end
    
    success = false;
end



%% 检测是否碰撞
function bool = Collision(state, obs, obsv, fortime, dt)
    for t = 0: dt: fortime
        obs_temp = obs + obsv * t;
        state = NextState(state, dt);
        for i = 1 : size(obs_temp, 1) 
            dist = norm([state(1), state(2)] - obs_temp(i, 1:2));
            if dist < obs_temp(i, 3)
                bool = true;
                return;
            end
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
    k_rep = 0.5;
    k_rho = 3;
    k_head = 0;
    k_vel = 0.5;

    f_val = Fatt(state(1), state(2), tar, k_att);
    for i = 1 : size(obs, 1)
        f_val = f_val + Frep(state(1), state(2), obs(i, :), k_rep, k_rho);
    end
    h_val = k_head * Head(state(1), state(2), state(3), tar);
    v_val = k_vel * Velocity(state(4), kinematic_limit(1));
    val = h_val + f_val + v_val;
end

function fatt = Fatt(x, y, tar, k_att)
    fatt = 1 / 2 * k_att * max(norm([x,y] - tar(1:2)) - tar(3), 0) ^ 2;
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
    traj = zeros(5, round(time_max / dt));
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
%
% end






