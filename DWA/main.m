function main(start, tar)
count_max = 500;
robot_state = [start(1), start(2), pi / 4, 0, 0]';
% tar = [9, 9, 0.5];
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
       9.1, 5.6, 0.3;
       2.1, 2.6, 0.5;
       0.8, 7.5, 0.6;
    ];
dt = 0.05;
kinematic_limit = [0.5, pi / 6, 0.5, pi / 3];
v_step = 0.01;
w_step = pi/180;
fortime = 3;
area = [0, 10, 0, 10];

axis([-2, 12, -2, 12]);
hold on;

set(gcf,'WindowButtonDownFcn',@ButttonDownFcn);

plot(tar(1), tar(2), '.', markersize=30, color='green');
rectangle('Position', [tar(1) - tar(3), tar(2)-tar(3), tar(3) * 2, tar(3) * 2], 'Curvature', [1 1], 'EdgeColor', '#aaaaaa', 'LineStyle', '--');
for i = 1: size(obs, 1)
    ob = obs(i, :);
    rectangle('Position', [ob(1) - ob(3), ob(2)-ob(3), ob(3) * 2, ob(3) * 2], 'Curvature', [1 1],  'FaceColor', '#aaaaaa');
end

[result, robot_state, success] = DWA(robot_state, kinematic_limit, tar, obs, count_max, dt, fortime);


disp(success)




