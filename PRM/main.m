%% 初始化参数
countMax = 50;
start = [1, 1]; target = [9, 9];
% start = [1, 9]; target = [9, 1];
% start = [5, 1]; target = [5, 9];
% start = [1, 5]; target = [9, 5];

mapLimit = [0, 10, 0, 10];
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


hold on;
axis(mapLimit);
for k = 1: size(obs, 1)
    ob = obs(k, :);
    rectangle('Position', [ob(1) - ob(3), ob(2)-ob(3), ob(3) * 2, ob(3) * 2], 'Curvature', [1 1],  'FaceColor', '#eeeeee');
end

result = PRM(mapLimit, start, target, obs, countMax);

%% 显示


plot(result(:,1), result(:,2), '-', color='blue', LineWidth=1);

hold off;

