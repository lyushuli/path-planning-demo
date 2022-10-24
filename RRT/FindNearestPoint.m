function index = FindNearestPoint(points, size, point)
%FINDNEARESTPOINT 寻找距离最近的点
% points 待查找数组, size 待查找数组有用大小，point 目标点
    minDistance = norm(points(1, :) - point);
    index = 1;
    for i = 1 : size
        if norm(points(i, :) - point) < minDistance
            minDistance = norm(points(i, :) - point);
            index = i;
        end
    end
end

