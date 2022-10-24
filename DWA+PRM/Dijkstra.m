function [result, length] = Dijkstra(mat, p1, p2)
    %% 初始化
    matSize = size(mat, 1);
    openList = zeros(matSize, 1);
    closeList = zeros(matSize, 1);
    fromList = zeros(matSize, 1);
    costList = zeros(matSize, 1);
    %% 开始搜索
    % 起始点添加到开列表
    openList(p1) = 1;
    index = 1;
    while index > 0
        openList(index) = 0;
        closeList(index) = 1;
        for i = 1: matSize
            % 如果不在关列表并可以到达
            if mat(index, i) > 0
                if closeList(i) == 0 && openList(i) == 0
                    % 添加到开列表，并记录相应信息
                    openList(i) = 1;
                    fromList(i) = index;
                    costList(i) = costList(index) + mat(index, i);
                elseif openList(i) > 0
                    % 已在开列表，判断从当前节点到达是否可以更近
                    if costList(index) + mat(index, i) < costList(i) 
                        fromList(i) = index;
                        costList(i) = costList(index) + mat(index, i);
                    end
                end
            end
        end
        if closeList(p2) > 0
            break;
        end
        index = FindMinCostPoint(openList, costList);
    end
    if index == 0
        result = [];
        length = 0;
        return;
    end

    resultSize = 1;
    i = p2;

    while i ~= p1
        resultSize = resultSize + 1;
        i = fromList(i);
    end

    i = p2;
    result = zeros(resultSize, 1);
    index = resultSize;
    while index > 0
        result(index) = i;
        i = fromList(i);
        index = index - 1;
    end
    length = costList(p2);
end

function index = FindMinCostPoint(openList, costList)
    index = 0;
    minVal = 0;
    for i = 1 : size(openList, 1)
        if openList(i) > 0
            if minVal == 0 || costList(i) < minVal
                minVal = costList(i);
                index = i;
            end
        end
    end
end