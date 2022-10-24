function randPoint = NewRandPoint(minX, maxX, minY, maxY)
%RANDPOINT 根据范围随机产生新节点
    randPoint = [minX + (maxX - minX) * rand, minY + (maxY - minY) * rand];    
end

