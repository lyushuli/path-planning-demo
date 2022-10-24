function newPoint = Grow(src, tar, step)
%GROW 根据源节点、目标节点和步长生成新节点
    newPoint = src +  (tar - src) / norm(tar - src) * step;
end

