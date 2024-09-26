function [ P ] = constrainP( P )
%CONSTRAINP 此处显示有关此函数的摘要
%   此处显示详细说明
row = size(P,1);
col = size(P,2);
for i = 1:row
    for j = 1:col
        if P(i,j) < 0
            P(i,j) = 0;
        end
    end
end

end

