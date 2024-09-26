function [ X ] = normQ( X )
%NORMQ 此处显示有关此函数的摘要
%   此处显示详细说明
len = X(7) * X(7) + X(8) * X(8) + X(9) * X(9) + X(10) * X(10);
X(7) = X(7) / len;
X(8) = X(8) / len;
X(9) = X(9) / len;
X(10) = X(10) / len;
end

