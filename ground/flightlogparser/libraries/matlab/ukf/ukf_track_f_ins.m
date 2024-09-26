function [ x_n ] = ukf_track_f_ins( x,param )
dT = param(1,1);
U = param(1,2:7);
x_n = RungeKutta(x, U, dT);
% normalize quaternion
len = x_n(7) * x_n(7) + x_n(8) * x_n(8) + x_n(9) * x_n(9) + x_n(10) * x_n(10);
invqmag = 1 / (len * len);
x_n(7) = x_n(7) * invqmag;
x_n(8) = x_n(8) * invqmag;
x_n(9) = x_n(9) * invqmag;
x_n(10) = x_n(10) * invqmag;

