function [M,P,D] = ukf_predict2(M,P,Q,f_param,alpha,beta,kappa,mat)
% In:
%   M - Nx1 mean state estimate of previous step
%   P - NxN state covariance of previous step
%   f - Dynamic model function as a matrix A defining
%       linear function a(x) = A*x, inline function,
%       function handle or name of function in
%       form a(x,param)                   (optional, default eye())
%   Q - Process noise of discrete model   (optional, default zero)
%   f_param - Parameters of f               (optional, default empty)
%   alpha - Transformation parameter      (optional)
%   beta  - Transformation parameter      (optional)
%   kappa - Transformation parameter      (optional)
%   mat   - If 1 uses matrix form         (optional, default 0)
%
% Out:
%   M - Updated state mean
%   P - Updated state covariance
%
% Description:
%   Perform additive form Unscented Kalman Filter prediction step.
%
tr_param = {alpha beta kappa mat};
[M,P,D] = ut_transform1(M,P,f_param,tr_param);
P = P + Q;