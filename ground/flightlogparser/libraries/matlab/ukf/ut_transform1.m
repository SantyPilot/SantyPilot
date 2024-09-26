%UT_TRANSFORM  Perform unscented transform
%
% Syntax:
%   [mu,S,C,X,Y,w] = UT_TRANSFORM(M,P,g,g_param,tr_param)
%
% In:
%   M - Random variable mean (Nx1 column vector)
%   P - Random variable covariance (NxN pos.def. matrix)
%   g - Transformation function of the form g(x,param) as
%       matrix, inline function, function name or function reference
%   g_param - Parameters of g               (optional, default empty)
%   tr_param - Parameters of the transformation as:       
%       alpha = tr_param{1} - Transformation parameter      (optional)
%       beta  = tr_param{2} - Transformation parameter      (optional)
%       kappa = tr_param{3} - Transformation parameter      (optional)
%       mat   = tr_param{4} - If 1 uses matrix form         (optional, default 0)
%       X     = tr_param{5} - Sigma points of x
%       w     = tr_param{6} - Weights as cell array {mean-weights,cov-weights,c}
%
% Out:
%   mu - Estimated mean of y
%    S - Estimated covariance of y
%    C - Estimated cross-covariance of x and y
%    X - Sigma points of x
%    Y - Sigma points of y
%    w - Weights as cell array {mean-weights,cov-weights,c}
%
% Description:
%   ...
%   For default values of parameters, see UT_WEIGHTS.
%
% See also
%   UT_WEIGHTS UT_MWEIGHTS UT_SIGMAS

% Copyright (C) 2006 Simo Sï¿½rkkï¿?%               2010 Jouni Hartikainen
%
% $Id: ut_transform.m 482 2010-10-18 07:53:23Z jmjharti $
%
% This software is distributed under the GNU General Public 
% Licence (version 2 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [mu,S,C,X,Y,w] = ut_transform1(M,P,g_param,tr_param)

alpha = tr_param{1};
beta = tr_param{2};
kappa = tr_param{3};
mat = tr_param{4};
X = [];
w = [];

[WM,W,c] = ut_mweights(size(M,1),alpha,beta,kappa);
X = ut_sigmas(M,P,c);
w = {WM,W,c};
  
%
% Propagate through the function
%
Y = [];
for i=1:size(X,2)
  Y = [Y ukf_track_h_ins(X(:,i),g_param)];
end

mu = Y*WM;
S  = Y*W*Y';
C  = X*W*Y';


  
