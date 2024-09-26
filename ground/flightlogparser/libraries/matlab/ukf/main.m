clc
clear all;

%% get mesurements

FORMAT_SIM  = 2;                                                                                                 
FORMAT_XLS  = 1;
FORMAT_TXT  = 0;
FORMAT      = FORMAT_TXT;
PI_RD = 180./pi;

Q = diag([0.2, 0.02, 0.002, 0.2, 0.02, 0.002]);
R = diag([0.5, 0.00087, 0.05]);

P = diag([0.6, 0.9, 0.1, 0.4, 0.3, 0.07]);
alpha = 0.1;
beta = 2;
kappa = 0; 
mat = 1;
f_func = @ukf_track_f;
h_func = @ukf_track_h;
%}
%% Ins Simulation
P = diag([25.0, 25.0, 25.0, 5.0, 5.0, 5.0, ...
    1e-4, 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6]);

%Q = diag([1e-3, 1e-3, 1e-3, 3e-3, 3e-3, 3e-3, ...
%    1e-6, 1e-6, 1e-6]);
Q = P;

R = diag([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 10, ...
    10, 10, 0.01]);

alpha = 0.1;
beta = 2;
kappa = 0; 
mat = 1;
f_func = @ukf_track_f_ins;
h_func = @ukf_track_h_ins;
hparam = [26000,400,40000];

%% UKF filtering
%[raIn, xyIn, time, n] = getMeasures(FORMAT);
data_path = 'E:\Booklist\video\msys2\home\Santy\SantyPilot\logs\2024_05_05_21_12_29_stylog.mat';
[raIn, xyIn, time, n, u] = getMeasuresIns(data_path);
raOut = zeros(size(raIn));
xyOut = xyIn;
X = xyIn(:,1);
errorCnt = 0;
hasError = 0;
error_idx=zeros(size(raIn,2),1);
for i = 2 : n
    f_param = []; % prepare params
    f_param(1,1) = time(i)-time(i-1);
    u(1,i) = u(1,i) - X(11) * 180 / 3.14159;
    u(2,i) = u(2,i) - X(12) * 180 / 3.14159;
    u(3,i) = u(3,i) - X(13) * 180 / 3.14159;
    f_param(1,2:7) = u(:,i)';
    [Xpre, Ppre] = ukf_predict1(X, P, f_func, Q, f_param, alpha, beta, kappa, mat);
    % [Xpre, Ppre] = ukf_predict2(X, P, Q, f_param, alpha, beta, kappa, mat);
    Zpre = ukf_track_h_ins(Xpre, hparam);
    %{
    if abs(Zpre(2) - raIn(2,i)) > (3./180*pi) || abs(Zpre(1) - raIn(1,i))>2.
        errorCnt = errorCnt + 1;
        hasError = 1;
    else
        errorCnt = 0;
        hasError = 0;
    end
    if errorCnt > 4
        errorCnt = 0;
        X = xyIn(:,i);
    end
    %}
    %if ~hasError
        [X, P] = ukf_update2(Xpre, Ppre, raIn(:,i), R, hparam, alpha, beta, kappa, mat);
    %end
    %X = normQ(X);
    xyOut(:,i) = X;
    
    P = (P + P') * (1/2); % sym
    P = constrainP(P); % contraint
end
% raOut = ukf_track_h(xyOut);
raOut = ukf_track_h_ins(xyOut, hparam);

%% errors------------------------------------------------------------------
xyErr = xyOut - xyIn;
raErr = raOut - raIn;

%% plot---------------------------------------------------------------------
index = [9,10];
figure(1);
clf;
subplot(2,1,1);
plot(xyIn(index(1),:),'b');
hold on;
plot(xyOut(index(1),:), 'r', 'linewidth', 1.5);
set(gca,'XGrid','on');
set(gca,'YGrid','on');
legend('EKF', 'UKF')
ylabel('q2');
title('EKF-UKF');

subplot(2,1,2);
plot(xyIn(index(2),:), 'b');
hold on;
plot(xyOut(index(2),:), 'r', 'linewidth', 1.5);
set(gca,'XGrid','on');
set(gca,'YGrid','on');
legend('EKF', 'UKF')
xlabel('time/0.005 sec');
ylabel('q3');

indexm = [1,4];
figure(2);
clf;
subplot(2,1,1);
plot(raIn(indexm(1),:),'b');
hold on;
plot(raOut(indexm(1),:), 'r', 'linewidth', 1.5);
set(gca,'XGrid','on');
set(gca,'YGrid','on');
legend('Measurement', 'UKF')
ylabel('X:m');
title('Measurement-UKF');

subplot(2,1,2);
plot(raIn(indexm(2),:), 'b');
hold on;
plot(raOut(indexm(2),:), 'r', 'linewidth', 1.5);
set(gca,'XGrid','on');
set(gca,'YGrid','on');
legend('Measurement', 'UKF')
xlabel('time/0.005 sec');
ylabel('Vx:m/s');


