%
% @brief: ins f,h implementation in matlab
% @author: zhangxin
% @date: 2024-5-4
%

% 1. test runge-kutta
clear all
clc

set(0,'defaultfigurecolor','w')
x0 = 0; %自变量初值
xn = 1; %自变量终值
y0 = 1; %因变量初值
h = 0.1; %步长
[x,y] = RungeKutta4(x0, xn, y0, h); %自定义4阶龙格库塔法
[x2,y2] = ode45(@fun, [0,1], y0);   %matlab内部函数
plot(x,y,'r.')
hold on
plot(x2,y2,'b')

function [x,y] = RungeKutta4(x0, xn, y0, h)
n = (xn-x0)/h;
x = zeros(n+1,1);
y = zeros(n+1,1);
x(1) = x0;
y(1) = y0;
for i = 1:n
   x(i+1) = x(i)+h;
   K1 = fun(x(i), y(i));
   K2 = fun(x(i)+h/2, y(i)+K1*h/2);
   K3 = fun(x(i)+h/2, y(i)+K2*h/2);
   K4 = fun(x(i)+h, y(i)+K3*h);
   y(i+1) = y(i)+h/6*(K1+2*K2+2*K3+K4);
end
end

function f = fun(x,y)
f = y-2*x/y;

% 1.predict state 
void INSStatePrediction(const float gyro_data[3], const float accel_data[3], float dT)
{
    RungeKutta(ekf.X, U, dT);
    // normalize
    invqmag   = invsqrtf(ekf.X[6] * ekf.X[6] + ekf.X[7] * ekf.X[7] + ekf.X[8] * ekf.X[8] + ekf.X[9] * ekf.X[9]);
    ekf.X[6] *= invqmag;
    ekf.X[7] *= invqmag;
    ekf.X[8] *= invqmag;
    ekf.X[9] *= invqmag;
}

% 2. Runge-Kutta implementation
function [x,y]=runge_kutta1(ufunc,y0,h,a,b)
    % 参数表顺序依次是微分方程组的函数名称，初始值向量，步长，时间起点，
    % 时间终点（参数形式参考了ode45函数）
n=floor((b-a)/h);%求步数
x(1)=a;%时间起点
y(:,1)=y0;%赋初值，可以是向量，但是要注意维数
for ii=1:n

x(ii+1)=x(ii)+h;

k1=ufunc(x(ii),y(:,ii));

k2=ufunc(x(ii)+h/2,y(:,ii)+h*k1/2);

k3=ufunc(x(ii)+h/2,y(:,ii)+h*k2/2);

k4=ufunc(x(ii)+h,y(:,ii)+h*k3);

y(:,ii+1)=y(:,ii)+h*(k1+2*k2+2*k3+k4)/6;
%按照龙格库塔方法进行数值求解
end

%3 state predition implementation
function []=StateEq(float X[NUMX], float U[NUMU], float Xdot[NUMX])
float ax, ay, az, wx, wy, wz, q0, q1, q2, q3;

// ax=U[3]-X[13]; ay=U[4]-X[14]; az=U[5]-X[15];  // subtract the biases on accels
ax = U[3];
ay = U[4];
az = U[5]; // NO BIAS STATES ON ACCELS
wx = U[0] - X[10];
wy = U[1] - X[11];
wz = U[2] - X[12]; // subtract the biases on gyros
q0 = X[6];
q1 = X[7];
q2 = X[8];
q3 = X[9];

// Pdot = V
Xdot[0] = X[3];
Xdot[1] = X[4];
Xdot[2] = X[5];

// Vdot = Reb*a
Xdot[3] =
    (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * ax + 2.0f * (q1 * q2 -
                                                           q0 * q3) *
    ay + 2.0f * (q1 * q3 + q0 * q2) * az;
Xdot[4] =
    2.0f * (q1 * q2 + q0 * q3) * ax + (q0 * q0 - q1 * q1 + q2 * q2 -
                                       q3 * q3) * ay + 2 * (q2 * q3 -
                                                            q0 * q1) *
    az;
Xdot[5] =
    2.0f * (q1 * q3 - q0 * q2) * ax + 2 * (q2 * q3 + q0 * q1) * ay +
    (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az + 9.81f;

// qdot = Q*w
Xdot[6]  = (-q1 * wx - q2 * wy - q3 * wz) / 2.0f;
Xdot[7]  = (q0 * wx - q3 * wy + q2 * wz) / 2.0f;
Xdot[8]  = (q3 * wx + q0 * wy - q1 * wz) / 2.0f;
Xdot[9]  = (-q2 * wx + q1 * wy + q0 * wz) / 2.0f;

// best guess is that bias stays constant
Xdot[10] = Xdot[11] = Xdot[12] = 0;


% x->y
% MeasurementEq(ekf.X, ekf.Be, Y);
%{
    float Z[10] = { 0 };
    float Y[10] = { 0 };

    // GPS Position in meters and in local NED frame
    Z[0] = Pos[0];
    Z[1] = Pos[1];
    Z[2] = Pos[2];

    // GPS Velocity in meters and in local NED frame
    Z[3] = Vel[0];
    Z[4] = Vel[1];
    Z[5] = Vel[2];

    if (SensorsUsed & MAG_SENSORS) {
        // magnetometer data in any units (use unit vector) and in body frame
        float invBmag = invsqrtf(mag_data[0] * mag_data[0] + mag_data[1] * mag_data[1] + mag_data[2] * mag_data[2]);
        Z[6] = mag_data[0] * invBmag;
        Z[7] = mag_data[1] * invBmag;
        Z[8] = mag_data[2] * invBmag;
    }

    // barometric altimeter in meters and in local NED frame
    Z[9] = BaroAlt;
    float invqmag = invsqrtf(ekf.X[6] * ekf.X[6] + ekf.X[7] * ekf.X[7] + ekf.X[8] * ekf.X[8] + ekf.X[9] * ekf.X[9]);
    ekf.X[6]  *= invqmag;
    ekf.X[7]  *= invqmag;
    ekf.X[8]  *= invqmag;
    ekf.X[9]  *= invqmag;
    // Update Nav solution structure
    Nav.Pos[0] = ekf.X[0];
    Nav.Pos[1] = ekf.X[1];
    Nav.Pos[2] = ekf.X[2];
    Nav.Vel[0] = ekf.X[3];
    Nav.Vel[1] = ekf.X[4];
    Nav.Vel[2] = ekf.X[5];
    Nav.q[0]   = ekf.X[6];
    Nav.q[1]   = ekf.X[7];
    Nav.q[2]   = ekf.X[8];
    Nav.q[3]   = ekf.X[9];
    Nav.gyro_bias[0] = ekf.X[10];
    Nav.gyro_bias[1] = ekf.X[11];
    Nav.gyro_bias[2] = ekf.X[12];
%}

void MeasurementEq(float X[NUMX], float Be[3], float Y[NUMV])
{
    float q0, q1, q2, q3;

    q0   = X[6];
    q1   = X[7];
    q2   = X[8];
    q3   = X[9];

    // first six outputs are P and V
    Y[0] = X[0];
    Y[1] = X[1];
    Y[2] = X[2];
    Y[3] = X[3];
    Y[4] = X[4];
    Y[5] = X[5];

    // Bb=Rbe*Be
    Y[6] =
        (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * Be[0] +
        2.0f * (q1 * q2 + q0 * q3) * Be[1] + 2.0f * (q1 * q3 -
                                                     q0 * q2) * Be[2];
    Y[7] =
        2.0f * (q1 * q2 - q0 * q3) * Be[0] + (q0 * q0 - q1 * q1 +
                                              q2 * q2 - q3 * q3) * Be[1] +
        2.0f * (q2 * q3 + q0 * q1) * Be[2];
    Y[8] =
        2.0f * (q1 * q3 + q0 * q2) * Be[0] + 2.0f * (q2 * q3 -
                                                     q0 * q1) * Be[1] +
        (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * Be[2];

    // Alt = -Pz
    Y[9] = -1.0f * X[2];
}
