function [ Xdot ] = ins_state_eq( X,U )
ax = U(4); ay = U(5); az = U(6);
wx = U(1) - X(11); wy = U(2) - X(12); wz = U(3) - X(13);
q0 = X(7); q1 = X(8); q2 = X(9); q3 = X(10);
% pdot = v
Xdot = zeros(1, 13);
Xdot(1) = X(4); Xdot(2) = X(5); Xdot(3) = X(6);
% vdot = reb * a
Xdot(4) = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * ax ...
    + 2 * (q1 * q2 - q0 * q3) * ay ...
    + 2 * (q1 * q3 + q0 * q2) * az;
Xdot(5) = (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * ay ...
    + 2 * (q1 * q2 + q0 * q3) * ax ...
    + 2 * (q2 * q3 - q0 * q1) * az;
Xdot(6) = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az ...
    + 2 * (q1 * q3 - q0 * q2) * ax ...
    + 2 * (q2 * q3 + q0 * q1) * ay + 9.81;
% qdot = q * w
Xdot(7)  = (-q1 * wx - q2 * wy - q3 * wz) / 2.0;
Xdot(8)  = (q0 * wx - q3 * wy + q2 * wz) / 2.0;
Xdot(9)  = (q3 * wx + q0 * wy - q1 * wz) / 2.0;
Xdot(10)  = (-q2 * wx + q1 * wy + q0 * wz) / 2.0;
% gyro bias guess 0
Xdot(11) = 0; Xdot(12) = 0; Xdot(13) = 0;
end

