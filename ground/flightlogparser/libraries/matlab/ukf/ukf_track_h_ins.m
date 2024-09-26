function [ y ] = ukf_track_h_ins( x ,hparam)
Be = [hparam(1), hparam(2), hparam(3)];

q0   = x(7,:);
q1   = x(8,:);
q2   = x(9,:);
q3   = x(10,:);

% first six outputs are P and V
y = zeros(10, size(x,2));
y(1,:) = x(1,:);
y(2,:) = x(2,:);
y(3,:) = x(3,:);
y(4,:) = x(4,:);
y(5,:) = x(5,:);
y(6,:) = x(6,:);

% Bb= Rbe * Be
y(7,:) = ...
    (q0 .* q0 + q1 .* q1 - q2 .* q2 - q3 .* q3) * Be(1) + ...
    2.0 * (q1 .* q2 + q0 .* q3) * Be(2) + 2.0 * (q1 .* q3 - ...
                                                 q0 .* q2) * Be(3);
y(8,:) = ...
    2.0 * (q1 .* q2 - q0 .* q3) * Be(1) + (q0 .* q0 - q1 .* q1 + ...
                                          q2 .* q2 - q3 .* q3) * Be(2) + ...
    2.0 * (q2 .* q3 + q0 .* q1) * Be(3);
y(9,:) = ...
    2.0 * (q1 .* q3 + q0 .* q2) * Be(1) + 2.0 * (q2 .* q3 - ...
                                                 q0 .* q1) * Be(2) + ...
    (q0 .* q0 - q1 .* q1 - q2 .* q2 + q3 .* q3) * Be(3);

% Alt = -Pz
y(10,:) = -1.0 * x(3,:);
end