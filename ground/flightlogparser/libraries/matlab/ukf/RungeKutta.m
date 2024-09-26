function Xnew = RungeKutta(X, U, dT)
dT2 = dT / 2;
Xlast = X;

K1 = ins_state_eq(X, U);
X = Xlast + dT2 * K1';
K2 = ins_state_eq(X, U);
X = Xlast + dT2 * K2';
K3 = ins_state_eq(X, U);
X = Xlast + dT * K3';
K4 = ins_state_eq(X, U);
% X = Xlast + dT2 * K4;

Xnew = Xlast + (dT * (K1 + 2 * K2 + 2 * K3 + K4) * (1 / 6))';
end

