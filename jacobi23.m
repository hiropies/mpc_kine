function [J] = jacobi23(q2,q3)
% q1,2,3はdegで与える
%clear variables;
%q1 = 0;
%q2 = 0;
%q3 = 0;

% リンク長
Lac = 0.2685;
Lb  = 0.16;
Ld  = 0.0880;
Le  = 0.56;
Lf  = 0.088;
Lg  = 0.13;
Lh1 = 0.145;
Lh2 = 0.455;
Lii = 0.04;
Lj  = 0.2;
Lk  = 0.05;

L1 = Lb;
L2 = Le;
L3 = Lg + Lii;
L4 = Lh1 + Lh2 + Lj + Lk;

% q2,3はdegで与える。ここでradに変換。
theta2 = q2 / 180 * pi;
theta3 = q3 / 180 * pi;

% 三角関数
C1 = cos(0);
S1 = sin(0);
S2 = sin(theta2);
C2 = cos(theta2);
S23 = sin(theta2 + theta3);
C23 = cos(theta2 + theta3);

% 各要素の偏微分
% dxdthe1 = -S1*(L1 + L2*S2 + L3*S23 + L4*C23);
dxdthe2 = C1*(L2*C2 + L3*C23 - L4*S23);
dxdthe3 = C1*(L3*C23 - L4*S23);
% dydthe1 = C1*(L1 + L2*S2 + L3*S23 + L4*C23);
dydthe2 = S1*(L2*C2 - L3*C23 - L4*S23);
dydthe3 = S1*(L3*C23 - L4*S23);
% dzdthe1 = 0;
dzdthe2 = -L2*S2 - L3*S23 - L4*C23;
dzdthe3 = -L3*S23 - L4*C23;

J = [dxdthe2 dxdthe3;
     dzdthe2 dzdthe3];
% J_inv = inv(J);

end