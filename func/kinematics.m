function matrix = kinematics(q1,q2,q3,q4,q5,q6)
% 引数は各関節角度。　Degで入力する。

q(1) = q1;
q(2) = q2;
q(3) = q3;

%% パラメータ
% 各関節角度(deg)
theta1 = q(1);
theta2 = q(2);
theta3 = q(3);
theta4 = q4;
theta5 = q5;
theta6 = q6;

%　各リンクの長さ
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

% 各関節の座標系から見て次の関節がどこにあるか
p01 = [0;0;0]; %絶対座標と原点が同じ
p12 = [Lb; 0; Lac];
p23 = [ 0; -Ld; Le];
p34 = [ Lh1; Lf; Lg];
p45 = [ Lh2; 0; 0];
p56 = [ Lj; 0; Lii];
p6 = [ Lk; 0; 0];

% rotの第二引数はdeg。内部でradに変換して計算される。
% 回転行列　''は回転軸方向
R1 = rot('z',theta1);
R2 = rot('y',theta2);
R3 = rot('y',theta3);
R4 = rot('x',theta4);
R5 = rot('y',theta5);
R6 = rot('x',theta6);
% R7（先端）は回転なしで長さのみなので省略
%% 手先位置・各関節位置 計算

E = eye(3);
zero = [0, 0, 0];

%　同次変換行列
% 0から1
T01 = [R1,p01;zero,1];
% 1から2
T12 = [R2,p12;zero,1];
% 2から3
T23 = [R3,p23;zero,1];
% 3から4
T34 = [R4,p34;zero,1];
% 4から5
T45 = [R5,p45;zero,1];
% 5から6
T56 = [R6,p56;zero,1];
%6から手先
T6h = [E,p6;zero,1];

%基準座標から各関節までの同次変換行列
T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;
T06 = T01*T12*T23*T34*T45*T56;
T0h = T01*T12*T23*T34*T45*T56*T6h;

%各関節位置　
r1 = T01(1:3, 4:4);
r2 = T02(1:3, 4:4);
r3 = T03(1:3, 4:4);
r4 = T04(1:3, 4:4);
r5 = T05(1:3, 4:4);
r6 = T06(1:3, 4:4);
rh = T0h(1:3, 4:4);

% グラフ描画用仮想関節(折れ曲がってるとこ)
sub_p12   = [0;0;Lac];
sub_p23   = [0;-Ld;0];
sub_p34_1 = [0;Lf;0];
sub_p34_2 = [0;Lf;Lg];
sub_p56   = [0;0;Lii]; 

r1_sub = T01*[sub_p12;1];
r2_sub = T02*[sub_p23;1];
r3_sub1 = T03*[sub_p34_1;1];
r3_sub2 = T03*[sub_p34_2;1];
r5_sub = T05*[sub_p56;1];

r1_sub = r1_sub(1:3, 1:1);
r2_sub = r2_sub(1:3, 1:1);
r3_sub1 = r3_sub1(1:3, 1:1);
r3_sub2 = r3_sub2(1:3, 1:1);
r5_sub = r5_sub(1:3, 1:1);

%関節位置行列
matrix.joint = [r1.';r2.';r3.';r4.';r5.';r6.';rh.'];
%描画用位置行列
matrix.full = [r1.'; r1_sub.'; r2.'; r2_sub.'; r3.'; r3_sub1.'; r3_sub2.'; r4.'; r5.'; r5_sub.'; r6.'; rh.'];
% disp(matrix.joint)