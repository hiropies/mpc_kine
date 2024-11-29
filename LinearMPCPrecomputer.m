function [Q,q1,q2,D,DU_MAX,DU_MIN,U_MAX,U_MIN, M, LT, W_E, W_DU, W_A, Sv, Sa, n, LH] = LinearMPCPrecomputer(J0,T,P,L,w_E,w_DU,w_A,u_MAX,u_MIN,du_MAX,du_MIN,Ts)
%LinearMPCPrecomputer
%   Precomputes matrices for linear MPC to reduce computational cost during
%   simulation

%Input arguments:
%{
J0: jacobian with q = 0

T: State constraint selection matrix, selects which states are to be
subject to constraints

[P,L]: Prediction and control horizons

[w_E, w_DU, w_A]: Penalizing costs for reference tracking,
joint velocity, joint accelaration 


[u_MAX,u_MIN,du_MAX,du_MIN]: Max and min value vectors for input, input variation.
%}

dim_T = size(T);
n = size(J0,2); %Dimensions of state vector (q_kの長さ)
M = size(J0,2); %Dimensions of input vector (q_kの長さ) 運動学MPCにおいては入出力次元は等しい。
LH = size(J0,1); %Dimensions of output vector (g(q)の行数、つまりJ(q)の行数)-> 作業空間の自由度
LT = dim_T(1); %Dimensions of selection vector


W_E = eye(LH*P)*w_E;      % J_k'*W_Y*(J_k)
W_DU = eye(n*P)*w_DU;   % (Sv*q + v)'*W_DU*(Sv*q + v)
W_A = eye(n*P)*w_A;   % (Sa*q + a)'*W_A*(Sa*q + a)

%% Sv,Saを宣言する。a,vに関してはループ内で。1/deltaT, 1/deltaT^2要素を含ませる必要あり。
% P = 10;
% M = 3;
SSv = zeros(P,P);
SSv(1,1) = 1;
for i = 1:P-1
    SSv(1+i,i:(i+1)) = [-1 1];
end
SSa = zeros(P,P);
SSa(1,1) = 1;
SSa(2,1:2) = [-2 1];
for i = 2:P-1
    SSa(1+i,(i-1):(i+1)) = [1 -2 1];
end
Sv = kron(SSv,eye(M))/Ts;
Sa = kron(SSa,eye(M))/(Ts*Ts);

%% For inequality constraints
U_MAX   = repmat(u_MAX,P,1);
U_MIN   = repmat(u_MIN,P,1);
DU_MAX  = repmat(du_MAX,P,1);
DU_MIN  = repmat(du_MIN,P,1);

%Constraints
SIGMA = [Sv;
         eye(n*P);];
D = [SIGMA;
    -SIGMA];

%Cost function 1\2 * x'*H*x + f'*x
%z = [Up | epsilon]^T , where Up-> R(MP x MP), epsilon-> R(1x1)
% Bbar_block = Bbar*Tbm;

% Svの項をここに示す。
q1 = Sv'*W_DU*Sv;
% Saの項
q2 = Sa'*W_A*Sa;
% 通常Hを返すが、KinematicsMPCではJk(qkによって変化する)を含むためそれ以外の要素Qのみを返す。
Q = q1 + q2;

end