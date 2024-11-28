function [Q,q1,q2,D,DU_MAX,DU_MIN,U_MAX,U_MIN, M, LT, W_E, W_DU, W_A, Sv, Sa, n, LH] = LinearMPCPrecomputer(J0,T,P,L,w_E,w_DU,w_A,u_MAX,u_MIN,du_MAX,du_MIN,Ts)
%LinearMPCPrecomputer
%   Precomputes matrices for linear MPC to reduce computational cost during
%   simulation

%Input arguments:
%{
disc_SS: Discrete-time representation of system discretized by
sampling Ts_MPC, which should contain [Ad, Bd, Cd]

T: State constraint selection matrix, selects which states are to be
subject to constraints

[P,L]: Prediction and control horizons

[w_Y, w_U, w_DU, w_EPSILON]: Penalizing costs for reference tracking,
input, input variation and slack variable

[Vi_max, Vi_min]: Scaling gains for slack variables
0 yields hard constraints, 1 -> soft constraint

[u_MAX,u_MIN,du_MAX,du_MIN,x_MAX,x_MIN]: Max and min value vectors for
input, input variation and selected states.
%}

%Ad = disc_SS.A;
%Bd = disc_SS.B;
%Cd = disc_SS.C;

%dim_A = size(Ad);
%dim_B = size(Bd);
%dim_C = size(Cd);
dim_T = size(T);
n = size(J0,2); %Dimensions of state vector (q_kの長さ)
M = size(J0,2); %Dimensions of input vector (q_kの長さ) 運動学MPCにおいては入出力次元は等しい。
% LY = size(disc_SS,2); %Dimensions of output vector (q_kの長さ) nに等しい。
LH = size(J0,1); %Dimensions of output vector (g(q)の行数、つまりJ(q)の行数)-> 作業空間の自由度
LT = dim_T(1); %Dimensions of selection vector


W_E = eye(LH*P)*w_E;      % J_k'*W_Y*(J_k)
W_DU = eye(n*P)*w_DU;   % (Sv*q + v)'*W_DU*(Sv*q + v)
W_A = eye(n*P)*w_A;   % (Sa*q + a)'*W_A*(Sa*q + a)

%% 要変更 適応するようA,Bを変更する。
% 制約行列の生成
% [Abar,Bbar,LB] = PredictionMatrices(Ad,Bd,P,Cd);

% P = 10;
% L = 3;
%Tconth = [eye(L);zeros(P-L,L-1) ones(P-L,1)];
%Tbm = kron(Tconth,eye(M)); %Blocking matrix: Tconth (x) I_mxm 

%{
D_U = eye(M*L); %for U_MIN <= D_U*Up <= U_MAX, usually identity matrix
D_DU = zeros(M*P,M*P);  %Computes difference vector [du{k} du{k+1} ... du{k+P-1}] from D_DU*Up (du{i} = u{i} - u{i-1})
D_DU_ineq = zeros(M*L,M*L); %Computes difference vector [du{k} du{k+1} ... du{k+L-1}] from D_DU*Up (du{i} = u{i} - u{i-1})
D_DU(1:M,1:M) = eye(M);
%Fill D_DU:
for i=1:P-1 
    D_DU(1+M*i:M*(i+1),1+M*(i-1):M*(i+1)) = [-eye(M) eye(M)]; %For cost function
end
%Fill D_DU_ineq
D_DU_ineq(1:M,1:M) = eye(M);  %For DUmin + c0_DU <= D_DU_ineq*DU <= DUmax + c0_DU
for i=1:L-1
    D_DU_ineq(1+M*i:M*(i+1),1+M*(i-1):M*(i+1)) = [-eye(M) eye(M)];
end
%}
%% Sv,Saを宣言する。a,vに関してはループ内で。1/deltaT要素を含ませる必要あり。
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

%{
% [AAbar,BBbar,LB] = PredictionMatrices(Ad,Bd,P,T); %Prediction for constrained states
% BBbar_block = BBbar*Tbm;

%% 制約の条件項D
SIG_R1 = [D_U zeros(M*L,1)];            %Input limitation: UMIN < D_U*Up < UMAX
SIG_R2 = [D_DU_ineq zeros(M*L,1)];           %Input rate limitation DU_MIN + C0_DU_ineq < D_DU_ineq*Up < DU_MAX + C0_DU_ineq
SIG_R3 = [BBbar_block -VX_MAX];               %States upper limitation   -Inf < BBbar_block*Up - VX_MAX*epsilon < BMAX_X
SIG_R4 = [BBbar_block VX_MIN];                %States lower limitation BMIN_X < BBbar_block*Up - VX_MIN*epsilon < Inf
SIG_R5 = [zeros(1,M*L) ones(1,1)];  %Slack variables lower limitation   0 < epsilon < Inf

SIGMA = [SIG_R1;
        SIG_R2;
        SIG_R3;
        SIG_R4;
        SIG_R5];
%}

%Cost function 1\2 * x'*H*x + f'*x
%z = [Up | epsilon]^T , where Up-> R(MP x MP), epsilon-> R(1x1)
% Bbar_block = Bbar*Tbm;
%% Svの項をここに示す。
q1 = Sv'*W_DU*Sv;
%% Saの項
q2 = Sa'*W_A*Sa;

% q3 = Tbm'*W_U*Tbm;
Q = q1 + q2;
%% 通常Hを返すが、KinematicsMPCではJk(qkによって変化する)を含むためそれ以外の要素Qのみを返す。
%% QとJkの合わせこみについては要検討か？ -> Jkのblkdiagを定義すれば解決
% H = 2*blkdiag(Q,w_EPSILON);
%R1 = Abar'*W_Y*Bbar_block;
%R2 = W_Y*Bbar_block;
%R3 = W_DU*D_DU*Tbm;

%Constraints
TTT = size(Sv);
disp(TTT);
SIGMA = [Sv;
         eye(n*P);];
D = [SIGMA;
    -SIGMA];

%%返すもの
% Q,Sv,Sa,Qd,Qa,Qe,D,n,M,LT,LH,Abar,Bbar,q1,q2,Tbm,Bbar_block,W_Y,D_DU,W_DU,W_U,LB
% H,R1,R2,R3,D,DU_MAX,DU_MIN,X_MAX,X_MIN,U_MAX,U_MIN,AAbar,M,LT,Abar,Bbar,q1,q2,q3,Tbm,Bbar_block,W_Y,D_DU,W_DU,W_U,LB
end