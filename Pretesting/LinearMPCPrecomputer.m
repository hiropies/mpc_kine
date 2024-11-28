function [H,R1,R2,R3,D,DU_MAX,DU_MIN,X_MAX,X_MIN,U_MAX,U_MIN,AAbar, M, LT] = LinearMPCPrecomputer(disc_SS,T,P,L,w_Y,w_U,w_DU,w_EPSILON,Vi_max,Vi_min,u_MAX,u_MIN,du_MAX,du_MIN,x_MAX,x_MIN)
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

Ad = disc_SS.A;
Bd = disc_SS.B;
Cd = disc_SS.C;

dim_A = size(Ad);
dim_B = size(Bd);
dim_C = size(Cd);
dim_T = size(T);
N = dim_A(2); %Dimensions of state vector
M = dim_B(2); %Dimensions of input vector
LY = dim_C(1); %Dimensions of output vector
LT = dim_T(1); %Dimensions of selection vector


W_Y = eye(LY*(P+1))*w_Y; % (Y-Rp)'*W_Y*(Y-Rp)
W_U = eye(M*P)*w_U;      % (Up-Uref)'*W_U*(Up-Uref)
W_DU = eye(M*P)*w_DU;    % dUp'*W_DU*dUp
[Abar,Bbar] = PredictionMatrices(Ad,Bd,P,Cd);
Tconth = [eye(L);zeros(P-L,L-1) ones(P-L,1)];
Tbm = kron(Tconth,eye(M)); %Blocking matrix: Tconth (x) I_mxm 
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

%For inequality constraints
U_MAX   = repmat(u_MAX,L,1);
U_MIN   = repmat(u_MIN,L,1);
DU_MAX  = repmat(du_MAX,L,1);
DU_MIN  = repmat(du_MIN,L,1);
X_MAX   = repmat(x_MAX,P+1,1);
X_MIN   = repmat(x_MIN,P+1,1);
VX_MAX  = repmat(Vi_max,LT*(P+1),1);  %1 for now, will modify in future
VX_MIN  = repmat(Vi_min,LT*(P+1),1);  %1 for now, will modify in future
[AAbar,BBbar] = PredictionMatrices(Ad,Bd,P,T); %Prediction for constrained states
BBbar_block = BBbar*Tbm;
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
    

%Cost function 1\2 * H'*z*H + f'*z
%z = [Up | epsilon]^T , where Up-> R(MP x MP), epsilon-> R(1x1)
Bbar_block = Bbar*Tbm;
q1 = Bbar_block'*W_Y*Bbar_block;
q2 = Tbm'*D_DU'*W_DU*D_DU*Tbm;
q3 = Tbm'*W_U*Tbm;
Q = q1 + q2 + q3;
H = 2*blkdiag(Q,w_EPSILON);
R1 = Abar'*W_Y*Bbar_block;
R2 = W_Y*Bbar_block;
R3 = W_DU*D_DU*Tbm;



%Constraints
D = [SIGMA;
    -SIGMA];



end