function [Up,qpDetails] = LinearMPCImplicitBlocking(Ad,Bd,Cd,T,P,L,w_Y,w_U,w_DU,P_EPSILON,Vi_max,Vi_min,u_MAX,u_MIN,du_MAX,du_MIN,x_MAX,x_MIN,xk0,uk0,y_ref)

%{
Linear MPC with implicit blocking (blocking is embedded in the cost
function and inequality constraints)

P: Prediction horizon
L: Control horizon
w_Y, w_U, w
%}


%% MPC main parameters
%System properties
dim_A = size(Ad);
dim_B = size(Bd);
dim_C = size(Cd);
dim_T = size(T);
N = dim_A(2); %Dimensions of state vector
M = dim_B(2); %Dimensions of input vector
LY = dim_C(1); %Dimensions of output vector
LT = dim_T(1); %Dimensions of selection vector
MFLN = 100000; %MFLN: Motherfuckingly large number -> Use this instead of Inf/-Inf for solver

%Necessary definitions
W_Y = eye(LY*(P+1))*w_Y; % (Y-Rp)'*W_Y*(Y-Rp)
W_U = eye(M*P)*w_U;      % (Up-Uref)'*W_U*(Up-Uref)
W_DU = eye(M*P)*w_DU;    % dUp'*W_DU*dUp
[Abar,Bbar] = PredictionMatrices(Ad,Bd,P,Cd);
Tconth = [eye(L);zeros(P-L,L-1) ones(P-L,1)];
Tbm = kron(Tconth,eye(M));
D_U = eye(M*L); %For Umin <= D_U*U <= Umax

D_DU = zeros(M*P,M*P);  %For use in cost function
%Fill D_DU:
D_DU(1:M,1:M) = eye(M);
for i=1:P-1
    D_DU(1+M*i:M*(i+1),1+M*(i-1):M*(i+1)) = [-eye(M) eye(M)];
end


D_DU_ineq = zeros(M*L,M*L);  %For DUmin + c0_DU <= D_DU_ineq*DU <= DUmax + c0_DU
%Fill D_DU_ineq:
D_DU_ineq(1:M,1:M) = eye(M);
for i=1:L-1
    D_DU_ineq(1+M*i:M*(i+1),1+M*(i-1):M*(i+1)) = [-eye(M) eye(M)];
end

R_P = repmat(y_ref,P+1,1); %Reference vector (No anticipative action for now)

C0_DU_ineq = zeros(M*L,1);
C0_DU_ineq(1:M,:) = uk0;
C0_DU = zeros(M*P,1);
C0_DU(1:M,:) = uk0;


%For inequality constraints
U_MAX   = repmat(u_MAX,L,1);
U_MIN   = repmat(u_MIN,L,1);
DU_MAX  = repmat(du_MAX,L,1);
DU_MIN  = repmat(du_MIN,L,1);
X_MAX   = repmat(x_MAX,P+1,1);
X_MIN   = repmat(x_MIN,P+1,1);
VX_MAX  = repmat(Vi_max,LT*(P+1),1); %TODO: Add defining constant later
VX_MIN  = repmat(Vi_min,LT*(P+1),1); %TODO: Add defining constant later
[AAbar,BBbar] = PredictionMatrices(Ad,Bd,P,T);
BBbar_block = BBbar*Tbm;
SIG_R1 = [D_U zeros(M*L,1)];            %Input limitation: UMIN < D_U*Up < UMAX
SIG_R2 = [D_DU_ineq zeros(M*L,1)];           %Input rate limitation DU_MIN + C0_DU < D_DU*Up < DU_MAX + C0_DU
SIG_R3 = [BBbar_block -VX_MAX];               %States upper limitation   -Inf < BBbar*Up - VX_MAX*epsilon < BMAX_X
SIG_R4 = [BBbar_block VX_MIN];                %States lower limitation BMIN_X < BBbar*Up - VX_MIN*epsilon < Inf
SIG_R5 = [zeros(1,M*L) ones(1,1)];  %Slack variables lower limitation   0 < epsilon < Inf

SIGMA = [SIG_R1;
        SIG_R2;
        SIG_R3;
        SIG_R4;
        SIG_R5];
    

BMAX_DU = DU_MAX + C0_DU_ineq;
BMIN_DU = DU_MIN + C0_DU_ineq;
BMAX_X = X_MAX - AAbar*xk0;
BMIN_X = X_MIN - AAbar*xk0;

BMAX = [U_MAX;
        BMAX_DU;
        BMAX_X;
        ones(LT*(P+1),1)*MFLN;
        ones(1,1)*MFLN;
        ];
    
    
BMIN = [U_MIN;
        BMIN_DU;        
        -ones(LT*(P+1),1)*MFLN;
        BMIN_X;
        zeros(1,1)
        ];
   

%%%%%%%%%%%%%%%%%

%Cost function 1\2 * H'*z*H + f'*z
%z = [Up | epsilon]^T , where Up-> R(MP x MP), epsilon-> R(1x1)
Bbar_block = Bbar*Tbm;
q1 = Bbar_block'*W_Y*Bbar_block;
q2 = Tbm'*D_DU'*W_DU*D_DU*Tbm;
q3 = Tbm'*W_U*Tbm;
Q = q1 + q2 + q3;
r1 = Abar'*W_Y*Bbar_block;
r2 = W_Y*Bbar_block;
r3 = W_DU*D_DU*Tbm;
R = 2*(xk0'*r1 - R_P'*r2 - C0_DU'*r3); %Assume U_ref=0
H = 2*blkdiag(Q,P_EPSILON);
f = [R';0];



%Constraints
%   D*z<=b     (inequality constraints)
%   Deq*z=beq  (equality constraints)
D = [SIGMA;
    -SIGMA];

b = [BMAX;
    -BMIN];


% [Up,fval,exitflag,qpDetails] = quadprog(H,f,D,b);
opt = mpcInteriorPointOptions;
x0 = zeros(L+1,1); %Cold start
Deq = zeros(0,L+1);
beq = zeros(0,1);
[x,exitflag] = mpcInteriorPointSolver(H,f,D,b,Deq,beq,x0,opt);
Up = x;
qpDetails = exitflag;
% Up = quadprog(H,f);

end