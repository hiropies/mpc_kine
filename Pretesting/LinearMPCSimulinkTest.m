function [u, epsilon] = LinearMPCSimulinkTest(xk0, y_ref, H, R1, R2, R3, D, DU_MAX, DU_MIN, X_MAX, X_MIN, U_MAX, U_MIN, AAbar, P, LT, M, L)


%{
LinearMPC for Simulink

Solve the following quadratic program for every sample:

min  0.5*z'*H*z + f'*z
z
    s.t.  Dz<=b

where z = [Up | epsilon]' = [u{k} u{k+2} u{k+3} ... u{k+l-1} epsilon]'


***Required parameters (Detailed explanation given below):
For cost function: [H, R1, R2, R3]
For inequality constraints: [D, DU_MAX, DU_MIN, X_MAX, X_MIN, U_MAX, U_MIN, AAbar]
For both: [P, M, LT, L]
(P: Prediction horizon, L: Control horizon, LT: Number of constrained states, M: Input dimension) 
(Use LinearMPCPrecomputer function to compute)


***Input arguments:
-xk0: Last sample state vector X
-uk0: Last sample input vector u
-y_ref: Reference

***Output:
-Up: Computed optimal input vector over prediction horizon
-epsilon: Slack variable value
-x_predic: Predicted state vector over prediction horizon

NOTE: Some matrices need to be multiplied either with xk0 or uk0 to form
the whole quadratic program to be solved, so the precomputed required
parameters are given as below:


%}


MFLN = 1e5;
persistent uk0;
if isempty(uk0)
    uk0 = 0;
end


%Reference and previous value generation
R_P = repmat(y_ref,P+1,1); %Reference vector (No anticipative action for now)
C0_DU = zeros(M*P,1);
C0_DU_ineq = zeros(M*L,1);
C0_DU(1:M,:) = uk0; 
C0_DU_ineq(1:M,:)=uk0;


 

%Cost function 1\2 * H'*z*H + f'*z
%H is already precomputed
%z = [Up | epsilon]^T , where Up-> R(MP x MP), epsilon-> R(1x1)
R = 2*(xk0'*R1 - R_P'*R2 - C0_DU'*R3); %Assume U_ref=0
f = [R';0];



%Constraints
%   D*z<=b     (inequality constraints)
%D is already precomputed, b needs to be computed
%Inequalities: [D, DU_MAX, DU_MIN, X_MAX, X_MIN, U_MAX, U_MIN, MFLN, AAbar] needs to be precomputed
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


b = [BMAX;
    -BMIN];



%Solve quadratic program
%{
min  0.5*z'*H*z + f'*z
z
    s.t.  Dz<=b

where z = [Up | epsilon]' = [u{k} u{k+2} u{k+3} ... u{k+L-1} epsilon]'
%}


opt = mpcInteriorPointOptions;
x0 = zeros(L+1,1); %Cold start
Deq = zeros(0,L+1);
beq = zeros(0,1);
[z,exitflag] = mpcInteriorPointSolver(H,f,D,b,Deq,beq,x0,opt);
u = z(1);
epsilon = z(L+1);
uk0 = u;