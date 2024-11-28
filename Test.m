

MFLN = 1e6;
% persistent uk0;
% if isempty(uk0)
%     uk0 = 0;
% end
uk0 = 0;
xk0 = [0;0];
y_ref = 1;

%Reference and previous value generation
R_P = repmat(y_ref,P+1,1); %Reference vector (No anticipative action for now)
C0_DU = zeros(M*P,1);
C0_DU_ineq = zeros(M*L,1);
C0_DU(1:M,:) = uk0; %
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