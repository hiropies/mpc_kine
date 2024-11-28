function [AA,BB] = PredictionMatrices(A,B,P,T)
%PredictionMatrices: Generates the prediction matrices for the MPC problem
%[A,B]: State-space representation of system
% P: Prediction horizon value
% T: Optional state selection matrix (Usually empty or C)
%{     
                             AA  = [T | TA | TA^2 ... TA^P]^T

                             BB  = [0 | 0 | 0 ....      0 ]
                                   [TB | 0 | 0 ....      0 ]
                                   [TAB| TB | 0 ....      0 ]
                                          ....
                                   [TA^P-1*B | TA^P-2*B ... TB]
where T ∈ Re(LB x N) where N: Dimension of state vector
                           LB: Dimension of selected state vector
(If T argument is empty, I_NxN will be used.
%}

dim_A = size(A);
dim_B = size(B);
N = dim_A(2);
M = dim_B(2);

if nargin < 4 || isempty(T)
    T = eye(size(A));  % Default value: Identity matrix NxN
    disp('PredictionMatrices: T is empty. Using default value');
end

dim_T = size(T);
LB = dim_T(1);
ROWS_AA = LB*(P+1);
COLS_AA = N;
ROWS_BB = LB*(P+1);
COLS_BB = M*P;

AA = zeros(ROWS_AA,COLS_AA);
BB = zeros(ROWS_BB,COLS_BB);
    
%Fill AA and BB
for i=0:P
    AA(1+LB*i:LB*(1+i),:) = T*A^i;
    for j=0:P-1
       if(i-j-1>=0)
           BB(1+LB*i:LB*(1+i),1+M*j:M*(1+j)) = T*A^(i-j-1)*B;
       end
    end
end

    


end