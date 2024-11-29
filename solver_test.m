clear variables;
H = [1 -1; -1 2];
f = [-2; -6];
A = [-1 0; 0 -1; 1 1; -1 2; 2 1];
b = [0; 0; 2; 2; 3];
n = length(f);
Aeq = zeros(0,n);
beq = zeros(0,1);
[~,p] = chol(H);
opt = mpcInteriorPointOptions;
x0 = zeros(n,1);
[x,exitflag] = mpcInteriorPointSolver(H,f,A,b,Aeq,beq,x0,opt);

