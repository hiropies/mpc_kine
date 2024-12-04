clear variables;
load("param.mat");
MFLN = 1e5;
k = 5451;
uk0 = zeros(M,1);
duk0 = zeros(M,1);
qref2 = pos_ref2.Var1(k:k+P-1);
qref3 = pos_ref3.Var1(k:k+P-1);%Reference and previous value generation
% qref_P = repmat(qref,P,1);
qref_P = zeros(M*P,1);
Jhat = zeros(M,M,P);
% Jhat_P = kron(eye(P),Jhat);
for c= 1:P
    qref_P(2*(c-1)+1,1) = qref2(c);
    qref_P(2*(c-1)+2,1) = qref3(c);

    % q1,2,3はdegで与える
    q2 = qref2(c);
    q3 = qref3(c);
    
    % リンク長
    Le  = 0.56;
    Lg  = 0.13;
    Lh1 = 0.145;
    Lh2 = 0.455;
    Lii = 0.04;
    Lj  = 0.2;
    Lk  = 0.05;
    
    L2 = Le;
    L3 = Lg + Lii;
    L4 = Lh1 + Lh2 + Lj + Lk;
    
    % q1,2,3はdegで与える。ここでradに変換。
    theta2 = q2 / 180 * pi;
    theta3 = q3 / 180 * pi;
    % 三角関数
    S2 = sin(theta2);
    C2 = cos(theta2);
    S23 = sin(theta2 + theta3);
    C23 = cos(theta2 + theta3);

    % 各要素の偏微分
    % dxdthe1 = -S1*(L1 + L2*S2 + L3*S23 + L4*C23);
    dxdthe2 = (L2*C2 + L3*C23 - L4*S23);
    dxdthe3 = (L3*C23 - L4*S23);
    % dydthe1 = C1*(L1 + L2*S2 + L3*S23 + L4*C23);
    % dydthe2 = S1*(L2*C2 - L3*C23 - L4*S23);
    % dydthe3 = S1*(L3*C23 - L4*S23);
    % dzdthe1 = 0;
    dzdthe2 = -L2*S2 - L3*S23 - L4*C23;
    dzdthe3 = -L3*S23 - L4*C23;
    
    Jhat(:,:,c) = [dxdthe2 dxdthe3;
                   dzdthe2 dzdthe3];
end
Jhat_P = blkdiag(Jhat(:,:,1),Jhat(:,:,2),Jhat(:,:,3),Jhat(:,:,4),Jhat(:,:,5),Jhat(:,:,6),Jhat(:,:,7),Jhat(:,:,8),Jhat(:,:,9),Jhat(:,:,10));