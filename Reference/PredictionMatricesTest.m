%PredictionMatrices - Unit test
%Test 1 - P=4, A:2x2, B:2x1, T=[1 0]
P1 = 4;
A1 = [1 2;
      3 4];

B1 = [5;
      5];

T1 = [1 0];
T1_ZERO = T1*A1*B1*0;

AA1 = [T1;
       T1*A1;
       T1*A1^2;
       T1*A1^3;
       T1*A1^4];

BB1 = [T1_ZERO T1_ZERO T1_ZERO T1_ZERO;
    T1*A1^(P1-4)*B1 T1_ZERO T1_ZERO T1_ZERO;
    T1*A1^(P1-3)*B1 T1*A1^(P1-4)*B1 T1_ZERO T1_ZERO;
    T1*A1^(P1-2)*B1 T1*A1^(P1-3)*B1 T1*A1^(P1-4)*B1 T1_ZERO;
    T1*A1^(P1-1)*B1 T1*A1^(P1-2)*B1 T1*A1^(P1-3)*B1 T1*A1^(P1-4)*B1];

[AA1_Result,BB1_Result] = PredictionMatrices(A1,B1,P1,T1);
assert(isequal(AA1,AA1_Result));
assert(isequal(BB1,BB1_Result));


%Test 2 - P=4, A:3x3, B:3x1, T= [1 0 0 ; 0 1 0] 
P2 = 4;
A2 = [1 2 3;
      4 5 6;
      7 8 9];

B2 = [1;
      2;
      3];

T2 = [1 0 0;
      0 1 0];
T2_ZERO = T2*A2*B2*0;

AA2 = [T2;
       T2*A2;
       T2*A2^2;
       T2*A2^3;
       T2*A2^4];

BB2 = [T2_ZERO T2_ZERO T2_ZERO T2_ZERO;
    T2*A2^(P2-4)*B2 T2_ZERO T2_ZERO T2_ZERO;
    T2*A2^(P2-3)*B2 T2*A2^(P2-4)*B2 T2_ZERO T2_ZERO;
    T2*A2^(P2-2)*B2 T2*A2^(P2-3)*B2 T2*A2^(P2-4)*B2 T2_ZERO;
    T2*A2^(P2-1)*B2 T2*A2^(P2-2)*B2 T2*A2^(P2-3)*B2 T2*A2^(P2-4)*B2];

[AA2_Result,BB2_Result] = PredictionMatrices(A2,B2,P2,T2);
assert(isequal(AA2,AA2_Result));
assert(isequal(BB2,BB2_Result));


%Test 3 - P=4, A:3x3, B:3x1, T= NULL (T=eye(3))
P3 = 4;
A3 = [1 2 3;
      4 5 6;
      7 8 9];

B3 = [1;
      2;
      3];

T3 = eye(3);
T3_ZERO = T3*A3*B3*0;

AA3 = [T3;
       T3*A3;
       T3*A3^2;
       T3*A3^3;
       T3*A3^4];

BB3 = [T3_ZERO T3_ZERO T3_ZERO T3_ZERO;
    T3*A3^(P3-4)*B3 T3_ZERO T3_ZERO T3_ZERO;
    T3*A3^(P3-3)*B3 T3*A3^(P3-4)*B3 T3_ZERO T3_ZERO;
    T3*A3^(P3-2)*B3 T3*A3^(P3-3)*B3 T3*A3^(P3-4)*B3 T3_ZERO;
    T3*A3^(P3-1)*B3 T3*A3^(P3-2)*B3 T3*A3^(P3-3)*B3 T3*A3^(P3-4)*B3];

[AA3_Result,BB3_Result] = PredictionMatrices(A3,B3,P3);
assert(isequal(AA3,AA3_Result));
assert(isequal(BB3,BB3_Result));



%Test 4 - P=1, A:3x3, B:3x1, T= NULL (T=eye(3))
P4 = 1;
A4 = [1 2 3;
      4 5 6;
      7 8 9];

B4 = [1;
      2;
      3];

T4 = eye(3);
T4_ZERO = T4*A4*B4*0;

AA4 = [T4;
       T4*A4;
       ];

BB4 = [T4_ZERO;
       T4*B4];

[AA4_Result,BB4_Result] = PredictionMatrices(A4,B4,P4);
assert(isequal(AA4,AA4_Result));
assert(isequal(BB4,BB4_Result));