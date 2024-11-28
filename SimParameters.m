clear variables
Ts_plant = 10e-6; % 10  [us]
Ts_control = 100e-6;  % 100 [us]
Ts_MPC = 0.01;    % 

%Let's use a large mechanical constant for now 

%Axis1.Jm = 8.61827e-4; %Motor inertia 
%Axis1.Kt = 4* 0.070504; %Torque constant
%Axis1.Dm = 0.0;
%Axis1.Jl = 41.518; %Load inertia 
%Axis1.Dl = 192.159;
%Axis1.Ks = 1.8071e5; %Torque constant
%Axis1.Rg = 120;

Jm1 = 8.61827e-4; %Motor inertia 
Kt1 = 4* 0.070504; %Torque constant
Dm1 = 0.0;
w1 = 50.0;
zeta1 = 1.0;
Kvp1 = (Jm1*w1*zeta1)/Kt1;
Kvi1 = (w1*w1*Jm1)/Kt1;
Kpp1 = 2.0;

Kt2 = (0.07354) * (4);
Rg2 = 121;
Jm2 = 4.6370e-04;
Dl2 = 183.8800;
Dm2 = 0;
Ks2 = 92831;
Jl2 = 23.9666;
w2 = 50.0;
zeta2 = 1.0;
Kvp2 = (Jm2*w2*zeta2)/Kt2;
Kvi2 = (w2*w2*Jm2)/Kt2;
Kpp2 = 2.0;

Kt3 = (0.0631765) * (4);
Rg3 = 121;
Jm3 = 2.4640e-04;
Dl3 = 89.42;
Dm3 = 0;
Ks3 = 1.7533e+04;
Jl3 = 8.3562;
w3 = 50.0;
zeta3 = 1.0;
Kvp3 = (Jm3*w3*zeta3)/Kt3;
Kvi3 = (w3*w3*Jm3)/Kt3;
Kpp3 = 2.0;

%% State-space representation
n= 2; %Dimension of state vector: We only have q2,q3
ng = 3; %Dimension of Flexibility of work space : x,y,z
% m = 1; %Dimension of input vector: we only have input current
lt = 2; %Dimension of output vector

%Input, Input rate of change and State constraints
T = [1,0; 0,1];        %Constraint selection vector. Select which states are to be subject to state constraints
%T = [1,0 0 ; 0,1,0 ; 0,0,1];        %Constraint selection vector. Select which states are to be subject to state constraints

%% 制約について 要変更
%u1c = 5; %Input constraint i.e. current limit
%du1c = 100; %Input rate of change constraint
%x1c = 100; %State x1 constraint: Angular velocity
%x2c = 50000; %State x2 constraint: Angular position.
%x_MAX = [x1c;
%         x2c];
%x_MIN = [-x1c;
%         -x2c];
u1c = 5;    %Input constraint i.e. q1 limit
u2c = 5;    %Input constraint i.e. q2 limit
%u3c = 5;    %Input constraint i.e. q3 limit
du1c = 100; %Input rate of change i.e. q1 limit
du2c = 100; %Input rate of change i.e. q2 limit
%du3c = 100; %Input rate of change i.e. q3 limit
u_MAX = [u1c;
         u2c];
u_MIN = [-u1c;
         -u2c];
du_MAX = [du1c;
          du2c;];
du_MIN = [-du1c;
          -du2c;];

%Weights (Reference tracking, input penalization and input rate
%penalization)
%w_Y = 1.0;
w_E = 0.0;
w_DU = 0.0;
w_A = 0.0;
%w_EPSILON = 10000; %Slack variable epsilon weight
Vi_max = 1; %Setting same value for all i's for now
Vi_min = 1;

%P: Prediction horizon / L: Control horizon
P = 10;
L = 2;

% [J0,J0_inv] = jacobi3(0,0,0);
J0 = jacobi23(0,0);

% Up = LinearMPC(Ad,Bd,Cd,T,P,L,w_Y,w_U,w_DU,w_EPSILON,u_MAX,u_MIN,du_MAX,du_MIN,x_MAX,x_MIN,xk0,uk0,y_ref);
%[H,R1,R2,R3,D,DU_MAX,DU_MIN,X_MAX,X_MIN,U_MAX,U_MIN,AAbar, M, LT, Abar, Bbar, q1, q2, q3, Tbm, Bbar_block, W_Y, D_DU, W_DU, W_U, LB] = LinearMPCPrecomputer(modelSS_DT,T,P,L,w_Y,w_U,w_DU,w_EPSILON,Vi_max,Vi_min,u_MAX,u_MIN,du_MAX,du_MIN,x_MAX,x_MIN);
[Q,q1,q2,D,DU_MAX,DU_MIN,U_MAX,U_MIN, M, LT, W_E, W_DU, W_A, Sv, Sa, n, ng] = LinearMPCPrecomputer(J0,T,P,L,w_E,w_DU,w_A,u_MAX,u_MIN,du_MAX,du_MIN,Ts_MPC);
% Q = q1+q2+q3;
tic;
%out = sim('OneMassMPC.slx');
toc;

%{
theta_m = out.theta_m;
u_vec = out.iq_ref;
omega_m = out.omega_m;
ref_vec = out.theta_m_ref;
slack_vec = out.slack_variable;

figure;
figure.WindowState = 'maximized';
tiledlayout(2, 2);
nexttile;
plot(theta_m);
hold
plot(ref_vec,'--');
xlabel('Time [s]');
ylabel('Position [rad]');
title('Position');
grid on;

%figure;
nexttile;
plot(omega_m);
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
title('Velocity');
grid on;

%figure;
nexttile;
plot(u_vec);
xlabel('Time [s]');
ylabel('Control effort [Nm]');
title('Torque');
grid on;

%figure;
nexttile;
plot(slack_vec);
xlabel('Time [s]');
ylabel('Slack variable');
title('Slack');
grid on;
%}
disp('Finished');