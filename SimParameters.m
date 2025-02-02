clear variables
% load("reference.mat");
addpath('func\');
run("make_cmd.m");
load("ref23.mat"); %ref23.matは~~~.mで作成

Ts = 100e-6;
Ts_plant = 10e-6; % 10  [us]
Ts_control = 100e-6;  % 100 [us]
%Ts_MPC = 0.01;    % 
Ts_MPC = Ts;

long_cmd = length(pos_ref2);
pos_ref2_tt = array2timetable(pos_ref2,'SampleRate',1/Ts);
pos_ref3_tt = array2timetable(pos_ref3,'SampleRate',1/Ts);
% q2_init = pos_ref2(1);
% q3_init = pos_ref3(1);

Kt1 = 4* 0.070504;
Rg1 = 120;
Jm1 = 8.61827e-4;
Dl1 = 192.159;
Dm1 = 0.0;
Ks1 = 1.8071e5;
Jl1 = 41.518;
w1 = 50.0;
zeta1 = 1.0;
Kvp1 = (Jm1*w1*zeta1)/Kt1;
Kvi1 = (w1*w1*Jm1)/Kt1;
Kpp1 = 10.0;

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
Kpp2 = 10.0;

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
Kpp3 = 10.0;

%% State-space representation
n= 2; %Dimension of state vector: We only have q2,q3
ng = 2; %Dimension of Flexibility of work space : x,z
lt = 2; %Dimension of output vector

%Input, Input rate of change and State constraints
T = [1,0; 0,1];        %Constraint selection vector. Select which states are to be subject to state constraints
%T = [1,0 0 ; 0,1,0 ; 0,0,1];        %Constraint selection vector. Select which states are to be subject to state constraints

%% 制約について
% u1c = 5;    %Input constraint i.e. q1 limit
u2c = 50 * pi / 180.0;    %Input constraint i.e. q2 limit
u3c = 50 * pi / 180.0;   %Input constraint i.e. q3 limit
% du1c = 100; %Input rate of change i.e. q1 limit
% du2c = 100; %Input rate of change i.e. q2 limit
% du3c = 100; %Input rate of change i.e. q3 limit
du2c = 200 * pi / 180; %Input rate of change i.e. q2 limit
du3c = 200 * pi / 180; %Input rate of change i.e. q3 limit
% du2c = 50 / Rg2; %Input rate of change i.e. q2 limit
% du3c = 50 / Rg3; %Input rate of change i.e. q3 limit

u_MAX = [u2c;
         u3c];
u_MIN = [-u2c;
         -u3c];
du_MAX = [du2c;
          du3c;];
du_MIN = [-du2c;
          -du3c;];

%Weights (Reference tracking, input penalization and input rate penalization)
w_E = 10.0;
w_DU = 0.001;
w_A = 0.0;

%P: Prediction horizon / L: Control horizon
P = 10;
L = 2;

% [J0,J0_inv] = jacobi3(0,0,0);
% J0 = jacobi23(q2_init,q3_init);
J0 = jacobi23(0,0);

[Q,q1,q2,D,DU_MAX,DU_MIN,U_MAX,U_MIN, M, LT, W_E, W_DU, W_A, Sv, Sa, n, ng] = LinearMPCPrecomputer(J0,T,P,L,w_E,w_DU,w_A,u_MAX,u_MIN,du_MAX,du_MIN,Ts_MPC);

tic;
% out = sim('KineMPC.slx');
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
save("param.mat");
params = load("param.mat");
disp('Finished');
%}