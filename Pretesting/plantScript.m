%% Linear MPC implementation for double integrator

Ts = 0.01; %1 ms
Tsim = 10;
i_max = Tsim/Ts;


%Let's use a large mechanical constant for now 
Jm = 0.1; %Motor inertia 
Kt = 1; %Torque constant
Dm = 1e-2;
u1c = 5; %Input constraint i.e. current limit
du1c = 100; %Input rate of change constraint
x1c = 20; %State x1 constraint: Angular velocity
x2c = 50000; %State x2 constraint: Angular position.

%% State-space representation
n= 2; %Dimension of state vector: We only have x1: ang. velocity and x2: ang. position
m = 1; %Dimension of input vector: we only have input current
lt = 1; %Dimension of output vector
Ac = [-Dm/Jm 0; 1 0];
Bc = [Kt/Jm ; 0];
Cc = [0, 1];
Dc = [0];
modelSS_CT = ss(Ac,Bc,Cc,Dc);
modelSS_DT = c2d(modelSS_CT,Ts); %ZOH discretization
Ad = modelSS_DT.A;
Bd = modelSS_DT.B;
Cd = modelSS_DT.C;

t_vec = zeros(1,i_max+1);
u_vec = zeros(m,i_max+1);
y_vec = zeros(lt,i_max+1);
ref_vec = zeros(lt,i_max+1);
slack_vec = zeros(1,i_max+1);
x_vec = zeros(n,i_max+1);


xk0 = zeros(n,1); %Initial condition
x = xk0;
uk0 = zeros(m,1); %Initial input condition
u = uk0;
y_ref = 0; %Reference to follow (step)
y = zeros(lt,1);
t = 0;

%Input, Input rate of change and State constraints
T = [1,0;0,1];        %Constraint selection vector. Select which states are to be subject to state constraints
u_MAX = [u1c];
u_MIN = [-u1c];
du_MAX = [du1c];
du_MIN = [-du1c];
x_MAX = [x1c;
    x2c];
x_MIN = [-x1c;
    -x2c];



%Weights (Reference tracking, input penalization and input rate
%penalization)
w_Y = 1;
w_U = 0;
w_DU = 0.1;
w_EPSILON = 10000; %Slack variable epsilon weight
Vi_max = 1; %Setting same value for all i's for now
Vi_min = 1;

%P: Prediction horizon / L: Control horizon
P = 20;
L = 2;

% Up = LinearMPC(Ad,Bd,Cd,T,P,L,w_Y,w_U,w_DU,w_EPSILON,u_MAX,u_MIN,du_MAX,du_MIN,x_MAX,x_MIN,xk0,uk0,y_ref);
[H,R1,R2,R3,D,DU_MAX,DU_MIN,X_MAX,X_MIN,U_MAX,U_MIN,AAbar, M, LT] = LinearMPCPrecomputer(modelSS_DT,T,P,L,w_Y,w_U,w_DU,w_EPSILON,Vi_max,Vi_min,u_MAX,u_MIN,du_MAX,du_MIN,x_MAX,x_MIN);

%Simulation

for i=1:i_max+1   

    t = t + Ts;

    if(t>=2)
        y_ref = 6*pi;
    end

    [u, slack] = LinearMPCSimulinkTest(xk0, y_ref, H, R1, R2, R3, D, DU_MAX, DU_MIN, X_MAX, X_MIN, U_MAX, U_MIN, AAbar, P, LT, M, L);
   % [Up,qpDetails] = LinearMPCImplicitBlocking(Ad,Bd,Cd,T,P,L,w_Y,w_U,w_DU,w_EPSILON,Vi_max,Vi_min,u_MAX,u_MIN,du_MAX,du_MIN,x_MAX,x_MIN,xk0,uk0,y_ref);
   % u = Up(1);
   % slack = Up(end);

    x = Ad*x + Bd*u;
    y = Cd*x;

    xk0 = x;
    uk0 = u;

    t_vec(:,i)=t;
    x_vec(:,i)=x;
    y_vec(:,i)=y;
    u_vec(:,i)=u;   
    slack_vec(:,i)=slack;
    ref_vec(:,i)=y_ref;

end

omega_m = x_vec(1,:);
theta_m = x_vec(2,:);
theta_m_sensor = y_vec;

figure;
plot(t_vec,theta_m);
hold
plot(t_vec,theta_m_sensor,'--');
plot(t_vec,ref_vec,'--');
xlabel('Time [s]');
ylabel('Position [rad]');
grid on;

figure;
plot(t_vec,omega_m);
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
grid on;

figure;
plot(t_vec,u_vec);
xlabel('Time [s]');
ylabel('Control effort [Nm]');
grid on;

figure;
plot(t_vec,slack_vec);
xlabel('Time [s]');
ylabel('Slack variable');
grid on;

