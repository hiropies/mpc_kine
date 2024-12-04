function goal = make_23link(scale,T,Ts)
X_max = scale(1)/2;
X_min = -scale(1)/2;
Y_max = scale(2)/2;
Y_min = -scale(2)/2;
%disp(X_max);
%disp(X_min);
%disp(Y_max);
%disp(Y_min);

miniT = (T/4);
mT_step = miniT/Ts;
step = (T/Ts)+1;
time = linspace(0,T,step);
L = length(time);
x = zeros(1,L);
y = zeros(1,L);
z = zeros(1,L);
z_up = 0.00;
start = [10,0];

T1 = miniT;
pos_x1 = trapezoidal(0,T1,Ts)+start(1);
pos_y1 = trapezoidal(0,T1,Ts)+start(2);
pos_x1(length(pos_x1)) =  [];
pos_y1(length(pos_y1)) =  [];

T2 = miniT;
start_x = pos_x1(length(pos_x1));
start_y = pos_y1(length(pos_y1));
pos_x2 = trapezoidal(X_max,T2,Ts)+start_x;
pos_y2 = trapezoidal(Y_max,T2,Ts)+start_y;
pos_x2(length(pos_x2)) =  [];
pos_y2(length(pos_y2)) =  [];

T3 = miniT;
start_x = pos_x2(length(pos_x2));
start_y = pos_y2(length(pos_y2));
pos_x3 = trapezoidal(0,T3,Ts)+start_x;
pos_y3 = trapezoidal(0,T3,Ts)+start_y;
pos_x3(length(pos_x3)) =  [];
pos_y3(length(pos_y3)) =  [];

T4 = miniT;
start_x = pos_x3(length(pos_x3));
start_y = pos_y3(length(pos_y3));
pos_x4 = trapezoidal(X_min,T4,Ts)+start_x;
pos_y4 = trapezoidal(Y_min,T4,Ts)+start_y;
%pos_x4(length(pos_x4)) =  [];
%pos_y4(length(pos_y4)) =  [];

x = cat(1,cat(1,cat(1,pos_x1,pos_x2),pos_x3),pos_x4)';
y = cat(1,cat(1,cat(1,pos_y1,pos_y2),pos_y3),pos_y4)';

goal.pos = [time; x; y; z];
goal.step = step;
goal.Xmin = X_min;
goal.Ymin = Y_min;