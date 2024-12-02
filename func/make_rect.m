function goal = make_rect(scale,T,Ts)
X_max = scale(1)/2;
X_min = -scale(1)/2;
Y_max = scale(2)/2;
Y_min = -scale(2)/2;

miniT = (T/8);
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
pos_y1 = trapezoidal(10,T1,Ts)+start(2);
pos_x1(length(pos_x1)) =  [];
pos_y1(length(pos_y1)) =  [];

T23 = miniT*2;
start_x = pos_x1(length(pos_x1));
start_y = pos_y1(length(pos_y1));
pos_x23 = trapezoidal(-20,T23,Ts)+start_x;
pos_y23 = trapezoidal(0,T23,Ts)+start_y;
pos_x23(length(pos_x23)) =  [];
pos_y23(length(pos_y23)) =  [];

T45 = miniT*2;
start_x = pos_x23(length(pos_x23));
start_y = pos_y23(length(pos_y23));
pos_x45 = trapezoidal(0,T45,Ts)+start_x;
pos_y45 = trapezoidal(-20,T45,Ts)+start_y;
pos_x45(length(pos_x45)) =  [];
pos_y45(length(pos_y45)) =  [];

T67 = miniT*2;
start_x = pos_x45(length(pos_x45));
start_y = pos_y45(length(pos_y45));
pos_x67 = trapezoidal(20,T67,Ts)+start_x;
pos_y67 = trapezoidal(0,T67,Ts)+start_y;
pos_x67(length(pos_x67)) =  [];
pos_y67(length(pos_y67)) =  [];

T8 = miniT;
start_x = pos_x67(length(pos_x67));
start_y = pos_y67(length(pos_y67));
pos_x8 = trapezoidal(0,T8,Ts)+start_x;
pos_y8 = trapezoidal(10,T8,Ts)+start_y;
% pos_x8(length(pos_x8)) =  [];
% pos_y8(length(pos_y8)) =  [];

x = cat(1,cat(1,cat(1,cat(1,pos_x1,pos_x23),pos_x45),pos_x67),pos_x8)';
y = cat(1,cat(1,cat(1,cat(1,pos_y1,pos_y23),pos_y45),pos_y67),pos_y8)';

goal.pos = [time; x; y; z];
goal.step = step;
goal.Xmin = X_min;
goal.Ymin = Y_min;