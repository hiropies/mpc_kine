clear variables
Ts = 100e-6;
size = [20,0];
T = 2.0;
theta = -90.0;
axis = 'y';
slide = [1274.6,0,246.6];
% goal = make_rect(size,T,Ts);
goal = make_23link(size,T,Ts);
goal_shift = shift(goal,slide,theta,axis);
%goal_shift = goal;

x = goal.pos(2,:);
y = goal.pos(3,:);
z = goal.pos(4,:);
x_ = goal_shift.pos(2,:);
y_ = goal_shift.pos(3,:);
z_ = goal_shift.pos(4,:);
time = goal_shift.pos(1,:);

L = length(goal_shift.pos(1,:));
joint = zeros(L,6);
motor = zeros(L,6);
motor_rad = zeros(L,6);
joint_rad = zeros(L,6);
for i = 1:L
    joint(i,:) = inv_kine(goal_shift.pos(2:4,i)');
    motor(i,:) = j2m(joint(i,:));
    motor_rad(i,:) = deg2rad(motor(i,:));
    joint_rad(i,:) = deg2rad(joint(i,:));
end
LH = length(motor_rad);
motor_rad = reshape(motor_rad,[LH(1),6]);
joint_rad = reshape(joint_rad,[LH(1),6]);
cmd_rad = joint_rad;

time_ramp = 1.0;
pos_ref_go = zeros((time_ramp+3.0)/Ts+1,6);
pos_ref_end = zeros((1.0)/Ts,6);
for i = 1:6
    pos_go = trapezoidal(cmd_rad(1,i),time_ramp,Ts);
    ll = length(pos_go);
    pos_ref_go(:,i) = pos_go(ll);
    pos_ref_go(1:ll,i) = pos_go;
    pos_ref_end(:,i) = cmd_rad(LH(1),i);
end

pos_go_traj = vertcat(pos_ref_go,cmd_rad);
pos_go_traj = vertcat(pos_go_traj,pos_ref_end);

pos_ref = array2timetable(pos_go_traj,'SampleRate',1/Ts);
pos_ref2 = array2timetable(pos_go_traj(:,2),'SampleRate',1/Ts);
pos_ref3 = array2timetable(pos_go_traj(:,3),'SampleRate',1/Ts);
qref2_M_deg = rad2deg_array(pos_ref2.Var1);
qref3_M_deg = rad2deg_array(pos_ref3.Var1);
qref2_M_rad = deg2rad_array(qref2_M_deg);
qref3_M_rad = deg2rad_array(qref3_M_deg);
pos_ref2 = qref2_M_rad;
pos_ref3 = qref3_M_rad;
%pos_ref2 = array2timetable(qref2_M_deg,'SampleRate',1/Ts,'VariableNames',{'Var1'});
%pos_ref3 = array2timetable(qref3_M_deg,'SampleRate',1/Ts,'VariableNames',{'Var1'});
%save('reference.mat',"pos_ref");
save('ref23.mat',"pos_ref2","pos_ref3");
% save('D:\user\Desktop\abe\matlab\sim\20240918_軌跡シミュ_四角\reference.mat',"pos_ref");

x_ = goal_shift.pos(2,:);
y_ = goal_shift.pos(3,:);
z_ = goal_shift.pos(4,:);
%% figure作成
set(0,'DefaultTextInterpreter','latex')
set(0,'DefaultLegendInterpreter','latex')
clf;
font=25;
legend_font=25;
linewidth = 3;

fig1 = figure(1);
fig1.WindowState = 'maximized';
tiledlayout(3,2)

%% 1枚目
nexttile(1,[3,1])
h11 = plot3(x_,y_,z_);
set(h11, 'linewidth',linewidth);
hold on;
% 表示範囲指定
caz = -38.5173;
cel =  10.2403;
view(caz,cel);
xrange = [-0.1 0.1];
yrange = [-0.2 1.0];
zrange = [-0.0 0.001];
%set(gca, 'Xlim',xrange); 
%set(gca, 'Ylim',yrange); 
%set(gca, 'Zlim',zrange);
xlabel({'X[m]'},'FontSize',font, 'color','k');
ylabel({'Y[m]'},'FontSize',font, 'color','k');
zlabel({'Z[m]'},'FontSize',font, 'color','k');
grid on;

%% 4枚目
nexttile
h41 = plot(time,z_);
set(h41, 'linewidth',linewidth);
hold on;
% 表示範囲指定
xrange = [0 2502];
yrange = [-0.2 1.0];
%set(gca, 'Xlim',xrange); 
%set(gca, 'Ylim',yrange);
xlabel({'time[s]'},'FontSize',font, 'color','k');
ylabel({'Z[m]'},'FontSize',font, 'color','k');
grid on;

%% 3枚目
nexttile
h31 = plot(pos_ref.Time,pos_ref2);
set(h31, 'linewidth',linewidth);
hold on;
% 表示範囲指定
%xrange = [0 2502];
%yrange = [-0.2 1.0];
%set(gca, 'Xlim',xrange); 
%set(gca, 'Ylim',yrange);
xlabel({'time[s]'},'FontSize',font, 'color','k');
ylabel({'Axis2_M[rad]'},'FontSize',font, 'color','k');
grid on;

%% 2枚目
nexttile
h21 = plot(pos_ref.Time,pos_ref3);
set(h21, 'linewidth',linewidth);
hold on;
% 表示範囲指定
%xrange = [0 2502];
%yrange = [-0.2 1.0];
%set(gca, 'Xlim',xrange); 
%set(gca, 'Ylim',yrange);
xlabel({'time[s]'},'FontSize',font, 'color','k');
ylabel({'Axis3_M[rad]'},'FontSize',font, 'color','k');
grid on;