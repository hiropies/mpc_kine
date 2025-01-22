function joint = inv_kine(Goal)
%Goal = [x,y,z]; mm
Lac = 0.2685;
Lb  = 0.16;
Ld  = 0.0880;
Le  = 0.56;
Lf  = 0.088;
Lg  = 0.13;
Lh1 = 0.145;
Lh2 = 0.455;
Lii = 0.04;
Lj  = 0.2;
Lk  = 0.05;

x = Goal(1)/1000;
y = Goal(2)/1000;
z = Goal(3)/1000;
%x = Goal(1);
%y = Goal(2);
%z = Goal(3);

Axis0 = atan2(y,x)/pi*180;
goal0 = kinematics(Axis0,0,0,0,0,0);
p1 = [goal0.joint(2,1),goal0.joint(2,2),goal0.joint(2,3)];
%disp(p1)

Larm = sqrt((Lh1+Lh2+Lj+Lk)^2 + (Lii+Lg)^2);
L2ph = sqrt((x-p1(1))^2 + (y-p1(2))^2 + (z-p1(3))^2);

aa = -(Le^2 + (Larm)^2 - L2ph^2)/(2*Le*Larm);
Theta2 = acos(aa)/pi*180;
Axis2 = Theta2 - (90 - acos((Lh1+Lh2+Lj+Lk)/Larm)/pi*180);

xx = x-p1(1);
yy = y-p1(2);
zz = z-0;
Lgph = sqrt(xx^2 + yy^2 + zz^2);
ab = (Le^2 + L2ph^2 - Larm^2)/(2*Le*L2ph);
ac = (Lac^2 + L2ph^2 - Lgph^2)/(2*Lac*L2ph);
Theta1_1 = acos(ab);
Theta1_2 = acos(ac);
Axis1 = (pi - (Theta1_1 + Theta1_2))/pi*180;

goal = kinematics(Axis0,Axis1,Axis2,0,0,0);
ph = [goal.full(12,1),goal.full(12,2),goal.full(12,3)];
disp(ph)

%% 上位プログラムと整合のため、1軸及び2軸に-1をかけていたが、バグの温床となるため消去。
%% 全体の見直しが必要
joint = [Axis0,Axis1,Axis2,0,0,0];