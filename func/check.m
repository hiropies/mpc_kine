clear variables;
x = 200;
y = 300;
z = 0;

goal = [x y z];

qj = inv_kine(goal);
hand = kinematics(qj(1),qj(2),qj(3),0,0,0);