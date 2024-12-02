function joint = m2j(motor_Q,Axis)
Rgn = [140.254,121.0,121.0,81.0,80.0,50.0];
joint = motor_Q/Rgn(Axis);