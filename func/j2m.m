function motor = j2m(joint)
Rgn = [140.254,121.0,121.0,81.0,80.0,50.0];
motor = [0 0 0 0 0 0];
for i = 1:6
    motor(i) = joint(i)*Rgn(i);
end